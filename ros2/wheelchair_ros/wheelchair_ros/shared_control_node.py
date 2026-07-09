#!/usr/bin/env python3
"""Semi-assisted shared control using joystick intent and LIDAR scans.

The joystick is primary.  This node only reduces speed, adds a limited steering
correction, or stops forward motion when the LIDAR says there is no safe
candidate direction.
"""

from __future__ import annotations

import json
import math
from typing import Optional

from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _as_bool(value: object) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class SharedControl(Node):
    def __init__(self) -> None:
        super().__init__("shared_control")

        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("slow_distance", 1.10)
        self.declare_parameter("max_deviation_deg", 45.0)

        # ── Geometria do LIDAR no base_link (centro do eixo traseiro) ────────
        # O cone angular antigo (front_offset_deg + cone_half_deg) assumia o
        # sensor NO CENTRO da cadeira. Como ele fica 33,6 cm à esquerda, um cone
        # apontado "para a frente" varre uma faixa deslocada: a metade DIREITA
        # da cadeira ficava fora dele e obstáculos ali eram atropelados. Não
        # existe um front_offset correto -- o certo é projetar o scan no
        # base_link e testar o corredor físico que a cadeira vai ocupar.
        # Calibrado 2026-07-09: laser_x por 3 métodos independentes (±1,3 cm),
        # laser_yaw por ajuste de reta em 2 paredes (±0,25°).
        self.declare_parameter("laser_x", 0.667)          # m à frente do eixo
        self.declare_parameter("laser_y", 0.336)          # m à esquerda (+)
        self.declare_parameter("laser_yaw_deg", -155.22)  # yaw do sensor
        # Meia-largura do corredor protegido (cadeira tem 61 cm).
        self.declare_parameter("corridor_half_width", 0.305)
        # Distância do eixo traseiro à frente do chassi (apoios dos pés).
        self.declare_parameter("front_extent", 0.667)
        # Folga p/ não confundir a própria estrutura com obstáculo.
        self.declare_parameter("corridor_margin", 0.02)
        self.declare_parameter("num_candidates", 19)
        self.declare_parameter("w_obstacle", 1.0)
        self.declare_parameter("w_deviation", 0.35)
        self.declare_parameter("blocked_cost", 10.0)
        self.declare_parameter("assist_gain", 0.8)
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("reverse_speed_cap", 0.5)
        # Zona morta do giro: |w| abaixo disto vira 0. Evita o "giro fantasma"
        # ao parar (o eixo X vaza um w pequeno quando se empurra reto p/ frente);
        # um giro deliberado fica acima e ainda passa p/ esterçar/sair.
        self.declare_parameter("turn_deadzone", 0.15)
        # Teto da taxa de giro que o desvio pode injetar (rad/s). Satura o
        # arqueamento para o supervisor nunca dominar o comando do usuário.
        self.declare_parameter("deviation_rate_max", 0.40)
        self.declare_parameter("scan_timeout_s", 0.40)
        self.declare_parameter("intent_timeout_s", 0.40)
        self.declare_parameter("control_rate_hz", 20.0)

        gp = self.get_parameter
        self.stop_d = float(gp("stop_distance").value)
        self.slow_d = float(gp("slow_distance").value)
        self.dev_max = math.radians(float(gp("max_deviation_deg").value))
        self.laser_x = float(gp("laser_x").value)
        self.laser_y = float(gp("laser_y").value)
        self.laser_yaw = math.radians(float(gp("laser_yaw_deg").value))
        self.half_w = float(gp("corridor_half_width").value)
        self.front_extent = float(gp("front_extent").value)
        self.corridor_margin = float(gp("corridor_margin").value)
        self.n_cand = max(3, int(gp("num_candidates").value))
        self.w_obs = float(gp("w_obstacle").value)
        self.w_dev = float(gp("w_deviation").value)
        self.blocked_cost = float(gp("blocked_cost").value)
        self.assist_gain = float(gp("assist_gain").value)
        self.allow_reverse = _as_bool(gp("allow_reverse").value)
        self.rev_cap = float(gp("reverse_speed_cap").value)
        self.turn_deadzone = float(gp("turn_deadzone").value)
        self.dev_rate_max = float(gp("deviation_rate_max").value)
        self.scan_timeout = float(gp("scan_timeout_s").value)
        self.intent_timeout = float(gp("intent_timeout_s").value)
        rate = float(gp("control_rate_hz").value)

        self._scan: Optional[LaserScan] = None
        self._scan_time = 0.0
        self._px: Optional[np.ndarray] = None
        self._py: Optional[np.ndarray] = None
        self._v_user = 0.0
        self._w_user = 0.0
        self._intent_time = 0.0

        self.create_subscription(
            LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "joystick_cmd_vel", self._on_intent, 10)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_status = self.create_publisher(
            String, "wheelchair/assist_status", 10)
        self.create_timer(1.0 / rate, self._control_step)

        # Ajuste ao vivo dos parâmetros de calibração (ros2 param set ...)
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            "shared_control pronto: "
            f"stop={self.stop_d:.2f} m, slow={self.slow_d:.2f} m, "
            f"corredor=±{self.half_w:.3f} m, "
            f"desvio_max=±{math.degrees(self.dev_max):.0f}°, "
            f"laser=({self.laser_x:.3f}, {self.laser_y:+.3f}) m "
            f"yaw={math.degrees(self.laser_yaw):.2f}°, "
            f"frente={self.front_extent:.3f} m")

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            v = p.value
            if p.name == "laser_x":
                self.laser_x = float(v)
            elif p.name == "laser_y":
                self.laser_y = float(v)
            elif p.name == "laser_yaw_deg":
                self.laser_yaw = math.radians(float(v))
            elif p.name == "corridor_half_width":
                self.half_w = float(v)
            elif p.name == "front_extent":
                self.front_extent = float(v)
            elif p.name == "corridor_margin":
                self.corridor_margin = float(v)
            elif p.name == "stop_distance":
                self.stop_d = float(v)
            elif p.name == "slow_distance":
                self.slow_d = float(v)
            elif p.name == "max_deviation_deg":
                self.dev_max = math.radians(float(v))
            elif p.name == "assist_gain":
                self.assist_gain = float(v)
            elif p.name == "turn_deadzone":
                self.turn_deadzone = float(v)
            elif p.name == "deviation_rate_max":
                self.dev_rate_max = float(v)
        return SetParametersResult(successful=True)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_time = self._now()
        # Projeta o scan no base_link uma unica vez por varredura. Os candidatos
        # de desvio depois so rotacionam estes pontos (barato), em vez de
        # re-varrer os feixes com trigonometria por candidato.
        n = len(msg.ranges)
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(n, dtype=np.float64) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
        ca = a[ok] + self.laser_yaw
        rr = r[ok]
        self._px = self.laser_x + rr * np.cos(ca)
        self._py = self.laser_y + rr * np.sin(ca)

    def _on_intent(self, msg: Twist) -> None:
        self._v_user = float(msg.linear.x)
        self._w_user = float(msg.angular.z)
        self._intent_time = self._now()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _control_step(self) -> None:
        now = self._now()
        if (now - self._intent_time) > self.intent_timeout:
            self._publish(0.0, 0.0, "sem_intencao")
            return

        # Zona morta do giro: mata o resíduo do eixo X (evita giro fantasma ao
        # parar) mas deixa passar um giro deliberado.
        w_user = self._w_user
        if abs(w_user) < self.turn_deadzone:
            w_user = 0.0

        if self._scan is None or (now - self._scan_time) > self.scan_timeout:
            v = min(0.0, self._v_user) if self.allow_reverse else 0.0
            self._publish(v, w_user, "sem_scan")
            self.get_logger().warn(
                "sem /scan recente; bloqueando avanco",
                throttle_duration_sec=2.0)
            return

        v_user = self._v_user

        if v_user < 0.0:
            v = v_user * (_clamp(self.rev_cap, 0.0, 1.0)
                          if self.allow_reverse else 0.0)
            self._publish(v, w_user, "re")
            return

        if v_user <= 1e-3:
            self._publish(0.0, w_user, "giro")
            return

        best_delta, front_clear, best_clear, all_blocked = self._best_direction(
            w_user)

        # Parada e velocidade se baseiam na folga REAL à frente (front_clear),
        # não na melhor direção teórica. Com assist_gain baixo a cadeira não
        # esterça o suficiente, então usar best_clear a mandaria a toda
        # velocidade reto contra o obstáculo "achando" que vai desviar. O desvio
        # (assist_gain*best_delta) ainda ajuda a arquear para o lado livre, mas
        # em velocidade reduzida e sem anular a parada frontal.
        if all_blocked or front_clear <= self.stop_d:
            self._publish(0.0, w_user, "para", front_clear, best_clear,
                          best_delta)
            self.get_logger().info(
                "obstaculo: parando avanco",
                throttle_duration_sec=1.0)
            return

        speed_scale = self._speed_scale(front_clear)
        out_v = v_user * speed_scale

        # best_delta é um OFFSET DE RUMO (rad), não uma velocidade angular.
        # Somá-lo direto em out_w o transforma numa taxa sustentada: a cadeira
        # gira sem parar enquanto o obstáculo estiver na zona amarela (chegava a
        # ~180°, piruetando em vez de desviar). O desvio correto é curvatura x
        # velocidade -> escala pelo avanço REAL (out_v). Assim ela ARQUEIA, o
        # giro some junto com o avanço e nunca há pirueta parada. A taxa ainda é
        # saturada p/ o supervisor jamais dominar o comando do usuário.
        dev_rate = _clamp(self.assist_gain * best_delta * out_v,
                          -self.dev_rate_max, self.dev_rate_max)

        # Frente totalmente livre: zero intervention.
        if front_clear >= self.slow_d or math.isinf(front_clear):
            best_delta = 0.0
            dev_rate = 0.0
            speed_scale = 1.0
            out_v = v_user
            mode = "livre"
        else:
            mode = "desvia" if abs(dev_rate) > 0.02 else "freia"

        out_w = w_user + dev_rate
        self._publish(out_v, out_w, mode, front_clear, best_clear, best_delta)

    def _best_direction(self, w_user: float) -> tuple[float, float, float, bool]:
        # Enviesa a escolha para o lado que o usuário já está esterçando. w_user
        # é rad/s e dev_max é ângulo: o clamp aqui é só um viés suave, não uma
        # conversão física.
        center = _clamp(w_user, -self.dev_max, self.dev_max)
        denom = max(self.dev_max, 1e-6)
        if self.n_cand == 1:
            candidates = [0.0]
        else:
            step = (2.0 * self.dev_max) / (self.n_cand - 1)
            candidates = [-self.dev_max + i * step for i in range(self.n_cand)]

        front_clear = self._clearance(0.0)
        best_delta = 0.0
        best_clear = 0.0
        best_cost = math.inf
        all_blocked = True

        for delta in candidates:
            clear = self._clearance(delta)
            obs = self._obstacle_term(clear)
            if obs < self.blocked_cost:
                all_blocked = False
            cost = self.w_obs * obs + self.w_dev * (abs(delta - center) / denom)
            if cost < best_cost:
                best_cost = cost
                best_delta = delta
                best_clear = clear

        return best_delta, front_clear, best_clear, all_blocked

    def _clearance(self, heading: float) -> float:
        """Folga à frente no corredor que a cadeira ocupa se seguir `heading`.

        Os pontos já estão no base_link; girar o corredor por `heading` é o
        mesmo que girar os pontos por -heading (a cadeira pivota em torno do
        eixo traseiro). Tudo que ficar atrás da frente do chassi é a própria
        estrutura e sai naturalmente -- sem precisar de min_obstacle_range.
        """
        if self._px is None or self._px.size == 0:
            return math.inf
        c, s = math.cos(heading), math.sin(heading)
        xr = self._px * c + self._py * s
        yr = -self._px * s + self._py * c
        floor = self.front_extent + self.corridor_margin
        hit = (np.abs(yr) <= self.half_w) & (xr > floor)
        if not hit.any():
            return math.inf
        return float(xr[hit].min() - self.front_extent)

    def _obstacle_term(self, clearance: float) -> float:
        if math.isinf(clearance) or clearance >= self.slow_d:
            return 0.0
        if clearance <= self.stop_d:
            return self.blocked_cost
        return (self.slow_d - clearance) / (self.slow_d - self.stop_d)

    def _speed_scale(self, clearance: float) -> float:
        if math.isinf(clearance) or clearance >= self.slow_d:
            return 1.0
        if clearance <= self.stop_d:
            return 0.0
        return _clamp(
            (clearance - self.stop_d) / (self.slow_d - self.stop_d),
            0.0, 1.0)

    def _publish(
        self,
        v: float,
        w: float,
        mode: str,
        front_clear: float = math.inf,
        best_clear: float = math.inf,
        best_delta: float = 0.0,
    ) -> None:
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub_cmd.publish(cmd)

        status = {
            "mode": mode,
            "v_user": round(self._v_user, 3),
            "w_user": round(self._w_user, 3),
            "v_out": round(v, 3),
            "w_out": round(w, 3),
            "front_clear_m": None if math.isinf(front_clear) else round(front_clear, 3),
            "best_clear_m": None if math.isinf(best_clear) else round(best_clear, 3),
            "best_delta_deg": round(math.degrees(best_delta), 1),
        }
        self.pub_status.publish(
            String(data=json.dumps(status, separators=(",", ":"))))

    def stop(self) -> None:
        self.pub_cmd.publish(Twist())


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = SharedControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
