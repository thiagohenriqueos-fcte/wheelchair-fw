#!/usr/bin/env python3
"""
shared_control_node.py — Controle COMPARTILHADO (semiassistido).

Filosofia:
  - O joystick (intenção da pessoa) é o comando PRIMÁRIO.
  - O LIDAR só MODULA esse comando. O nó nunca cria movimento que a
    pessoa não pediu: ele apenas reduz velocidade, corrige a direção
    dentro de uma janela limitada, ou para.
  - Intervenção mínima: frente livre => o joystick passa intacto.

Entradas:
  /joystick_cmd_vel (geometry_msgs/Twist)  -> intenção do usuário (do esp_bridge)
  /scan             (sensor_msgs/LaserScan) -> do sllidar_ros2

Saída:
  /cmd_vel (geometry_msgs/Twist) -> consumido pelo esp_bridge

DECISÃO POR MENOR CUSTO
-----------------------
A cada ciclo, quando o usuário pede ir para frente (v_user > 0), avaliamos
um leque de direções candidatas (offsets de heading) em torno da direção
que o usuário está pedindo. Para cada candidata δ:

    custo(δ) = w_obs * termo_obstaculo(folga(δ))  +  w_dev * |δ| / Δmax

  - folga(δ)   : menor distância medida num cone em torno de δ
  - termo_obstaculo: 0 se folga >= slow_d ; cresce até 1 perto de stop_d ;
                     "bloqueado" (>=1, alto) se folga <= stop_d
  - termo de desvio: penaliza afastar-se da direção pedida (suavidade)

Escolhemos δ* = argmin custo(δ). Então:

  - Todas as candidatas bloqueadas (folga <= stop_d)  -> PARA a frente
    (v=0), mas deixa a rotação do usuário passar para ele escapar.   ["para"]
  - Caso contrário                                    -> segue, com:
      * v reduzido proporcional à folga à frente (freio suave)
      * pequena correção de curva rumo a δ*                          ["desvia"]
  - Frente totalmente livre (folga(0) >= slow_d)      -> passa intacto.

SEGURANÇA / LIMITES:
  - Um RPLIDAR vê só ~360° no plano dele. Se montado para frente, NÃO vê
    atrás: por isso ré (v_user < 0) passa direto (com cap opcional) e isso
    é uma limitação consciente — documente e, se precisar, adicione sensor
    traseiro.
  - Sem /scan recente => PARA a frente (failsafe de sensor).
  - Sempre teste com as rodas suspensas primeiro.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SharedControl(Node):
    def __init__(self):
        super().__init__("shared_control")

        # ---- Distâncias (m) ----
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("slow_distance", 1.10)
        # ---- Geometria da busca ----
        self.declare_parameter("cone_half_deg", 15.0)    # meia-largura do cone p/ folga
        self.declare_parameter("max_deviation_deg", 45.0)  # Δmax: até onde pode desviar
        self.declare_parameter("num_candidates", 19)      # nº de direções avaliadas
        # ---- Pesos do custo ----
        self.declare_parameter("w_obstacle", 1.0)
        self.declare_parameter("w_deviation", 0.35)
        self.declare_parameter("blocked_cost", 10.0)      # custo de candidata bloqueada
        # ---- Ação ----
        self.declare_parameter("assist_gain", 0.8)        # quanto da correção δ* aplicar
        self.declare_parameter("allow_reverse", True)     # ré passa direto (sem visão traseira)
        self.declare_parameter("reverse_speed_cap", 0.5)  # fração do v_user na ré
        self.declare_parameter("scan_timeout_s", 0.4)
        self.declare_parameter("control_rate_hz", 20.0)

        gp = self.get_parameter
        self.stop_d = float(gp("stop_distance").value)
        self.slow_d = float(gp("slow_distance").value)
        self.cone_half = math.radians(float(gp("cone_half_deg").value))
        self.dev_max = math.radians(float(gp("max_deviation_deg").value))
        self.n_cand = int(gp("num_candidates").value)
        self.w_obs = float(gp("w_obstacle").value)
        self.w_dev = float(gp("w_deviation").value)
        self.blocked_cost = float(gp("blocked_cost").value)
        self.assist_gain = float(gp("assist_gain").value)
        self.allow_reverse = bool(gp("allow_reverse").value)
        self.rev_cap = float(gp("reverse_speed_cap").value)
        self.scan_timeout = float(gp("scan_timeout_s").value)
        rate = float(gp("control_rate_hz").value)

        self._scan = None
        self._scan_time = 0.0
        self._v_user = 0.0
        self._w_user = 0.0
        self._intent_time = 0.0

        self.create_subscription(LaserScan, "scan", self._on_scan, 10)
        self.create_subscription(Twist, "joystick_cmd_vel", self._on_intent, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Loop de controle em taxa fixa (não dependemos do ritmo do scan)
        self.create_timer(1.0 / rate, self._control_step)

        self.get_logger().info(
            f"shared_control pronto. stop={self.stop_d} m, slow={self.slow_d} m, "
            f"Δmax={math.degrees(self.dev_max):.0f}°, candidatas={self.n_cand}"
        )

    # ------------------------------------------------------------------ #
    def _on_scan(self, msg: LaserScan):
        self._scan = msg
        self._scan_time = self._now()

    def _on_intent(self, msg: Twist):
        self._v_user = msg.linear.x
        self._w_user = msg.angular.z
        self._intent_time = self._now()

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ #
    #  Folga (clearance) num cone centrado em 'heading' (rad, 0 = frente)
    # ------------------------------------------------------------------ #
    def _clearance(self, scan: LaserScan, heading: float) -> float:
        lo = heading - self.cone_half
        hi = heading + self.cone_half
        best = math.inf
        angle = scan.angle_min
        for r in scan.ranges:
            if lo <= angle <= hi:
                if scan.range_min <= r <= scan.range_max and not math.isinf(r) and not math.isnan(r):
                    best = min(best, r)
            angle += scan.angle_increment
        return best  # inf se nada visível => livre

    def _obstacle_term(self, clearance: float) -> float:
        if math.isinf(clearance) or clearance >= self.slow_d:
            return 0.0
        if clearance <= self.stop_d:
            return self.blocked_cost
        # Interpola 0..1 entre slow_d e stop_d
        return (self.slow_d - clearance) / (self.slow_d - self.stop_d)

    # ------------------------------------------------------------------ #
    #  Núcleo: avalia candidatas e devolve (delta_otimo, folga_frente, bloqueado)
    # ------------------------------------------------------------------ #
    def _best_direction(self, scan: LaserScan):
        # Centro da busca segue a intenção de curva do usuário (limitada a Δmax),
        # para que "desviar" fique alinhado com o que a pessoa já quer.
        center = max(-self.dev_max, min(self.dev_max, self._w_user))
        deltas = [(-self.dev_max + i * (2 * self.dev_max) / (self.n_cand - 1))
                  for i in range(self.n_cand)]

        best_delta, best_cost = 0.0, math.inf
        clear_front = self._clearance(scan, 0.0)
        all_blocked = True

        for d in deltas:
            clr = self._clearance(scan, d)
            obs = self._obstacle_term(clr)
            if obs < self.blocked_cost:
                all_blocked = False
            cost = self.w_obs * obs + self.w_dev * (abs(d - center) / self.dev_max)
            if cost < best_cost:
                best_cost, best_delta = cost, d

        return best_delta, clear_front, all_blocked

    # ------------------------------------------------------------------ #
    def _control_step(self):
        out = Twist()

        # Failsafe de sensor
        if self._scan is None or (self._now() - self._scan_time) > self.scan_timeout:
            # Sem LIDAR confiável: bloqueia avanço, deixa girar/recuar manualmente.
            out.linear.x = min(0.0, self._v_user) if self.allow_reverse else 0.0
            out.angular.z = self._w_user
            self.pub.publish(out)
            self.get_logger().warn("Sem /scan — frente bloqueada.",
                                   throttle_duration_sec=2.0)
            return

        v_user, w_user = self._v_user, self._w_user

        # Ré: o LIDAR frontal não enxerga atrás. Passa direto (com cap).
        if v_user < 0.0:
            out.linear.x = v_user * (self.rev_cap if self.allow_reverse else 0.0)
            out.angular.z = w_user
            self.pub.publish(out)
            return

        # Parado / quase parado: só repassa rotação (girar no lugar é seguro).
        if v_user <= 1e-3:
            out.linear.x = 0.0
            out.angular.z = w_user
            self.pub.publish(out)
            return

        # --- Indo para frente: aplica o custo ---
        best_delta, clear_front, all_blocked = self._best_direction(self._scan)

        if all_blocked or clear_front <= self.stop_d:
            # PARA o avanço; mantém rotação do usuário para ele escapar.
            out.linear.x = 0.0
            out.angular.z = w_user
            self.pub.publish(out)
            self.get_logger().info("Obstáculo: parando avanço.",
                                   throttle_duration_sec=1.0)
            return

        # Freio suave: escala v conforme a folga à frente.
        if math.isinf(clear_front) or clear_front >= self.slow_d:
            speed_scale = 1.0  # frente livre -> passa intacto
        else:
            speed_scale = (clear_front - self.stop_d) / (self.slow_d - self.stop_d)
            speed_scale = max(0.0, min(1.0, speed_scale))

        out.linear.x = v_user * speed_scale
        # Correção de curva rumo à direção de menor custo (nudge, não comando).
        out.angular.z = w_user + self.assist_gain * best_delta
        self.pub.publish(out)

    # ------------------------------------------------------------------ #
    def stop(self):
        t = Twist()
        self.pub.publish(t)


def main(args=None):
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
