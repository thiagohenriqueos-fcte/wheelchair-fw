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


def _angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class SharedControl(Node):
    def __init__(self) -> None:
        super().__init__("shared_control")

        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("slow_distance", 1.10)
        self.declare_parameter("cone_half_deg", 15.0)
        self.declare_parameter("max_deviation_deg", 45.0)
        # Frente real do chassi vs 0 do sensor (o LIDAR pode estar montado girado)
        self.declare_parameter("front_offset_deg", 0.0)
        # Ignora retornos mais perto que isto (estrutura da própria cadeira)
        self.declare_parameter("min_obstacle_range", 0.12)
        self.declare_parameter("num_candidates", 19)
        self.declare_parameter("w_obstacle", 1.0)
        self.declare_parameter("w_deviation", 0.35)
        self.declare_parameter("blocked_cost", 10.0)
        self.declare_parameter("assist_gain", 0.8)
        self.declare_parameter("allow_reverse", True)
        self.declare_parameter("reverse_speed_cap", 0.5)
        self.declare_parameter("scan_timeout_s", 0.40)
        self.declare_parameter("intent_timeout_s", 0.40)
        self.declare_parameter("control_rate_hz", 20.0)

        gp = self.get_parameter
        self.stop_d = float(gp("stop_distance").value)
        self.slow_d = float(gp("slow_distance").value)
        self.cone_half = math.radians(float(gp("cone_half_deg").value))
        self.dev_max = math.radians(float(gp("max_deviation_deg").value))
        self.front_offset = math.radians(float(gp("front_offset_deg").value))
        self.min_range = float(gp("min_obstacle_range").value)
        self.n_cand = max(3, int(gp("num_candidates").value))
        self.w_obs = float(gp("w_obstacle").value)
        self.w_dev = float(gp("w_deviation").value)
        self.blocked_cost = float(gp("blocked_cost").value)
        self.assist_gain = float(gp("assist_gain").value)
        self.allow_reverse = _as_bool(gp("allow_reverse").value)
        self.rev_cap = float(gp("reverse_speed_cap").value)
        self.scan_timeout = float(gp("scan_timeout_s").value)
        self.intent_timeout = float(gp("intent_timeout_s").value)
        rate = float(gp("control_rate_hz").value)

        self._scan: Optional[LaserScan] = None
        self._scan_time = 0.0
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
            f"cone=±{math.degrees(self.cone_half):.0f}°, "
            f"desvio_max=±{math.degrees(self.dev_max):.0f}°, "
            f"front_offset={math.degrees(self.front_offset):.0f}°, "
            f"min_range={self.min_range:.2f} m")

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            v = p.value
            if p.name == "front_offset_deg":
                self.front_offset = math.radians(float(v))
            elif p.name == "min_obstacle_range":
                self.min_range = float(v)
            elif p.name == "stop_distance":
                self.stop_d = float(v)
            elif p.name == "slow_distance":
                self.slow_d = float(v)
            elif p.name == "cone_half_deg":
                self.cone_half = math.radians(float(v))
            elif p.name == "max_deviation_deg":
                self.dev_max = math.radians(float(v))
            elif p.name == "assist_gain":
                self.assist_gain = float(v)
        return SetParametersResult(successful=True)

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg
        self._scan_time = self._now()

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

        if self._scan is None or (now - self._scan_time) > self.scan_timeout:
            v = min(0.0, self._v_user) if self.allow_reverse else 0.0
            self._publish(v, self._w_user, "sem_scan")
            self.get_logger().warn(
                "sem /scan recente; bloqueando avanco",
                throttle_duration_sec=2.0)
            return

        v_user = self._v_user
        w_user = self._w_user

        if v_user < 0.0:
            v = v_user * (_clamp(self.rev_cap, 0.0, 1.0)
                          if self.allow_reverse else 0.0)
            self._publish(v, w_user, "re")
            return

        if v_user <= 1e-3:
            self._publish(0.0, w_user, "giro")
            return

        best_delta, front_clear, best_clear, all_blocked = self._best_direction(
            self._scan, w_user)

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
        applied_delta = self.assist_gain * best_delta

        # Frente totalmente livre: zero intervention.
        if front_clear >= self.slow_d or math.isinf(front_clear):
            best_delta = 0.0
            applied_delta = 0.0
            speed_scale = 1.0
            mode = "livre"
        else:
            mode = "desvia" if abs(applied_delta) > math.radians(1.0) else "freia"

        out_v = v_user * speed_scale
        out_w = w_user + applied_delta
        self._publish(out_v, out_w, mode, front_clear, best_clear, best_delta)

    def _best_direction(
        self, scan: LaserScan, w_user: float
    ) -> tuple[float, float, float, bool]:
        center = _clamp(w_user, -self.dev_max, self.dev_max)
        denom = max(self.dev_max, 1e-6)
        if self.n_cand == 1:
            candidates = [0.0]
        else:
            step = (2.0 * self.dev_max) / (self.n_cand - 1)
            candidates = [-self.dev_max + i * step for i in range(self.n_cand)]

        front_clear = self._clearance(scan, 0.0)
        best_delta = 0.0
        best_clear = 0.0
        best_cost = math.inf
        all_blocked = True

        for delta in candidates:
            clear = self._clearance(scan, delta)
            obs = self._obstacle_term(clear)
            if obs < self.blocked_cost:
                all_blocked = False
            cost = self.w_obs * obs + self.w_dev * (abs(delta - center) / denom)
            if cost < best_cost:
                best_cost = cost
                best_delta = delta
                best_clear = clear

        return best_delta, front_clear, best_clear, all_blocked

    def _clearance(self, scan: LaserScan, heading: float) -> float:
        # heading é relativo à frente do chassi; soma o offset de montagem para
        # comparar com o ângulo do feixe (0 do sensor).
        aim = heading + self.front_offset
        floor = max(scan.range_min, self.min_range)
        best = math.inf
        angle = scan.angle_min
        for distance in scan.ranges:
            if abs(_angle_diff(angle, aim)) <= self.cone_half:
                if (math.isfinite(distance)
                        and floor <= distance <= scan.range_max):
                    best = min(best, float(distance))
            angle += scan.angle_increment
        return best

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
