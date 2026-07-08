#!/usr/bin/env python3
"""Serial bridge between ROS 2 and the ESP wheelchair firmware.

The ESP keeps the physical joystick and motor PWM loop.  This node reads the
ESP telemetry, publishes the joystick intent as Twist, receives the assisted
/cmd_vel, converts it to normalized wheel commands, and sends:

  - drive_cfg: safety gate, max duty and ramps
  - drive_cmd: assisted left/right wheel requests in [-1, 1]

The firmware still applies max_duty, ramping, and watchdogs.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Optional

from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

import serial


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def _as_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class EspBridge(Node):
    def __init__(self) -> None:
        super().__init__("esp_bridge")

        self.declare_parameter("port", "/dev/ttyUSB1")
        self.declare_parameter("baud", 460800)  # firmware >= 0.9.x usa 460800
        self.declare_parameter("cmd_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_s", 0.30)
        self.declare_parameter("armed", False)
        self.declare_parameter("max_duty", 0.30)
        self.declare_parameter("accel", 1.5)
        self.declare_parameter("decel", 3.0)
        self.declare_parameter("gain_lin", 1.0)
        self.declare_parameter("gain_ang", 0.5)
        self.declare_parameter("joy_v_scale", 1.0)
        self.declare_parameter("joy_w_scale", 1.0)
        # Mapeamento dos eixos do joystick do ESP -> Twist (REP-103). Mapeado em
        # bancada: avanço no eixo X (0, +), giro no eixo Y (1; direita = y neg).
        # Índice do eixo: 0 = x do ESP, 1 = y do ESP (evita "Norway problem").
        self.declare_parameter("joy_v_axis", 0)
        self.declare_parameter("joy_v_sign", 1.0)
        self.declare_parameter("joy_w_axis", 1)
        self.declare_parameter("joy_w_sign", 1.0)
        # Odometria por encoder (para EKF/futuro PID). O firmware emite as
        # contagens acumuladas "enc":[esq,dir] e "enc_cpr" (contagens/rev).
        self.declare_parameter("publish_wheel_odom", True)
        self.declare_parameter("wheel_radius", 0.165)   # m — MEDIR/CALIBRAR
        self.declare_parameter("wheel_base", 0.60)       # m — entre rodas motorizadas
        self.declare_parameter("enc_left_sign", 1)       # -1 se frente der contagem negativa
        self.declare_parameter("enc_right_sign", 1)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("enc_reset_jump", 100000)  # salto de contagem tratado como reset
        self.declare_parameter("reset_odom", False)       # set true -> zera a pose acumulada

        self.port = str(self.get_parameter("port").value)
        self.baud = int(self.get_parameter("baud").value)
        self.cmd_rate = float(self.get_parameter("cmd_rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self.armed = _as_bool(self.get_parameter("armed").value)
        self.max_duty = float(self.get_parameter("max_duty").value)
        self.accel = float(self.get_parameter("accel").value)
        self.decel = float(self.get_parameter("decel").value)
        self.gain_lin = float(self.get_parameter("gain_lin").value)
        self.gain_ang = float(self.get_parameter("gain_ang").value)
        self.joy_v_scale = float(self.get_parameter("joy_v_scale").value)
        self.joy_w_scale = float(self.get_parameter("joy_w_scale").value)
        self.joy_v_axis = int(self.get_parameter("joy_v_axis").value)
        self.joy_v_sign = float(self.get_parameter("joy_v_sign").value)
        self.joy_w_axis = int(self.get_parameter("joy_w_axis").value)
        self.joy_w_sign = float(self.get_parameter("joy_w_sign").value)
        self.wheel_odom = _as_bool(self.get_parameter("publish_wheel_odom").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_base = float(self.get_parameter("wheel_base").value)
        self.enc_lsign = int(self.get_parameter("enc_left_sign").value)
        self.enc_rsign = int(self.get_parameter("enc_right_sign").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.enc_reset_jump = int(self.get_parameter("enc_reset_jump").value)

        self._seq = 0
        self._last_v = 0.0
        self._last_w = 0.0
        self._last_cmd_time = 0.0
        self._lock = threading.Lock()
        self._running = True

        # Estado da integração de odometria por encoder
        self._enc_last: Optional[tuple[float, float]] = None
        self._enc_time = 0.0
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._dist_l = 0.0
        self._dist_r = 0.0

        self.pub_raw = self.create_publisher(
            String, "wheelchair/telemetry_json", 10)
        self.pub_joy_cmd = self.create_publisher(Twist, "joystick_cmd_vel", 10)
        self.pub_fw_armed = self.create_publisher(Bool, "wheelchair/armed", 10)
        self.pub_fw_driving = self.create_publisher(
            Bool, "wheelchair/driving", 10)
        self.pub_bridge_armed = self.create_publisher(
            Bool, "wheelchair/bridge_armed", 10)
        self.pub_wheel_odom = self.create_publisher(Odometry, "wheel/odom", 10)
        # [dist_l, dist_r, vel_l, vel_r] — para calibração e futuro PID.
        self.pub_wheel_tel = self.create_publisher(
            Float32MultiArray, "wheel/telemetry", 10)
        # Estado do PID de roda do firmware (fw >= 0.9), para tuning:
        # [sp_l, sp_r, vel_l, vel_r, err_l, err_r, out_l, out_r] em rad/s.
        self.pub_pid_state = self.create_publisher(
            Float32MultiArray, "wheel/pid_state", 10)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        # Ajuste ao vivo da calibração de odometria (ros2 param set), para
        # calibrar wheel_radius/wheel_base e corrigir sinais sem reiniciar.
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(f"abrindo ESP em {self.port} @ {self.baud}")
        try:
            self.ser = serial.Serial(
                self.port, self.baud, timeout=0.1, exclusive=True)
        except TypeError:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(0.3)
        self.ser.reset_input_buffer()

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.create_timer(1.0 / self.cmd_rate, self._send_loop)

        state = "ARMADO" if self.armed else "desarmado"
        self.get_logger().info(
            f"esp_bridge pronto ({state}, max_duty={self.max_duty:.2f})")

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._last_v = float(msg.linear.x)
            self._last_w = float(msg.angular.z)
            self._last_cmd_time = time.monotonic()

    def _send_loop(self) -> None:
        self.pub_bridge_armed.publish(Bool(data=self.armed))
        if not self.armed:
            self._send_stop()
            return

        with self._lock:
            v = self._last_v
            w = self._last_w
            last_t = self._last_cmd_time

        if (time.monotonic() - last_t) > self.cmd_timeout:
            self._send_stop()
            self.get_logger().warn(
                "sem /cmd_vel recente; enviando stop",
                throttle_duration_sec=2.0)
            return

        left, right = self._twist_to_wheels(v, w)
        self._send_drive_cfg(True)
        self._send_drive_cmd(left, right)

    def _twist_to_wheels(self, v: float, w: float) -> tuple[float, float]:
        left = self.gain_lin * v - self.gain_ang * w
        right = self.gain_lin * v + self.gain_ang * w
        mag = max(abs(left), abs(right), 1.0)
        return (_clamp(left / mag, -1.0, 1.0),
                _clamp(right / mag, -1.0, 1.0))

    def _send_drive_cfg(self, armed: bool) -> None:
        self._write({
            "type": "drive_cfg",
            "accel": round(self.accel, 2),
            "decel": round(self.decel, 2),
            "max_duty": round(_clamp(self.max_duty, 0.0, 1.0), 3),
            "armed": bool(armed),
        })

    def _send_drive_cmd(self, left: float, right: float) -> None:
        self._write({
            "type": "drive_cmd",
            "left": round(left, 3),
            "right": round(right, 3),
        })

    def _send_stop(self) -> None:
        self._write({"type": "stop"})

    def _write(self, packet: dict[str, Any]) -> None:
        self._seq += 1
        packet["seq"] = self._seq
        line = json.dumps(packet, separators=(",", ":")) + "\n"
        try:
            self.ser.write(line.encode("ascii"))
        except serial.SerialException as exc:
            self.get_logger().error(f"falha na escrita serial: {exc}")

    def _read_loop(self) -> None:
        buf = b""
        while self._running and rclpy.ok():
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as exc:
                self.get_logger().error(f"falha na leitura serial: {exc}")
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if line:
                    self._handle_line(line)

    def _handle_line(self, raw: bytes) -> None:
        try:
            pkt = json.loads(raw.decode("utf-8", errors="replace"))
        except ValueError:
            return
        if not isinstance(pkt, dict):
            return

        self.pub_raw.publish(String(data=json.dumps(pkt, separators=(",", ":"))))
        if pkt.get("type") in ("drive", "joy", "joystick"):
            self._publish_joystick_intent(pkt)
            self.pub_fw_armed.publish(Bool(data=bool(pkt.get("armed", False))))
            self.pub_fw_driving.publish(
                Bool(data=bool(pkt.get("driving", False))))
            if self.wheel_odom:
                self._update_wheel_odom(pkt)
            self._publish_pid_state(pkt)
        elif pkt.get("type") == "err":
            self.get_logger().warn(f"ESP err: {pkt.get('code')}")

    def _publish_pid_state(self, pkt: dict[str, Any]) -> None:
        """Republica o estado do PID de roda (fw >= 0.9) para tuning."""
        sp = pkt.get("wheel_sp_rad_s")
        vel = pkt.get("wheel_vel_rad_s")
        if not (isinstance(sp, list) and isinstance(vel, list)):
            return  # firmware antigo (sem PID) — nada a publicar
        err = pkt.get("wheel_err_rad_s") or [0.0, 0.0]
        out = pkt.get("wheel_pid_out") or [0.0, 0.0]

        def pair(a):
            try:
                return [float(a[0]), float(a[1])]
            except (TypeError, ValueError, IndexError):
                return [0.0, 0.0]

        data = pair(sp) + pair(vel) + pair(err) + pair(out)
        self.pub_pid_state.publish(Float32MultiArray(data=data))

    def _update_wheel_odom(self, pkt: dict[str, Any]) -> None:
        """Integra a odometria diferencial a partir das contagens de encoder.

        O firmware emite "enc":[esq,dir] (contagens acumuladas, assinadas) e
        "enc_cpr". Convertemos contagens -> distância por roda, integramos a
        pose 2D (arco médio) e publicamos /wheel/odom (o EKF usa vx e vyaw)."""
        enc = pkt.get("enc")
        cpr = pkt.get("enc_cpr")
        if not (isinstance(enc, list) and len(enc) >= 2 and cpr):
            return
        left = self.enc_lsign * self._as_float(enc[0])
        right = self.enc_rsign * self._as_float(enc[1])
        if left is None or right is None:
            return

        now = time.monotonic()
        if self._enc_last is None:
            self._enc_last = (left, right)
            self._enc_time = now
            return

        d_left = left - self._enc_last[0]
        d_right = right - self._enc_last[1]
        dt = now - self._enc_time
        self._enc_last = (left, right)
        self._enc_time = now

        # Reset do microcontrolador: as contagens voltam a ~0, gerando um salto
        # enorme. Descarta esse delta em vez de contaminar a pose.
        if (abs(d_left) > self.enc_reset_jump
                or abs(d_right) > self.enc_reset_jump):
            self.get_logger().warn("encoder: salto tratado como reset")
            return

        circ = 2.0 * math.pi * self.wheel_radius
        ds_l = (d_left / float(cpr)) * circ
        ds_r = (d_right / float(cpr)) * circ
        ds = 0.5 * (ds_l + ds_r)
        dyaw = (ds_r - ds_l) / self.wheel_base if self.wheel_base > 1e-6 else 0.0

        mid = self._odom_yaw + 0.5 * dyaw
        self._odom_x += ds * math.cos(mid)
        self._odom_y += ds * math.sin(mid)
        self._odom_yaw += dyaw

        self._dist_l += ds_l
        self._dist_r += ds_r
        vel_l = ds_l / dt if dt > 1e-6 else 0.0
        vel_r = ds_r / dt if dt > 1e-6 else 0.0
        vx = ds / dt if dt > 1e-6 else 0.0
        vyaw = dyaw / dt if dt > 1e-6 else 0.0
        self._publish_wheel_odom(vx, vyaw)
        self.pub_wheel_tel.publish(Float32MultiArray(
            data=[self._dist_l, self._dist_r, vel_l, vel_r]))

    def _publish_wheel_odom(self, vx: float, vyaw: float) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        half = 0.5 * self._odom_yaw
        odom.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vyaw
        self.pub_wheel_odom.publish(odom)

    def _reset_odom(self) -> None:
        self._enc_last = None
        self._odom_x = self._odom_y = self._odom_yaw = 0.0
        self._dist_l = self._dist_r = 0.0

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            v = p.value
            if p.name == "wheel_radius":
                self.wheel_radius = float(v)
            elif p.name == "wheel_base":
                self.wheel_base = float(v)
            elif p.name == "enc_left_sign":
                self.enc_lsign = int(v)
                self._enc_last = None   # re-baseline: o sinal mudou
            elif p.name == "enc_right_sign":
                self.enc_rsign = int(v)
                self._enc_last = None
            elif p.name == "max_duty":
                self.max_duty = float(v)
            elif p.name == "reset_odom" and _as_bool(v):
                self._reset_odom()
                self.get_logger().info("odometria zerada")
        return SetParametersResult(successful=True)

    def _publish_joystick_intent(self, pkt: dict[str, Any]) -> None:
        x = self._as_float(pkt.get("x"))
        y = self._as_float(pkt.get("y"))
        if x is None or y is None:
            return
        axes = (x, y)
        v_raw = axes[self.joy_v_axis] if self.joy_v_axis in (0, 1) else y
        w_raw = axes[self.joy_w_axis] if self.joy_w_axis in (0, 1) else x
        msg = Twist()
        msg.linear.x = self.joy_v_scale * self.joy_v_sign * v_raw
        msg.angular.z = self.joy_w_scale * self.joy_w_sign * w_raw
        self.pub_joy_cmd.publish(msg)

    @staticmethod
    def _as_float(value: Any) -> Optional[float]:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def destroy_node(self) -> None:
        try:
            self._send_stop()
            time.sleep(0.05)
        except Exception:
            pass
        self._running = False
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = EspBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
