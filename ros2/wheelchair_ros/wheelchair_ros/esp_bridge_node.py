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
import threading
import time
from typing import Any, Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

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
        self.declare_parameter("baud", 115200)
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

        self._seq = 0
        self._last_v = 0.0
        self._last_w = 0.0
        self._last_cmd_time = 0.0
        self._lock = threading.Lock()
        self._running = True

        self.pub_raw = self.create_publisher(
            String, "wheelchair/telemetry_json", 10)
        self.pub_joy_cmd = self.create_publisher(Twist, "joystick_cmd_vel", 10)
        self.pub_fw_armed = self.create_publisher(Bool, "wheelchair/armed", 10)
        self.pub_fw_driving = self.create_publisher(
            Bool, "wheelchair/driving", 10)
        self.pub_bridge_armed = self.create_publisher(
            Bool, "wheelchair/bridge_armed", 10)

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

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
        elif pkt.get("type") == "err":
            self.get_logger().warn(f"ESP err: {pkt.get('code')}")

    def _publish_joystick_intent(self, pkt: dict[str, Any]) -> None:
        x = self._as_float(pkt.get("x"))
        y = self._as_float(pkt.get("y"))
        if x is None or y is None:
            return
        msg = Twist()
        msg.linear.x = self.joy_v_scale * y
        msg.angular.z = -self.joy_w_scale * x
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
