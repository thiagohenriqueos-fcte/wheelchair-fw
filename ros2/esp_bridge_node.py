#!/usr/bin/env python3
"""
esp_bridge_node.py — Ponte ROS 2 <-> ESP32-S3 (firmware v0.5+ da cadeira).

O que faz:
  - Abre a porta serial (115200) e lê pacotes JSON do firmware
    (joy / ack / err / status / heartbeat) numa thread separada.
  - Publica telemetria útil em tópicos ROS 2.
  - Assina /cmd_vel (geometry_msgs/Twist) e, num timer, converte
    velocidade -> duty por roda (open-loop, igual ao que o v0.9 faria
    no firmware) e envia comandos `pwm_test` por roda.
  - Alimenta o watchdog de 500 ms do firmware enviando a uma taxa fixa.
  - Failsafe próprio na Pi: se /cmd_vel parar de chegar (timeout) ou
    o comando for ~zero, envia `stop`.

ATENÇÃO DE SEGURANÇA:
  - Teste SEMPRE com as rodas suspensas (igual ao TEST_PLAN_V0_6).
  - max_duty começa em 0.30 — o mesmo gate de segurança do GUI.
    Só aumente com a cadeira suspensa e motivo claro.
  - A conversão v/w -> duty é OPEN-LOOP e NÃO calibrada. Os ganhos
    são aproximados; calibre com a cadeira no lugar antes de confiar.

Dependências: rclpy, pyserial (python3 -m pip install pyserial)
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Int32

import serial  # pyserial


class EspBridge(Node):
    def __init__(self):
        super().__init__("esp_bridge")

        # ---- Parâmetros (ros2 run ... --ros-args -p port:=/dev/ttyACM0) ----
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("cmd_rate_hz", 20.0)      # taxa de envio p/ watchdog
        self.declare_parameter("cmd_timeout_s", 0.3)     # sem /cmd_vel -> stop
        self.declare_parameter("max_duty", 0.30)         # gate de segurança (== GUI)
        # Ganhos open-loop: duty = gain_lin*v +/- gain_ang*w  (depois clamp)
        self.declare_parameter("gain_lin", 1.0)          # por unidade de linear.x
        self.declare_parameter("gain_ang", 0.5)          # por unidade de angular.z
        # Decodificação do joystick (telemetria x/y) -> intenção em Twist.
        # Convenção: y>0 = frente; x>0 = direita. ROS REP-103: +angular.z = esquerda,
        # então angular.z = -x. Ajuste os sinais conforme o seu joystick.
        self.declare_parameter("joy_v_scale", 1.0)       #  y -> linear.x
        self.declare_parameter("joy_w_scale", 1.0)       # -x -> angular.z

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.cmd_rate = float(self.get_parameter("cmd_rate_hz").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)
        self.max_duty = float(self.get_parameter("max_duty").value)
        self.gain_lin = float(self.get_parameter("gain_lin").value)
        self.gain_ang = float(self.get_parameter("gain_ang").value)
        self.joy_v_scale = float(self.get_parameter("joy_v_scale").value)
        self.joy_w_scale = float(self.get_parameter("joy_w_scale").value)

        # ---- Estado de comando ----
        self._last_v = 0.0
        self._last_w = 0.0
        self._last_cmd_time = 0.0
        self._seq = 0
        self._lock = threading.Lock()

        # ---- Publishers de telemetria ----
        self.pub_raw = self.create_publisher(String, "wheelchair/telemetry_json", 10)
        self.pub_motor_active = self.create_publisher(Bool, "wheelchair/motor_active", 10)
        self.pub_enc_left = self.create_publisher(Int32, "wheelchair/enc_left_count", 10)
        self.pub_enc_right = self.create_publisher(Int32, "wheelchair/enc_right_count", 10)
        # Intenção do usuário decodificada do joystick -> consumida pelo nó de assistência
        self.pub_joy_cmd = self.create_publisher(Twist, "joystick_cmd_vel", 10)

        # ---- Subscriber de comando ----
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)

        # ---- Serial ----
        self.get_logger().info(f"Abrindo {self.port} @ {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        # Dá tempo do ESP resetar caso a abertura da porta cause reset por DTR
        time.sleep(0.3)
        self.ser.reset_input_buffer()

        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        # Timer que envia comandos na taxa fixa (mantém watchdog vivo)
        self.create_timer(1.0 / self.cmd_rate, self._send_loop)

        self.get_logger().info(
            f"esp_bridge pronto. max_duty={self.max_duty:.2f}, "
            f"cmd_rate={self.cmd_rate:.0f} Hz, timeout={self.cmd_timeout:.2f} s"
        )

    # ------------------------------------------------------------------ #
    #  /cmd_vel -> guarda último comando
    # ------------------------------------------------------------------ #
    def _on_cmd_vel(self, msg: Twist):
        with self._lock:
            self._last_v = msg.linear.x
            self._last_w = msg.angular.z
            self._last_cmd_time = time.monotonic()

    # ------------------------------------------------------------------ #
    #  Conversão open-loop v/w -> duty por roda  (NÃO calibrada)
    # ------------------------------------------------------------------ #
    def _twist_to_duty(self, v, w):
        left = self.gain_lin * v - self.gain_ang * w
        right = self.gain_lin * v + self.gain_ang * w
        # Clamp simétrico ao gate de segurança
        left = max(-self.max_duty, min(self.max_duty, left))
        right = max(-self.max_duty, min(self.max_duty, right))
        return left, right

    # ------------------------------------------------------------------ #
    #  Timer de envio: alimenta watchdog, aplica failsafe
    # ------------------------------------------------------------------ #
    def _send_loop(self):
        with self._lock:
            v, w, t = self._last_v, self._last_w, self._last_cmd_time

        stale = (time.monotonic() - t) > self.cmd_timeout
        if stale or (abs(v) < 1e-3 and abs(w) < 1e-3):
            self._send_stop()
            return

        left, right = self._twist_to_duty(v, w)
        self._send_pwm(left, right)

    def _send_pwm(self, left, right):
        self._seq += 1
        pkt = {"type": "pwm_test", "seq": self._seq,
               "left": round(left, 3), "right": round(right, 3)}
        self._write(pkt)

    def _send_stop(self):
        self._seq += 1
        self._write({"type": "stop", "seq": self._seq})

    def _write(self, pkt: dict):
        line = json.dumps(pkt) + "\n"
        try:
            self.ser.write(line.encode("ascii"))
        except serial.SerialException as e:
            self.get_logger().error(f"Falha na escrita serial: {e}")

    # ------------------------------------------------------------------ #
    #  Thread de leitura: parseia JSON do firmware e publica
    # ------------------------------------------------------------------ #
    def _read_loop(self):
        buf = b""
        while self._running and rclpy.ok():
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as e:
                self.get_logger().error(f"Falha na leitura serial: {e}")
                time.sleep(0.5)
                continue
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                self._handle_line(line)

    def _handle_line(self, raw: bytes):
        try:
            pkt = json.loads(raw.decode("ascii", errors="replace"))
        except (ValueError, UnicodeDecodeError):
            # Linhas de log do ESP-IDF / boot não são JSON — ignore.
            return
        if not isinstance(pkt, dict):
            return

        # Publica o JSON cru para quem quiser inspecionar
        self.pub_raw.publish(String(data=json.dumps(pkt)))

        ptype = pkt.get("type")
        if ptype in ("joy", "joystick"):
            # Decodifica a INTENÇÃO do usuário (joystick) e publica como Twist.
            # É isso que o nó de controle compartilhado usa como comando primário.
            if "x" in pkt and "y" in pkt:
                intent = Twist()
                intent.linear.x = self.joy_v_scale * float(pkt["y"])    #  frente/trás
                intent.angular.z = -self.joy_w_scale * float(pkt["x"])  #  +z = esquerda
                self.pub_joy_cmd.publish(intent)
            if "motor_test_active" in pkt:
                self.pub_motor_active.publish(Bool(data=bool(pkt["motor_test_active"])))
            if "enc_left_count" in pkt:
                self.pub_enc_left.publish(Int32(data=int(pkt["enc_left_count"])))
            if "enc_right_count" in pkt:
                self.pub_enc_right.publish(Int32(data=int(pkt["enc_right_count"])))
        elif ptype == "err":
            self.get_logger().warn(f"ESP err: {pkt.get('code')}")

    # ------------------------------------------------------------------ #
    def destroy_node(self):
        # Failsafe na saída: manda stop antes de fechar.
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


def main(args=None):
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
