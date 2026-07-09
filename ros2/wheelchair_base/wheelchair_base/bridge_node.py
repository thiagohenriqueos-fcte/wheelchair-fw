#!/usr/bin/env python3
"""
No ponte serial entre o ESP32 da cadeira de rodas e o ROS2.

Entrada (linha de texto a 20 Hz produzida pelo firmware atual):
    D:<m>,V:<m/s>,EL:<ticks>,ER:<ticks>,DL:<m>,DR:<m>,
    VL:<m/s>,VR:<m/s>,ML:<-1..1>,MR:<-1..1>[,JX:<float>,JY:<float>]

Saidas ROS2:
    /wheel/odom        nav_msgs/Odometry
        pose integrada de DL/DR + twist (vx, vyaw) derivado de VL/VR.
        IMPORTANTE: nao publica TF aqui; o EKF do robot_localization
        publica odom->base_link a partir desta odometria + IMU.

    /joy               sensor_msgs/Joy
        publicado apenas quando JX/JY estao presentes na linha.
        Hoje o firmware ainda nao emite esses campos; assim que voce
        adicionar JX:%.2f,JY:%.2f ao printf, este topico passa a
        publicar automaticamente.

    /wheel/telemetry   std_msgs/Float32MultiArray
        layout (8 floats, todos crus do ESP32 — uteis para debug):
            [ticks_l, ticks_r, dist_l, dist_r, vel_l, vel_r, pwm_l, pwm_r]

Robustez:
    - Detecta reset do ESP32 (DL/DR voltam a 0): re-baseliza sem aplicar
      o salto na pose. Sem isso, um reboot do micro chuta a cadeira
      varios metros no RViz.
    - O thread de leitura sobrevive a linhas corrompidas e a remocoes
      temporarias do dispositivo (loga e continua).
"""

import math
import threading

import rclpy
import serial
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Quaternion 2D (apenas componente Z): rotacao no plano."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def parse_line(raw: str):
    """Extrai 'K:valor' separados por virgula. Retorna None se invalido."""
    if 'D:' not in raw:
        return None
    fields = {}
    for token in raw.split(','):
        if ':' not in token:
            continue
        key, value = token.split(':', 1)
        try:
            fields[key.strip()] = float(value)
        except ValueError:
            return None
    return fields if 'DL' in fields and 'DR' in fields else None


class BridgeNode(Node):
    def __init__(self):
        super().__init__('wheelchair_bridge')

        # Parametros (override pelo YAML/launch)
        self.declare_parameter('port', '/dev/wheelchair/esp32')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_base', 0.60)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('joy_max', 1.0)
        # Salto absoluto em DL/DR (em metros) acima do qual consideramos
        # um reset do ESP32 e re-baselizamos a integracao sem mover a pose.
        self.declare_parameter('reset_jump_m', 0.30)

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.joy_max = float(self.get_parameter('joy_max').value)
        self.reset_jump_m = float(self.get_parameter('reset_jump_m').value)

        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self.tel_pub = self.create_publisher(
            Float32MultiArray, '/wheel/telemetry', 10)

        self.serial = serial.Serial(port, baud, timeout=0.2)

        # Estado da odometria diferencial (portado do monitor matplotlib)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_dl = None
        self.last_dr = None

        self.stop_event = threading.Event()
        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

        self.get_logger().info(
            f'Ponte ativa em {port} @ {baud} '
            f'(wheel_base={self.wheel_base:.3f} m)')

    # ---------------------------------------------------- thread de leitura

    def read_loop(self):
        while not self.stop_event.is_set():
            try:
                raw = self.serial.readline().decode(
                    'utf-8', errors='ignore').strip()
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'Serial caiu: {exc}')
                break
            if not raw:
                continue
            fields = parse_line(raw)
            if fields is None:
                continue
            self.handle_telemetry(fields)

    # ---------------------------------------------------- publicacao ROS

    def handle_telemetry(self, f: dict):
        now = self.get_clock().now().to_msg()
        dl, dr = f['DL'], f['DR']

        # Integracao incremental — so depois da primeira amostra de
        # referencia, e ignora saltos compativeis com reset do ESP32.
        if self.last_dl is not None:
            delta_l = dl - self.last_dl
            delta_r = dr - self.last_dr
            if (abs(delta_l) > self.reset_jump_m
                    or abs(delta_r) > self.reset_jump_m):
                self.get_logger().warning(
                    'Salto na odometria — possivel reset do ESP32. '
                    'Re-baselizando sem mover a pose.')
            else:
                delta_s = (delta_l + delta_r) / 2.0
                delta_th = (delta_r - delta_l) / self.wheel_base
                mid = self.theta + delta_th / 2.0
                self.x += delta_s * math.cos(mid)
                self.y += delta_s * math.sin(mid)
                # Normaliza theta em (-pi, pi]
                self.theta = math.atan2(
                    math.sin(self.theta + delta_th),
                    math.cos(self.theta + delta_th))
        self.last_dl, self.last_dr = dl, dr

        vl = f.get('VL', 0.0)
        vr = f.get('VR', 0.0)
        vx = (vl + vr) / 2.0
        vyaw = (vr - vl) / self.wheel_base

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.theta)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vyaw
        # Covariancias modestas — o EKF refina com o IMU. Indices:
        # 0=x, 7=y, 35=yaw (pose); 0=vx, 35=vyaw (twist).
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if 'JX' in f and 'JY' in f:
            joy = Joy()
            joy.header.stamp = now
            joy.header.frame_id = self.base_frame
            m = self.joy_max if self.joy_max else 1.0
            joy.axes = [
                max(-1.0, min(1.0, f['JX'] / m)),
                max(-1.0, min(1.0, f['JY'] / m)),
            ]
            self.joy_pub.publish(joy)

        tel = Float32MultiArray()
        tel.data = [
            f.get('EL', 0.0), f.get('ER', 0.0),
            dl, dr, vl, vr,
            f.get('ML', 0.0), f.get('MR', 0.0),
        ]
        self.tel_pub.publish(tel)


def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_event.set()
        try:
            node.serial.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
