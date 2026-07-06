#!/usr/bin/env python3
"""Sonda de calibração do LIDAR para o supervisor semi-assistido.

Uso (com o /scan publicando e um objeto ~1 m EXATAMENTE à frente da cadeira,
o resto ~1,5 m livre):

    python3 ~/wheelchair-fw/scripts/lidar_calibration.py

Mostra a menor distância por setor de 15 graus e o ponto mais próximo. O ângulo
onde o objeto aparece é o front_offset_deg. Aplique ao vivo (sem reiniciar):

    ros2 param set /shared_control front_offset_deg <ANG>

Retornos muito perto (< ~0,4 m) em vários setores são a própria estrutura da
cadeira; use min_obstacle_range para ignorá-los.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Probe(Node):
    def __init__(self):
        super().__init__("lidar_calib_probe")
        self.msg = None
        self.create_subscription(
            LaserScan, "/scan", lambda m: setattr(self, "msg", m),
            qos_profile_sensor_data)


def main():
    rclpy.init()
    n = Probe()
    t0 = time.time()
    while n.msg is None and time.time() - t0 < 6.0:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = n.msg
    if m is None:
        print("sem /scan - o LIDAR esta publicando? (ros2 topic hz /scan)")
        rclpy.shutdown()
        return

    pts = []
    ang = m.angle_min
    for r in m.ranges:
        if math.isfinite(r) and m.range_min <= r <= m.range_max:
            pts.append((math.degrees(ang) % 360.0, float(r)))
        ang += m.angle_increment
    if not pts:
        print("scan sem retornos validos")
        rclpy.shutdown()
        return

    gmin = min(pts, key=lambda p: p[1])
    print("\nMAIS PROXIMO no total: %.2f m @ %.0f graus\n" % (gmin[1], gmin[0]))
    print("setor(g)   dist_min(m)  @ang(g)  obs")
    for b in range(0, 360, 15):
        seg = [p for p in pts if b <= p[0] < b + 15]
        if not seg:
            print("  %3d-%3d    (vazio)" % (b, b + 15))
            continue
        mn = min(seg, key=lambda p: p[1])
        tag = ""
        if mn[1] < 0.40:
            tag = "<-- MUITO perto (estrutura?)"
        elif mn[1] < 0.60:
            tag = "<-- perto"
        print("  %3d-%3d    %.2f      @%3.0f   %s" % (b, b + 15, mn[1], mn[0], tag))
    print("\nDica: o objeto a ~1 m e o retorno num setor que antes estava livre. "
          "O angulo dele e o front_offset_deg.\n")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
