#!/usr/bin/env python3
"""Captura varreduras REAIS do /scan para servir de estimulo ao testbench.

Grava:
  data/scan_NNN.csv   uma varredura: indice, range_m  (inf -> -1)
  data/scan_meta.json angle_min, angle_increment, range_min, range_max, n
"""
import json, math, sys, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

OUT = "/home/wheelchair/wheelchair-fw/fpga/data"
N = int(sys.argv[1]) if len(sys.argv) > 1 else 3

class C(Node):
    def __init__(self):
        super().__init__("cap"); self.k = 0
        self.create_subscription(LaserScan, "/scan", self.cb, qos_profile_sensor_data)
    def cb(self, m):
        if self.k >= N: return
        if self.k == 0:
            json.dump({"angle_min": m.angle_min, "angle_increment": m.angle_increment,
                       "range_min": m.range_min, "range_max": m.range_max,
                       "n": len(m.ranges)}, open(f"{OUT}/scan_meta.json", "w"), indent=2)
        with open(f"{OUT}/scan_{self.k:03d}.csv", "w") as f:
            f.write("i,range_m\n")
            for i, r in enumerate(m.ranges):
                ok = math.isfinite(r) and m.range_min <= r <= m.range_max
                f.write(f"{i},{r if ok else -1.0:.5f}\n")
        print(f"scan_{self.k:03d}.csv  ({len(m.ranges)} feixes)")
        self.k += 1

rclpy.init(); n = C()
while rclpy.ok() and n.k < N: rclpy.spin_once(n, timeout_sec=1.0)
rclpy.shutdown()
