#!/usr/bin/env python3
"""Janela ao vivo para calibrar a odometria por encoder.

Mostra distância e velocidade por roda (de /wheel/telemetry) e a pose integrada
(de /wheel/odom). O botão "Zerar" fixa o ponto atual como origem, para medir um
trecho limpo (ex.: empurrar 1,00 m reto, ou girar 360 graus no lugar).

Uso (com a esp_bridge publicando):

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 ~/wheelchair-fw/scripts/wheel_calib_monitor.py

Calibração:
  * wheel_radius: zere, empurre 1,00 m reto; se dist_esq/dir != 1,00, multiplique
    wheel_radius por (1.00 / dist_medida).
  * wheel_base: zere, gire 360 graus no lugar; se o yaw != 360, ajuste wheel_base
    (fechou menos que 360 -> diminua; mais que 360 -> aumente).
"""
import math
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


class OdomListener(Node):
    def __init__(self):
        super().__init__("wheel_calib_monitor")
        self.dist_l = self.dist_r = 0.0
        self.vel_l = self.vel_r = 0.0
        self.x = self.y = self.yaw = 0.0
        self.create_subscription(
            Float32MultiArray, "/wheel/telemetry", self._on_tel, 10)
        self.create_subscription(Odometry, "/wheel/odom", self._on_odom, 10)

    def _on_tel(self, msg):
        d = list(msg.data)
        if len(d) >= 4:
            self.dist_l, self.dist_r, self.vel_l, self.vel_r = d[:4]

    def _on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.z * q.z + q.y * q.y))


class App:
    def __init__(self, root, node):
        self.node = node
        self.root = root
        root.title("Calibração de odometria")
        root.configure(bg="#1e1e2e")
        self.off = dict(dl=0.0, dr=0.0, x=0.0, y=0.0, yaw=0.0)

        big = ("Courier", 20, "bold")
        lbl = ("Courier", 11)
        self.vars = {}

        def row(parent, key, text):
            f = tk.Frame(parent, bg="#1e1e2e")
            f.pack(fill="x", padx=14, pady=2)
            tk.Label(f, text=text, width=16, anchor="w", font=lbl,
                     fg="#a6adc8", bg="#1e1e2e").pack(side="left")
            v = tk.StringVar(value="—")
            tk.Label(f, textvariable=v, font=big, fg="#89b4fa",
                     bg="#1e1e2e").pack(side="left")
            self.vars[key] = v

        tk.Label(root, text="RODAS", font=("Courier", 10, "bold"),
                 fg="#f9e2af", bg="#1e1e2e").pack(anchor="w", padx=14, pady=(10, 0))
        row(root, "dl", "dist esq (m)")
        row(root, "dr", "dist dir (m)")
        row(root, "vl", "vel esq (m/s)")
        row(root, "vr", "vel dir (m/s)")

        tk.Label(root, text="POSE", font=("Courier", 10, "bold"),
                 fg="#f9e2af", bg="#1e1e2e").pack(anchor="w", padx=14, pady=(10, 0))
        row(root, "x", "x (m)")
        row(root, "y", "y (m)")
        row(root, "yaw", "yaw (graus)")
        row(root, "path", "|desloc| (m)")

        tk.Button(root, text="ZERAR", command=self._zero, font=("Courier", 12, "bold"),
                  bg="#f38ba8", fg="#11111b", padx=20, pady=6).pack(pady=12)

        self._tick()

    def _zero(self):
        n = self.node
        self.off = dict(dl=n.dist_l, dr=n.dist_r, x=n.x, y=n.y, yaw=n.yaw)

    def _tick(self):
        n = self.node
        dl = n.dist_l - self.off["dl"]
        dr = n.dist_r - self.off["dr"]
        dx = n.x - self.off["x"]
        dy = n.y - self.off["y"]
        dyaw = math.degrees(n.yaw - self.off["yaw"])
        self.vars["dl"].set(f"{dl:+.3f}")
        self.vars["dr"].set(f"{dr:+.3f}")
        self.vars["vl"].set(f"{n.vel_l:+.3f}")
        self.vars["vr"].set(f"{n.vel_r:+.3f}")
        self.vars["x"].set(f"{dx:+.3f}")
        self.vars["y"].set(f"{dy:+.3f}")
        self.vars["yaw"].set(f"{dyaw:+.1f}")
        self.vars["path"].set(f"{math.hypot(dx, dy):.3f}")
        self.root.after(100, self._tick)


def main():
    rclpy.init()
    node = OdomListener()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    root = tk.Tk()
    App(root, node)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
