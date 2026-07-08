#!/usr/bin/env python3
"""Painel de demonstração da cadeira semi-assistida (terminal, colorido).

Mostra ao vivo, para a plateia: a intenção do joystick, a distância do LIDAR à
frente, a DECISÃO do supervisor (livre / reduz / desvia / para) e a saída para
o motor. Assina apenas /wheelchair/assist_status (não interfere no controle).

    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    python3 ~/wheelchair-fw/scripts/demo_monitor.py
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

R = "\033[0m"; B = "\033[1m"
GREEN = "\033[92m"; YELLOW = "\033[93m"; RED = "\033[91m"
CYAN = "\033[96m"; GREY = "\033[90m"
CLEAR = "\033[2J\033[H"

MODE = {
    "livre":       (GREEN,  "LIVRE  — segue o usuário"),
    "freia":       (YELLOW, "FREIA  — reduzindo velocidade"),
    "reduz":       (YELLOW, "REDUZ  — zona amarela"),
    "desvia":      (CYAN,   "DESVIA — arqueando p/ o lado livre"),
    "para":        (RED,    "PARA   — obstáculo à frente!"),
    "re":          (CYAN,   "RÉ"),
    "giro":        (GREY,   "GIRO no lugar"),
    "sem_scan":    (RED,    "SEM LIDAR — avanço bloqueado"),
    "sem_intencao":(GREY,   "joystick parado"),
}


def bar(frac, color=GREEN, n=22):
    frac = max(0.0, min(1.0, abs(frac)))
    f = int(frac * n)
    return color + "█" * f + GREY + "░" * (n - f) + R


class Demo(Node):
    def __init__(self):
        super().__init__("demo_monitor")
        self.st = {}
        self.create_subscription(
            String, "/wheelchair/assist_status", self._cb, 10)
        self.create_timer(0.1, self._render)

    def _cb(self, msg):
        try:
            self.st = json.loads(msg.data)
        except ValueError:
            pass

    def _render(self):
        s = self.st
        mode = s.get("mode", "—")
        color, label = MODE.get(mode, (R, mode))
        vu = float(s.get("v_user", 0.0) or 0.0)
        vo = float(s.get("v_out", 0.0) or 0.0)
        fc = s.get("front_clear_m")
        fc_s = "livre (> 3 m)" if fc is None else f"{float(fc):.2f} m"
        lines = [
            CLEAR,
            B + CYAN + "   CADEIRA DE RODAS — SEMI-ASSISTÊNCIA POR LIDAR" + R,
            "   " + "─" * 48,
            f"   Usuário (joystick):   {bar(vu)} {abs(vu) * 100:3.0f}%",
            f"   LIDAR à frente:       {B}{fc_s}{R}",
            "",
            f"   DECISÃO:   {B}{color}{label}{R}",
            f"   Saída p/ motor:       {bar(vo, color)} {abs(vo) * 100:3.0f}%",
            "   " + "─" * 48,
            GREY + "   Regra: o sistema pode FREAR/DESVIAR — nunca ACELERAR." + R,
            GREY + "   A iniciativa de movimento é sempre do usuário." + R,
        ]
        print("\n".join(lines), flush=True)


def main():
    rclpy.init()
    node = Demo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
