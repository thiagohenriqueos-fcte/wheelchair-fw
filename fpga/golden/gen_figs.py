#!/usr/bin/env python3
"""Gera os dados das figuras do relatorio, em formato PGFPlots (vetorial)."""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import (load_scan, load_meta, clearances_float, clearances_fixed, to_float,
                   LASER_X, LASER_Y, LASER_YAW, HALF_W, FRONT_EXT, MARGIN, DELTAS, DATA)

F = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "docs", "figs")
os.makedirs(F, exist_ok=True)
meta = load_meta()
ranges = load_scan(os.path.join(DATA, "scan_000.csv"))
a0, inc = meta["angle_min"], meta["angle_increment"]

# ── Fig 1: a varredura real projetada no base_link, com o corredor ───────────
dentro, fora = [], []
for i, rm in enumerate(ranges):
    if rm <= 0 or rm > 3.0:
        continue
    ca = a0 + i*inc + LASER_YAW
    X = LASER_X + rm*math.cos(ca)
    Y = LASER_Y + rm*math.sin(ca)
    (dentro if (abs(Y) <= HALF_W and X > FRONT_EXT+MARGIN) else fora).append((X, Y))

with open(f"{F}/scan_in.dat", "w") as f:
    f.write("x y\n")
    for x, y in dentro: f.write(f"{x:.4f} {y:.4f}\n")
with open(f"{F}/scan_out.dat", "w") as f:
    f.write("x y\n")
    for x, y in fora: f.write(f"{x:.4f} {y:.4f}\n")
print(f"scan: {len(dentro)} pontos no corredor, {len(fora)} fora")

# ── Fig 2: folga por candidato -- float vs ponto fixo ────────────────────────
fl = clearances_float(ranges, meta)
fx = [to_float(v) for v in clearances_fixed(ranges, meta)]
with open(f"{F}/clear_cmp.dat", "w") as f:
    f.write("delta float fixo erro_mm\n")
    for k, (a, b) in enumerate(zip(fl, fx)):
        d = math.degrees(DELTAS[k])
        if math.isinf(a) or math.isinf(b):
            continue
        f.write(f"{d:.0f} {a:.4f} {b:.4f} {abs(a-b)*1000:.2f}\n")
print("clear_cmp.dat escrito")

# ── Fig 3: cobertura do cone antigo vs corredor (o defeito que motivou tudo) ─
with open(f"{F}/cobertura.dat", "w") as f:
    f.write("d cone30 cone20\n")
    for d in [0.3+0.05*i for i in range(25)]:
        row = [d]
        for hc in (30, 20):
            half = d*math.tan(math.radians(hc))
            lo, hi = LASER_Y-half, LASER_Y+half
            a, b = max(lo, -HALF_W), min(hi, HALF_W)
            row.append(100*max(0.0, b-a)/(2*HALF_W))
        f.write(f"{row[0]:.2f} {row[1]:.1f} {row[2]:.1f}\n")
print("cobertura.dat escrito")
