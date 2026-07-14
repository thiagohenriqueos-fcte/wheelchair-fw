#!/usr/bin/env python3
"""Gera as figuras de comparacao HW x SW.

IMPORTANTE: o "HW" aqui e a saida REAL da simulacao VHDL (tb/rtl_out_*.txt),
nao o modelo. O "SW" e a referencia em ponto flutuante -- a mesma matematica que
roda hoje no no ROS 2 da cadeira.
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import (load_scan, load_meta, clearances_float, DELTAS, DATA, Q_R, INF_Q)

HERE = os.path.dirname(os.path.abspath(__file__))
TB = os.path.join(HERE, "..", "tb")
F = os.path.join(HERE, "..", "docs", "figs")
os.makedirs(F, exist_ok=True)

def read_rtl(sid):
    out = []
    for line in open(os.path.join(TB, f"rtl_out_{sid:03d}.txt")):
        v = int(line.strip(), 16)
        if v >= 0x8000: v -= 0x10000        # complemento de 2
        out.append(math.inf if v == INF_Q else v / (1 << Q_R))
    return out

meta = load_meta()

# ── Fig: folga por candidato, SW (float) vs HW (RTL simulado) ────────────────
rows, errs = [], []
for sid in range(3):
    ranges = load_scan(os.path.join(DATA, f"scan_{sid:03d}.csv"))
    sw = clearances_float(ranges, meta)
    hw = read_rtl(sid)
    for k, (a, b) in enumerate(zip(sw, hw)):
        if math.isinf(a) or math.isinf(b):
            continue
        e = (b - a) * 1000.0                 # erro com SINAL, em mm
        rows.append((sid, math.degrees(DELTAS[k]), a, b, e))
        errs.append(abs(e))

# curva SW vs HW (varredura 0) -- as duas se sobrepoem: e esse o ponto
with open(f"{F}/sw_hw_scan0.dat", "w") as f:
    f.write("delta sw hw\n")
    for sid, d, a, b, e in rows:
        if sid == 0:
            f.write(f"{d:.0f} {a:.4f} {b:.4f}\n")

# erro (com sinal) por candidato, as 3 varreduras
for sid in range(3):
    with open(f"{F}/err_scan{sid}.dat", "w") as f:
        f.write("delta err_mm\n")
        for s, d, a, b, e in rows:
            if s == sid:
                f.write(f"{d:.0f} {e:+.3f}\n")

lsb = 1000.0 / (1 << Q_R)
print(f"amostras comparadas : {len(rows)}  (3 varreduras x 19 candidatos, sem os INF)")
print(f"erro |HW - SW|      : medio {sum(errs)/len(errs):.2f} mm | maximo {max(errs):.2f} mm")
print(f"LSB de Q6.10        : {lsb:.2f} mm  -> erro maximo = {max(errs)/lsb:.1f} LSB")

# ── Fig: tempo de execucao (barras, escala log) ─────────────────────────────
with open(f"{F}/tempos.dat", "w") as f:
    f.write("impl tempo_us\n")
    f.write("escalar 3542\n")
    f.write("NumPy 488\n")
    f.write("HW100 7.51\n")
    f.write("HW150 5.01\n")
print("tempos.dat escrito")
