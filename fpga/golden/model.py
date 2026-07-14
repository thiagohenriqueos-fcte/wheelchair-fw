#!/usr/bin/env python3
"""Modelo de referencia do supervisor LiDAR: float (ouro) e ponto fixo bit-exato.

O modelo de ponto fixo emula EXATAMENTE o RTL (mesmo CORDIC, mesmos shifts e
truncamentos), de modo que a simulacao VHDL deve casar bit a bit com ele. O
modelo float e a referencia fisica, usada para medir o erro de quantizacao --
"precisao como parametro de projeto".

Uso:
  model.py stimulus   -> gera tb/stimulus_*.txt e tb/expected_*.txt
  model.py error      -> erro float vs ponto fixo sobre os scans reais
  model.py bench      -> baseline de tempo em software (Pi 5)
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DATA = os.path.join(ROOT, "data")
TB = os.path.join(ROOT, "tb")

# ── Calibracao da cadeira (medida em 2026-07-09; ver relatorio, Exp. 4) ───────
LASER_X = 0.667      # m, a frente do eixo traseiro
LASER_Y = 0.336      # m, a esquerda do eixo longitudinal
LASER_YAW = math.radians(-155.22)
HALF_W = 0.305       # m, meia-largura do corredor (cadeira: 0,61 m)
FRONT_EXT = 0.667    # m, frente do chassi
MARGIN = 0.02        # m
DEV_MAX = math.radians(45.0)
K_CAND = 19          # candidatos de desvio

# ── Formato de ponto fixo (identico ao RTL) ──────────────────────────────────
W = 16               # largura das palavras
Q_R = 10             # Q6.10  -> distancias/coordenadas (LSB = 0,977 mm)
Q_T = 14             # Q1.14  -> cosseno/seno          (LSB = 6,1e-5)
Q_A = 13             # Q3.13  -> angulos em rad        (LSB = 1,22e-4 rad)
CORDIC_N = 16        # iteracoes
INF_Q = 0x7FFF       # sentinela "sem obstaculo"

CORDIC_K = 0.6072529350088812561694  # ganho: prod 1/sqrt(1+2^-2i)


def q(x: float, frac: int) -> int:
    """Converte float -> inteiro em ponto fixo (arredondamento p/ o mais proximo)."""
    return int(math.floor(x * (1 << frac) + 0.5))


def sat16(v: int) -> int:
    return max(-32768, min(32767, v))


def asr(v: int, s: int) -> int:
    """Arithmetic shift right -- em Python o >> ja e aritmetico p/ negativos."""
    return v >> s


def wrap_pi(a: float) -> float:
    """Envolve o angulo em [-pi, pi).

    Indispensavel: theta = angle_min + i*inc + laser_yaw chega a -5,85 rad, o que
    NAO cabe em Q3.13 (+-4,0). Sem isto o angulo satura e o ponto e projetado no
    lugar errado -- foi a origem de um erro de 402 mm no primeiro modelo.
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


ATAN_TAB = [q(math.atan(2.0 ** -i), Q_A) for i in range(CORDIC_N)]
PI_Q = q(math.pi, Q_A)
HALF_PI_Q = q(math.pi / 2, Q_A)


def cordic_fixed(theta_q: int) -> tuple[int, int]:
    """cos/sin em Q1.14 a partir de angulo Q3.13. Bit-exato com o RTL."""
    # reducao de quadrante: CORDIC so converge em |z| <= ~1.7434 rad
    neg = False
    z = theta_q
    while z > PI_Q:
        z -= 2 * PI_Q
    while z < -PI_Q:
        z += 2 * PI_Q
    if z > HALF_PI_Q:
        z -= PI_Q
        neg = True
    elif z < -HALF_PI_Q:
        z += PI_Q
        neg = True

    x = q(CORDIC_K, Q_T)
    y = 0
    for i in range(CORDIC_N):
        dx = asr(y, i)
        dy = asr(x, i)
        if z >= 0:
            x, y, z = x - dx, y + dy, z - ATAN_TAB[i]
        else:
            x, y, z = x + dx, y - dy, z + ATAN_TAB[i]
    if neg:
        x, y = -x, -y
    return sat16(x), sat16(y)


# constantes por pista (cos/sin de cada candidato) -- Q1.14
DELTAS = [(-DEV_MAX + i * (2 * DEV_MAX / (K_CAND - 1))) for i in range(K_CAND)]
LANE_COS = [q(math.cos(d), Q_T) for d in DELTAS]
LANE_SIN = [q(math.sin(d), Q_T) for d in DELTAS]

LX_Q = q(LASER_X, Q_R)
LY_Q = q(LASER_Y, Q_R)
HALF_W_Q = q(HALF_W, Q_R)
FLOOR_Q = q(FRONT_EXT + MARGIN, Q_R)
FRONT_Q = q(FRONT_EXT, Q_R)


def load_scan(path: str) -> list[float]:
    with open(path) as f:
        return [float(row["range_m"]) for row in csv.DictReader(f)]


def load_meta() -> dict:
    return json.load(open(os.path.join(DATA, "scan_meta.json")))


# ── Referencia FLOAT (mesma matematica do no ROS 2) ──────────────────────────
def clearances_float(ranges: list[float], meta: dict) -> list[float]:
    a0, inc = meta["angle_min"], meta["angle_increment"]
    r = np.asarray(ranges)
    ok = r > 0
    a = a0 + np.arange(len(r)) * inc
    ca = a[ok] + LASER_YAW
    rr = r[ok]
    X = LASER_X + rr * np.cos(ca)
    Y = LASER_Y + rr * np.sin(ca)
    out = []
    for d in DELTAS:
        c, s = math.cos(d), math.sin(d)
        xr = X * c + Y * s
        yr = -X * s + Y * c
        hit = (np.abs(yr) <= HALF_W) & (xr > FRONT_EXT + MARGIN)
        out.append(float(xr[hit].min() - FRONT_EXT) if hit.any() else math.inf)
    return out


# ── Modelo PONTO FIXO bit-exato (o que o RTL faz) ────────────────────────────
def clearances_fixed(ranges: list[float], meta: dict) -> list[int]:
    a0, inc = meta["angle_min"], meta["angle_increment"]
    yaw_q = q(LASER_YAW, Q_A)
    best = [INF_Q] * K_CAND
    for i, rm in enumerate(ranges):
        if rm <= 0:
            continue                       # feixe invalido: descartado
        r_q = sat16(q(rm, Q_R))
        th_q = sat16(q(wrap_pi(a0 + i * inc + LASER_YAW), Q_A))
        c, s = cordic_fixed(th_q)
        X = LX_Q + asr(r_q * c, Q_T)
        Y = LY_Q + asr(r_q * s, Q_T)
        for k in range(K_CAND):
            xr = asr(X * LANE_COS[k] + Y * LANE_SIN[k], Q_T)
            yr = asr(-X * LANE_SIN[k] + Y * LANE_COS[k], Q_T)
            if abs(yr) <= HALF_W_Q and xr > FLOOR_Q:
                d = xr - FRONT_Q
                if d < best[k]:
                    best[k] = d
    return best


def to_float(v: int) -> float:
    return math.inf if v == INF_Q else v / (1 << Q_R)


# ── Geracao de estimulo e esperado para o testbench ──────────────────────────
def gen_stimulus() -> None:
    os.makedirs(TB, exist_ok=True)
    meta = load_meta()
    scans = sorted(f for f in os.listdir(DATA) if f.startswith("scan_") and f.endswith(".csv"))
    a0, inc = meta["angle_min"], meta["angle_increment"]
    yaw_q = q(LASER_YAW, Q_A)

    for si, fn in enumerate(scans):
        ranges = load_scan(os.path.join(DATA, fn))
        # estimulo: um feixe por linha -> "range_q theta_q valid"
        with open(os.path.join(TB, f"stimulus_{si:03d}.txt"), "w") as f:
            for i, rm in enumerate(ranges):
                valid = 1 if rm > 0 else 0
                r_q = sat16(q(rm, Q_R)) if valid else 0
                th_q = sat16(q(wrap_pi(a0 + i * inc + LASER_YAW), Q_A))
                f.write(f"{r_q & 0xFFFF:04X} {th_q & 0xFFFF:04X} {valid}\n")
        exp = clearances_fixed(ranges, meta)
        with open(os.path.join(TB, f"expected_{si:03d}.txt"), "w") as f:
            for v in exp:
                f.write(f"{v & 0xFFFF:04X}\n")
        print(f"{fn}: {len(ranges)} feixes -> stimulus_{si:03d}.txt, expected_{si:03d}.txt")

    # parametros do RTL, para conferencia
    with open(os.path.join(TB, "params.txt"), "w") as f:
        f.write(f"LX_Q     {LX_Q}\nLY_Q     {LY_Q}\n")
        f.write(f"HALF_W_Q {HALF_W_Q}\nFLOOR_Q  {FLOOR_Q}\nFRONT_Q  {FRONT_Q}\n")
        for k in range(K_CAND):
            f.write(f"LANE {k:2d}  delta={math.degrees(DELTAS[k]):+6.1f}  "
                    f"cos={LANE_COS[k]:6d}  sin={LANE_SIN[k]:6d}\n")
    print(f"params.txt escrito ({K_CAND} pistas)")


# ── Analise de erro: float vs ponto fixo ─────────────────────────────────────
def error_analysis() -> None:
    meta = load_meta()
    scans = sorted(f for f in os.listdir(DATA) if f.startswith("scan_") and f.endswith(".csv"))
    print(f"{'scan':>10} {'cand':>5} {'float (m)':>10} {'fixo (m)':>10} {'erro (mm)':>10}")
    print("-" * 52)
    worst = 0.0
    errs = []
    for fn in scans:
        ranges = load_scan(os.path.join(DATA, fn))
        fl = clearances_float(ranges, meta)
        fx = [to_float(v) for v in clearances_fixed(ranges, meta)]
        for k, (a, b) in enumerate(zip(fl, fx)):
            if math.isinf(a) and math.isinf(b):
                continue
            if math.isinf(a) or math.isinf(b):
                print(f"{fn:>10} {k:5d}  DIVERGENCIA inf: float={a} fixo={b}")
                continue
            e = abs(a - b) * 1000
            errs.append(e)
            if e > worst:
                worst = e
            if k in (0, 9, 18):
                print(f"{fn:>10} {k:5d} {a:10.4f} {b:10.4f} {e:10.2f}")
    if errs:
        print("-" * 52)
        print(f"erro medio = {sum(errs)/len(errs):.2f} mm   maximo = {worst:.2f} mm")
        print(f"LSB de Q6.10 = {1000.0/(1 << Q_R):.2f} mm   "
              f"(erro maximo = {worst/(1000.0/(1 << Q_R)):.1f} LSB)")


# ── Baseline de software (para a tabela de speedup) ──────────────────────────
def bench() -> None:
    meta = load_meta()
    ranges = load_scan(os.path.join(DATA, "scan_000.csv"))
    a0, inc = meta["angle_min"], meta["angle_increment"]

    # (a) escalar puro -- a implementacao original do no ROS 2
    def scalar():
        best = [math.inf] * K_CAND
        for i, rm in enumerate(ranges):
            if rm <= 0:
                continue
            ca = a0 + i * inc + LASER_YAW
            X = LASER_X + rm * math.cos(ca)
            Y = LASER_Y + rm * math.sin(ca)
            for k, d in enumerate(DELTAS):
                c, s = math.cos(d), math.sin(d)
                xr = X * c + Y * s
                yr = -X * s + Y * c
                if abs(yr) <= HALF_W and xr > FRONT_EXT + MARGIN:
                    best[k] = min(best[k], xr - FRONT_EXT)
        return best

    reps = 50
    t = time.perf_counter()
    for _ in range(reps):
        scalar()
    t_scalar = (time.perf_counter() - t) / reps

    t = time.perf_counter()
    for _ in range(reps):
        clearances_float(ranges, meta)
    t_numpy = (time.perf_counter() - t) / reps

    n = len(ranges)
    nval = sum(1 for r in ranges if r > 0)
    print(f"pontos por varredura : {n} ({nval} validos)")
    print(f"candidatos           : {K_CAND}")
    print(f"avaliacoes/varredura : {nval * K_CAND}")
    print()
    print(f"software escalar (Python puro) : {t_scalar*1e3:8.3f} ms/varredura")
    print(f"software vetorizado (NumPy)    : {t_numpy*1e3:8.3f} ms/varredura")
    print()
    # Ciclos MEDIDOS na simulacao GHDL (tb_corridor_core, scan real): 7515 ns
    # @ 100 MHz. O hardware consome os 720 feixes a 1/ciclo -- os invalidos
    # tambem ocupam um ciclo, apenas nao entram no CORDIC -- mais a drenagem do
    # pipeline. Nao e estimativa: e o numero que o testbench produziu.
    CYCLES = 751
    for f_mhz in (100, 150):
        t_hw = CYCLES / (f_mhz * 1e6)
        print(f"HW @ {f_mhz:3d} MHz: {CYCLES} ciclos = {t_hw*1e6:7.2f} us  "
              f"-> speedup {t_scalar/t_hw:6.0f}x (escalar) / {t_numpy/t_hw:5.0f}x (NumPy)")
    print()
    print("Latencia do supervisor: o LiDAR entrega uma varredura a cada 100 ms.")
    print(f"  software (NumPy): {t_numpy*1e3:.2f} ms  -> {100*t_numpy*1e3/100:.1f}% do orcamento")
    print(f"  hardware        : {751/100e6*1e3:.3f} ms  -> {100*(751/100e6*1e3)/100:.3f}% do orcamento")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "stimulus"
    {"stimulus": gen_stimulus, "error": error_analysis, "bench": bench}[cmd]()
