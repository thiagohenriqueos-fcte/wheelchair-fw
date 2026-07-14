#!/usr/bin/env python3
"""Gera rtl/cordic_pkg.vhd A PARTIR do golden model.

As constantes do RTL nao podem ser transcritas a mao: qualquer divergencia
quebra a equivalencia bit a bit. Este script e a unica fonte delas.
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import (W, Q_R, Q_T, Q_A, CORDIC_N, K_CAND, ATAN_TAB, PI_Q, HALF_PI_Q,
                   CORDIC_K, LANE_COS, LANE_SIN, DELTAS, q, LX_Q, LY_Q,
                   HALF_W_Q, FLOOR_Q, FRONT_Q)

def rows(vals, per=4, ind="    "):
    out = []
    for i in range(0, len(vals), per):
        chunk = ", ".join(f"to_signed({v:6d}, W)" for v in vals[i:i+per])
        out.append(ind + chunk)
    return ",\n".join(out)

vhd = f"""--------------------------------------------------------------------------------
-- cordic_pkg.vhd  -- GERADO por golden/gen_pkg.py. NAO EDITAR A MAO.
--
-- Formato de ponto fixo e constantes do acelerador do supervisor LiDAR.
-- A unica fonte destes valores e golden/model.py, de modo que o RTL e o modelo
-- casam por construcao -- o testbench compara as saidas bit a bit.
--
--   Q{W-1-Q_R}.{Q_R}  distancias e coordenadas, em metros  (LSB = {1000/(1<<Q_R):.3f} mm)
--   Q1.{Q_T}  cosseno e seno                       (LSB = {1/(1<<Q_T):.2e})
--   Q3.{Q_A}  angulos, em radianos                 (LSB = {1/(1<<Q_A):.2e} rad)
--
-- Nota de projeto: Q3.{Q_A} comporta +-4,0 rad. O angulo do feixe,
-- theta = angle_min + i*inc + laser_yaw, chega a -5,85 rad -- fora da faixa. O
-- host entrega theta ja ENVOLVIDO em [-pi, pi). Sem isso o valor satura e o
-- ponto e projetado no lugar errado (erro medido de 402 mm no 1o modelo).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package cordic_pkg is

  constant W      : integer := {W};
  constant Q_R    : integer := {Q_R};
  constant Q_T    : integer := {Q_T};
  constant Q_A    : integer := {Q_A};
  constant N_ITER : integer := {CORDIC_N};
  constant K_CAND : integer := {K_CAND};

  subtype word_t is signed(W-1 downto 0);
  type    word_array_t is array (natural range <>) of word_t;

  -- sentinela "nenhum obstaculo no corredor"
  constant INF_Q : word_t := to_signed(32767, W);

  constant PI_Q      : word_t := to_signed({PI_Q}, W);
  constant HALF_PI_Q : word_t := to_signed({HALF_PI_Q}, W);

  -- ganho do CORDIC: prod 1/sqrt(1+2^-2i) = {CORDIC_K:.7f}
  constant CORDIC_GAIN : word_t := to_signed({q(CORDIC_K, Q_T)}, W);

  -- geometria calibrada da cadeira (ver relatorio final, Experimento 4)
  constant LX_Q     : word_t := to_signed({LX_Q}, W);  -- laser_x
  constant LY_Q     : word_t := to_signed({LY_Q}, W);  -- laser_y
  constant HALF_W_Q : word_t := to_signed({HALF_W_Q}, W);  -- meia-largura
  constant FLOOR_Q  : word_t := to_signed({FLOOR_Q}, W);  -- frente + margem
  constant FRONT_Q  : word_t := to_signed({FRONT_Q}, W);  -- frente do chassi

  -- atan(2^-i) em Q3.{Q_A}
  type atan_tab_t is array (0 to N_ITER-1) of word_t;
  constant ATAN_TAB : atan_tab_t := (
{rows(ATAN_TAB)});

  -- cos/sin de cada candidato ({math.degrees(DELTAS[0]):+.0f}..{math.degrees(DELTAS[-1]):+.0f} graus, passo
  -- {math.degrees(DELTAS[1]-DELTAS[0]):.0f}). Sao CONSTANTES: cada pista vira um multiplicador de
  -- coeficiente fixo, que a sintese mapeia em DSP48 sem custo de roteamento.
  type lane_tab_t is array (0 to K_CAND-1) of word_t;
  constant LANE_COS : lane_tab_t := (
{rows(LANE_COS)});
  constant LANE_SIN : lane_tab_t := (
{rows(LANE_SIN)});

end package cordic_pkg;
"""
open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rtl", "cordic_pkg.vhd"), "w").write(vhd)
print("rtl/cordic_pkg.vhd gerado")
print(f"  conferencia: LANE_COS[9] (delta=0) = {LANE_COS[9]}  (deve ser {1<<Q_T} = 1,0)")
print(f"               LANE_SIN[9]           = {LANE_SIN[9]}  (deve ser 0)")
print(f"               LANE_COS[0] (delta=-45) = {LANE_COS[0]}")
