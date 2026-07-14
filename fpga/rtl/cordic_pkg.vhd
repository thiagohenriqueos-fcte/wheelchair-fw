--------------------------------------------------------------------------------
-- cordic_pkg.vhd  -- GERADO por golden/gen_pkg.py. NAO EDITAR A MAO.
--
-- Formato de ponto fixo e constantes do acelerador do supervisor LiDAR.
-- A unica fonte destes valores e golden/model.py, de modo que o RTL e o modelo
-- casam por construcao -- o testbench compara as saidas bit a bit.
--
--   Q5.10  distancias e coordenadas, em metros  (LSB = 0.977 mm)
--   Q1.14  cosseno e seno                       (LSB = 6.10e-05)
--   Q3.13  angulos, em radianos                 (LSB = 1.22e-04 rad)
--
-- Nota de projeto: Q3.13 comporta +-4,0 rad. O angulo do feixe,
-- theta = angle_min + i*inc + laser_yaw, chega a -5,85 rad -- fora da faixa. O
-- host entrega theta ja ENVOLVIDO em [-pi, pi). Sem isso o valor satura e o
-- ponto e projetado no lugar errado (erro medido de 402 mm no 1o modelo).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package cordic_pkg is

  constant W      : integer := 16;
  constant Q_R    : integer := 10;
  constant Q_T    : integer := 14;
  constant Q_A    : integer := 13;
  constant N_ITER : integer := 16;
  constant K_CAND : integer := 19;

  subtype word_t is signed(W-1 downto 0);
  type    word_array_t is array (natural range <>) of word_t;

  -- sentinela "nenhum obstaculo no corredor"
  constant INF_Q : word_t := to_signed(32767, W);

  constant PI_Q      : word_t := to_signed(25736, W);
  constant HALF_PI_Q : word_t := to_signed(12868, W);

  -- ganho do CORDIC: prod 1/sqrt(1+2^-2i) = 0.6072529
  constant CORDIC_GAIN : word_t := to_signed(9949, W);

  -- geometria calibrada da cadeira (ver relatorio final, Experimento 4)
  constant LX_Q     : word_t := to_signed(683, W);  -- laser_x
  constant LY_Q     : word_t := to_signed(344, W);  -- laser_y
  constant HALF_W_Q : word_t := to_signed(312, W);  -- meia-largura
  constant FLOOR_Q  : word_t := to_signed(703, W);  -- frente + margem
  constant FRONT_Q  : word_t := to_signed(683, W);  -- frente do chassi

  -- atan(2^-i) em Q3.13
  type atan_tab_t is array (0 to N_ITER-1) of word_t;
  constant ATAN_TAB : atan_tab_t := (
    to_signed(  6434, W), to_signed(  3798, W), to_signed(  2007, W), to_signed(  1019, W),
    to_signed(   511, W), to_signed(   256, W), to_signed(   128, W), to_signed(    64, W),
    to_signed(    32, W), to_signed(    16, W), to_signed(     8, W), to_signed(     4, W),
    to_signed(     2, W), to_signed(     1, W), to_signed(     0, W), to_signed(     0, W));

  -- cos/sin de cada candidato (-45..+45 graus, passo
  -- 5). Sao CONSTANTES: cada pista vira um multiplicador de
  -- coeficiente fixo, que a sintese mapeia em DSP48 sem custo de roteamento.
  type lane_tab_t is array (0 to K_CAND-1) of word_t;
  constant LANE_COS : lane_tab_t := (
    to_signed( 11585, W), to_signed( 12551, W), to_signed( 13421, W), to_signed( 14189, W),
    to_signed( 14849, W), to_signed( 15396, W), to_signed( 15826, W), to_signed( 16135, W),
    to_signed( 16322, W), to_signed( 16384, W), to_signed( 16322, W), to_signed( 16135, W),
    to_signed( 15826, W), to_signed( 15396, W), to_signed( 14849, W), to_signed( 14189, W),
    to_signed( 13421, W), to_signed( 12551, W), to_signed( 11585, W));
  constant LANE_SIN : lane_tab_t := (
    to_signed(-11585, W), to_signed(-10531, W), to_signed( -9397, W), to_signed( -8192, W),
    to_signed( -6924, W), to_signed( -5604, W), to_signed( -4240, W), to_signed( -2845, W),
    to_signed( -1428, W), to_signed(     0, W), to_signed(  1428, W), to_signed(  2845, W),
    to_signed(  4240, W), to_signed(  5604, W), to_signed(  6924, W), to_signed(  8192, W),
    to_signed(  9397, W), to_signed( 10531, W), to_signed( 11585, W));

end package cordic_pkg;
