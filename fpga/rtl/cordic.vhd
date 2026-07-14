--------------------------------------------------------------------------------
-- cordic.vhd
--
-- CORDIC em modo rotacao, pipelinado: entrega cos(theta) e sin(theta) em Q1.14 a
-- partir de theta em Q3.13. Um resultado por ciclo depois de N_ITER+2 ciclos de
-- latencia.
--
-- Escolha de projeto: CORDIC em vez de LUT de seno. A LUT exigiria uma ROM
-- indexada pelo indice do feixe, o que so funciona se angle_min, o incremento e
-- o yaw forem fixos em sintese -- mas eles sao parametros de calibracao,
-- reescritos por AXI4-Lite. O CORDIC aceita qualquer angulo em tempo de execucao
-- e nao gasta um unico DSP48 (so somadores e deslocamentos cabeados).
--
-- O laco converge apenas para |z| <= 1,7434 rad, dai o estagio de reducao de
-- quadrante na entrada: theta fora de [-pi/2, pi/2] e trazido para dentro
-- somando/subtraindo pi, e o sinal de cos/sin e invertido na saida.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cordic_pkg.all;

entity cordic is
  port (
    clk     : in  std_logic;
    rst     : in  std_logic;
    -- entrada
    in_vld  : in  std_logic;
    theta   : in  word_t;                 -- Q3.13, ja envolvido em [-pi, pi)
    in_tag  : in  word_t;                 -- carona: viaja com o dado (o range)
    -- saida
    out_vld : out std_logic;
    cos_o   : out word_t;                 -- Q1.14
    sin_o   : out word_t;                 -- Q1.14
    out_tag : out word_t
  );
end entity cordic;

architecture rtl of cordic is

  -- estagios do pipeline: 0 = pos-reducao de quadrante; 1..N_ITER = iteracoes
  type vec_t is array (0 to N_ITER) of word_t;
  signal x, y, z, tg : vec_t := (others => (others => '0'));
  signal neg         : std_logic_vector(0 to N_ITER) := (others => '0');
  signal vld         : std_logic_vector(0 to N_ITER) := (others => '0');

begin

  --------------------------------------------------------------------------
  -- Estagio 0: reducao de quadrante
  --------------------------------------------------------------------------
  process (clk)
    variable zr : word_t;
    variable ng : std_logic;
  begin
    if rising_edge(clk) then
      if rst = '1' then
        vld(0) <= '0';
      else
        zr := theta;
        ng := '0';
        if theta > HALF_PI_Q then
          zr := theta - PI_Q;
          ng := '1';
        elsif theta < -HALF_PI_Q then
          zr := theta + PI_Q;
          ng := '1';
        end if;
        x(0)   <= CORDIC_GAIN;   -- pre-escala pelo ganho: dispensa normalizar depois
        y(0)   <= (others => '0');
        z(0)   <= zr;
        neg(0) <= ng;
        tg(0)  <= in_tag;
        vld(0) <= in_vld;
      end if;
    end if;
  end process;

  --------------------------------------------------------------------------
  -- Iteracoes: um estagio por micro-rotacao (somador + deslocamento cabeado)
  --------------------------------------------------------------------------
  gen_iter : for i in 0 to N_ITER-1 generate
    process (clk)
      variable dx, dy : word_t;
    begin
      if rising_edge(clk) then
        if rst = '1' then
          vld(i+1) <= '0';
        else
          -- deslocamento aritmetico: sem custo em hardware (fiacao)
          dx := shift_right(y(i), i);
          dy := shift_right(x(i), i);
          if z(i) >= 0 then
            x(i+1) <= x(i) - dx;
            y(i+1) <= y(i) + dy;
            z(i+1) <= z(i) - ATAN_TAB(i);
          else
            x(i+1) <= x(i) + dx;
            y(i+1) <= y(i) - dy;
            z(i+1) <= z(i) + ATAN_TAB(i);
          end if;
          neg(i+1) <= neg(i);
          tg(i+1)  <= tg(i);
          vld(i+1) <= vld(i);
        end if;
      end if;
    end process;
  end generate;

  --------------------------------------------------------------------------
  -- Saida: desfaz a reducao de quadrante
  --------------------------------------------------------------------------
  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        out_vld <= '0';
      else
        if neg(N_ITER) = '1' then
          cos_o <= -x(N_ITER);
          sin_o <= -y(N_ITER);
        else
          cos_o <= x(N_ITER);
          sin_o <= y(N_ITER);
        end if;
        out_tag <= tg(N_ITER);
        out_vld <= vld(N_ITER);
      end if;
    end if;
  end process;

end architecture rtl;
