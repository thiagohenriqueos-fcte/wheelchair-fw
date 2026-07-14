--------------------------------------------------------------------------------
-- lane.vhd
--
-- Uma pista de candidato de desvio. Recebe o ponto ja projetado no base_link
-- (X, Y) e responde: "se a cadeira seguir na direcao delta_k, este ponto esta no
-- corredor que ela vai ocupar? Se sim, a que folga?"
--
-- Girar o corredor por delta e o mesmo que girar o ponto por -delta em torno do
-- eixo traseiro (o centro instantaneo de rotacao do acionamento diferencial):
--
--   xr = ( X*cos(d) + Y*sin(d)) >> Q_T
--   yr = (-X*sin(d) + Y*cos(d)) >> Q_T
--
-- cos(d) e sin(d) sao CONSTANTES de sintese (GENERIC), portanto cada pista custa
-- 4 multiplicadores de coeficiente fixo -- mapeados diretamente em DSP48.
--
-- O ponto conta como obstaculo se estiver dentro da largura da cadeira
-- (|yr| <= HALF_W_Q) e a frente do chassi (xr > FLOOR_Q). A estrutura da propria
-- cadeira cai atras de FLOOR_Q e e descartada por construcao: nao existe
-- "distancia minima" arbitraria a ajustar.
--
-- A pista mantem o MINIMO corrente da folga. Sem obstaculo, o resultado e INF_Q.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cordic_pkg.all;

entity lane is
  generic (
    COS_D : integer;   -- cos(delta_k) em Q1.14
    SIN_D : integer    -- sin(delta_k) em Q1.14
  );
  port (
    clk     : in  std_logic;
    rst     : in  std_logic;
    clr     : in  std_logic;             -- zera o minimo (inicio de varredura)
    in_vld  : in  std_logic;
    x_in    : in  word_t;                -- Q6.10
    y_in    : in  word_t;                -- Q6.10
    min_o   : out word_t                 -- Q6.10, ou INF_Q
  );
end entity lane;

architecture rtl of lane is
  constant CQ : signed(W-1 downto 0) := to_signed(COS_D, W);
  constant SQ : signed(W-1 downto 0) := to_signed(SIN_D, W);

  -- estagio 1: produtos (32 bits) e soma
  signal xr_1, yr_1 : signed(2*W-1 downto 0) := (others => '0');
  signal vld_1      : std_logic := '0';
  -- estagio 2: truncado de volta a Q6.10
  signal xr_2, yr_2 : word_t := (others => '0');
  signal vld_2      : std_logic := '0';
  -- acumulador de minimo
  signal best       : word_t := INF_Q;
begin

  -- Estagio 1: rotacao por coeficiente constante
  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        vld_1 <= '0';
      else
        xr_1  <= (x_in * CQ) + (y_in * SQ);
        yr_1  <= (-(x_in * SQ)) + (y_in * CQ);
        vld_1 <= in_vld;
      end if;
    end if;
  end process;

  -- Estagio 2: >> Q_T (truncamento, igual ao modelo de referencia)
  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        vld_2 <= '0';
      else
        xr_2  <= resize(shift_right(xr_1, Q_T), W);
        yr_2  <= resize(shift_right(yr_1, Q_T), W);
        vld_2 <= vld_1;
      end if;
    end if;
  end process;

  -- Estagio 3: teste de corredor e minimo corrente
  process (clk)
    variable d : word_t;
  begin
    if rising_edge(clk) then
      if rst = '1' or clr = '1' then
        best <= INF_Q;
      elsif vld_2 = '1' then
        if (abs(yr_2) <= HALF_W_Q) and (xr_2 > FLOOR_Q) then
          d := xr_2 - FRONT_Q;
          if d < best then
            best <= d;
          end if;
        end if;
      end if;
    end if;
  end process;

  min_o <= best;

end architecture rtl;
