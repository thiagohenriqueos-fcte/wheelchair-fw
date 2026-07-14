--------------------------------------------------------------------------------
-- corridor_core.vhd
--
-- Nucleo do acelerador. Consome a varredura (um feixe por ciclo) e devolve, para
-- cada um dos K_CAND candidatos de desvio, a folga do corredor.
--
-- Fluxo:
--
--   (r, theta) --> CORDIC --> (cos, sin) --> projecao --> (X, Y) --> 19 pistas
--                                                                     em paralelo
--
--   X = LX_Q + (r*cos) >> Q_T        Y = LY_Q + (r*sin) >> Q_T
--
-- O CORDIC e compartilhado (uma unica instancia, pipelinada) porque a projecao e
-- comum a todos os candidatos: o custo O(N) e pago uma vez. O que se replica sao
-- as pistas, que carregam o custo O(N*K) -- e e exatamente ai que o paralelismo
-- de hardware paga: as 19 pistas rodam no mesmo ciclo, enquanto o software
-- precisa iterar sobre elas.
--
-- Vazao: 1 feixe/ciclo. Latencia: N_ITER + 5 ciclos (irrelevante frente aos 720
-- feixes de uma varredura).
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cordic_pkg.all;

entity corridor_core is
  port (
    clk       : in  std_logic;
    rst       : in  std_logic;
    -- controle
    start     : in  std_logic;                     -- pulso: inicia uma varredura
    -- entrada de feixes
    in_vld    : in  std_logic;
    in_last   : in  std_logic;                     -- ultimo feixe da varredura
    in_range  : in  word_t;                        -- Q6.10 (0 se invalido)
    in_theta  : in  word_t;                        -- Q3.13, envolvido em [-pi,pi)
    in_valid_beam : in std_logic;                  -- feixe util (range dentro da faixa)
    in_rdy    : out std_logic;
    -- resultado
    done      : out std_logic;                     -- pulso: varredura concluida
    clear_o   : out word_array_t(0 to K_CAND-1)    -- folga por candidato (Q6.10)
  );
end entity corridor_core;

architecture rtl of corridor_core is

  constant TAIL : integer := N_ITER + 6;  -- ciclos p/ drenar o pipeline

  signal clr        : std_logic := '0';
  signal cor_vld    : std_logic;
  signal cos_v, sin_v, tag_v : word_t;

  -- projecao (2 estagios)
  signal px_1, py_1 : signed(2*W-1 downto 0) := (others => '0');
  signal pv_1       : std_logic := '0';
  signal px_2, py_2 : word_t := (others => '0');
  signal pv_2       : std_logic := '0';

  signal mins       : word_array_t(0 to K_CAND-1);

  -- drenagem
  signal draining   : std_logic := '0';
  signal drain_cnt  : integer range 0 to TAIL := 0;
  signal cordic_vld : std_logic;

begin

  in_rdy <= '1';   -- o pipeline aceita um feixe por ciclo, sem bolhas

  -- so entram no CORDIC os feixes uteis; os invalidos sao simplesmente
  -- descartados aqui (o modelo de referencia faz o mesmo).
  cordic_vld <= in_vld and in_valid_beam;

  u_cordic : entity work.cordic
    port map (
      clk     => clk,
      rst     => rst,
      in_vld  => cordic_vld,
      theta   => in_theta,
      in_tag  => in_range,      -- o range viaja junto com o angulo
      out_vld => cor_vld,
      cos_o   => cos_v,
      sin_o   => sin_v,
      out_tag => tag_v
    );

  -- Projecao, estagio 1: r * cos, r * sin
  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        pv_1 <= '0';
      else
        px_1 <= tag_v * cos_v;
        py_1 <= tag_v * sin_v;
        pv_1 <= cor_vld;
      end if;
    end if;
  end process;

  -- Projecao, estagio 2: >> Q_T e soma da posicao do sensor
  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        pv_2 <= '0';
      else
        px_2 <= LX_Q + resize(shift_right(px_1, Q_T), W);
        py_2 <= LY_Q + resize(shift_right(py_1, Q_T), W);
        pv_2 <= pv_1;
      end if;
    end if;
  end process;

  -- As K_CAND pistas: mesmo (X, Y), coeficientes distintos, todas no mesmo ciclo
  gen_lanes : for k in 0 to K_CAND-1 generate
    u_lane : entity work.lane
      generic map (
        COS_D => to_integer(LANE_COS(k)),
        SIN_D => to_integer(LANE_SIN(k))
      )
      port map (
        clk    => clk,
        rst    => rst,
        clr    => clr,
        in_vld => pv_2,
        x_in   => px_2,
        y_in   => py_2,
        min_o  => mins(k)
      );
  end generate;

  clear_o <= mins;

  -- Controle: 'start' zera os minimos; 'in_last' dispara a drenagem do pipeline,
  -- e so entao o resultado e valido.
  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        clr <= '0'; done <= '0'; draining <= '0'; drain_cnt <= 0;
      else
        clr  <= start;
        done <= '0';
        if in_vld = '1' and in_last = '1' then
          draining  <= '1';
          drain_cnt <= TAIL;
        elsif draining = '1' then
          if drain_cnt = 0 then
            draining <= '0';
            done     <= '1';        -- pulso: clear_o ja esta estavel
          else
            drain_cnt <= drain_cnt - 1;
          end if;
        end if;
      end if;
    end if;
  end process;

end architecture rtl;
