--------------------------------------------------------------------------------
-- lidar_accel_axi.vhd
--
-- Envelope AXI4 do acelerador, para conexao ao ARM Cortex-A9 do Zynq (ZedBoard /
-- PYNQ-Z1, ambos xc7z020).
--
-- Particionamento HW/SW
-- ---------------------
-- O supervisor tem duas partes de custo bem distinto:
--
--   O(N*K)  projetar 720 feixes e testar 19 corredores  -> 9.918 avaliacoes
--   O(K)    somar a funcao custo e escolher o melhor    ->    19 comparacoes
--
-- So a primeira vai para hardware. A segunda fica no ARM: sao 19 operacoes, o
-- ganho seria irrisorio, e e ali que moram os pesos ajustaveis (w_obstacle,
-- w_deviation, assist_gain), que o usuario retoca em campo. Congelar isso em RTL
-- trocaria flexibilidade por nada.
--
-- Interfaces
-- ----------
--   S_AXIS  (AXI4-Stream) : os feixes, 1 por ciclo. TDATA = [theta(31:16) |
--                           range(15:0)], TUSER = feixe valido, TLAST = fim da
--                           varredura. Casa com o AXI-DMA levando /scan da DDR.
--   S_AXI   (AXI4-Lite)   : controle e resultado.
--
-- Mapa de registradores (AXI4-Lite, offsets em bytes)
-- ---------------------------------------------------
--   0x00  CTRL   [0] start (auto-limpa)   -- zera os minimos
--   0x04  STATUS [0] done   [1] busy
--   0x08  ...    reservado
--   0x40  CLEAR0 folga do candidato  0 (delta = -45 graus), Q6.10 com sinal
--   0x44  CLEAR1 ...
--   ...
--   0x88  CLEAR18 folga do candidato 18 (delta = +45 graus)
--
-- As folgas saem estendidas em sinal para 32 bits; o driver as le como int32 e
-- divide por 2^10 para obter metros.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.cordic_pkg.all;

entity lidar_accel_axi is
  generic (
    C_S_AXI_ADDR_WIDTH : integer := 8;
    C_S_AXI_DATA_WIDTH : integer := 32
  );
  port (
    -- ── AXI4-Stream: entrada dos feixes ────────────────────────────────────
    s_axis_aclk    : in  std_logic;
    s_axis_aresetn : in  std_logic;
    s_axis_tvalid  : in  std_logic;
    s_axis_tready  : out std_logic;
    s_axis_tdata   : in  std_logic_vector(31 downto 0);
    s_axis_tuser   : in  std_logic;                     -- '1' = feixe valido
    s_axis_tlast   : in  std_logic;

    -- ── AXI4-Lite: controle e resultado ────────────────────────────────────
    s_axi_aclk     : in  std_logic;
    s_axi_aresetn  : in  std_logic;
    s_axi_awaddr   : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    s_axi_awvalid  : in  std_logic;
    s_axi_awready  : out std_logic;
    s_axi_wdata    : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    s_axi_wstrb    : in  std_logic_vector(3 downto 0);
    s_axi_wvalid   : in  std_logic;
    s_axi_wready   : out std_logic;
    s_axi_bresp    : out std_logic_vector(1 downto 0);
    s_axi_bvalid   : out std_logic;
    s_axi_bready   : in  std_logic;
    s_axi_araddr   : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    s_axi_arvalid  : in  std_logic;
    s_axi_arready  : out std_logic;
    s_axi_rdata    : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    s_axi_rresp    : out std_logic_vector(1 downto 0);
    s_axi_rvalid   : out std_logic;
    s_axi_rready   : in  std_logic;

    irq            : out std_logic          -- pulso ao concluir a varredura
  );
end entity lidar_accel_axi;

architecture rtl of lidar_accel_axi is

  -- Nota: usa-se um unico dominio de relogio (s_axi_aclk = s_axis_aclk, como no
  -- bloco Zynq padrao). Com relogios distintos seria preciso um FIFO CDC.
  signal clk : std_logic;
  signal rst : std_logic;

  signal start_p  : std_logic := '0';
  signal core_done: std_logic;
  signal busy     : std_logic := '0';
  signal done_lat : std_logic := '0';
  signal clears   : word_array_t(0 to K_CAND-1);

  -- AXI4-Lite
  signal awready_i, wready_i, bvalid_i : std_logic := '0';
  signal arready_i, rvalid_i           : std_logic := '0';
  signal rdata_i : std_logic_vector(31 downto 0) := (others => '0');
  signal araddr_q : unsigned(C_S_AXI_ADDR_WIDTH-1 downto 0) := (others => '0');

begin

  clk <= s_axi_aclk;
  rst <= not s_axi_aresetn;

  s_axis_tready <= '1';   -- o pipeline nunca causa contrapressao

  u_core : entity work.corridor_core
    port map (
      clk           => clk,
      rst           => rst,
      start         => start_p,
      in_vld        => s_axis_tvalid,
      in_last       => s_axis_tlast,
      in_range      => signed(s_axis_tdata(15 downto 0)),
      in_theta      => signed(s_axis_tdata(31 downto 16)),
      in_valid_beam => s_axis_tuser,
      in_rdy        => open,
      done          => core_done,
      clear_o       => clears
    );

  -- estado busy/done
  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        busy <= '0'; done_lat <= '0'; irq <= '0';
      else
        irq <= core_done;
        if start_p = '1' then
          busy <= '1'; done_lat <= '0';
        elsif core_done = '1' then
          busy <= '0'; done_lat <= '1';
        end if;
      end if;
    end if;
  end process;

  ------------------------------------------------------------------------------
  -- AXI4-Lite: escrita
  ------------------------------------------------------------------------------
  process (clk)
  begin
    if rising_edge(clk) then
      if rst = '1' then
        awready_i <= '0'; wready_i <= '0'; bvalid_i <= '0'; start_p <= '0';
      else
        start_p <= '0';   -- start e um pulso de 1 ciclo

        if awready_i = '0' and s_axi_awvalid = '1' and s_axi_wvalid = '1' then
          awready_i <= '1';
          wready_i  <= '1';
          -- CTRL (offset 0x00), bit 0 -> start
          if unsigned(s_axi_awaddr(7 downto 2)) = 0 and s_axi_wstrb(0) = '1' then
            start_p <= s_axi_wdata(0);
          end if;
        else
          awready_i <= '0';
          wready_i  <= '0';
        end if;

        if awready_i = '1' then
          bvalid_i <= '1';
        elsif s_axi_bready = '1' then
          bvalid_i <= '0';
        end if;
      end if;
    end if;
  end process;

  s_axi_awready <= awready_i;
  s_axi_wready  <= wready_i;
  s_axi_bvalid  <= bvalid_i;
  s_axi_bresp   <= "00";   -- OKAY

  ------------------------------------------------------------------------------
  -- AXI4-Lite: leitura
  ------------------------------------------------------------------------------
  process (clk)
    variable idx : integer;
  begin
    if rising_edge(clk) then
      if rst = '1' then
        arready_i <= '0'; rvalid_i <= '0'; rdata_i <= (others => '0');
      else
        if arready_i = '0' and s_axi_arvalid = '1' then
          arready_i <= '1';
          araddr_q  <= unsigned(s_axi_araddr);
        else
          arready_i <= '0';
        end if;

        if arready_i = '1' then
          rvalid_i <= '1';
          case to_integer(araddr_q(7 downto 2)) is
            when 0 =>                                  -- 0x00 CTRL
              rdata_i <= (others => '0');
            when 1 =>                                  -- 0x04 STATUS
              rdata_i <= (0 => done_lat, 1 => busy, others => '0');
            when others =>
              -- 0x40..0x88 -> CLEAR0..CLEAR18
              idx := to_integer(araddr_q(7 downto 2)) - 16;
              if idx >= 0 and idx < K_CAND then
                -- extensao de sinal p/ 32 bits: o driver le como int32
                rdata_i <= std_logic_vector(resize(clears(idx), 32));
              else
                rdata_i <= (others => '0');
              end if;
          end case;
        elsif s_axi_rready = '1' then
          rvalid_i <= '0';
        end if;
      end if;
    end if;
  end process;

  s_axi_arready <= arready_i;
  s_axi_rvalid  <= rvalid_i;
  s_axi_rdata   <= rdata_i;
  s_axi_rresp   <= "00";   -- OKAY

end architecture rtl;
