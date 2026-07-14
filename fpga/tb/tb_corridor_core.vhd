--------------------------------------------------------------------------------
-- tb_corridor_core.vhd
--
-- Testbench automatico do nucleo do acelerador.
--
-- Le uma varredura REAL do LiDAR da cadeira (capturada em campo, com um balde no
-- antigo ponto cego), injeta os 720 feixes a um por ciclo, e compara as 19 folgas
-- produzidas pelo RTL com as do modelo de referencia em ponto fixo
-- (golden/model.py). A comparacao e BIT A BIT: qualquer divergencia e falha.
--
-- Arquivos (gerados por golden/model.py stimulus):
--   stimulus_NNN.txt : "range_q theta_q valid", um feixe por linha, em hexa
--   expected_NNN.txt : 19 folgas em hexa, uma por linha
--
-- Uso (GHDL):
--   ghdl -a --std=08 rtl/*.vhd tb/tb_corridor_core.vhd
--   ghdl -e --std=08 tb_corridor_core
--   ghdl -r --std=08 tb_corridor_core -gSCAN_ID=0
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;

library work;
use work.cordic_pkg.all;

entity tb_corridor_core is
  generic (
    SCAN_ID  : integer := 0;
    TB_DIR   : string  := "tb/"
  );
end entity tb_corridor_core;

architecture sim of tb_corridor_core is

  constant CLK_P : time := 10 ns;    -- 100 MHz

  signal clk, rst : std_logic := '0';
  signal start    : std_logic := '0';
  signal in_vld   : std_logic := '0';
  signal in_last  : std_logic := '0';
  signal in_range : word_t := (others => '0');
  signal in_theta : word_t := (others => '0');
  signal in_beam  : std_logic := '0';
  signal in_rdy   : std_logic;
  signal done     : std_logic;
  signal clear_o  : word_array_t(0 to K_CAND-1);

  signal sim_done : boolean := false;

  function scan_file(prefix : string; id : integer) return string is
    variable s : string(1 to 3);
  begin
    s := (character'val(48 + (id / 100) mod 10),
          character'val(48 + (id / 10) mod 10),
          character'val(48 + id mod 10));
    return TB_DIR & prefix & "_" & s & ".txt";
  end function;

begin

  clk <= not clk after CLK_P/2 when not sim_done else '0';

  dut : entity work.corridor_core
    port map (
      clk           => clk,
      rst           => rst,
      start         => start,
      in_vld        => in_vld,
      in_last       => in_last,
      in_range      => in_range,
      in_theta      => in_theta,
      in_valid_beam => in_beam,
      in_rdy        => in_rdy,
      done          => done,
      clear_o       => clear_o
    );

  stim : process
    file     fstim  : text;
    file     fexp   : text;
    file     fout   : text;
    variable l      : line;
    variable lo     : line;
    variable r_hex  : std_logic_vector(W-1 downto 0);
    variable t_hex  : std_logic_vector(W-1 downto 0);
    variable v_int  : integer;
    variable e_hex  : std_logic_vector(W-1 downto 0);
    variable nbeam  : integer := 0;
    variable nvalid : integer := 0;
    variable errors : integer := 0;
    variable got, exp : word_t;
    variable status : file_open_status;

    -- pre-leitura: quantos feixes o arquivo tem (p/ marcar o ultimo)
    variable total  : integer := 0;
  begin
    rst <= '1';
    wait for 5*CLK_P;
    rst <= '0';
    wait until rising_edge(clk);

    -- conta as linhas do estimulo
    file_open(status, fstim, scan_file("stimulus", SCAN_ID), read_mode);
    assert status = open_ok
      report "NAO ABRIU " & scan_file("stimulus", SCAN_ID) &
             " -- rode 'python3 golden/model.py stimulus' antes"
      severity failure;
    while not endfile(fstim) loop
      readline(fstim, l);
      total := total + 1;
    end loop;
    file_close(fstim);
    report "varredura " & integer'image(SCAN_ID) & ": " &
           integer'image(total) & " feixes";

    -- zera os minimos
    start <= '1';
    wait until rising_edge(clk);
    start <= '0';
    wait until rising_edge(clk);

    -- injeta um feixe por ciclo
    file_open(fstim, scan_file("stimulus", SCAN_ID), read_mode);
    while not endfile(fstim) loop
      readline(fstim, l);
      hread(l, r_hex);
      hread(l, t_hex);
      read(l, v_int);

      in_range <= signed(r_hex);
      in_theta <= signed(t_hex);
      in_beam  <= '1' when v_int = 1 else '0';
      in_vld   <= '1';
      nbeam    := nbeam + 1;
      if v_int = 1 then
        nvalid := nvalid + 1;
      end if;
      in_last  <= '1' when nbeam = total else '0';
      wait until rising_edge(clk);
    end loop;
    file_close(fstim);

    in_vld  <= '0';
    in_last <= '0';
    in_beam <= '0';

    -- espera a drenagem do pipeline
    wait until done = '1';
    wait until rising_edge(clk);

    report "feixes injetados: " & integer'image(nbeam) &
           " (validos: " & integer'image(nvalid) & ")";

    -- compara com o modelo de referencia, bit a bit
    file_open(status, fexp, scan_file("expected", SCAN_ID), read_mode);
    assert status = open_ok report "NAO ABRIU expected" severity failure;
    file_open(fout, TB_DIR & "rtl_out.txt", write_mode);

    for k in 0 to K_CAND-1 loop
      readline(fexp, l);
      hread(l, e_hex);
      exp := signed(e_hex);
      got := clear_o(k);

      hwrite(lo, std_logic_vector(got));
      writeline(fout, lo);

      if got /= exp then
        errors := errors + 1;
        report "PISTA " & integer'image(k) &
               ": RTL=" & integer'image(to_integer(got)) &
               "  modelo=" & integer'image(to_integer(exp)) &
               "  (delta=" & integer'image(to_integer(got) - to_integer(exp)) & ")"
          severity error;
      end if;
    end loop;
    file_close(fexp);
    file_close(fout);

    report "----------------------------------------";
    if errors = 0 then
      report "EQUIVALENCIA OK: as " & integer'image(K_CAND) &
             " folgas casam BIT A BIT com o modelo de referencia."
        severity note;
    else
      report integer'image(errors) & " de " & integer'image(K_CAND) &
             " pistas DIVERGEM do modelo." severity failure;
    end if;
    report "----------------------------------------";

    sim_done <= true;
    wait;
  end process;

end architecture sim;
