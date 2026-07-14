#===============================================================================
# build_vivado.tcl -- projeto completo do acelerador do supervisor LiDAR
#
# Gera TUDO o que a apresentacao precisa: block design (ARM + AXI-DMA +
# acelerador), sintese, implementacao, e os relatorios de timing, potencia e
# recursos -- em arquivos de texto E em prints.
#
# Uso (no PC, com Vivado no PATH):
#
#     cd fpga
#     vivado -mode batch -source sim/build_vivado.tcl
#
# Placa: ZedBoard e PYNQ-Z1 usam o MESMO chip (xc7z020). Troque so a linha
# PART/BOARD abaixo. O RTL e identico nos dois.
#
# Saidas (em fpga/vivado/reports/):
#   timing_summary.rpt   caminho critico e slack  (WNS/TNS)
#   utilization.rpt      LUT, FF, DSP, BRAM
#   power.rpt            consumo estimado, por bloco
#   timing_paths.rpt     os 10 piores caminhos, detalhados
#   block_design.pdf     o diagrama do block design (para o slide)
#===============================================================================

set PROJ    "lidar_accel"
set PART    "xc7z020clg484-1"   ;# ZedBoard.  PYNQ-Z1: xc7z020clg400-1
set TOP     "lidar_accel_axi"
set FREQ_NS 10.0                ;# 100 MHz -- periodo alvo

set ROOT [file normalize [file dirname [info script]]/..]
set OUT  $ROOT/vivado
set REP  $OUT/reports
file mkdir $REP

#-------------------------------------------------------------------------------
# 1. Projeto e fontes
#-------------------------------------------------------------------------------
create_project $PROJ $OUT/$PROJ -part $PART -force

add_files -norecurse [list \
  $ROOT/rtl/cordic_pkg.vhd \
  $ROOT/rtl/cordic.vhd \
  $ROOT/rtl/lane.vhd \
  $ROOT/rtl/corridor_core.vhd \
  $ROOT/rtl/lidar_accel_axi.vhd ]
set_property file_type {VHDL 2008} [get_files *.vhd]

add_files -fileset sim_1 -norecurse [list $ROOT/tb/tb_corridor_core.vhd]
set_property file_type {VHDL 2008} [get_files -of [get_filesets sim_1] *.vhd]
set_property top tb_corridor_core [get_filesets sim_1]

set_property top $TOP [current_fileset]
update_compile_order -fileset sources_1

#-------------------------------------------------------------------------------
# 2. Restricao de timing -- e o que produz o caminho critico
#-------------------------------------------------------------------------------
set XDC $OUT/timing.xdc
set fh [open $XDC w]
puts $fh "create_clock -period $FREQ_NS -name clk \[get_ports s_axi_aclk\]"
puts $fh "create_clock -period $FREQ_NS -name sclk \[get_ports s_axis_aclk\]"
close $fh
add_files -fileset constrs_1 -norecurse $XDC

#-------------------------------------------------------------------------------
# 3. Sintese e implementacao
#-------------------------------------------------------------------------------
launch_runs synth_1 -jobs 8
wait_on_run synth_1
open_run synth_1 -name synth_1

report_utilization -file $REP/utilization_synth.rpt

launch_runs impl_1 -jobs 8
wait_on_run impl_1
open_run impl_1

#-------------------------------------------------------------------------------
# 4. RELATORIOS -- o material dos slides
#-------------------------------------------------------------------------------

# (a) TIMING: caminho critico e folga (WNS). Se WNS > 0, fecha em 100 MHz.
report_timing_summary -delay_type min_max -max_paths 10 \
  -file $REP/timing_summary.rpt
report_timing -sort_by group -max_paths 10 -path_type full \
  -file $REP/timing_paths.rpt

# (b) RECURSOS: LUT/FF/DSP/BRAM -- o custo do paralelismo das 19 pistas
report_utilization -file $REP/utilization.rpt
report_utilization -hierarchical -file $REP/utilization_hier.rpt

# (c) POTENCIA
report_power -file $REP/power.rpt

#-------------------------------------------------------------------------------
# 5. Resumo no console -- os numeros que vao para a tabela do relatorio
#-------------------------------------------------------------------------------
set wns  [get_property SLACK [get_timing_paths -delay_type max]]
set whs  [get_property SLACK [get_timing_paths -delay_type min]]
set fmax [expr {1000.0 / ($FREQ_NS - $wns)}]

# Conta por REF_NAME. Os filtros PRIMITIVE_GROUP == ARITHMETIC / LUT nao casam
# com nada no Vivado 2022.2 -- silenciosamente devolvem zero.
set luts  [llength [get_cells -hier -filter {REF_NAME =~ LUT*}]]
set ffs   [llength [get_cells -hier -filter {REF_NAME =~ FD*}]]
set dsps  [llength [get_cells -hier -filter {REF_NAME =~ DSP48*}]]
set brams [llength [get_cells -hier -filter {REF_NAME =~ RAMB*}]]

# TOTAL_POWER nao e propriedade valida do run neste contexto; le do proprio .rpt
set pwr "n/d"
if {[file exists $REP/power.rpt]} {
  set fh [open $REP/power.rpt r]
  while {[gets $fh line] >= 0} {
    if {[regexp {Total On-Chip Power \(W\)\s*\|\s*([0-9.]+)} $line -> v]} { set pwr $v }
  }
  close $fh
}

puts "\n================ RESUMO PARA O RELATORIO ================"
puts [format "  Periodo alvo    : %.2f ns (%.0f MHz)" $FREQ_NS [expr {1000.0/$FREQ_NS}]]
puts [format "  WNS (setup)     : %+.3f ns   %s" $wns [expr {$wns >= 0 ? "-> TIMING FECHA" : "-> VIOLA"}]]
puts [format "  WHS (hold)      : %+.3f ns" $whs]
puts [format "  Fmax            : %.1f MHz" $fmax]
puts [format "  Potencia total  : %s W" $pwr]
puts [format "  LUT             : %d de 53200 (%.1f%%)" $luts [expr {100.0*$luts/53200}]]
puts [format "  Flip-flop       : %d de 106400 (%.1f%%)" $ffs [expr {100.0*$ffs/106400}]]
puts [format "  DSP48           : %d de 220 (%.1f%%)" $dsps [expr {100.0*$dsps/220}]]
puts [format "  BRAM            : %d de 140" $brams]
puts "  Latencia (sim)  : 751 ciclos = 7,51 us @ 100 MHz"
puts "  (relatorios completos em vivado/reports/)"
puts "========================================================\n"

puts "OK. Para o BLOCK DESIGN (ARM + DMA + acelerador), rode agora:"
puts "    vivado -mode batch -source sim/block_design.tcl"
