#===============================================================================
# report_only.tcl -- extrai os numeros do relatorio SEM re-sintetizar.
#
# Abre o checkpoint ja roteado (gerado pelo build_vivado.tcl) e imprime tudo o
# que vai para as tabelas. Leva ~30 s, nao os 5 min da sintese.
#
# Uso (de dentro da pasta fpga/):
#     vivado -mode batch -source sim/report_only.tcl
#===============================================================================

set ROOT [file normalize [file dirname [info script]]/..]
set OUT  $ROOT/vivado
set REP  $OUT/reports
file mkdir $REP

set DCP $OUT/lidar_accel/lidar_accel.runs/impl_1/lidar_accel_axi_routed.dcp
if {![file exists $DCP]} {
  puts "ERRO: nao achei o checkpoint roteado:"
  puts "  $DCP"
  puts "Rode antes:  vivado -mode batch -source sim/build_vivado.tcl"
  exit 1
}

open_checkpoint $DCP

#-------------------------------------------------------------------------------
# Relatorios em arquivo
#-------------------------------------------------------------------------------
report_timing_summary -delay_type min_max -max_paths 10 -file $REP/timing_summary.rpt
report_timing -sort_by group -max_paths 10 -path_type full -file $REP/timing_paths.rpt
report_utilization -file $REP/utilization.rpt
report_utilization -hierarchical -file $REP/utilization_hier.rpt
report_power -file $REP/power.rpt

#-------------------------------------------------------------------------------
# Timing
#-------------------------------------------------------------------------------
set FREQ_NS 10.0
set wns  [get_property SLACK [get_timing_paths -delay_type max]]
set whs  [get_property SLACK [get_timing_paths -delay_type min]]
set fmax [expr {1000.0 / ($FREQ_NS - $wns)}]

#-------------------------------------------------------------------------------
# Recursos -- conta por REF_NAME (o filtro PRIMITIVE_GROUP == ARITHMETIC nao casa
# com nada nesta versao do Vivado; foi o bug do build_vivado.tcl).
#-------------------------------------------------------------------------------
set luts  [llength [get_cells -hier -filter {REF_NAME =~ LUT*}]]
set ffs   [llength [get_cells -hier -filter {REF_NAME =~ FD*}]]
set dsps  [llength [get_cells -hier -filter {REF_NAME =~ DSP48*}]]
set brams [llength [get_cells -hier -filter {REF_NAME =~ RAMB*}]]
set srls  [llength [get_cells -hier -filter {REF_NAME =~ SRL*}]]

# quantos DSP estao dentro do CORDIC? (esperado: ZERO)
set dsp_cordic [llength [get_cells -hier -filter {REF_NAME =~ DSP48* && NAME =~ *u_cordic*}]]

#-------------------------------------------------------------------------------
# Potencia -- TOTAL_POWER nao e propriedade valida do run; le do proprio .rpt
#-------------------------------------------------------------------------------
set pwr_total "n/d"
set pwr_dyn   "n/d"
set pwr_stat  "n/d"
if {[file exists $REP/power.rpt]} {
  set fh [open $REP/power.rpt r]
  while {[gets $fh line] >= 0} {
    if {[regexp {Total On-Chip Power \(W\)\s*\|\s*([0-9.]+)} $line -> v]} { set pwr_total $v }
    if {[regexp {Dynamic \(W\)\s*\|\s*([0-9.]+)}             $line -> v]} { set pwr_dyn   $v }
    if {[regexp {Device Static \(W\)\s*\|\s*([0-9.]+)}       $line -> v]} { set pwr_stat  $v }
  }
  close $fh
}

#-------------------------------------------------------------------------------
# Resumo
#-------------------------------------------------------------------------------
puts "\n================ RESUMO PARA O RELATORIO ================"
puts [format "  Periodo alvo      : %.2f ns (%.0f MHz)" $FREQ_NS [expr {1000.0/$FREQ_NS}]]
puts [format "  WNS (setup)       : %+.3f ns   %s" $wns [expr {$wns >= 0 ? "-> TIMING FECHA" : "-> VIOLA"}]]
puts [format "  WHS (hold)        : %+.3f ns" $whs]
puts [format "  Fmax              : %.1f MHz" $fmax]
puts ""
puts [format "  LUT               : %5d  de  53200  (%.1f%%)" $luts [expr {100.0*$luts/53200}]]
puts [format "  Flip-flop         : %5d  de 106400  (%.1f%%)" $ffs  [expr {100.0*$ffs/106400}]]
puts [format "  DSP48             : %5d  de    220  (%.1f%%)" $dsps [expr {100.0*$dsps/220}]]
puts [format "  BRAM              : %5d  de    140  (%.1f%%)" $brams [expr {100.0*$brams/140}]]
puts [format "  SRL               : %5d" $srls]
puts ""
puts [format "  DSP dentro do CORDIC : %d   (esperado: 0 -- so somadores e shifts)" $dsp_cordic]
puts ""
puts [format "  Potencia total    : %s W" $pwr_total]
puts [format "    dinamica        : %s W" $pwr_dyn]
puts [format "    estatica        : %s W" $pwr_stat]
puts ""
puts "  Latencia (simulacao GHDL): 751 ciclos = 7,51 us @ 100 MHz"
puts "========================================================\n"
puts "Relatorios completos em: vivado/reports/"
