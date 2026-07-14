#===============================================================================
# block_design.tcl -- Sistema em Chip: ARM Cortex-A9 + AXI-DMA + acelerador
#
# Monta o block design que demonstra o co-projeto HW/SW:
#
#   +------------------+   AXI4-Lite (GP0)   +--------------------+
#   |  Zynq PS         |-------------------->|  lidar_accel_axi   |
#   |  (ARM Cortex-A9) |                     |                    |
#   |                  |   AXI4-Stream       |  CORDIC + 19 lanes |
#   |  DDR: /scan      |==== AXI-DMA ========>|                    |
#   +------------------+   (HP0 -> S_AXIS)   +--------------------+
#            ^                                          |
#            +---------------- IRQ ---------------------+
#
# O ARM guarda a varredura na DDR, o AXI-DMA a transmite como AXI4-Stream para o
# acelerador, e o resultado (19 folgas) volta por AXI4-Lite. A interrupcao avisa
# o fim -- o ARM nao fica em espera ocupada.
#
# Uso:
#     vivado -mode batch -source sim/block_design.tcl
#
# Gera vivado/reports/block_design.pdf -- a figura do slide/relatorio.
#===============================================================================

set PROJ "lidar_accel_bd"
set PART "xc7z020clg484-1"    ;# ZedBoard. PYNQ-Z1: xc7z020clg400-1

set ROOT [file normalize [file dirname [info script]]/..]
set OUT  $ROOT/vivado
set REP  $OUT/reports
file mkdir $REP

create_project $PROJ $OUT/$PROJ -part $PART -force

add_files -norecurse [list \
  $ROOT/rtl/cordic_pkg.vhd \
  $ROOT/rtl/cordic.vhd \
  $ROOT/rtl/lane.vhd \
  $ROOT/rtl/corridor_core.vhd \
  $ROOT/rtl/lidar_accel_axi.vhd ]
set_property file_type {VHDL 2008} [get_files *.vhd]
update_compile_order -fileset sources_1

#-------------------------------------------------------------------------------
# Empacota o acelerador como IP (para instanciar no block design)
#-------------------------------------------------------------------------------
ipx::package_project -root_dir $OUT/ip_repo -vendor unb -library user \
  -taxonomy /UserIP -module lidar_accel_axi -import_files -force
ipx::unload_core $OUT/ip_repo/component.xml
set_property ip_repo_paths $OUT/ip_repo [current_project]
update_ip_catalog -rebuild

#-------------------------------------------------------------------------------
# Block design
#-------------------------------------------------------------------------------
create_bd_design "system"

# Zynq PS (o ARM)
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7 zynq
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
  -config {make_external "FIXED_IO, DDR" apply_board_preset "1"} [get_bd_cells zynq]
# habilita porta HP (para o DMA acessar a DDR) e a entrada de interrupcao
set_property -dict [list \
  CONFIG.PCW_USE_S_AXI_HP0 {1} \
  CONFIG.PCW_USE_FABRIC_INTERRUPT {1} \
  CONFIG.PCW_IRQ_F2P_INTR {1}] [get_bd_cells zynq]

# AXI-DMA: leva a varredura da DDR ate o acelerador, como AXI4-Stream
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma dma
set_property -dict [list \
  CONFIG.c_include_sg {0} \
  CONFIG.c_include_s2mm {0} \
  CONFIG.c_sg_include_stscntrl_strm {0} \
  CONFIG.c_m_axi_mm2s_data_width {32} \
  CONFIG.c_m_axis_mm2s_tdata_width {32}] [get_bd_cells dma]

# O acelerador
create_bd_cell -type ip -vlnv unb:user:lidar_accel_axi accel

# Conexoes
connect_bd_intf_net [get_bd_intf_pins dma/M_AXIS_MM2S] [get_bd_intf_pins accel/S_AXIS]
connect_bd_net [get_bd_pins accel/irq] [get_bd_pins zynq/IRQ_F2P]

apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
  -config {Master "/zynq/M_AXI_GP0" Clk "Auto"} [get_bd_intf_pins accel/S_AXI]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
  -config {Master "/zynq/M_AXI_GP0" Clk "Auto"} [get_bd_intf_pins dma/S_AXI_LITE]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
  -config {Master "/dma/M_AXI_MM2S" Slave "/zynq/S_AXI_HP0" Clk "Auto"} \
  [get_bd_intf_pins zynq/S_AXI_HP0]

regenerate_bd_layout
validate_bd_design
save_bd_design

#-------------------------------------------------------------------------------
# Exporta a FIGURA do block design (o print do slide)
#-------------------------------------------------------------------------------
write_bd_layout -force -format pdf -orientation landscape $REP/block_design.pdf
write_bd_layout -force -format png -orientation landscape $REP/block_design.png

# Wrapper HDL e implementacao (para os numeros do sistema completo)
make_wrapper -files [get_files system.bd] -top
add_files -norecurse $OUT/$PROJ/$PROJ.gen/sources_1/bd/system/hdl/system_wrapper.v
set_property top system_wrapper [current_fileset]
update_compile_order -fileset sources_1

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
open_run impl_1

report_utilization        -file $REP/bd_utilization.rpt
report_power              -file $REP/bd_power.rpt
report_timing_summary     -file $REP/bd_timing_summary.rpt

# Exporta o XSA (para o Vitis, se forem rodar no ARM de verdade)
write_hw_platform -fixed -include_bit -force $OUT/system.xsa

puts "\n=========================================================="
puts "  block_design.pdf / .png  -> vivado/reports/  (para o slide)"
puts "  bd_timing_summary.rpt, bd_power.rpt, bd_utilization.rpt"
puts "  system.xsa               -> para o Vitis (driver no ARM)"
puts "==========================================================\n"
