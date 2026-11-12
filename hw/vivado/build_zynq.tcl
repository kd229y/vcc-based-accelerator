# Zynq flow: PS drives vcc_axil_top over AXI4-Lite (M_AXI_GP0 / M_AXI_HPM0), IRQ to PL->PS interrupt.
# Windows:  vivado -mode batch -source build_zynq.tcl -tclargs pynq-z2
#           vivado -mode batch -source build_zynq.tcl -tclargs zybo-z7-20
#           vivado -mode batch -source build_zynq.tcl -tclargs kv260
#           vivado -mode batch -source build_zynq.tcl -tclargs custom7 <part> | customzu <part>
# Result: out_<board>/vcc_zynq.xsa (+ .bit, .hwh for PYNQ).  Accelerator base address: 0x40000000 (7-series)
#         or 0xA0000000 (UltraScale+), 16 MiB window.
set here  [file dirname [file normalize [info script]]]
set rtl   [file normalize $here/../rtl]
set board [lindex $argv 0]
set HASH_AW 14; set CELL_AW 12; set MAXN_AW 9; set RAM_STYLE block
switch $board {
  pynq-z2    { set part xc7z020clg400-1;     set ps 7 }
  zybo-z7-20 { set part xc7z020clg400-1;     set ps 7 }
  kv260      { set part xck26-sfvc784-2LV-c; set ps zu; set HASH_AW 15; set MAXN_AW 10; set RAM_STYLE ultra }
  custom7    { set part [lindex $argv 1];    set ps 7 }
  customzu   { set part [lindex $argv 1];    set ps zu; set HASH_AW 15; set MAXN_AW 10; set RAM_STYLE ultra }
  default    { puts "usage: -tclargs pynq-z2|zybo-z7-20|kv260|custom7 <part>|customzu <part>"; exit 1 }
}
set out $here/out_$board
file delete -force $out/proj
create_project vcc_zynq $out/proj -part $part
foreach f {ram_sdp.v stage_a.v build_engine.v query_engine.v panda_consts.v fk_engine.v arith_units.v rrt_engine.v vcc_core.v vcc_top.v} {
  add_files $rtl/$f
}
update_compile_order -fileset sources_1
create_bd_design vcc_bd

if {$ps == 7} {
  set p [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7 ps]
  # DDR/MIO presets: apply board preset when the board files are installed, otherwise use defaults.
  catch { apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 -config {make_external "FIXED_IO, DDR" apply_board_preset "1"} $p }
  set_property -dict [list CONFIG.PCW_USE_M_AXI_GP0 {1} CONFIG.PCW_USE_FABRIC_INTERRUPT {1} CONFIG.PCW_IRQ_F2P_INTR {1} \
                           CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100}] $p
  set clk  [get_bd_pins ps/FCLK_CLK0]
  set rstn [get_bd_pins ps/FCLK_RESET0_N]
  set maxi [get_bd_intf_pins ps/M_AXI_GP0]
  set base 0x40000000
} else {
  set p [create_bd_cell -type ip -vlnv xilinx.com:ip:zynq_ultra_ps_e ps]
  catch { apply_bd_automation -rule xilinx.com:bd_rule:zynq_ultra_ps_e -config {apply_board_preset "1"} $p }
  set_property -dict [list CONFIG.PSU__USE__M_AXI_GP0 {1} CONFIG.PSU__USE__M_AXI_GP1 {0} CONFIG.PSU__USE__M_AXI_GP2 {0} \
                           CONFIG.PSU__USE__IRQ0 {1} CONFIG.PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100}] $p
  set clk  [get_bd_pins ps/pl_clk0]
  set rstn [get_bd_pins ps/pl_resetn0]
  set maxi [get_bd_intf_pins ps/M_AXI_HPM0_FPD]
  connect_bd_net $clk [get_bd_pins ps/maxihpm0_fpd_aclk]
  set base 0xA0000000
}

set acc [create_bd_cell -type module -reference vcc_axil_top vcc]
set_property -dict [list CONFIG.HASH_AW $HASH_AW CONFIG.CELL_AW $CELL_AW CONFIG.MAXN_AW $MAXN_AW CONFIG.RAM_STYLE $RAM_STYLE] $acc
set rst [create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst100]
set ic  [create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect sc]
set_property CONFIG.NUM_SI 1 $ic
connect_bd_net $clk [get_bd_pins rst100/slowest_sync_clk] [get_bd_pins sc/aclk] [get_bd_pins vcc/aclk]
if {$ps == 7} { connect_bd_net $clk [get_bd_pins ps/M_AXI_GP0_ACLK] }
connect_bd_net $rstn [get_bd_pins rst100/ext_reset_in]
connect_bd_net [get_bd_pins rst100/peripheral_aresetn] [get_bd_pins sc/aresetn] [get_bd_pins vcc/aresetn]
connect_bd_intf_net $maxi [get_bd_intf_pins sc/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins sc/M00_AXI] [get_bd_intf_pins vcc/s_axi]
# points arrive over AXI-Lite (PT_WORD) in this minimal design; tie the stream input off.
# To add AXI DMA: connect axi_dma/M_AXIS_MM2S (128-bit) -> vcc/s_axis; one point (x,y,z,pad) per beat.
set c0 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant zero32]
set_property -dict [list CONFIG.CONST_WIDTH {128} CONFIG.CONST_VAL {0}] $c0
set c1 [create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant zero1]
set_property -dict [list CONFIG.CONST_WIDTH {1} CONFIG.CONST_VAL {0}] $c1
connect_bd_net [get_bd_pins zero32/dout] [get_bd_pins vcc/s_axis_tdata]
connect_bd_net [get_bd_pins zero1/dout]  [get_bd_pins vcc/s_axis_tvalid]
if {$ps == 7} { connect_bd_net [get_bd_pins vcc/irq] [get_bd_pins ps/IRQ_F2P] } else { connect_bd_net [get_bd_pins vcc/irq] [get_bd_pins ps/pl_ps_irq0] }

assign_bd_address
set seg [lindex [get_bd_addr_segs -of_objects [get_bd_addr_spaces -of_objects $maxi] -filter {NAME =~ *vcc*}] 0]
if {$seg eq ""} { set seg [lindex [get_bd_addr_segs -excluded false] end] }
catch { set_property offset $base $seg; set_property range 16M $seg }
validate_bd_design
save_bd_design
make_wrapper -files [get_files vcc_bd.bd] -top
add_files -norecurse [glob $out/proj/vcc_zynq.gen/sources_1/bd/vcc_bd/hdl/vcc_bd_wrapper.v]
set_property top vcc_bd_wrapper [current_fileset]
update_compile_order -fileset sources_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
open_run impl_1
report_utilization -file $out/util_impl.rpt
report_timing_summary -file $out/timing.rpt
write_hw_platform -fixed -include_bit -force $out/vcc_zynq.xsa
file copy -force [glob $out/proj/vcc_zynq.runs/impl_1/*.bit] $out/vcc_zynq.bit
catch { file copy -force [glob $out/proj/vcc_zynq.gen/sources_1/bd/vcc_bd/hw_handoff/vcc_bd.hwh] $out/vcc_zynq.hwh }
puts "Done: $out/vcc_zynq.xsa  (accelerator at $base)"
