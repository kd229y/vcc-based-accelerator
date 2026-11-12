# Non-project batch flow for UART boards (Arty A7-100T / Nexys A7-100T / any 7-series with USB-UART).
# Windows:  vivado -mode batch -source build_uart.tcl -tclargs arty
#           vivado -mode batch -source build_uart.tcl -tclargs nexys
#           vivado -mode batch -source build_uart.tcl -tclargs custom <part> <xdc> [HASH_AW CELL_AW MAXN_AW]
set here [file dirname [file normalize [info script]]]
set rtl  [file normalize $here/../rtl]
set board [lindex $argv 0]
set HASH_AW 14; set CELL_AW 12; set MAXN_AW 9
switch $board {
  arty   { set part xc7a100tcsg324-1; set xdc $here/xdc/arty_a7_100t.xdc }
  nexys  { set part xc7a100tcsg324-1; set xdc $here/xdc/nexys_a7_100t.xdc }
  nexys50 { set part xc7a50ticsg324-1L; set xdc $here/xdc/nexys_a7_100t.xdc; set HASH_AW 12; set CELL_AW 10; set MAXN_AW 8 }
  basys3 { set part xc7a35tcpg236-1;  set xdc $here/xdc/basys3.xdc; set HASH_AW 11; set CELL_AW 9; set MAXN_AW 7 }
  custom { set part [lindex $argv 1]; set xdc [file normalize [lindex $argv 2]]
           if {[llength $argv] > 3} { set HASH_AW [lindex $argv 3]; set CELL_AW [lindex $argv 4]; set MAXN_AW [lindex $argv 5] } }
  default { puts "usage: -tclargs arty|nexys|nexys50|basys3|custom <part> <xdc> [HASH_AW CELL_AW MAXN_AW]"; exit 1 }
}
set out $here/out_$board
file mkdir $out
foreach f {ram_sdp.v stage_a.v build_engine.v query_engine.v panda_consts.v fk_engine.v arith_units.v rrt_engine.v vcc_core.v uart_bridge.v vcc_top.v} {
  read_verilog $rtl/$f
}
read_xdc $xdc
synth_design -top vcc_uart_top -part $part -generic HASH_AW=$HASH_AW -generic CELL_AW=$CELL_AW -generic MAXN_AW=$MAXN_AW
report_utilization -file $out/util_synth.rpt
opt_design -directive Explore
place_design -directive Explore
phys_opt_design -directive AggressiveExplore
route_design -directive Explore
if {[get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]] < 0} { phys_opt_design -directive AggressiveExplore }
report_utilization -file $out/util_impl.rpt
report_timing_summary -file $out/timing.rpt
report_timing -max_paths 40 -nworst 1 -unique_pins -sort_by group -file $out/timing_paths.rpt
write_checkpoint -force $out/routed.dcp
write_bitstream -force $out/vcc_uart_top.bit
puts "Bitstream: $out/vcc_uart_top.bit"
