# Integrated design: build_engine_v0 + PLE-CSE ple_cse_top (N_CU) over UART, Nexys A7-100T, 75 MHz.
#   vivado -mode batch -source build_v0.tcl [-tclargs <N_CU>]
set here [file dirname [file normalize [info script]]]
set rtl  [file normalize $here/../rtl]
set pc   [file normalize $here/../ple-cse/rtl]
set ncu  [expr {[llength $argv] > 0 ? [lindex $argv 0] : 1}]
set out  $here/out_v0_ncu$ncu
set f [open $pc/common/ple_cse_pkg.sv]; set pkg [read $f]; close $f
if {![string match "*3 \\* AOFF_W*" $pkg]} { puts "ERROR: hw/ple-cse is not patched (run: make ple-patch)"; exit 1 }
file mkdir $out
read_verilog -sv [list $pc/common/ple_cse_pkg.sv $pc/ple/ple_pkg.sv $pc/ple/ple_robot_pkg.sv $pc/cse/cse_cu_pkg.sv]
read_verilog -sv [lsearch -all -inline -not -glob [lsort [glob $pc/ple/*.sv]] *_pkg.sv]
read_verilog -sv [lsearch -all -inline -not -glob [lsort [glob $pc/cse/*.sv]] *_pkg.sv]
read_verilog -sv $pc/ple_cse_top.sv
read_verilog [list $rtl/ram_sdp.v $rtl/stage_a_v0.v $rtl/build_engine_v0.v $rtl/uart_bridge.v]
read_verilog -sv $rtl/vcc_v0_top.sv
read_xdc $here/xdc/nexys_a7_100t.xdc
synth_design -top vcc_v0_uart_top -part xc7a100tcsg324-1 -generic N_CU=$ncu -include_dirs $pc/common
report_utilization -hierarchical -hierarchical_depth 3 -file $out/util_synth_hier.rpt
opt_design -directive Explore
place_design -directive Explore
phys_opt_design -directive AggressiveExplore
route_design -directive Explore
if {[get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]] < 0} { phys_opt_design -directive AggressiveExplore }
report_utilization -file $out/util_impl.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file $out/util_impl_hier.rpt
report_timing_summary -file $out/timing.rpt
report_timing -max_paths 40 -nworst 1 -unique_pins -sort_by group -file $out/timing_paths.rpt
write_checkpoint -force $out/routed.dcp
write_bitstream -force $out/vcc_v0_uart_top.bit
puts "Bitstream: $out/vcc_v0_uart_top.bit"
