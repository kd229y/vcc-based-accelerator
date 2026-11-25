# hw/ddr/vivado/build_ddr.tcl -- non-project build of the DDR test design (docs/DDR_PORT.md section 6).
# usage: vivado -mode batch -nojournal -log <out>/vivado.log -source build_ddr.tcl -tclargs <out> [ip|synth|all]
#   ip    : generate the MIG IP (mig_ddr2 from mig_ddr2.prj) only
#   synth : + synthesize ddr_uart_top
#   all   : + place, route, timing reports, bitstream (default)
# Last line: DDRBUILD,<stage>,wns=..,tns=..,whs=..,ths=.. or DDRBUILD_FAIL,<reason>.
set here [file dirname [file normalize [info script]]]
set root [file normalize $here/../../..]
set out  [file normalize [lindex $argv 0]]
set mode [expr {[llength $argv] > 1 ? [lindex $argv 1] : "all"}]
set part xc7a100tcsg324-1
file mkdir $out
set_param general.maxThreads 8

create_project -in_memory -part $part
set_property target_language Verilog [current_project]

# ---- MIG 7 Series 4.2, settings from the SoC (mig_ddr2.prj = SoC mig_a.prj with ModuleName mig_ddr2) ----
set ipdir $out/ip
file delete -force $ipdir
file mkdir $ipdir
create_ip -name mig_7series -vendor xilinx.com -library ip -version 4.2 -module_name mig_ddr2 -dir $ipdir
file copy -force $here/mig_ddr2.prj $ipdir/mig_ddr2/mig_ddr2.prj
set_property -dict [list CONFIG.XML_INPUT_FILE {mig_ddr2.prj} CONFIG.RESET_BOARD_INTERFACE {Custom} \
    CONFIG.MIG_DONT_TOUCH_PARAM {Custom} CONFIG.BOARD_MIG_PARAM {Custom}] [get_ips mig_ddr2]
generate_target {instantiation_template synthesis simulation} [get_ips mig_ddr2]
if {$mode eq "ip"} { puts "DDRBUILD,ip"; exit 0 }
synth_ip [get_ips mig_ddr2]

# ---- our RTL ----
read_verilog [list $root/hw/rtl/uart_bridge.v \
    $root/hw/ddr/rtl/ddr_fifo.v $root/hw/ddr/rtl/ddr_port.v $root/hw/ddr/rtl/ddr_test.v $root/hw/ddr/rtl/ddr_uart_top.v]
read_xdc $here/ddr_top.xdc
if {[catch {synth_design -top ddr_uart_top -part $part} msg]} { puts "DDRBUILD_FAIL,synth: $msg"; exit 1 }
report_utilization -file $out/util_synth.rpt
report_utilization -hierarchical -file $out/util_synth_hier.rpt
report_timing_summary -file $out/timing_synth.rpt
if {$mode eq "synth"} {
    set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
    puts "DDRBUILD,synth,wns=$wns"
    exit 0
}

opt_design
place_design
phys_opt_design
route_design
if {[get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]] < 0} { phys_opt_design -directive AggressiveExplore }
report_utilization -file $out/util_impl.rpt
report_utilization -hierarchical -file $out/util_impl_hier.rpt
report_timing_summary -max_paths 10 -file $out/timing.rpt
report_timing -max_paths 40 -nworst 1 -unique_pins -sort_by group -file $out/timing_paths.rpt
report_clocks -file $out/clocks.rpt
report_clock_interaction -file $out/clock_interaction.rpt
report_drc -file $out/drc.rpt
write_checkpoint -force $out/routed.dcp

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -hold]]
set tns [expr {[llength [get_timing_paths -quiet -setup -slack_lesser_than 0 -max_paths 1000]]}]
set ths [expr {[llength [get_timing_paths -quiet -hold -slack_lesser_than 0 -max_paths 1000]]}]
if {$wns < 0 || $whs < 0} { puts "DDRBUILD_FAIL,timing,wns=$wns,whs=$whs (no bitstream)"; exit 1 }
if {[catch {write_bitstream -force $out/ddr_uart_top.bit} msg]} { puts "DDRBUILD_FAIL,bitstream: $msg"; exit 1 }
puts "DDRBUILD,all,wns=$wns,whs=$whs,setup_fail_paths=$tns,hold_fail_paths=$ths,bit=$out/ddr_uart_top.bit"
