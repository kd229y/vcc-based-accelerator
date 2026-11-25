# hw/ddr/vivado/program.tcl -- program the Nexys A7-100T over hw_server (WSL2: localhost:3121), as SoC Platform/hw/program.tcl.
# usage: vivado -mode batch -nojournal -log <out>/program.log -source program.tcl -tclargs <bitstream>
if { $argc != 1 } {
    puts "Usage: vivado -mode batch -source program.tcl -tclargs <bitstream_file>"
    exit 1
}
set bit_file [lindex $argv 0]

open_hw_manager
connect_hw_server -url localhost:3121
open_hw_target

set device [get_hw_devices xc7a100t_0]
current_hw_device $device
set_property PROGRAM.FILE $bit_file $device
program_hw_devices $device
puts "PROGRAM_DONE $bit_file"

close_hw_target
disconnect_hw_server
close_hw_manager
