# vivado -mode batch -source program.tcl -tclargs <bitfile> [hw_server url, default localhost:3121]
# (if Windows reserved the port range containing 3121, run hw_server -s TCP::3400 and pass localhost:3400)
# Retries (flaky USB connector): reconnects to hw_server / target up to 8 times.
set bit [lindex $argv 0]
set url [expr {[llength $argv] > 1 ? [lindex $argv 1] : "localhost:3121"}]
open_hw_manager
set ok 0
for {set i 1} {$i <= 8 && !$ok} {incr i} {
  if {[catch {
    catch {disconnect_hw_server}
    connect_hw_server -url $url
    open_hw_target
    set dev [lindex [get_hw_devices -filter {NAME =~ xc7*}] 0]
    current_hw_device $dev
    set_property PROGRAM.FILE $bit $dev
    program_hw_devices $dev
    refresh_hw_device $dev
    if {[get_property REGISTER.IR.BIT5_DONE $dev] != 1} { error "DONE pin not high" }
  } msg]} {
    puts "PROGRAM attempt $i failed: $msg"
    catch {close_hw_target}
    after 3000
  } else {
    set ok 1
  }
}
close_hw_manager
if {!$ok} { puts "PROGRAM FAILED"; exit 1 }
puts "PROGRAM OK"
