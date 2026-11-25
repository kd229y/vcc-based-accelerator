# ddr_uart_top on Nexys A7-100T: clock, button, UART, LEDs. DDR2 pins and their timing come from the MIG xdc.
set_property -dict {PACKAGE_PIN E3  IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 10.000 -name sys_clk [get_ports clk]
set_property -dict {PACKAGE_PIN N17 IOSTANDARD LVCMOS33} [get_ports rst_btn]
set_property -dict {PACKAGE_PIN C4  IOSTANDARD LVCMOS33} [get_ports uart_rx]
set_property -dict {PACKAGE_PIN D4  IOSTANDARD LVCMOS33} [get_ports uart_tx]
set_property -dict {PACKAGE_PIN H17 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN K15 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN J13 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN N14 IOSTANDARD LVCMOS33} [get_ports {led[3]}]

# asynchronous board I/O (uart_rx has a 2-flop synchronizer; the MIG synchronizes sys_rst)
set_false_path -from [get_ports {rst_btn uart_rx}]
set_false_path -to   [get_ports {uart_tx led[*]}]

set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
