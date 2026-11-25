# hw/ddr -- DDR2 port (docs/DDR_PORT.md): L1 Verilator tests, mutants, L2 MIG sim, L3 board build, programming.
ROOT    := $(abspath ../..)
BUILD   ?= $(ROOT)/build/ddr
VIVADO  ?= /tools/Xilinx/Vivado/2024.1/bin/vivado
RTL     := rtl/ddr_fifo.v rtl/ddr_port.v rtl/ddr_test.v
BIT     ?= $(BUILD)/impl/ddr_uart_top.bit

.PHONY: help lint test test-l1 test-mutants sim-mig build program

help: ## List the targets
	@grep -E '^[a-z0-9-]+:.*## ' $(MAKEFILE_LIST) | sed 's/:.*## /\t/'

lint: ## Verilator -Wall on ddr_fifo, ddr_port (3 parameter sets), ddr_test
	verilator --lint-only -Wall rtl/ddr_fifo.v --top-module ddr_fifo
	verilator --lint-only -Wall rtl/ddr_fifo.v rtl/ddr_port.v --top-module ddr_port
	verilator --lint-only -Wall -GAXI_AW=27 -GBURST_MAX=256 -GRD_DEPTH=256 -GWR_DEPTH=256 rtl/ddr_fifo.v rtl/ddr_port.v --top-module ddr_port
	verilator --lint-only -Wall -GBURST_MAX=1 -GRD_DEPTH=2 -GWR_DEPTH=1 -GRD_OUTSTANDING=1 rtl/ddr_fifo.v rtl/ddr_port.v --top-module ddr_port
	verilator --lint-only -Wall rtl/ddr_test.v --top-module ddr_test

test-l1: ## L1: tb_ddr_port (5 parameter sets x 4 latency sets) + tb_ddr_test (MODE 0-4, 3 latency sets)
	bash tb/run_l1.sh $(BUILD)/l1

test-mutants: ## L1 mutants (~4 min): every planted bug must make L1 fail
	python3 tb/mutants.py $(BUILD)/mutants

test: lint test-l1 ## lint + L1 (in make regress); run test-mutants after changing hw/ddr/rtl

sim-mig: ## L2: real MIG RTL (FAST calibration) + Micron DDR2 model in xsim, whole board top over UART (slow)
	bash tb/run_mig_sim.sh $(BUILD)/l2

build: ## L3: MIG IP + synth + impl + bitstream (non-project); reports in $(BUILD)/impl
	mkdir -p $(BUILD)/impl
	cd $(BUILD)/impl && $(VIVADO) -mode batch -nojournal -log vivado.log -source $(CURDIR)/vivado/build_ddr.tcl -tclargs $(BUILD)/impl all
	@grep -E '^DDRBUILD' $(BUILD)/impl/vivado.log | tail -n 1

program: ## Program the board (WSL2: hw_server on localhost:3121); BIT=<file>
	mkdir -p $(BUILD)/board
	cd $(BUILD)/board && $(VIVADO) -mode batch -nojournal -log program.log -source $(CURDIR)/vivado/program.tcl -tclargs $(abspath $(BIT))
