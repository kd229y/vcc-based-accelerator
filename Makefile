# VCC Morton accelerator: entry point (make help). hw/ keeps its own scripts.
SHELL := /bin/bash
SCENE ?= s1_shelf
HASH  ?= 15

.PHONY: help vectors consts sim sim-build sim-fk sim-plan sim-uart clean

help: ## List the targets
	@grep -E '^[a-z-]+:.*## ' $(MAKEFILE_LIST) | sed 's/:.*## /\t/'

consts: ## Regenerate rtl/panda_consts.v from the package's panda_generated.hh
	python3 hw/golden/gen_consts.py

vectors: ## Rebuild golden vectors from soc_model/core.cpp (needs g++, numpy)
	python3 hw/golden/make_vectors.py

sim: ## Full regression (18 testbenches, iverilog)
	hw/sim/run_all.sh

sim-build: ## One build+query scene: make sim-build SCENE=s1_shelf HASH=15
	hw/sim/run_build.sh $(SCENE) $(HASH) +fullrate

sim-fk: ## FK against the package's 128 golden cases
	mkdir -p hw/sim/out
	iverilog -g2005 -o hw/sim/out/r_fk hw/tb/tb_fk.v hw/rtl/fk_engine.v hw/rtl/panda_consts.v
	cd hw && vvp -n sim/out/r_fk +dir=vectors/fk_pkg +n=128 | grep -v WARNING

sim-plan: ## RRT-Connect plan through the register bus (detour problem)
	hw/sim/run_plan.sh s0_pkgvec p0_detour

clean: ## Remove simulation outputs
	rm -rf hw/sim/out build
