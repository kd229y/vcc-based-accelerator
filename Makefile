# VCC Morton accelerator: entry point (make help). hw/ keeps its own scripts.
SHELL := /bin/bash
SCENE ?= s1_shelf
HASH  ?= 15

.PHONY: help vectors consts sim sim-build sim-fk sim-plan sim-uart clean ple-patch vectors-v0 sim-v0 sim-sys sim-sys-all

help: ## List the targets
	@grep -E '^[a-z0-9-]+:.*## ' $(MAKEFILE_LIST) | sed 's/:.*## /\t/'

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

# ---- integration with PLE-CSE (ext/PLE-CSE submodule) ----
PLE   := ext/PLE-CSE
PATCH := hw/patches/ple-cse-pts-offset.patch
V0_PROBS ?= 0 100 200 300 400 500 600 542 482

ple-patch: ## Apply our CSE change (PTS as 11-bit in-cell offsets) to ext/PLE-CSE; needed by the integrated design
	@if git -C $(PLE) apply --reverse --check ../../$(PATCH) 2>/dev/null; then echo "ple-patch: already applied"; \
	else git -C $(PLE) apply ../../$(PATCH) && echo "ple-patch: applied"; fi

vectors-v0: ## Format-v0 golden vectors from PLE-CSE's fx_env_build / fx_ref_solve (hw/vectors_v0)
	hw/golden_v0/make_vectors_v0.sh $(V0_PROBS)

sim-v0: ## build_engine_v0 vs fx_env_build on every vector set (iverilog)
	mkdir -p hw/sim/out
	iverilog -g2005 -o hw/sim/out/tb_bv0 hw/tb/tb_build_v0.v hw/rtl/build_engine_v0.v hw/rtl/stage_a_v0.v hw/rtl/ram_sdp.v
	cd hw && for d in vectors_v0/*/; do vvp -n sim/out/tb_bv0 +dir=$${d%/} | grep -E "PASS|FAIL"; done

sim-sys: ple-patch ## Our build engine + PLE-CSE (N_CU=1) planning, vs golden fx_ref_solve (Verilator)
	hw/sim/run_sys_v0.sh 1 $(V0_PROBS)

sim-sys-all: ple-patch ## Same, all 700 problems (clouds from the C model through DPI), 7 parallel chunks
	hw/sim/run_sys_v0.sh 1 +first=0 +last=0 > /dev/null
	mkdir -p build/all700
	for c in 0 1 2 3 4 5 6; do build/sys_v0/ncu1/Vtb_sys_v0 +first=$$((c*100)) +last=$$((c*100+99)) > build/all700/chunk$$c.log & done; wait
	grep -h "TB_SYS N_CU\|TB_SYS_AVG" build/all700/chunk*.log

clean: ## Remove simulation outputs
	rm -rf hw/sim/out build
