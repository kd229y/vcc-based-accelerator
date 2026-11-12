#!/bin/bash
# usage: sim/run_plan.sh <cloud scene> <plan dir>
set -e
cd "$(dirname "$0")/.."
mkdir -p sim/out
iverilog -g2005 -o sim/out/tb_plan_$2 tb/tb_plan.v rtl/vcc_core.v rtl/rrt_engine.v rtl/arith_units.v \
  rtl/build_engine.v rtl/query_engine.v rtl/fk_engine.v rtl/stage_a.v rtl/ram_sdp.v rtl/panda_consts.v 2>&1 \
  | grep -iv "warning\|sensitive\|inherit" || true
vvp -n sim/out/tb_plan_$2 +cloud=vectors/$1 +plan=vectors/$2 | grep -v WARNING
