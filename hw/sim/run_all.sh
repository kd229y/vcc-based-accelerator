#!/bin/bash
# Full regression: every testbench against golden vectors from soc_model/core.cpp.
# Needs iverilog (>= 11). Runs scenes in parallel; prints a PASS/FAIL table.
cd "$(dirname "$0")/.."
mkdir -p sim/out
RTL="rtl/ram_sdp.v rtl/stage_a.v rtl/build_engine.v rtl/query_engine.v rtl/panda_consts.v rtl/fk_engine.v rtl/arith_units.v rtl/rrt_engine.v rtl/vcc_core.v rtl/uart_bridge.v rtl/vcc_top.v"
cc() { iverilog -g2005 -o "$@" 2>&1 | grep -iv "warning\|sensitive\|inherit" ; }
cc sim/out/r_bq15 -P tb_build_query.HASH_AW=15 tb/tb_build_query.v $RTL
cc sim/out/r_bq10 -P tb_build_query.HASH_AW=10 tb/tb_build_query.v $RTL
cc sim/out/r_bq14 -P tb_build_query.HASH_AW=14 tb/tb_build_query.v $RTL   # FPGA board profile
cc sim/out/r_fk tb/tb_fk.v $RTL
cc sim/out/r_plan tb/tb_plan.v $RTL
cc sim/out/r_uart tb/tb_uart.v $RTL
run() { local name=$1; shift; vvp -n "$@" > sim/out/log_$name.txt 2>&1; }
for s in s0_pkgvec s1_shelf s2_edge s4_inf s5_empty s6_dense; do run $s sim/out/r_bq15 +dir=vectors/$s & done
run s3_cap1024 sim/out/r_bq10 +dir=vectors/s3_cap1024 &
for s in s1_shelf s2_edge s5_empty s6_dense; do run ${s}_h14 sim/out/r_bq14 +dir=vectors/$s & done
run fk_pkg sim/out/r_fk +dir=vectors/fk_pkg +n=128 &
run fk_rand sim/out/r_fk +dir=vectors/fk +n=200 &
run uart sim/out/r_uart &
run p1_free sim/out/r_plan +cloud=vectors/s5_empty +plan=vectors/p1_free &
run p0_detour sim/out/r_plan +cloud=vectors/s0_pkgvec +plan=vectors/p0_detour &
run p2_shelf_s1 sim/out/r_plan +cloud=vectors/s1_shelf +plan=vectors/p2_shelf_s1 &
run p2_shelf_s3 sim/out/r_plan +cloud=vectors/s1_shelf +plan=vectors/p2_shelf_s3 &
wait
echo "================ regression summary ================"
for f in sim/out/log_*.txt; do
  n=$(basename $f .txt); n=${n#log_}
  if grep -q "^PASS" $f; then r=PASS; else r=FAIL; fi
  printf "%-14s %s   %s\n" $n $r "$(grep -E 'build done|queries:|fk:|plan cycles' $f | tr '\n' ' ' | cut -c1-110)"
done
