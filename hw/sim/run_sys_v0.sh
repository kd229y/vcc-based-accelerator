#!/bin/bash
# System sim: build_engine_v0 + PLE-CSE ple_cse_top (real CSE) vs golden fx_ref_solve.
# usage: hw/sim/run_sys_v0.sh [N_CU] [problems...]      (needs Verilator >= 5, gcc; vectors from `make vectors-v0`)
set -e
cd "$(dirname "$0")/../.."
NCU=${1:-1}; shift || true
PROBS=${@:-0}
export PATH="$HOME/.conda-envs/verilator/bin:$PATH"
A=ext/PLE-CSE/accel
grep -q "3 \* AOFF_W" $A/rtl/common/ple_cse_pkg.sv || { echo "ext/PLE-CSE is not patched: run make ple-patch"; exit 1; }
B=$PWD/build/sys_v0
mkdir -p $B
make -s -C $A BUILD=$B $B/libfxref.a
PKG="$A/rtl/common/ple_cse_pkg.sv $A/rtl/ple/ple_pkg.sv $A/rtl/ple/ple_robot_pkg.sv $A/rtl/cse/cse_cu_pkg.sv"
PLE=$(ls $A/rtl/ple/*.sv | grep -v _pkg.sv)
CSE=$(ls $A/rtl/cse/*.sv | grep -v _pkg.sv)
OBJ=$B/ncu$NCU
if [ ! -x $OBJ/Vtb_sys_v0 ] || [ -n "$(find hw/rtl hw/tb $A/rtl -newer $OBJ/Vtb_sys_v0 -name '*.*v' -o -newer $OBJ/Vtb_sys_v0 -name '*.c' | head -1)" ]; then
  verilator --binary --timing --assert -Wno-fatal -Wno-lint -Wno-style -I$A/rtl/common -Mdir $OBJ -GN_CU=$NCU \
    --top-module tb_sys_v0 $PKG $PLE $CSE $A/rtl/ple_cse_top.sv \
    hw/rtl/ram_sdp.v hw/rtl/stage_a_v0.v hw/rtl/build_engine_v0.v hw/tb/tb_sys_v0.sv hw/tb/sys_dpi.c \
    -CFLAGS "-I$PWD/ext/PLE-CSE/sw_rrtc/src -I$PWD/ext/PLE-CSE/sw_rrtc/fx" \
    -LDFLAGS $B/libfxref.a -LDFLAGS -lm -o Vtb_sys_v0 > $OBJ.build.log 2>&1 || { tail -30 $OBJ.build.log; exit 1; }
fi
ARGS=""; i=0; for p in $PROBS; do case $p in +*) ARGS="$ARGS $p";; *) ARGS="$ARGS +p$i=$p"; i=$((i+1));; esac; done
$OBJ/Vtb_sys_v0 +vec=hw/vectors_v0 $ARGS | grep -E "TB_SYS|ERROR|fatal"
