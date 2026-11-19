#!/bin/bash
# Build the golden tools (link PLE-CSE's C model) and write hw/vectors_v0/p<i> for the given problems.
#   hw/golden_v0/make_vectors_v0.sh 0 100 200 ...
set -e
cd "$(dirname "$0")/../.."
P=sw_rrtc
B=$PWD/build/sys_v0
mkdir -p $B
make -s -C hw/ple-cse BUILD=$B $B/libfxref.a
CF="-O2 -std=gnu11 -ffp-contract=off -fno-math-errno -I$P/src -I$P/fx"
gcc $CF -o hw/golden_v0/gen_env_v0 hw/golden_v0/gen_env_v0.c $P/fx/fx_env.c $P/fx/fx_pose.c $P/fx/fx_fk_gen.c \
    $P/src/env.c $P/src/cc.c $P/src/nn.c $P/src/rrtc.c $P/src/bench.c $P/gen/panda_fk.c $P/gen/problems_panda.c -lm
gcc $CF -o hw/golden_v0/gen_plan_v0 hw/golden_v0/gen_plan_v0.c $B/libfxref.a -lm
s=0
for p in "$@"; do
  mkdir -p hw/vectors_v0/p$p
  hw/golden_v0/gen_env_v0 prob $p hw/vectors_v0/p$p
  hw/golden_v0/gen_plan_v0 $p $s 3000 hw/vectors_v0/p$p/plan_s$s.txt
  s=$((s+1))
done
