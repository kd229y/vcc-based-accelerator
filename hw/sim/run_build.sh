#!/bin/bash
# usage: sim/run_build.sh <scene> [HASH_AW]
set -e
cd "$(dirname "$0")/.."
SCENE=$1; HAW=${2:-15}
mkdir -p sim/out
iverilog -g2005 -Wall -o sim/out/tb_bq_${SCENE}_$HAW -P tb_build_query.HASH_AW=$HAW \
  tb/tb_build_query.v rtl/build_engine.v rtl/query_engine.v rtl/stage_a.v rtl/ram_sdp.v 2>&1 | grep -v "timescale\|inherit" || true
vvp -n sim/out/tb_bq_${SCENE}_$HAW +dir=vectors/$SCENE $3
