#!/bin/bash
# hw/ddr/tb/run_l1.sh -- L1: tb_ddr_port over parameter sets x latency sets (docs/DDR_PORT.md section 7).
# usage: run_l1.sh [out_dir] ; env RTL_DIR=<dir with the hw/ddr/rtl files> (mutants), CFGS, LATS, SEEDS, TEST_LATS.
# Then tb_ddr_test (ddr_test MODE 0-4) at TEST_LATS.
# Prints one TB_DDR_PORT line per run; exit status = number of failed runs (errors, proto errors, timeout, no result).
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${1:-$HERE/../../../build/ddr/l1}
RTL_DIR=${RTL_DIR:-$HERE/../rtl}
# BURST_MAX,RD_OUTSTANDING,RD_DEPTH,WR_DEPTH
CFGS=${CFGS:-"16,4,32,16 1,1,2,1 256,4,256,256 16,1,16,16 4,8,64,4"}
LATS=${LATS:-"min def max slow"}
SEEDS=${SEEDS:-"1"}
TEST_LATS=${TEST_LATS:-"min def max"}
mkdir -p "$OUT"
fail=0
for c in $CFGS; do
    IFS=, read -r bm ro rd wd <<< "$c"
    dir="$OUT/c${bm}_${ro}_${rd}_${wd}"
    mkdir -p "$dir"
    if ! verilator --binary --timing --assert -Wno-fatal -Wno-lint -Wno-style --top-module tb_ddr_port \
            -GBURST_MAX="$bm" -GRD_OUTSTANDING="$ro" -GRD_DEPTH="$rd" -GWR_DEPTH="$wd" \
            "$HERE/ddr_tb_pkg.sv" "$HERE/axi_mem_model.sv" "$HERE/tb_ddr_port.sv" \
            "$RTL_DIR/ddr_fifo.v" "$RTL_DIR/ddr_port.v" -Mdir "$dir/obj" -o Vtb > "$dir/build.log" 2>&1; then
        echo "FAIL build cfg=$c (see $dir/build.log)"; fail=$((fail + 1)); continue
    fi
    for l in $LATS; do
        case $l in
            min) pa="+gap=0 +lat=1 +blat=1 +ugap=0" ;;
            def) pa="+gap=30 +lat=20 +blat=8 +ugap=30" ;;
            max) pa="+gap=80 +lat=100 +blat=50 +ugap=80" ;;
            slow) pa="+gap=0 +lat=20 +blat=8 +ugap=90" ;;   # fast memory, slow user: fills the read FIFO
        esac
        for s in $SEEDS; do
            log="$dir/$l.s$s.log"
            # shellcheck disable=SC2086
            timeout -k 5 300 "$dir/obj/Vtb" +seed="$s" $pa > "$log" 2>&1
            line=$(grep '^TB_DDR_PORT' "$log" | tail -n 1)
            if [ -z "$line" ]; then line="TB_DDR_PORT cfg=$c lat=$l seed=$s NO RESULT (rc=$?)"; fi
            echo "$line"
            if ! [[ "$line" =~ errors=0\ proto_errors=0$ ]]; then fail=$((fail + 1)); grep -m3 ERROR "$log"; fi
        done
    done
done
# ddr_test (MODE 0-4, window, planted errors) on ddr_port + the memory model
dir="$OUT/ddr_test"
mkdir -p "$dir"
if ! verilator --binary --timing --assert -Wno-fatal -Wno-lint -Wno-style --top-module tb_ddr_test \
        "$HERE/ddr_tb_pkg.sv" "$HERE/axi_mem_model.sv" "$HERE/tb_ddr_test.sv" \
        "$RTL_DIR/ddr_fifo.v" "$RTL_DIR/ddr_port.v" "$RTL_DIR/ddr_test.v" -Mdir "$dir/obj" -o Vtb > "$dir/build.log" 2>&1; then
    echo "FAIL build tb_ddr_test (see $dir/build.log)"; fail=$((fail + 1))
else
    for l in $TEST_LATS; do
        case $l in
            min) pa="+gap=0 +lat=1 +blat=1" ;;
            def) pa="+gap=30 +lat=20 +blat=8" ;;
            max) pa="+gap=80 +lat=100 +blat=50" ;;
        esac
        log="$dir/$l.log"
        # shellcheck disable=SC2086
        timeout -k 5 600 "$dir/obj/Vtb" $pa > "$log" 2>&1
        line=$(grep '^TB_DDR_TEST' "$log" | tail -n 1)
        [ -z "$line" ] && line="TB_DDR_TEST lat=$l NO RESULT"
        echo "$line $(grep -m1 '^MODE4' "$log")"
        if ! [[ "$line" =~ errors=0$ ]]; then fail=$((fail + 1)); grep -m3 ERROR "$log"; fi
    done
fi
echo "L1 failed_runs=$fail"
[ "$fail" -gt 125 ] && fail=125
exit "$fail"
