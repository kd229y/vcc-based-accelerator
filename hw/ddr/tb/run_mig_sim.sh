#!/bin/bash
# hw/ddr/tb/run_mig_sim.sh -- L2: tb_ddr_mig in xsim (real MIG RTL, FAST calibration, Micron ddr2_model).
# usage: run_mig_sim.sh [out_dir]   (default build/ddr/l2). Generates the MIG IP there on the first run.
# Last line: TB_DDR_MIG errors=N; exit status 0 only when errors=0.
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../.." && pwd)
OUT=$(mkdir -p "${1:-$ROOT/build/ddr/l2}" && cd "${1:-$ROOT/build/ddr/l2}" && pwd)
VIVADO_BIN=${VIVADO_BIN:-/tools/Xilinx/Vivado/2024.1/bin}
XV=$(cd "$VIVADO_BIN/.." && pwd)
MODEL=$XV/data/ip/xilinx/mig_7series_v4_2/data/dlib/7series/ddr2_sdram/sim
MIG=$OUT/ip/mig_ddr2/mig_ddr2/user_design/rtl

cd "$OUT" || exit 1   # Vivado and xsim write side files into the working directory
if [ ! -f "$MIG/mig_ddr2.v" ]; then
    echo "L2: generating the MIG IP in $OUT/ip"
    timeout -k 30 1200 "$VIVADO_BIN/vivado" -mode batch -nojournal -log "$OUT/ipgen.log" \
        -source "$ROOT/hw/ddr/vivado/build_ddr.tcl" -tclargs "$OUT" ip > "$OUT/ipgen.out" 2>&1 || { echo "L2 FAIL ipgen"; exit 1; }
fi

# simulation core: mig_ddr2_mig_sim.v (SIM_BYPASS_INIT_CAL = FAST) replaces mig_ddr2_mig.v
mapfile -t MIG_SRC < <(find "$MIG" -name '*.v' ! -name 'mig_ddr2_mig.v' | sort)
{
    "$VIVADO_BIN/xvlog" --relax -d x1Gb -d sg25E -d x16 -i "$MODEL" "$MODEL/ddr2_model.v" &&
    "$VIVADO_BIN/xvlog" --relax "$XV/data/verilog/src/glbl.v" "${MIG_SRC[@]}" \
        "$ROOT/hw/rtl/uart_bridge.v" "$ROOT/hw/ddr/rtl/ddr_fifo.v" "$ROOT/hw/ddr/rtl/ddr_port.v" \
        "$ROOT/hw/ddr/rtl/ddr_test.v" "$ROOT/hw/ddr/rtl/ddr_uart_top.v" &&
    "$VIVADO_BIN/xvlog" -sv "$HERE/tb_ddr_mig.sv" &&
    "$VIVADO_BIN/xelab" tb_ddr_mig glbl -L unisims_ver -L secureip -s tb_ddr_mig_snap --relax
} > compile.log 2>&1 || { grep -m5 -E "ERROR" compile.log; echo "L2 FAIL compile (see $OUT/compile.log)"; exit 1; }

timeout -k 30 "${SIM_TIMEOUT:-7200}" "$VIVADO_BIN/xsim" tb_ddr_mig_snap -R > sim.log 2>&1
grep -E "^(ok|ERROR|L2|MODE|TB_DDR_MIG)" sim.log
grep -q "^TB_DDR_MIG errors=0$" sim.log
