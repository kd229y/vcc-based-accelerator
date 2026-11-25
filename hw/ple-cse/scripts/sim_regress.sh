#!/bin/bash
# hw/ple-cse/scripts/sim_regress.sh (make regress) -- run the PLE-CSE and preprocessing regressions, one line per target.
# usage: bash hw/ple-cse/scripts/sim_regress.sh                  the default list below
#        bash hw/ple-cse/scripts/sim_regress.sh "make foo" ...   only these commands (run from the repo root)
# Targets run one at a time (they share build/accel/libfxref.a and the golden traces). Full logs: build/regress/NN.log.
# Exit status is the number of failed targets. TARGET_TIMEOUT=<seconds> per target (default 3600); timeout = FAIL.
cd "$(dirname "$0")/../../.." || exit 1
OUT=${REGRESS_OUT:-build/regress}
TARGET_TIMEOUT=${TARGET_TIMEOUT:-3600}
mkdir -p "$OUT" && rm -f "$OUT"/[0-9][0-9].log

if [ $# -gt 0 ]; then
    TARGETS=("$@")
else
    # "make trace" must come before the CSE targets (they read build/sw_rrtc_host/cse_trace).
    TARGETS=(
        "make lint"
        "make -C hw/ple-cse test-ple-unit"
        "make -C hw/ple-cse test-ple-core"
        "make -C hw/ple-cse test-ple-top"
        "make -C hw/ple-cse test-ple-drv"
        "make -C hw/ple-cse test-ple-cse NCU=2"
        "make trace"
        "make -C hw/ple-cse test-checker"
        "make -C hw/ple-cse test-env-mem"
        "make -C hw/ple-cse test-cse"
        "make -C hw/ddr test"
        "make sim"
    )
fi

# Lines that carry the result or key numbers.
KEY_RE='^TB|^DRV|errors=|killed:|SURVIVED|DIFF|control \('
# Output that means "ran zero tests"; it is forced to FAIL.
EMPTY_RE='runs=0 |queries=0 '
# Output that means failure even when make returns 0 (test-env-mem, test-checker and make sim only print their result).
FAIL_RE=' DIFF$|^[a-z0-9_]+ +FAIL |SURVIVED|bug=0 .*checker_errors=[1-9]|bug=[1-9] .*checker_errors=0$|nonzero_reads=[1-9]|port_errors=[1-9]|absent buffer is visible'

n=0; failed=0
for cmd in "${TARGETS[@]}"; do
    n=$((n + 1))
    log=$(printf '%s/%02d.log' "$OUT" "$n")
    start=$SECONDS
    # shellcheck disable=SC2086
    timeout -k 30 "$TARGET_TIMEOUT" $cmd > "$log" 2>&1 < /dev/null
    rc=$?
    [ "$rc" -eq 0 ] && grep -Eq "$EMPTY_RE" "$log" && rc=99
    [ "$rc" -eq 0 ] && grep -Eq "$FAIL_RE" "$log" && rc=98
    status=PASS
    [ "$rc" -ne 0 ] && { status=FAIL; failed=$((failed + 1)); }
    key=$(grep -E "$KEY_RE" "$log" | tail -n 2 | tr '\n' ' ' | cut -c1-160)
    printf '%s rc=%d %4ds %-36s %s\n' "$status" "$rc" "$((SECONDS - start))" "$cmd" "$key"
done
echo "== $((n - failed))/$n passed, logs in $OUT"
[ "$failed" -gt 125 ] && failed=125
exit "$failed"
