#!/usr/bin/env python3
"""hw/ddr/tb/mutants.py -- L1 mutants (docs/DDR_PORT.md section 7): each planted bug must make run_l1.sh FAIL.
usage: mutants.py [out_dir]   prints one line per mutant: killed / SURVIVED; exit status = survivors."""
import os, shutil, subprocess, sys

HERE = os.path.dirname(os.path.abspath(__file__))
RTL = os.path.join(HERE, "..", "rtl")
OUT = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "../../../build/ddr/mutants"))

# name: (file, original text, mutated text)
MUTANTS = {
    "wlast_early":   ("ddr_port.v", "assign m_axi_wlast  = (w_beat == lq_head);",
                      "assign m_axi_wlast  = (w_beat + 8'd1 >= lq_head);"),
    "cross_4k":      ("ddr_port.v", "b4k = 9'd256 - {1'b0, a4k};", "b4k = 9'd256;"),
    "ignore_strobe": ("ddr_port.v", "assign m_axi_wstrb  = wf_dout[143:128];", "assign m_axi_wstrb  = 16'hFFFF;"),
    "rd_fifo_short": ("ddr_port.v", "ddr_fifo #(.W(128), .DEPTH(RD_DEPTH)) u_rf", "ddr_fifo #(.W(128), .DEPTH(RD_DEPTH - 1)) u_rf"),
    "rd_credit_p1":  ("ddr_port.v", "(resv + {8'd0, rb} <= RD_ROOM)", "(resv + {8'd0, rb} <= RD_ROOM + 17'd1)"),
    "wr_fin_no_b":   ("ddr_port.v", "(w_owed == {WCW{1'b0}}) && (b_pend == 17'd0) && !m_axi_awvalid",
                      "(w_owed == {WCW{1'b0}}) && !m_axi_awvalid"),
    "w_avail_owed":  ("ddr_port.v", "wire [WCW-1:0]   w_avail = wf_count - w_owed;", "wire [WCW-1:0]   w_avail = wf_count;"),
    # ddr_test (tb_ddr_test)
    "t_no_strobe":   ("ddr_test.v", "(r_mode == 3'd3 && st_r[k])", "(1'b0 && st_r[k])"),
    "t_cmp_low64":   ("ddr_test.v", "wire         c_bad = c_v && (c_got != c_exp);",
                      "wire         c_bad = c_v && (c_got[63:0] != c_exp[63:0]);"),
    "t_err_swap":    ("ddr_test.v", "err_got <= c_got[31:0]; err_exp <= c_exp[31:0];",
                      "err_got <= c_exp[31:0]; err_exp <= c_got[31:0];"),
    "t_lat_addr":    ("ddr_test.v", "rk_addr <= r_base + {l_prod[38:16], 4'd0};", "rk_addr <= r_base;"),
    "t_pat_swap":    ("ddr_test.v", "hw({a, 2'd1}, s, sk), hw({a, 2'd0}, s, sk)}", "hw({a, 2'd0}, s, sk), hw({a, 2'd1}, s, sk)}"),
}

survivors = 0
for name, (fn, old, new) in MUTANTS.items():
    d = os.path.join(OUT, name)
    rtl = os.path.join(d, "rtl")
    os.makedirs(rtl, exist_ok=True)
    for f in ("ddr_fifo.v", "ddr_port.v", "ddr_test.v"):
        shutil.copy(os.path.join(RTL, f), rtl)
    src = open(os.path.join(RTL, fn)).read()
    if src.count(old) != 1:
        print(f"{name:14s} ERROR: original text found {src.count(old)} times")
        survivors += 1
        continue
    open(os.path.join(rtl, fn), "w").write(src.replace(old, new))
    env = dict(os.environ, RTL_DIR=rtl, CFGS="16,4,32,16 4,8,64,4", LATS="min def max slow")
    r = subprocess.run(["bash", os.path.join(HERE, "run_l1.sh"), os.path.join(d, "l1")], env=env,
                       capture_output=True, text=True)
    first = next((l for l in r.stdout.splitlines() if "ERROR" in l), "")
    if r.returncode != 0:
        print(f"{name:14s} killed   failed_runs={r.returncode} {first[:90]}")
    else:
        print(f"{name:14s} SURVIVED")
        survivors += 1
print(f"MUTANTS killed={len(MUTANTS) - survivors}/{len(MUTANTS)}")
sys.exit(survivors)
