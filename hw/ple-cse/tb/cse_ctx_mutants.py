#!/usr/bin/env python3
"""Mutants of the cse_cu context entries (ids, allocation, N_BUF); every one must fail a CSE or PLE+CSE test.
usage: cse_ctx_mutants.py <build_dir> <trace_dir> [name ...]   (run from hw/ple-cse; needs build/accel/libfxref.a)"""
import os, re, subprocess, sys

MUTANTS = {
    'ctx_s3_id':     ("          s3_id          <= cell_payload.id;\n", ""),
    'ctx_s2_end_id': ("s2_out_payload.id    <= voxel_payload.id;", "s2_out_payload.id    <= s2_voxel.id;"),
    'ctx_full':      (" && (c_cnt != CNT_W'(NCTX))", ""),
    'ctx_s1_id':     ("s1_id     <= s0_id;", "s1_id     <= wp;"),
    'ctx_canc_s2':   ("if (canc[s2_voxel.id] || !dir_word.occ)", "if (canc[s1_id] || !dir_word.occ)"),
    'ctx_tag':       ("s4_payload.tag  <= c_tag[s3_id];", "s4_payload.tag  <= c_tag[wp];"),
    'ctx_bnd':       ("assign s2_bnd = c_bnd[s2_voxel.id];", "assign s2_bnd = c_bnd[s1_id];"),
    'ctx_pt_wr':     ("c_pt[wp]  <= pt_new;", "c_pt[s0_id]  <= pt_new;"),
    'nbuf_err':      (" || (!DBUF && q_payload.query.env_buf)", ""),
    'nbuf_hdr':      ("hb = DBUF ? q_payload.query.env_buf : 1'b0;", "hb = 1'b0;"),
    'nbuf_rdbuf':    ("if (s3_ball_active) mem_rd_buf = c_buf[s3_id];", "if (s3_ball_active) mem_rd_buf = 1'b0;"),
}

PKG = ['rtl/common/ple_cse_pkg.sv', 'rtl/cse/cse_cu_pkg.sv']
CSE = ['rtl/cse/cse_fifo.sv', 'rtl/cse/cse_query_fifo.sv', 'rtl/cse/cse_dispatcher.sv', 'rtl/cse/cse_receiver.sv',
       'CU', 'rtl/cse/cse_env_mem.sv', 'rtl/cse/cse_top.sv', 'rtl/cse/cse_system.sv']
PLE = ['rtl/ple/ple_pkg.sv', 'rtl/ple/ple_robot_pkg.sv', 'rtl/ple/ple_rkg.sv', 'rtl/ple/ple_fku.sv', 'rtl/ple/ple_scu.sv',
       'rtl/ple/ple_qd.sv', 'rtl/ple/ple_smp.sv', 'rtl/ple/ple_tree.sv', 'rtl/ple/ple_nnu.sv', 'rtl/ple/ple_isqrt.sv',
       'rtl/ple/ple_div7.sv', 'rtl/ple/ple_evu.sv', 'rtl/ple/ple_core.sv']


def tests(build, trace):
    lib = os.path.join(build, 'libfxref.a')
    return [  # name, top, extra sources, verilator args, plusargs, pass pattern(s)
        ('cuctx', 'tb_cse_cu_ctx', PKG + ['rtl/cse/cse_fifo.sv', 'CU', 'tb/tb_cse_cu_ctx.sv'], [], [],
         [r'TB_CSE_CU_CTX .*errors=0']),
        ('trace', 'tb_cse_trace', PKG + CSE + ['tb/ple_cse_if_checker.sv', 'tb/tb_cse_trace.sv'], [],
         [f'+env0={trace}/s0_p1', f'+queries={trace}/s0_p1_seed0_queries.txt'], [r'TB_CSE_TRACE .*errors=0 checker_errors=0']),
        ('tracekill', 'tb_cse_trace', PKG + CSE + ['tb/ple_cse_if_checker.sv', 'tb/tb_cse_trace.sv'], [],
         [f'+env0={trace}/s0_p3', f'+queries={trace}/s0_p3_seed0_queries.txt', '+kill=50'],
         [r'TB_CSE_TRACE .*errors=0 checker_errors=0']),
        ('twobuf', 'tb_cse_trace', PKG + CSE + ['tb/ple_cse_if_checker.sv', 'tb/tb_cse_trace.sv'], ['-GNB=2'],
         [f'+env0={trace}/s0_p1', f'+queries={trace}/s0_p1_seed0_queries.txt',
          f'+env1={trace}/s0_p2', f'+queries1={trace}/s0_p2_seed0_queries.txt'], [r'TB_CSE_TRACE .*errors=0 checker_errors=0']),
        ('kill', 'tb_cse_kill', PKG + CSE + ['tb/ple_cse_if_checker.sv', 'tb/tb_cse_kill.sv'], [],
         [f'+env0={trace}/s0_p1'], [r'TB_CSE_KILL .*errors=0 checker_errors=0']),
        ('core', 'tb_core', PKG[:1] + PLE + PKG[1:] + CSE + ['tb/cse_rtl.sv', 'tb/ple_cse_if_checker.sv', 'tb/tb_core.sv'],
         ['+define+RTL_CSE', '+define+CSE_N_CU=2', '-LDFLAGS', lib, '-LDFLAGS', '-lm'], ['+probs=6', '+seeds=1'],
         [r'^TB_CORE flat.*errors=0', r'\[if_checker\].*errors=0']),
    ]


def run_mutant(name, cu_src, build, trace):
    d = os.path.join(build, 'cse_ctx_mut', name)
    os.makedirs(d, exist_ok=True)
    cu = os.path.join(d, 'cse_cu.sv')
    with open(cu, 'w') as f:
        f.write(cu_src)
    for tname, top, srcs, vargs, pargs, pats in tests(build, trace):
        mdir = os.path.join(d, tname)
        srcs = [cu if s == 'CU' else s for s in srcs]
        cmd = ['verilator', '--binary', '--timing', '--assert', '-Wno-fatal', '-Irtl/common', '-Mdir', mdir,
               '--top-module', top, '-o', 'Vsim', '--build-jobs', '4'] + vargs + srcs
        if subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0:
            return f'killed ({tname}: does not build)'
        try:
            p = subprocess.run([os.path.join(mdir, 'Vsim')] + pargs, capture_output=True, text=True, timeout=600)
        except subprocess.TimeoutExpired:
            return f'killed ({tname}: timeout)'
        out = p.stdout + p.stderr
        bad = re.search(r'%Error|%Fatal|ERROR', out)
        if p.returncode != 0 or bad or not all(re.search(x, out, re.M) for x in pats):
            first = bad.group(0) if bad else f'rc={p.returncode}'
            line = next((l for l in out.splitlines() if first in l), first)
            return f'killed ({tname}: {line.strip()[:150]})'
    return 'SURVIVED'


def main():
    if len(sys.argv) < 3:
        raise SystemExit(__doc__)
    build, trace = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    names = sys.argv[3:] or list(MUTANTS)
    src = open('rtl/cse/cse_cu.sv').read()
    survived = 0
    for n in names:
        old, new = MUTANTS[n]
        if src.count(old) != 1:
            raise SystemExit(f'{n}: expected one match of {old!r} in cse_cu.sv')
        r = run_mutant(n, src.replace(old, new), build, trace)
        survived += r == 'SURVIVED'
        print(f'cse ctx mutant {n} {r}', flush=True)
    print(f'CSE_CTX_MUTANTS killed={len(names) - survived}/{len(names)}')
    sys.exit(1 if survived else 0)


if __name__ == '__main__':
    main()
