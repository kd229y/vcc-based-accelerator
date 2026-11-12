"""Generate input clouds and golden vectors (via gen_golden, i.e. the user's core.cpp) for the RTL testbenches.

usage: python3 make_vectors.py            (writes hw/vectors/*)
"""
from pathlib import Path
import json
import shutil
import subprocess
import sys

import numpy as np

HW = Path(__file__).resolve().parents[1]
PKG = HW.parent / 'SoC_軟體驗證資料包'
OUT = HW / 'vectors'
EXE = HW / 'golden' / 'gen_golden'


def compile_golden():
    src = HW / 'golden' / 'gen_golden.cpp'
    if EXE.exists() and EXE.stat().st_mtime > src.stat().st_mtime:
        return
    subprocess.run(['g++', '-O2', '-std=c++17', f'-I{PKG / "build"}', str(src), '-o', str(EXE)], check=True)


def sample_cloud(problem, n, seed=0):
    # Same algorithm as runner/data.py::sample_cloud (copied to avoid importing the package runner).
    rng = np.random.default_rng(seed)
    parts = []
    for shape in problem['primitives']:
        dims = shape['dimensions']
        if shape['type'] == 'box':
            dx, dy, dz = dims
            w = np.array([dy * dz, dy * dz, dx * dz, dx * dz, dx * dy, dx * dy]); w /= w.sum()
            sides = rng.choice(6, n, p=w); p = rng.uniform(-.5, .5, (n, 3)) * dims
            for s in range(6): p[sides == s, s // 2] = dims[s // 2] / 2 * (1 if s % 2 == 0 else -1)
        elif shape['type'] == 'cylinder':
            h, r = dims
            w = np.array([np.pi * r * r, 2 * np.pi * r * h, np.pi * r * r]); w /= w.sum()
            sides = rng.choice(3, n, p=w); ang = rng.uniform(-np.pi, np.pi, n)
            rho = np.where(sides == 1, r, r * np.sqrt(rng.random(n))); z = rng.uniform(-h / 2, h / 2, n)
            z[sides == 0] = -h / 2; z[sides == 2] = h / 2
            p = np.column_stack((rho * np.cos(ang), rho * np.sin(ang), z))
        elif shape['type'] == 'sphere':
            z = rng.uniform(-1, 1, n); ang = rng.uniform(-np.pi, np.pi, n); rho = np.sqrt(1 - z * z)
            p = np.column_stack((rho * np.cos(ang), rho * np.sin(ang), z)) * dims[0]
        else:
            raise ValueError(shape['type'])
        t = np.asarray(shape['world_from_local']); parts.append(p @ t[:3, :3].T + t[:3, 3])
    return np.ascontiguousarray(np.concatenate(parts) if parts else np.empty((0, 3)), dtype=np.float32)


def edge_cloud(rng):
    pts = list(rng.uniform(-2.5, 2.5, (3000, 3)))
    special = [0.0, -0.0, 1e-45, -1e-45, 1e-39, 1.19, -1.19, 1.1899, 2.0, -2.0, 1.99999, -1.99999, 1e30, -1e30,
               0.03125, -0.03125, 0.0312499, 0.5, -0.5, 6.103515625e-05, -6.103515625e-05, 3.0e-5, -3.0e-5]
    for a in special:
        for b in special[:8]:
            pts.append([a, b, 0.25]); pts.append([0.1, a, b]); pts.append([b, 0.2, a])
    # Many points in the same fine voxel with tied selector distances (keep first).
    base = np.array([0.40625, -0.3125, 0.21875], np.float64)
    for _ in range(400):
        o = rng.integers(0, 512, 3) / 16384.0
        pts.append(base + o)
        pts.append(base + o)  # exact duplicate -> tie
    # Mirror-symmetric tie positions inside one voxel
    for sx in (1, -1):
        for sy in (1, -1):
            pts.append(base + 0.015625 + np.array([sx, sy, 1]) * 0.004)
    a = np.array(pts, np.float64)
    rng.shuffle(a)
    return a.astype(np.float32)


def ball(rng, n, radius):
    v = rng.normal(size=(n, 3)); v /= np.linalg.norm(v, axis=1, keepdims=True)
    return (v * radius * rng.random((n, 1)) ** (1 / 3)).astype(np.float32)


def run(scene, pts, cap=32768, nq=600, seed=7):
    d = OUT / scene; d.mkdir(parents=True, exist_ok=True)
    pts.astype('<f4').tofile(d / 'input_points_f32.bin')
    subprocess.run([str(EXE), 'build', str(d / 'input_points_f32.bin'), str(d), str(cap), str(nq), str(seed)], check=True)


def main():
    compile_golden()
    if OUT.exists(): shutil.rmtree(OUT)
    OUT.mkdir()
    rng = np.random.default_rng(2026)
    # s0: the exact cloud used by the package's rtl_vectors/user_original_v1
    s0 = np.fromfile(PKG / 'rtl_vectors/user_original_v1/input_points_f32.bin', '<f4').reshape(-1, 3)
    run('s0_pkgvec', s0)
    # s1: a real MotionBenchMaker Panda scene (bookshelf_thin) surface-sampled
    idx = json.loads((PKG / 'data/panda_mbm/index.json').read_text())
    entry = next(e for e in idx if e['scenario'] == 'bookshelf_thin')
    prob = json.loads((PKG / entry['problem_path']).read_text())
    run('s1_shelf', sample_cloud(prob, 2500, seed=0))
    # s2: numeric edge cases, culling, ties, denormals, huge values
    run('s2_edge', edge_cloud(rng))
    # s3: hash capacity overflow at a small capacity (HW must be built with HASH_CAP=1024)
    run('s3_cap1024', ball(rng, 3000, 1.1), cap=1024)
    # s4: non-finite input in the middle -> INVALID_INPUT
    s4 = ball(rng, 500, 1.0); s4[250, 1] = np.inf
    run('s4_inf', s4)
    # s5: empty cloud
    run('s5_empty', np.empty((0, 3), np.float32), nq=50)
    # s6: dense ball -> thousands of coarse cells
    run('s6_dense', ball(rng, 12000, 1.15), nq=800)
    # FK vectors
    (OUT / 'fk').mkdir()
    subprocess.run([str(EXE), 'fk', str(OUT / 'fk'), '200', '11'], check=True)
    # also the package's own 128 FK cases, converted to hex
    fin = np.fromfile(PKG / 'rtl_vectors/user_original_v1/fk_input_q24_i32.bin', '<i4')
    fout = np.fromfile(PKG / 'rtl_vectors/user_original_v1/fk_expected_q14_u16.bin', '<u2').reshape(-1, 59, 3)
    (OUT / 'fk_pkg').mkdir()
    (OUT / 'fk_pkg/fk_in.hex').write_text(''.join(f'{int(v) & 0xffffffff:08x}\n' for v in fin))
    (OUT / 'fk_pkg/fk_out.hex').write_text(''.join(f'{s[2]:04x}{s[1]:04x}{s[0]:04x}\n' for c in fout for s in c))
    # package query vectors against s0 environment
    qc = np.fromfile(PKG / 'rtl_vectors/user_original_v1/query_centers_q14_u16.bin', '<u2').reshape(-1, 3)
    qr = np.fromfile(PKG / 'rtl_vectors/user_original_v1/query_radii_q14_u16.bin', '<u2')
    qe = np.fromfile(PKG / 'rtl_vectors/user_original_v1/query_expected_u8.bin', '<u1')
    d = OUT / 's0_pkgvec'
    (d / 'queries.hex').write_text(''.join(f'{r:04x}{c[2]:04x}{c[1]:04x}{c[0]:04x}\n' for c, r in zip(qc, qr)))
    (d / 'exp_query.hex').write_text(''.join(f'{v:x}\n' for v in qe))
    meta = [int(x, 16) for x in (d / 'meta.hex').read_text().split()]
    meta[4] = len(qr)
    (d / 'meta.hex').write_text(''.join(f'{v:08x}\n' for v in meta))
    print('vectors written to', OUT)


if __name__ == '__main__':
    sys.exit(main())
