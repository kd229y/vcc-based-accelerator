"""Board test of the integrated design (vcc_v0_uart_top): our build engine builds the format-v0 environment
from the raw point cloud, PLE-CSE plans on it, everything compared with PLE-CSE's golden model.

  python vcc_sys_host.py --port COM4 [--bit ..\\vivado\\out_v0_ncu1\\vcc_v0_uart_top.bit] run
  python vcc_sys_host.py --port COM4 run --probs 0 100 300

Vectors: hw/vectors_v0/p<i>/ (points_in.hex, meta.hex from gen_env_v0; plan_s<seed>.txt from gen_plan_v0,
seed = position of the problem in the default list).
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vcc_host import Uart, LinkLost, hexfile  # noqa: E402

VEC = Path(__file__).resolve().parents[1] / 'vectors_v0'
DEFAULT = [0, 100, 200, 300, 400, 500, 600, 542, 482]
BE = 0x010000
BE_CTRL, BE_STATUS, BE_NPTS, BE_WBUF, BE_KEPT, BE_NCELLS, BE_CULLED, BE_CYC, BE_PROBES, BE_PT, BE_ID = (
    BE + 0x00, BE + 0x04, BE + 0x08, BE + 0x0C, BE + 0x10, BE + 0x14, BE + 0x18, BE + 0x1C, BE + 0x20, BE + 0x3C, BE + 0xFC)
# PLE host interface (PLE-CSE accel/rtl/ple/ple_top.sv)
P_CTRL, P_STATUS, P_IRQCLR, P_MODE, P_MAXIT = 0x000, 0x004, 0x008, 0x010, 0x028
P_START, P_GOAL, P_SHIFT = 0x040, 0x050, 0x060
P_ITERS, P_SIZES, P_PLEN, P_CYC, P_EDGES, P_DD, P_NN, P_PATH = 0x080, 0x084, 0x088, 0x0C0, 0x0C4, 0x0D8, 0x0DC, 0x400


def s16(v):
    return v & 0xffff


def pair(a, b):
    return (s16(b) << 16) | s16(a)


def run_one(t, prob, seed, buf, iters=3000):
    d = VEC / f'p{prob}'
    meta = hexfile(d / 'meta.hex')
    words = hexfile(d / 'points_in.hex')
    plan = [int(x) for x in (d / f'plan_s{seed}.txt').read_text().split()]
    sg, sh, st, path = plan[:14], plan[14:21], plan[21:29], plan[29:]
    t0 = time.time()
    t.wr(BE_NPTS, meta[0]); t.wr(BE_WBUF, buf); t.wr(BE_CTRL, 1)
    t.stream(BE_PT, words)
    while t.rd(BE_STATUS) & 1:
        pass
    status = (t.rd(BE_STATUS) >> 4) & 15
    kept, cells, bcyc = t.rd(BE_KEPT), t.rd(BE_NCELLS), t.rd(BE_CYC)
    err = int(status != 0) + int(kept != meta[2]) + int(cells != meta[3])
    print(f'  p{prob} build -> buf{buf}: status={status} kept={kept}/{meta[2]} cells={cells}/{meta[3]} '
          f'({time.time() - t0:.1f}s incl. {len(words) * 4 / 1e6:.2f} MB over UART)', flush=True)
    for k in range(4):
        t.wr(P_START + 4 * k, pair(sg[2 * k], sg[2 * k + 1]) if k < 3 else s16(sg[6]))
        t.wr(P_GOAL + 4 * k, pair(sg[7 + 2 * k], sg[8 + 2 * k]) if k < 3 else s16(sg[13]))
    for j in range(7):
        t.wr(P_SHIFT + 4 * j, sh[j])
    t.wr(P_MAXIT, iters)
    t.wr(P_MODE, 0xC | buf)
    t.wr(P_CTRL, 0x5)
    t1 = time.time()
    while not (t.rd(P_STATUS) & 2):
        if time.time() - t1 > 120:
            raise TimeoutError('PLE did not finish')
    v = t.rd(P_STATUS)
    got = dict(result=(v >> 4) & 15, iters=t.rd(P_ITERS), sizes=t.rd(P_SIZES), path_len=t.rd(P_PLEN),
               edges=t.rd(P_EDGES), dd=t.rd(P_DD), nn=t.rd(P_NN))
    exp = dict(result=st[0], iters=st[1], sizes=(s16(st[3]) << 16) | s16(st[2]), path_len=st[7],
               edges=st[5], dd=st[4], nn=st[6])
    bad = [k for k in exp if got[k] != exp[k]]
    words_path = t.burst(P_PATH, 4 * got['path_len']) if got['path_len'] == st[7] else []
    for i in range(min(st[7], len(words_path) // 4)):
        w = path[7 * i:7 * i + 7]
        expw = [pair(w[0], w[1]), pair(w[2], w[3]), pair(w[4], w[5]), s16(w[6])]
        if words_path[4 * i:4 * i + 4] != expw:
            bad.append(f'path[{i}]')
    pcyc = t.rd(P_CYC)
    t.wr(P_IRQCLR, 1)
    print(f'  p{prob} plan  (buf{buf}): result={got["result"]} iters={got["iters"]} path={got["path_len"]} '
          f'cycles={pcyc} ({pcyc / 75e3:.2f} ms @75 MHz) | build cycles={bcyc} -> '
          f'{"MATCH" if not bad else "MISMATCH " + ",".join(bad[:6])}', flush=True)
    return err + len(bad)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--bit')
    ap.add_argument('--hw-url', help='hw_server url for re-programming, e.g. localhost:3400')
    ap.add_argument('--vivado', default=r'C:\Xilinx\2025.1\Vivado\bin\vivado.bat')
    ap.add_argument('cmd', choices=['id', 'run'])
    ap.add_argument('--probs', type=int, nargs='*')
    a = ap.parse_args()
    t = Uart(a.port, a.baud, a.bit, a.vivado)
    t.id_addr = BE_ID
    t.hw_url = a.hw_url
    ident = t.rd(BE_ID)
    print(f'ID {ident:08x}: N_CU={ident >> 8 & 255}, {ident & 255} MHz')
    if a.cmd == 'id':
        return 0
    probs = a.probs or DEFAULT
    err = 0
    for n, p in enumerate(probs):
        seed = DEFAULT.index(p) if p in DEFAULT else n
        for attempt in range(6):
            try:
                err += run_one(t, p, seed, n % 2)
                break
            except LinkLost as e:
                print(f'  !! {e} -- retrying p{p}', flush=True)
                t.recover()
    print('SYSTEST', 'PASS' if err == 0 else f'FAIL ({err})')
    return 1 if err else 0


if __name__ == '__main__':
    sys.exit(main())
