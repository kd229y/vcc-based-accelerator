"""Host driver + on-board self-test for the VCC Morton accelerator.

Transports
  UART  (any FPGA board, vcc_uart_top):   python vcc_host.py --port COM5 selftest
  MMIO  (Zynq / PYNQ, vcc_axil_top):       python vcc_host.py --mmio 0x40000000 selftest

Self-test replays the golden vectors in hw/vectors (generated from soc_model/core.cpp)
and compares every result bit-for-bit: environment (points, offsets, cell ids, cells),
queries, FK and RRT-Connect plans.

  python vcc_host.py --port COM5 build  --scene s1_shelf
  python vcc_host.py --port COM5 query  --scene s1_shelf
  python vcc_host.py --port COM5 fk     --set fk_pkg
  python vcc_host.py --port COM5 plan   --cloud s0_pkgvec --plan p0_detour
  python vcc_host.py --port COM5 selftest
"""
import argparse
import struct
import sys
import time
from pathlib import Path

VEC = Path(__file__).resolve().parents[1] / 'vectors'

# register map (see rtl/vcc_core.v)
CTRL, STATUS, NPOINTS, KEPT, NCELLS, CULLED, BCYC, PROBES = 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C
Q_XY, Q_ZR, Q_RES, PT_WORD = 0x30, 0x34, 0x38, 0x3C
QREG, GREG = 0x40, 0x60
RANGE, MAXN, MAXIT, BUDGET, SEED, BIAS, MARGIN = 0x80, 0x84, 0x88, 0x8C, 0x90, 0x94, 0x98
PLEN, ITERS, NODES, STATES, EDGES, QUERIES, PCYC, FKCYC, ID = 0xA0, 0xA4, 0xA8, 0xAC, 0xB0, 0xB4, 0xB8, 0xBC, 0xFC
W_PTS, W_OFF, W_CID, W_CELL, W_FK, W_PATH = 0x100000, 0x200000, 0x300000, 0x400000, 0x500000, 0x600000
BUILD_STATUS = {0: 'SUCCESS', 4: 'CAPACITY_EXCEEDED', 5: 'INVALID_INPUT'}
PLAN_STATUS = ['SUCCESS', 'BUDGET_EXHAUSTED', 'INVALID_START', 'INVALID_GOAL', 'CAPACITY_EXCEEDED', 'INVALID_INPUT', 'INTERNAL_ERROR']


class LinkLost(Exception):
    """Serial link dropped (flaky USB) or the FPGA lost its configuration."""


class Uart:
    """UART transport that survives a flaky USB connector.

    Any serial error / timeout raises LinkLost after closing the port; Accel.retry()
    then calls recover(): wait for the COM port to come back, let the FPGA bridge's
    idle timeout (~84 ms) discard any half-received command, check the ID register,
    and re-program the bitstream if the board lost power (Nexys A7 is USB powered).
    """

    def __init__(self, port, baud=921600, bit=None, vivado=None):
        self.port, self.baud, self.bit, self.vivado = port, baud, bit, vivado
        self.s = None
        self._open(wait=30)

    def _open(self, wait):
        import serial  # pip install pyserial
        t0 = time.time()
        while True:
            try:
                self.s = serial.Serial(self.port, self.baud, timeout=3, write_timeout=10)
                time.sleep(0.25)                  # > bridge idle timeout
                self.s.reset_input_buffer()
                return
            except (serial.SerialException, OSError):
                if time.time() - t0 > wait:
                    raise
                time.sleep(0.5)

    def _close(self):
        try:
            if self.s: self.s.close()
        except Exception:
            pass
        self.s = None

    def _io(self, fn):
        import serial
        if self.s is None:
            raise LinkLost('port closed')
        try:
            return fn()
        except (serial.SerialException, OSError, TimeoutError, AssertionError) as e:
            self._close()
            raise LinkLost(str(e) or type(e).__name__)

    def _rx(self, n):
        d = self.s.read(n)
        if len(d) != n:
            raise TimeoutError('UART timeout')
        return d

    def recover(self):
        """Reconnect; re-program the FPGA if it no longer answers. Returns True if re-programmed."""
        self._close()
        print('  !! link lost -- waiting for the board to come back', flush=True)
        self._open(wait=120)
        for _ in range(2):
            try:
                if self._io(lambda: self._rd(ID)) >> 24 == 0x5C:
                    return False
            except LinkLost:
                self._open(wait=120)
        if not self.bit:
            raise LinkLost('FPGA does not answer (power lost?); pass --bit to auto re-program')
        print(f'  !! FPGA not configured -- re-programming {self.bit}', flush=True)
        self._close()
        import subprocess
        here = Path(__file__).resolve().parents[1] / 'vivado'
        subprocess.run([self.vivado, '-mode', 'batch', '-nojournal', '-nolog', '-source', str(here / 'program.tcl'),
                        '-tclargs', str(Path(self.bit).resolve())], check=True, cwd=str(here))
        self._open(wait=60)
        if self._io(lambda: self._rd(ID)) >> 24 != 0x5C:
            raise LinkLost('re-programmed but the FPGA still does not answer')
        return True

    def _wr(self, a, d):
        self.s.write(b'W' + struct.pack('<II', a, d & 0xffffffff))
        assert self._rx(1) == b'K'

    def _rd(self, a):
        self.s.write(b'R' + struct.pack('<I', a))
        return struct.unpack('<I', self._rx(4))[0]

    def wr(self, a, d): return self._io(lambda: self._wr(a, d))
    def rd(self, a): return self._io(lambda: self._rd(a))
    def stream(self, a, words): return self._io(lambda: self._stream(a, words))
    def burst(self, a, n): return self._io(lambda: self._burst(a, n))

    def _stream(self, a, words):
        if not words:
            return
        self.s.write(b'P' + struct.pack('<II', a, len(words)))
        chunk = 4096
        for i in range(0, len(words), chunk):
            self.s.write(struct.pack(f'<{len(words[i:i + chunk])}I', *words[i:i + chunk]))
        if words:
            assert self._rx(1) == b'K'

    def _burst(self, a, n):
        out = []
        while n:
            k = min(n, 8192)
            self.s.write(b'M' + struct.pack('<II', a, k))
            out += struct.unpack(f'<{k}I', self._rx(4 * k))
            a += 4 * k; n -= k
        return out


class Mmio:
    def __init__(self, base):
        try:
            from pynq import MMIO
            self.m = MMIO(base, 1 << 24)
            self.wr = lambda a, d: self.m.write(a, d & 0xffffffff)
            self.rd = lambda a: self.m.read(a)
        except ImportError:  # plain Linux /dev/mem
            import mmap, os
            fd = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
            mm = mmap.mmap(fd, 1 << 24, offset=base)
            self.wr = lambda a, d: mm.__setitem__(slice(a, a + 4), struct.pack('<I', d & 0xffffffff))
            self.rd = lambda a: struct.unpack('<I', mm[a:a + 4])[0]

    def stream(self, a, words):
        for w in words: self.wr(a, w)

    def burst(self, a, n):
        return [self.rd(a + 4 * i) for i in range(n)]


def hexfile(path, width=None):
    p = Path(path)
    if not p.exists() or p.stat().st_size == 0:
        return []
    return [int(t, 16) for t in p.read_text().split()]


class Accel:
    def __init__(self, t):
        self.t = t

    def retry(self, fn, *args, tries=6):
        """Run one self-contained test (it rebuilds its own environment); redo it after a link drop."""
        for k in range(tries):
            try:
                return fn(self, *args)
            except LinkLost as e:
                if not hasattr(self.t, 'recover') or k == tries - 1:
                    raise
                print(f'  !! {e} -- retrying ({k + 1}/{tries - 1})', flush=True)
                self.t.recover()

    def wait(self, bit, timeout=60):
        t0 = time.time()
        while self.t.rd(STATUS) >> bit & 1:
            if time.time() - t0 > timeout: raise TimeoutError(f'busy bit {bit} stuck')
        return self.t.rd(STATUS)

    def build(self, words):
        self.t.wr(NPOINTS, len(words) // 3)
        self.t.wr(CTRL, 1)
        self.t.stream(PT_WORD, words)
        s = self.wait(0)
        return (s >> 4) & 15

    def query(self, cx, cy, cz, r):
        self.t.wr(Q_XY, cy << 16 | cx)
        self.t.wr(Q_ZR, r << 16 | cz)
        while True:
            v = self.t.rd(Q_RES)
            if not v & 2: return v & 1, v >> 8

    def fk(self, q):
        for j, v in enumerate(q): self.t.wr(QREG + 4 * j, v)
        self.t.wr(CTRL, 4)
        self.wait(12)
        w = self.t.burst(W_FK, 59 * 2)
        return [(w[2 * i + 1] & 0xffff) << 32 | w[2 * i] for i in range(59)]

    def plan(self, start, goal, cfg=None):
        if cfg:
            for reg, idx in [(RANGE, 4), (MAXN, 3), (MAXIT, 12), (BUDGET, 11), (SEED, 9), (BIAS, 16), (MARGIN, 6)]:
                self.t.wr(reg, cfg[idx])
        for j in range(7):
            self.t.wr(QREG + 4 * j, start[j]); self.t.wr(GREG + 4 * j, goal[j])
        self.t.wr(CTRL, 8)
        s = self.wait(16, timeout=600)
        n = self.t.rd(PLEN)
        path = []
        for i in range(n):
            path.append(self.t.burst(W_PATH + 32 * i, 7))
        stats = {k: self.t.rd(a) for k, a in [('iterations', ITERS), ('nodes', NODES), ('state_calls', STATES),
                                             ('edge_calls', EDGES), ('queries', QUERIES), ('cycles', PCYC)]}
        return (s >> 20) & 15, path, stats


def check_build(acc, scene, full=True):
    d = VEC / scene
    meta = hexfile(d / 'meta.hex')
    words = hexfile(d / 'points_in.hex')
    t0 = time.time()
    st = acc.build(words)
    t = acc.t
    kept, ncells, cyc = t.rd(KEPT), t.rd(NCELLS), t.rd(BCYC)
    print(f'[{scene}] build {BUILD_STATUS.get(st, st)} kept={kept} cells={ncells} culled={t.rd(CULLED)} '
          f'hw_cycles={cyc} (host {time.time() - t0:.1f}s incl. transfer)')
    err = 0
    if st != meta[1]: print('  status mismatch, golden', meta[1]); return 1
    if st != 0: return 0
    err += kept != meta[2]; err += ncells != meta[3]
    if full:
        exp_p = hexfile(d / 'exp_points.hex')
        w = t.burst(W_PTS, 2 * kept) if kept else []
        got = [(w[2 * i + 1] & 0xffff) << 32 | w[2 * i] for i in range(kept)]
        bad = sum(a != b for a, b in zip(got, exp_p)); err += bad
        print(f'  points: {kept - bad}/{kept} match')
        off = t.burst(W_OFF, 32769); bad = sum(a != b for a, b in zip(off, hexfile(d / 'exp_offsets.hex'))); err += bad
        cid = t.burst(W_CID, 32768); bad2 = sum(a != b for a, b in zip(cid, hexfile(d / 'exp_cellid.hex'))); err += bad2
        print(f'  offsets mismatches={bad}  cell-id mismatches={bad2}')
        ec = hexfile(d / 'exp_cells.hex'); w = t.burst(W_CELL, 4 * ncells) if ncells else []
        bad = 0
        for c in range(ncells):
            v = w[4 * c:4 * c + 4]
            got = (v[0] & 0xffff) << 112 | (v[0] >> 16) << 96 | (v[3] << 64) | (v[2] << 32) | v[1]
            bad += got != ec[c]
        err += bad
        print(f'  cells mismatches={bad}')
    return err


def check_queries(acc, scene, limit=None):
    d = VEC / scene
    qs = hexfile(d / 'queries.hex'); qe = hexfile(d / 'exp_query.hex')
    if limit: qs, qe = qs[:limit], qe[:limit]
    bad, cyc = 0, []
    for v, e in zip(qs, qe):
        cx, cy, cz, r = v & 0xffff, v >> 16 & 0xffff, v >> 32 & 0xffff, v >> 48
        hit, c = acc.query(cx, cy, cz, r); cyc.append(c)
        bad += hit != e
    print(f'[{scene}] queries: {len(qs) - bad}/{len(qs)} match, avg {sum(cyc) / max(1, len(cyc)):.0f} cycles')
    return bad


def check_scene(acc, scene, nq=200):
    err = check_build(acc, scene)
    if hexfile(VEC / scene / 'meta.hex')[1] == 0:
        err += check_queries(acc, scene, limit=nq)
    return err


def check_fk(acc, name, n=None):
    d = VEC / name
    qin = hexfile(d / 'fk_in.hex'); qout = hexfile(d / 'fk_out.hex')
    cases = len(qin) // 7 if n is None else n
    bad = 0
    for c in range(cases):
        q = qin[7 * c:7 * c + 7]
        bad += sum(a != b for a, b in zip(acc.fk(q), qout[59 * c:59 * c + 59]))
    print(f'[{name}] fk: {cases} cases, {bad} sphere mismatches, {acc.t.rd(FKCYC)} cycles/FK')
    return bad


def check_plan(acc, cloud, plan):
    d = VEC / plan
    check_build(acc, cloud, full=False)
    pin = hexfile(d / 'plan_in.hex'); cfg = hexfile(d / 'plan_cfg.hex'); meta = hexfile(d / 'plan_meta.hex')
    ep = hexfile(d / 'exp_path.hex')
    st, path, stats = acc.plan(pin[:7], pin[7:], cfg)
    flat = [v for p in path for v in p]
    ok = st == meta[0] and flat == ep and [stats[k] for k in ['iterations', 'nodes', 'state_calls', 'edge_calls', 'queries']] == meta[2:7]
    print(f'[{plan}] plan {PLAN_STATUS[st]} waypoints={len(path)} {stats} -> {"MATCH" if ok else "MISMATCH"}')
    print(f'  hw time {stats["cycles"] / 100:.0f} us @100 MHz')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port'); ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--mmio', type=lambda s: int(s, 0))
    ap.add_argument('--bit', help='bitstream used to auto re-program after a power loss')
    ap.add_argument('--vivado', default=r'C:\Xilinx\2025.1\Vivado\bin\vivado.bat')
    ap.add_argument('cmd', choices=['id', 'build', 'query', 'fk', 'plan', 'selftest'])
    ap.add_argument('--scene', default='s1_shelf'); ap.add_argument('--set', default='fk_pkg')
    ap.add_argument('--cloud', default='s0_pkgvec'); ap.add_argument('--plan', default='p0_detour')
    a = ap.parse_args()
    t = Mmio(a.mmio) if a.mmio is not None else Uart(a.port, a.baud, a.bit, a.vivado)
    acc = Accel(t)
    ident = acc.retry(lambda acc: acc.t.rd(ID))
    print(f'ID {ident:08x}: hash 2^{ident >> 16 & 15}, cells 2^{ident >> 8 & 15}, tree 2^{ident & 15}')
    if a.cmd == 'id': return 0
    if a.cmd == 'build': return acc.retry(check_build, a.scene)
    if a.cmd == 'query': return acc.retry(check_scene, a.scene, None)
    if a.cmd == 'fk': return acc.retry(check_fk, a.set)
    if a.cmd == 'plan': return acc.retry(check_plan, a.cloud, a.plan)
    err = 0
    hash_aw = ident >> 16 & 15
    for s in ['s0_pkgvec', 's1_shelf', 's2_edge', 's4_inf', 's5_empty', 's6_dense']:
        if (s == 's6_dense' or s == 's1_shelf') and hash_aw < 14:
            continue
        err += acc.retry(check_scene, s)
    err += acc.retry(check_fk, 'fk_pkg', 20)
    err += acc.retry(check_plan, 's5_empty', 'p1_free')
    err += acc.retry(check_plan, 's0_pkgvec', 'p0_detour')
    err += acc.retry(check_plan, 's1_shelf', 'p2_shelf_s1')
    print('SELFTEST', 'PASS' if err == 0 else f'FAIL ({err})')
    return 1 if err else 0


if __name__ == '__main__':
    sys.exit(main())
