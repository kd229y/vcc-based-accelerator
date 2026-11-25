#!/usr/bin/env python3
"""hw/ddr/host/ddr_host.py -- L4 board test of ddr_uart_top (docs/DDR_PORT.md section 7). Run it in your own terminal.

usage: python3 hw/ddr/host/ddr_host.py [--port /dev/ttyUSB1] [--baud 115200] [--rounds 3] [--log FILE] [--quick]
Steps per round: calibration -> window word test -> 128 MB MODE 0 -> MODE 3 (strobes) -> burst sweep -> MODE 4 latency.
Every step prints one line with PASS/FAIL and its numbers; the log file gets the same lines.
Last line: DDR_HOST PASS  or  DDR_HOST FAIL failed=N.
UART protocol (hw/rtl/uart_bridge.v): 'W' a[4] d[4] -> 'K';  'R' a[4] -> d[4]  (little endian).
No third-party packages (termios only)."""
import argparse, os, sys, termios, time, tty

UI_HZ = 75_000_000
MEM_BEATS = (128 << 20) // 16
M32 = 0xFFFF_FFFF
R = dict(CTRL=0x000, STATUS=0x004, BASE=0x008, LEN=0x00C, BURST=0x010, MODE=0x014, SEED=0x018, ERRS=0x020,
         ERR_ADDR=0x024, ERR_GOT=0x028, ERR_EXP=0x02C, WR_CYC=0x030, RD_CYC=0x034, LAT_MIN=0x038,
         LAT_MAX=0x03C, LAT_SUM=0x040, WIN_ADDR=0x100, WIN_DATA=0x104)


def hw(widx, s):
    """One pattern word (same formula as hw/ddr/rtl/ddr_test.v): widx = byte address >> 2."""
    x = (widx ^ s) & M32
    x ^= (x << 13) & M32; x ^= x >> 17; x ^= (x << 5) & M32
    x = (x + (((s & 0xFFFF) << 16) | (s >> 16)) + 0x9E37_79B9) & M32
    return x ^ (x >> 15)


class Link:
    def __init__(self, port, baud):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
        tty.setraw(self.fd)
        a = termios.tcgetattr(self.fd)
        speed = getattr(termios, f"B{baud}")
        a[4] = a[5] = speed
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 10          # 1 s read timeout
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def _rd(self, n):
        b = b""
        t0 = time.time()
        while len(b) < n:
            c = os.read(self.fd, n - len(b))
            if c:
                b += c
            elif time.time() - t0 > 3:
                raise TimeoutError(f"UART: got {len(b)} of {n} bytes")
        return b

    def w(self, a, d):
        os.write(self.fd, b"W" + a.to_bytes(4, "little") + (d & M32).to_bytes(4, "little"))
        k = self._rd(1)
        if k != b"K":
            raise IOError(f"write {a:#x}: reply {k!r}")

    def r(self, a):
        os.write(self.fd, b"R" + a.to_bytes(4, "little"))
        return int.from_bytes(self._rd(4), "little")


class Test:
    def __init__(self, link, logf):
        self.l, self.logf, self.failed = link, logf, 0

    def out(self, s):
        print(s, flush=True)
        self.logf.write(s + "\n"); self.logf.flush()

    def check(self, name, ok, info):
        self.out(f"{'PASS' if ok else 'FAIL'} {name:28s} {info}")
        if not ok:
            self.failed += 1
        return ok

    def run(self, mode, base, length, burst, seed, timeout=120):
        l = self.l
        for k, v in (("BASE", base), ("LEN", length), ("BURST", burst), ("MODE", mode), ("SEED", seed)):
            l.w(R[k], v)
        l.w(R["CTRL"], 1)
        t0 = time.time()
        while True:
            st = l.r(R["STATUS"])
            if not st & 2:
                break
            if time.time() - t0 > timeout:
                return dict(st=st, timeout=True)
            time.sleep(0.05)
        res = dict(st=st, timeout=False)
        for k in ("ERRS", "ERR_ADDR", "ERR_GOT", "ERR_EXP", "WR_CYC", "RD_CYC", "LAT_MIN", "LAT_MAX", "LAT_SUM"):
            res[k] = l.r(R[k])
        return res

    def result(self, name, res, nbeats, extra=""):
        st = res["st"]
        ok = (not res["timeout"]) and (st & 0x1D) == 0x05 and res["ERRS"] == 0
        bw = []
        for k, lab in (("WR_CYC", "wr"), ("RD_CYC", "rd")):
            c = res.get(k, 0)
            if c:
                bw.append(f"{lab}={nbeats * 16 / (c / UI_HZ) / 1e6:.0f}MB/s")
        info = f"status={st:#x} errs={res.get('ERRS')} {' '.join(bw)} {extra}"
        if res["timeout"]:
            info += " TIMEOUT (busy)"
        elif res["ERRS"]:
            a = res["ERR_ADDR"]
            info += (f" first_err addr={a:#x} got={res['ERR_GOT']:#010x} exp={res['ERR_EXP']:#010x}")
        if st & 0x10:
            info += " AXI_ERR"
        return self.check(name, ok, info)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rounds", type=int, default=3)
    ap.add_argument("--log", default=None)
    ap.add_argument("--quick", action="store_true", help="16 MB instead of 128 MB")
    a = ap.parse_args()
    logp = a.log or time.strftime("build/ddr/board/ddr_host_%Y%m%d_%H%M%S.log")
    os.makedirs(os.path.dirname(os.path.abspath(logp)), exist_ok=True)
    with open(logp, "w") as logf:
        t = Test(Link(a.port, a.baud), logf)
        t.out(f"DDR_HOST start {time.strftime('%Y-%m-%d %H:%M:%S')} port={a.port} baud={a.baud} rounds={a.rounds}")
        try:
            # calibration
            t0 = time.time()
            st = t.l.r(R["STATUS"])
            while not st & 1 and time.time() - t0 < 5:
                time.sleep(0.1); st = t.l.r(R["STATUS"])
            if not t.check("calibration", st & 1 == 1, f"status={st:#x}"):
                raise RuntimeError("no calibration")
            total = MEM_BEATS if not a.quick else MEM_BEATS // 8
            for rnd in range(a.rounds):
                seed = (0x1357_9BDF * (rnd + 1)) & M32
                t.out(f"-- round {rnd} seed={seed:#010x}")
                # window: single words, then the pattern formula against the hardware
                ws = [(0x40 + 4 * i, (0xA5000000 | (rnd << 16) | i)) for i in range(8)]
                for ad, d in ws:
                    t.l.w(R["WIN_ADDR"], ad); t.l.w(R["WIN_DATA"], d)
                bad = 0
                for ad, d in ws:
                    t.l.w(R["WIN_ADDR"], ad); bad += t.l.r(R["WIN_DATA"]) != d
                t.check("window words", bad == 0, f"bad={bad}/8")
                # full range write + check
                res = t.run(0, 0, total, 256, seed)
                t.result(f"MODE0 {total * 16 >> 20}MB burst256", res, total)
                # pattern formula: words read through the window equal the host hash
                bad = 0
                for ad in (0, 0x1234560, (total - 1) * 16 + 12, 0x7FFF0, 0x40_0004):
                    if ad < total * 16:
                        t.l.w(R["WIN_ADDR"], ad); bad += t.l.r(R["WIN_DATA"]) != hw(ad >> 2, seed)
                t.check("pattern = host formula", bad == 0, f"bad={bad}")
                # MODE 4 latency over the whole range (data checked against the same seed)
                res = t.run(4, 0, total, 10000, seed)
                n = 10000
                extra = f"lat min={res['LAT_MIN']} max={res['LAT_MAX']} avg={res['LAT_SUM'] / n:.1f} cycles"
                t.result("MODE4 random reads x10000", res, n, extra)
                # MODE 3 strobes on 8 MB
                res = t.run(3, 0x0100_0000, 8 << 16, 64, seed ^ 0xFFFF)
                t.result("MODE3 strobe 8MB", res, 8 << 16)
                # burst sweep, 1 MB each, odd base offsets
                for i, bl in enumerate((1, 4, 16, 64, 256)):
                    base = 0x0200_0000 + i * 0x10_0000 + 0x30
                    res = t.run(0, base, 1 << 16, bl, seed + i)
                    t.result(f"MODE0 1MB burst{bl}", res, 1 << 16)
        except Exception as e:  # report and stop
            t.check("exception", False, repr(e))
        t.out(f"DDR_HOST {'PASS' if t.failed == 0 else 'FAIL failed=%d' % t.failed}  log={logp}")
    sys.exit(1 if t.failed else 0)


if __name__ == "__main__":
    main()
