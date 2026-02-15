"""
LIN Live Monitor & Development Tool — 81A 955 555 A

Real-time frame viewer with change detection, master command injection,
timing analysis, and byte statistics. Self-contained.

Usage (from REPL):
    from decoder import *
    live()                                  # Passive live view
    live(cmd=0x20, data=[0x81, 0x04, ...])  # Live with master command
    inject(0x20, [0x81, 0x04, 0x02, ...])   # Inject and watch for changes
    timing()                                # Measure frame response time
    stats(count=100)                        # Capture + byte statistics
    stimulus("flashlight")                  # Before/after comparison
"""

import rp2
from machine import Pin, UART
import time


# ================================================================
# PIO + LIN (self-contained, identical to main.py)
# ================================================================

@rp2.asm_pio(out_init=rp2.PIO.OUT_LOW, set_init=rp2.PIO.OUT_LOW,
             out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _pio_tx():
    wrap_target()
    label("entry")
    pull(block)
    out(y, 8)
    out(x, 1)
    jmp(not_x, "byte")
    set(pins, 1)           [7]
    set(x, 12)             [7]
    label("brk")
    nop()                  [7]
    jmp(x_dec, "brk")     [7]
    set(pins, 0)           [7]
    jmp("entry")
    label("byte")
    mov(osr, y)            [7]
    set(pins, 1)           [7]
    set(x, 7)
    label("bit")
    out(pins, 1)           [6]
    jmp(x_dec, "bit")
    set(pins, 0)           [7]
    wrap()


class LIN:
    _BRK = 1 << 8

    def __init__(self, tx=0, rx=1, baud=19200):
        self.uart = UART(0, baudrate=baud, tx=Pin(12), rx=Pin(rx),
                         bits=8, parity=None, stop=1)
        self.sm = rp2.StateMachine(
            0, _pio_tx, freq=baud * 8,
            out_base=Pin(tx), set_base=Pin(tx))
        self.sm.active(1)
        print("LIN: TX=GPIO%d RX=GPIO%d %dbaud" % (tx, rx, baud))

    @staticmethod
    def pid(fid):
        b = [(fid >> i) & 1 for i in range(6)]
        p0 = b[0] ^ b[1] ^ b[2] ^ b[4]
        p1 = ~(b[1] ^ b[3] ^ b[4] ^ b[5]) & 1
        return (fid & 0x3F) | (p0 << 6) | (p1 << 7)

    @staticmethod
    def chk(data, pid=None):
        s = pid if pid is not None else 0
        for v in data:
            s += v
            if s > 255:
                s -= 255
        return (~s) & 0xFF

    def _brk(self):
        self.sm.put(self._BRK)
        time.sleep_us(200)

    def _tx(self, b):
        self.sm.put(b ^ 0xFF)
        time.sleep_us(200)

    def _flush(self):
        while self.uart.any():
            self.uart.read()

    def _rx(self, n=11, tmo=50):
        t0 = time.ticks_ms()
        buf = b""
        while time.ticks_diff(time.ticks_ms(), t0) < tmo:
            if self.uart.any():
                c = self.uart.read()
                if c:
                    buf += c
                if len(buf) >= n:
                    break
        return buf

    def header(self, fid):
        p = self.pid(fid)
        self._flush()
        self._brk()
        self._tx(0x55)
        self._tx(p)
        return p

    def send(self, fid, data, enhanced=True):
        p = self.header(fid)
        for b in data:
            self._tx(b)
        self._tx(self.chk(data, p if enhanced else None))
        time.sleep_us(300)

    def recv(self, fid, tmo=50):
        p = self.header(fid)
        raw = self._rx(11, tmo)
        if not raw or len(raw) < 3:
            return None, False
        try:
            idx = raw.index(bytes([p]))
        except ValueError:
            return None, False
        pay = raw[idx + 1:]
        if len(pay) < 2:
            return None, False
        data = list(pay[:-1])
        return data, pay[-1] == self.chk(data, p)


# ================================================================
# Frame Tracker
# ================================================================

FRAME_IDS = [(0x23, "Light"), (0x29, "Env"), (0x30, "Rain")]


class Frame:
    """Track one frame: data, changes, per-byte statistics."""

    def __init__(self, fid, name):
        self.fid = fid
        self.name = name
        self.data = None
        self.prev = None
        self.count = 0
        self.fails = 0
        self.mn = [0xFF] * 8
        self.mx = [0x00] * 8
        self.uniq = [set() for _ in range(8)]

    def update(self, data):
        if data is None:
            self.fails += 1
            return
        self.prev = self.data
        self.data = list(data[:8])
        while len(self.data) < 8:
            self.data.append(0xFF)
        self.count += 1
        for i, b in enumerate(self.data):
            if b < self.mn[i]:
                self.mn[i] = b
            if b > self.mx[i]:
                self.mx[i] = b
            self.uniq[i].add(b)

    def changed_bytes(self):
        if not self.prev or not self.data:
            return []
        return [i for i in range(8) if self.prev[i] != self.data[i]]

    def hex_line(self):
        if not self.data:
            return "-- no data --"
        ch = self.changed_bytes()
        parts = []
        for i, b in enumerate(self.data):
            parts.append(">%02X<" % b if i in ch else " %02X " % b)
        return "".join(parts)

    def stats_table(self):
        lines = [
            "  Byte | Min  | Max  | Range | Uniq | Type",
            "  -----+------+------+-------+------+--------",
        ]
        for i in range(8):
            rng = self.mx[i] - self.mn[i]
            u = len(self.uniq[i])
            if u <= 1:
                t = "static"
            elif rng <= 0x0F:
                t = "counter?"
            elif u <= 4:
                t = "flags?"
            else:
                t = "DYNAMIC"
            lines.append("  [%d]  | 0x%02X | 0x%02X |  %3d  | %4d | %s" % (
                i, self.mn[i], self.mx[i], rng, u, t))
        return "\n".join(lines)

    def bit_table(self):
        """Bit-level analysis: which bits change across all samples."""
        if self.count < 2:
            return "  (need 2+ samples)"
        lines = [
            "  Byte | Pattern (. = always same, X = changes)",
            "  -----+------------------------------------------",
        ]
        for i in range(8):
            all_or = 0x00
            all_and = 0xFF
            for val in self.uniq[i]:
                all_or |= val
                all_and &= val
            varying = all_or ^ all_and
            pattern = ""
            for bit in range(7, -1, -1):
                if varying & (1 << bit):
                    pattern += "X"
                else:
                    pattern += str((all_and >> bit) & 1)
            lines.append("  [%d]  | %s  (mask=0x%02X)" % (i, pattern, varying))
        return "\n".join(lines)

    def reset(self):
        self.data = self.prev = None
        self.count = self.fails = 0
        self.mn = [0xFF] * 8
        self.mx = [0x00] * 8
        self.uniq = [set() for _ in range(8)]


# ================================================================
# Decoded summaries
# ================================================================

def _dec_light(d):
    if not d or len(d) < 6:
        return ""
    light = d[4] | (d[5] << 8)
    return "Light=%d(0x%04X) cnt=%X" % (light, light, d[0] & 0x0F)


def _dec_env(d):
    if not d or len(d) < 6:
        return ""
    t1 = d[2] * 0.5 - 40
    t2 = d[5] * 0.5 - 40
    return "T=%.1fC T2=%.1fC solar=0x%02X" % (t1, t2, d[0])


def _dec_rain(d):
    if not d or len(d) < 2:
        return ""
    tag = "FIR" if (d[0] or d[1]) else "idle"
    return "Rain=%s(%02X,%02X)" % (tag, d[0], d[1])


_DECODERS = {0x23: _dec_light, 0x29: _dec_env, 0x30: _dec_rain}


# ================================================================
# Live Monitor
# ================================================================

def live(cmd=None, data=None, interval=500):
    """
    Continuous live view of all frames with decoded values and
    byte-level change detection. Ctrl+C to stop.

    cmd:      Master command frame ID (None = passive mode)
    data:     8-byte payload for master command
    interval: Display refresh interval in ms (default 500)
    """
    lin = LIN()
    frames = {}
    for fid, name in FRAME_IDS:
        frames[fid] = Frame(fid, name)

    if cmd is not None and data is None:
        data = [0x81, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]

    mode = "PASSIVE"
    if cmd is not None:
        mode = "CMD 0x%02X [%s]" % (cmd, " ".join("%02X" % b for b in data))

    print("\n=== Live: %s | refresh %dms ===" % (mode, interval))
    print("Ctrl+C to stop\n")

    cycle = 0
    last_t = 0
    try:
        while True:
            # Master command (if configured)
            if cmd is not None:
                lin.send(cmd, data)
                time.sleep_ms(5)

            # Poll all slave frames
            for fid, name in FRAME_IDS:
                d, ok = lin.recv(fid, tmo=40)
                frames[fid].update(d if ok else None)
                time.sleep_ms(2)

            # Display at interval
            now = time.ticks_ms()
            if time.ticks_diff(now, last_t) >= interval:
                last_t = now
                print("--- #%d ---" % cycle)
                for fid, name in FRAME_IDS:
                    f = frames[fid]
                    dec = _DECODERS.get(fid, lambda x: "")(f.data)
                    print("  0x%02X %-5s: %s  n=%d" % (
                        fid, name, f.hex_line(), f.count))
                    if dec:
                        print("           > %s" % dec)
                print()

            cycle += 1
            time.sleep_ms(10)
    except KeyboardInterrupt:
        print("\nStopped. %d cycles." % cycle)
        for fid, name in FRAME_IDS:
            f = frames[fid]
            print("  0x%02X: %d ok, %d fail" % (fid, f.count, f.fails))


# ================================================================
# Master Command Injection
# ================================================================

def inject(fid=0x20, data=None, seconds=5):
    """
    Send master command for N seconds, watching frame 0x30 for changes.
    Reports any change immediately.

    fid:     Master command frame ID
    data:    8-byte payload (default: KL15 + AutoWiper + sensitivity 2)
    seconds: Test duration
    """
    if data is None:
        data = [0x81, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]

    lin = LIN()

    # Capture baseline
    print("Baseline (0x30)...")
    baseline = None
    for _ in range(5):
        d, ok = lin.recv(0x30, tmo=50)
        if d and ok:
            baseline = d
        time.sleep_ms(20)
    if baseline:
        print("  [%s]" % " ".join("%02X" % b for b in baseline))
    else:
        print("  no response")

    # Inject
    print("\nInjecting: 0x%02X [%s] for %ds..." % (
        fid, " ".join("%02X" % b for b in data), seconds))

    t0 = time.ticks_ms()
    n = 0
    changed = False
    while time.ticks_diff(time.ticks_ms(), t0) < seconds * 1000:
        lin.send(fid, data)
        time.sleep_ms(5)
        d, ok = lin.recv(0x30, tmo=40)
        if d and ok:
            if baseline and d != baseline:
                dt = time.ticks_diff(time.ticks_ms(), t0)
                print("  +%dms CHANGE: [%s]" % (
                    dt, " ".join("%02X" % b for b in d)))
                changed = True
                baseline = d
            elif n % 50 == 0:
                dt = time.ticks_diff(time.ticks_ms(), t0)
                print("  +%dms: [%s]" % (
                    dt, " ".join("%02X" % b for b in d)))
        # Keep sensor alive
        lin.recv(0x23, tmo=30)
        n += 1
        time.sleep_ms(15)

    if changed:
        print("\n>>> Frame 0x30 changed! This command has effect.")
    else:
        print("\nNo change on 0x30 after %d cycles." % n)


# ================================================================
# Timing Measurement
# ================================================================

def timing(samples=100):
    """
    Measure total frame slot time (header TX + slave response)
    for each frame ID. Reports min/max/avg in microseconds.
    """
    lin = LIN()

    print("\nMeasuring %d samples per frame..." % samples)
    print("  Frame  | Min     | Max     | Avg     | Fail")
    print("  -------+---------+---------+---------+-----")

    for fid, name in FRAME_IDS:
        times = []
        fails = 0
        for _ in range(samples):
            t0 = time.ticks_us()
            d, ok = lin.recv(fid, tmo=50)
            dt = time.ticks_diff(time.ticks_us(), t0)
            if d and ok:
                times.append(dt)
            else:
                fails += 1
            time.sleep_ms(5)

        if times:
            mn = min(times)
            mx = max(times)
            avg = sum(times) // len(times)
            print("  0x%02X %s | %5dus | %5dus | %5dus | %d" % (
                fid, name, mn, mx, avg, fails))
        else:
            print("  0x%02X %s | --- no response --- | %d" % (
                fid, name, fails))


# ================================================================
# Statistics Capture
# ================================================================

def stats(count=100, cmd=None, data=None):
    """
    Capture N poll cycles, then show per-byte statistics and bit analysis.

    count: Number of poll cycles
    cmd:   Optional master command frame ID
    data:  Optional master command payload
    """
    lin = LIN()
    frames = {}
    for fid, name in FRAME_IDS:
        frames[fid] = Frame(fid, name)

    if cmd is not None and data is None:
        data = [0x81, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]

    mode = "passive"
    if cmd is not None:
        mode = "CMD 0x%02X" % cmd

    print("\nCapturing %d cycles (%s)..." % (count, mode))
    for n in range(count):
        if cmd is not None:
            lin.send(cmd, data)
            time.sleep_ms(5)
        for fid, name in FRAME_IDS:
            d, ok = lin.recv(fid, tmo=40)
            frames[fid].update(d if ok else None)
            time.sleep_ms(2)
        if n % 25 == 0:
            print("  %d/%d..." % (n, count))
        time.sleep_ms(10)

    print("\n" + "=" * 55)
    print(" STATISTICS (%d cycles, %s)" % (count, mode))
    print("=" * 55)

    for fid, name in FRAME_IDS:
        f = frames[fid]
        if f.count > 0:
            print("\n0x%02X (%s) -- %d samples, %d fails:" % (
                fid, name, f.count, f.fails))
            print(f.stats_table())
            print()
            print(f.bit_table())
            vol = [i for i in range(8) if len(f.uniq[i]) > 1]
            sta = [i for i in range(8) if len(f.uniq[i]) <= 1]
            print("  Volatile: %s" % (vol if vol else "none"))
            print("  Static:   %s" % (sta if sta else "none"))
    print()


# ================================================================
# Stimulus Test
# ================================================================

def stimulus(name="test"):
    """
    Before/after comparison. Captures baseline, waits for user to
    apply stimulus (cover sensor, flashlight, heat, water), then
    captures again and shows byte-level differences.

    name: Label for the test
    """
    lin = LIN()
    frames_before = {}
    frames_after = {}
    for fid, fname in FRAME_IDS:
        frames_before[fid] = Frame(fid, fname)
        frames_after[fid] = Frame(fid, fname)

    # Baseline
    print("\nCapturing baseline (20 readings)...")
    for _ in range(20):
        for fid, _ in FRAME_IDS:
            d, ok = lin.recv(fid, tmo=40)
            frames_before[fid].update(d if ok else None)
            time.sleep_ms(2)
        time.sleep_ms(30)

    print("Baseline:")
    for fid, _ in FRAME_IDS:
        f = frames_before[fid]
        if f.data:
            print("  0x%02X: [%s]" % (
                fid, " ".join("%02X" % b for b in f.data)))

    print("\n>>> Apply stimulus: '%s' <<<" % name)
    print(">>> Press Enter when ready...")
    try:
        input()
    except Exception:
        time.sleep(5)

    # Post-stimulus
    print("Capturing post-stimulus (20 readings)...")
    for _ in range(20):
        for fid, _ in FRAME_IDS:
            d, ok = lin.recv(fid, tmo=40)
            frames_after[fid].update(d if ok else None)
            time.sleep_ms(2)
        time.sleep_ms(30)

    # Compare
    print("\n--- Stimulus '%s' Results ---" % name)
    for fid, fname in FRAME_IDS:
        fb = frames_before[fid]
        fa = frames_after[fid]
        if fb.data and fa.data:
            b = fb.data
            c = fa.data
            print("\n0x%02X (%s):" % (fid, fname))
            print("  Before: [%s]" % " ".join("%02X" % v for v in b))
            print("  After:  [%s]" % " ".join("%02X" % v for v in c))
            diff = [i for i in range(min(len(b), len(c)))
                    if b[i] != c[i]]
            if diff:
                print("  Changed bytes: %s" % diff)
                for i in diff:
                    delta = c[i] - b[i]
                    sign = "+" if delta > 0 else ""
                    print("    [%d]: 0x%02X -> 0x%02X (%s%d)" % (
                        i, b[i], c[i], sign, delta))
            else:
                print("  No changes")


# ================================================================
# Entry
# ================================================================

if __name__ == "__main__":
    live()
