"""
LIN Diagnostic Tool — 81A 955 555 A

KWP2000-style diagnostics over LIN transport.
Read measuring blocks (LIDs), DIDs, coding.
Live LID monitoring with change detection.

Usage (REPL):
    from diag import *
    monitor()             # Live LID dashboard (dynamic LIDs)
    monitor(full=True)    # All 32 LIDs live
    read_lid(0x04)        # Single LID
    read_all_lids()       # One-shot dump
    info()                # Sensor identification
    read_coding()         # Read coding DID 0x0611
"""

from lin import LIN, hex_str
import time


# ================================================================
# Configuration
# ================================================================

# Ambient light LID (decoded as combined value on top of monitor)
AMBIENT_LID = 0x02

# Forward light LID (inverted: dark=1023, bright=0)
FRONT_LID = 0x07

# Solar / sunlight intensity LID
SOLAR_LID = 0x08

# LIDs to monitor raw channels (excluding decoded ones)
DYNAMIC_LIDS = [0x05, 0x09, 0x0A, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15]

# Master command to keep sensor alive during diagnostics
_KEEPALIVE = [0x80, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00]


# ================================================================
# Global LIN instance
# ================================================================

_lin = None


def _get():
    global _lin
    if _lin is None:
        _lin = LIN()
        # Wait for sensor to respond
        for _ in range(10):
            d, ok = _lin.recv(0x23, tmo=80)
            if d and ok:
                break
            time.sleep_ms(50)
    return _lin


# ================================================================
# LID reading — SID 0x21 (ReadDataByLocalIdentifier)
# ================================================================
# Each LID returns 12 bytes = 4 channels of [formula, high, low].
# Formula 0x87 -> value = (high << 8) | low
#
# Channel pairs encode split 16-bit values:
#   Value_A = ((ch0 & 0xFF) << 8) | (ch1 & 0xFF)
#   Value_B = ((ch2 & 0xFF) << 8) | (ch3 & 0xFF)
# Even channel low byte = high byte of combined value (multiplier).
# Odd channel low byte  = low byte of combined value.

def decode_lid_channels(data):
    """Decode 12-byte LID response into 4 channel values.
    Returns list of (formula, value) tuples."""
    channels = []
    for i in range(0, len(data) - 2, 3):
        formula = data[i]
        val = (data[i + 1] << 8) | data[i + 2]
        channels.append((formula, val))
    return channels


def lid_values(channels):
    """Combine 4 raw channels into 2 decoded values.
    channels: list of (formula, raw_16bit) from decode_lid_channels.
    Returns (value_A, value_B)."""
    if len(channels) < 4:
        return None, None
    a = ((channels[0][1] & 0xFF) << 8) | (channels[1][1] & 0xFF)
    b = ((channels[2][1] & 0xFF) << 8) | (channels[3][1] & 0xFF)
    return a, b


def read_lid(lid, verbose=True):
    """Read local identifier with multi-frame support."""
    lin = _get()
    resp = lin.diag(0x21, [lid])
    if resp and len(resp) >= 2 and resp[0] == 0x61:
        data = resp[2:]
        if verbose:
            chs = decode_lid_channels(data)
            vals = " | ".join("ch%d=%5d(0x%04X)" % (i, v, v)
                              for i, (f, v) in enumerate(chs))
            print("LID 0x%02X: %s" % (lid, vals))
            print("     raw: [%s]" % hex_str(data))
        return data
    elif resp and len(resp) >= 3 and resp[0] == 0x7F:
        if verbose:
            print("LID 0x%02X: NRC 0x%02X" % (lid, resp[2]))
    elif verbose:
        print("LID 0x%02X: %s" % (lid, hex_str(resp) if resp else "no response"))
    return None


def read_all_lids(start=0x00, end=0x1F):
    """Read all local identifiers in range."""
    results = {}
    print("Reading LIDs 0x%02X-0x%02X..." % (start, end))
    for lid in range(start, end + 1):
        data = read_lid(lid)
        if data is not None:
            results[lid] = data
        time.sleep_ms(10)
    print("Found %d LIDs." % len(results))
    return results


# ================================================================
# DID reading — SID 0x22 (ReadDataByIdentifier)
# ================================================================

def read_did(did, verbose=True):
    """Read DID via SID 0x22."""
    lin = _get()
    dh = (did >> 8) & 0xFF
    dl = did & 0xFF
    resp = lin.diag(0x22, [dh, dl])
    if resp and len(resp) >= 3 and resp[0] == 0x62:
        data = resp[3:]
        if verbose:
            asc = ""
            for b in data:
                asc += chr(b) if 0x20 <= b <= 0x7E else "."
            print("DID 0x%04X [%2d]: [%s]  %s" % (
                did, len(data), hex_str(data), asc))
        return data
    return None


def scan_dids(start=0x0580, end=0x0800):
    """Scan DID range, report hits."""
    lin = _get()
    hits = {}
    print("Scanning DIDs 0x%04X-0x%04X..." % (start, end))
    for did in range(start, end + 1):
        dh = (did >> 8) & 0xFF
        dl = did & 0xFF
        resp = lin.diag(0x22, [dh, dl])
        if resp and len(resp) >= 3 and resp[0] == 0x62:
            data = resp[3:]
            asc = ""
            for b in data:
                asc += chr(b) if 0x20 <= b <= 0x7E else "."
            print("  0x%04X [%2d]: [%s]  %s" % (
                did, len(data), hex_str(data), asc))
            hits[did] = data
        time.sleep_ms(5)
        if (did - start) % 128 == 127:
            print("  ... 0x%04X" % did)
    print("Found %d DIDs." % len(hits))
    return hits


# ================================================================
# Coding — DID 0x0611
# ================================================================

CODING_DID = 0x0611


def read_coding():
    """Read current coding (DID 0x0611)."""
    data = read_did(CODING_DID)
    if data:
        print("Coding: [%s]" % hex_str(data))
    return data


def write_coding(data, verbose=True):
    """Write 3-byte coding to DID 0x0611."""
    lin = _get()
    dh = (CODING_DID >> 8) & 0xFF
    dl = CODING_DID & 0xFF
    resp = lin.diag(0x2E, [dh, dl] + list(data))
    if resp and len(resp) >= 1 and resp[0] == 0x6E:
        if verbose:
            print("Coding written: [%s]" % hex_str(data))
        return True
    if verbose:
        print("Write failed: %s" % (hex_str(resp) if resp else "no response"))
    return False


def restore_coding():
    """Restore default coding [02 00 5D]."""
    return write_coding([0x02, 0x00, 0x5D])


# ================================================================
# Quick info
# ================================================================

def info():
    """Print sensor identification and coding."""
    print("\n--- Sensor Info ---")
    read_did(0x0641)   # Part number
    read_did(0x06A1)   # Internal part
    read_did(0x06D1)   # HW version
    read_did(0x0701)   # Full ident string
    read_did(0x0731)   # Type
    read_coding()
    print()


# ================================================================
# Live LID Monitor
# ================================================================

def monitor(lids=None, full=False):
    """
    Live LID dashboard with change detection and min/max tracking.
    Ambient light (LID 0x02) shown as decoded value on top.
    Raw channels displayed below for diagnosis.

    Default: dynamic LIDs only (~500ms/sweep).
    full=True: all 32 LIDs (~3s/sweep).
    lids=[...]: custom list.
    Ctrl+C to stop. Returns (current, min, max) dicts.
    """
    lin = _get()

    if full:
        lids = list(range(0x20))
    elif lids is None:
        lids = list(DYNAMIC_LIDS)

    cur = {}     # lid -> [v0, v1, v2, v3]  raw channels
    prev = {}
    lo = {}      # lid -> [min0..min3]
    hi = {}      # lid -> [max0..max3]

    # Decoded ambient tracking (single LID)
    amb_v = 0
    amb_lo = 0
    amb_hi = 0
    amb_ok = False

    # Decoded front light tracking (inverted)
    fwd_v = 0
    fwd_lo = 0
    fwd_hi = 0
    fwd_ok = False

    # Decoded solar tracking
    sol_v = 0
    sol_lo = 0
    sol_hi = 0
    sol_ok = False

    sweep = 0
    BAR_W = 20

    # Build read list: decoded LIDs first, then display LIDs
    decoded = [AMBIENT_LID, FRONT_LID, SOLAR_LID]
    read_lids = list(decoded)
    for l in lids:
        if l not in read_lids:
            read_lids.append(l)

    print("LID Monitor -- ambient+front + %d LIDs -- Ctrl+C to stop" % len(lids))
    time.sleep_ms(500)

    try:
        while True:
            t0 = time.ticks_ms()
            sweep += 1

            # Keep sensor alive
            lin.send(0x20, _KEEPALIVE)
            time.sleep_ms(5)

            for lid in read_lids:
                resp = lin.diag(0x21, [lid])
                if resp and len(resp) >= 2 and resp[0] == 0x61:
                    data = resp[2:]
                    chs = decode_lid_channels(data)
                    vals = [v for _, v in chs]

                    # Ambient decode
                    if lid == AMBIENT_LID and len(chs) >= 4:
                        a, b = lid_values(chs)
                        if a is not None:
                            amb_v = (a + b) // 2
                            if not amb_ok:
                                amb_lo = amb_v
                                amb_hi = amb_v
                                amb_ok = True
                            else:
                                amb_lo = min(amb_lo, amb_v)
                                amb_hi = max(amb_hi, amb_v)

                    # Front light decode (inverted: 1023=dark, 0=bright)
                    if lid == FRONT_LID and len(chs) >= 4:
                        a, b = lid_values(chs)
                        if a is not None:
                            fwd_v = 1023 - (a + b) // 2
                            if not fwd_ok:
                                fwd_lo = fwd_v
                                fwd_hi = fwd_v
                                fwd_ok = True
                            else:
                                fwd_lo = min(fwd_lo, fwd_v)
                                fwd_hi = max(fwd_hi, fwd_v)

                    # Solar decode
                    if lid == SOLAR_LID and len(chs) >= 4:
                        a, b = lid_values(chs)
                        if a is not None:
                            sol_v = (a + b) // 2
                            if not sol_ok:
                                sol_lo = sol_v
                                sol_hi = sol_v
                                sol_ok = True
                            else:
                                sol_lo = min(sol_lo, sol_v)
                                sol_hi = max(sol_hi, sol_v)

                    # Raw channel tracking (display LIDs only)
                    if lid in lids:
                        prev[lid] = cur.get(lid)
                        cur[lid] = vals

                        if lid not in lo:
                            lo[lid] = list(vals)
                            hi[lid] = list(vals)
                        else:
                            for i, v in enumerate(vals):
                                lo[lid][i] = min(lo[lid][i], v)
                                hi[lid][i] = max(hi[lid][i], v)

                time.sleep_ms(2)

            dt = time.ticks_diff(time.ticks_ms(), t0)

            # Render dashboard
            print("\x1b[2J\x1b[H", end="")
            print("=== LID Monitor  #%d  %dms ===" % (sweep, dt))

            # --- Decoded light sensors ---
            if amb_ok:
                rng = amb_hi - amb_lo
                if rng > 0:
                    pos = (amb_v - amb_lo) * 30 // rng
                    bar = "#" * min(pos, 30) + "-" * max(30 - pos, 0)
                else:
                    bar = "=" * 30
                print("  AMBIENT %5d |%s| %d..%d" % (
                    amb_v, bar, amb_lo, amb_hi))
            if fwd_ok:
                rng = fwd_hi - fwd_lo
                if rng > 0:
                    pos = (fwd_v - fwd_lo) * 30 // rng
                    bar = "#" * min(pos, 30) + "-" * max(30 - pos, 0)
                else:
                    bar = "=" * 30
                print("  FRONT   %5d |%s| %d..%d" % (
                    fwd_v, bar, fwd_lo, fwd_hi))
            if sol_ok:
                rng = sol_hi - sol_lo
                if rng > 0:
                    pos = (sol_v - sol_lo) * 30 // rng
                    bar = "#" * min(pos, 30) + "-" * max(30 - pos, 0)
                else:
                    bar = "=" * 30
                print("  SOLAR   %5d |%s| %d..%d" % (
                    sol_v, bar, sol_lo, sol_hi))

            # --- Raw channel bars ---
            for lid in lids:
                if lid not in cur:
                    print("\n 0x%02X  --" % lid)
                    continue

                vals = cur[lid]
                p = prev.get(lid)
                n = min(4, len(vals))

                print()
                for i in range(n):
                    v = vals[i]
                    vlo = lo[lid][i]
                    vhi = hi[lid][i]
                    rng = vhi - vlo

                    ch = p is not None and len(p) > i and p[i] != v
                    mark = "*" if ch else " "

                    if rng > 0:
                        pos = (v - vlo) * BAR_W // rng
                        pos = min(pos, BAR_W)
                        bar = "#" * pos + "-" * (BAR_W - pos)
                        print(" %s0x%02X.%d %04X |%s| %04X..%04X" % (
                            mark, lid, i, v, bar, vlo, vhi))
                    else:
                        bar = "=" * BAR_W
                        print("  0x%02X.%d %04X |%s| (static)" % (
                            lid, i, v, bar))

            print("\n * = changed | Ctrl+C to stop")

    except KeyboardInterrupt:
        print("\nStopped after %d sweeps." % sweep)
        if amb_ok:
            print("\n--- Ambient light (LID 0x%02X) ---" % AMBIENT_LID)
            print("  value: %5d  (range %d..%d)" % (amb_v, amb_lo, amb_hi))
        if fwd_ok:
            print("\n--- Front light (LID 0x%02X, inverted) ---" % FRONT_LID)
            print("  value: %5d  (range %d..%d)" % (fwd_v, fwd_lo, fwd_hi))
        if sol_ok:
            print("\n--- Solar (LID 0x%02X) ---" % SOLAR_LID)
            print("  value: %5d  (range %d..%d)" % (sol_v, sol_lo, sol_hi))
        print("\n--- Dynamic channels ---")
        for lid in sorted(lo.keys()):
            for i in range(min(4, len(lo[lid]))):
                rng = hi[lid][i] - lo[lid][i]
                if rng > 0:
                    print("  LID 0x%02X ch%d: %d..%d (range %d)" % (
                        lid, i, lo[lid][i], hi[lid][i], rng))
        return cur, lo, hi


# ================================================================
# Machine-parseable stream (for chart.py on PC)
# ================================================================

def stream(lids=None):
    """Stream LID data as parseable lines for chart.py.

    Output format:
        STREAM:N         header (N = number of LIDs)
        LID:XX           one per LID (hex)
        D:XX:v0,v1,v2,v3  data line (lid hex, values decimal)
        S:N:ms           sweep number + elapsed ms
    """
    lin = _get()
    if lids is None:
        lids = list(DYNAMIC_LIDS)

    print("STREAM:%d" % len(lids))
    for lid in lids:
        print("LID:%02X" % lid)

    sweep = 0
    try:
        while True:
            t0 = time.ticks_ms()
            sweep += 1

            lin.send(0x20, _KEEPALIVE)
            time.sleep_ms(5)

            for lid in lids:
                resp = lin.diag(0x21, [lid])
                if resp and len(resp) >= 2 and resp[0] == 0x61:
                    data = resp[2:]
                    chs = decode_lid_channels(data)
                    vals = ",".join(str(v) for _, v in chs)
                    print("D:%02X:%s" % (lid, vals))
                time.sleep_ms(2)

            dt = time.ticks_diff(time.ticks_ms(), t0)
            print("S:%d:%d" % (sweep, dt))
    except KeyboardInterrupt:
        print("STOP")


# ================================================================
# One-shot dump
# ================================================================

def run():
    """Full diagnostic dump: info + all LIDs decoded."""
    info()
    print("--- Measuring Blocks ---")
    lids = read_all_lids()

    print("\n--- Channel Decode ---")
    for lid in sorted(lids.keys()):
        data = lids[lid]
        channels = decode_lid_channels(data)
        vals = " | ".join("ch%d=%5d(0x%04X)" % (i, v, v)
                          for i, (f, v) in enumerate(channels))
        print("LID 0x%02X: %s" % (lid, vals))


if __name__ == "__main__":
    monitor()
