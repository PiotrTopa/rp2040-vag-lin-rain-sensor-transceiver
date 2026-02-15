"""
Combo Fuzzer — 81A 955 555 A

KNOWN FACTS:
  NAD = 0x02, KWP2000-style, multi-frame auto-sends (no FC needed)
  SID 0x21: ReadDataByLocalIdentifier (32 LIDs, 0x00-0x1F)
  SID 0x22: ReadDataByIdentifier (7 DIDs at 0x30 spacing)
  SID 0x2E: WriteDataByIdentifier — DID 0x0611 is writable (3 bytes)
  0x0611 current value: [02 00 5D]

STRATEGY:
  1. Read full multi-frame LID data (no FC, just poll)
  2. Find ALL DIDs (fine-grained scan around known region)
  3. Read 0x0611 coding, write coding value, then sweep master commands
  4. Systematically explore coding bits + master cmd combos

Usage:
    from diag4 import *
    read_lid(0x01)        # full multi-frame read
    read_all_lids()       # read LIDs 0x00-0x1F with MF
    scan_dids()           # fine-grained DID scan
    read_coding()         # read current 0x0611 value
    write_coding(data)    # write 3-byte coding
    combo()               # main: coding × master command sweep
    run()                 # full automated scan
"""

import rp2
from machine import Pin, UART
import time


# ================================================================
# PIO + LIN
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
    NAD = 0x02

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
            return None
        try:
            idx = raw.index(bytes([p]))
        except ValueError:
            return None
        pay = raw[idx + 1:]
        if len(pay) < 2:
            return None
        data = list(pay[:-1])
        if pay[-1] == self.chk(data, p):
            return data
        return None


# ================================================================
# Diagnostic primitives
# ================================================================

def _h(data):
    return " ".join("%02X" % b for b in data)


def _send_3c(lin, payload):
    while len(payload) < 8:
        payload.append(0xFF)
    lin.send(0x3C, payload[:8], enhanced=False)


def _recv_3d(lin, tmo=100):
    p = lin.header(0x3D)
    raw = lin._rx(11, tmo)
    if not raw or len(raw) < 3:
        return None
    try:
        idx = raw.index(bytes([p]))
    except ValueError:
        return None
    pay = raw[idx + 1:]
    if len(pay) < 2:
        return None
    data = list(pay[:-1])
    if pay[-1] == lin.chk(data):
        return data
    return None


def _diag(lin, sid, data, nad=None):
    """Send diagnostic request, handle SF and auto-CF multi-frame (no FC)."""
    if nad is None:
        nad = lin.NAD

    pci = 1 + len(data)
    _send_3c(lin, [nad, pci, sid] + list(data))
    time.sleep_ms(15)

    resp = _recv_3d(lin, tmo=100)
    if not resp or resp[0] != nad:
        return None

    pci_type = (resp[1] >> 4) & 0x0F

    # Single Frame
    if pci_type == 0:
        pci_len = resp[1] & 0x0F
        return resp[2:2 + pci_len]

    # First Frame — sensor auto-sends CFs, NO Flow Control needed
    if pci_type == 1:
        total_len = ((resp[1] & 0x0F) << 8) | resp[2]
        collected = list(resp[3:])  # up to 5 bytes from FF

        retries = 0
        while len(collected) < total_len and retries < 20:
            cf = _recv_3d(lin, tmo=120)
            if not cf or cf[0] != nad:
                retries += 1
                time.sleep_ms(5)
                continue
            cf_type = (cf[1] >> 4) & 0x0F
            if cf_type != 2:
                retries += 1
                continue
            collected.extend(cf[2:])
            retries = 0
            time.sleep_ms(2)

        return collected[:total_len]

    return None


# ================================================================
# Global LIN instance
# ================================================================

_lin = None


def _get():
    global _lin
    if _lin is None:
        _lin = LIN()
        for _ in range(10):
            d = _lin.recv(0x23, tmo=80)
            if d:
                break
            time.sleep_ms(50)
    return _lin


# ================================================================
# LID reading (SID 0x21) with proper multi-frame
# ================================================================

def read_lid(lid, verbose=True):
    """Read local identifier with multi-frame support."""
    lin = _get()
    resp = _diag(lin, 0x21, [lid])
    if resp and len(resp) >= 2 and resp[0] == 0x61:
        data = resp[2:]
        if verbose:
            # Try ASCII decode
            asc = ""
            for b in data:
                if 0x20 <= b <= 0x7E:
                    asc += chr(b)
                else:
                    asc += "."
            print("LID 0x%02X [%2d]: [%s]  %s" % (lid, len(data), _h(data), asc))
        return data
    elif resp and len(resp) >= 3 and resp[0] == 0x7F:
        nrc = resp[2]
        if verbose:
            print("LID 0x%02X: NRC 0x%02X" % (lid, nrc))
    elif verbose:
        print("LID 0x%02X: %s" % (lid, _h(resp) if resp else "no response"))
    return None


def read_all_lids():
    """Read all 32 local identifiers with full multi-frame."""
    results = {}
    print("Reading LIDs 0x00-0x1F (multi-frame)...")
    for lid in range(0x20):
        data = read_lid(lid)
        if data is not None:
            results[lid] = data
        time.sleep_ms(10)
    print("Found %d LIDs." % len(results))
    return results


# ================================================================
# DID reading (SID 0x22)
# ================================================================

def read_did(did, verbose=True):
    """Read DID via SID 0x22."""
    lin = _get()
    dh = (did >> 8) & 0xFF
    dl = did & 0xFF
    resp = _diag(lin, 0x22, [dh, dl])
    if resp and len(resp) >= 3 and resp[0] == 0x62:
        data = resp[3:]
        if verbose:
            asc = ""
            for b in data:
                if 0x20 <= b <= 0x7E:
                    asc += chr(b)
                else:
                    asc += "."
            print("DID 0x%04X [%2d]: [%s]  %s" % (did, len(data), _h(data), asc))
        return data
    return None


def scan_dids(start=0x0580, end=0x0800):
    """Fine-grained DID scan at step=1."""
    lin = _get()
    hits = {}
    print("Scanning DIDs 0x%04X-0x%04X..." % (start, end))
    for did in range(start, end + 1):
        dh = (did >> 8) & 0xFF
        dl = did & 0xFF
        resp = _diag(lin, 0x22, [dh, dl])
        if resp and len(resp) >= 3 and resp[0] == 0x62:
            data = resp[3:]
            asc = ""
            for b in data:
                asc += chr(b) if 0x20 <= b <= 0x7E else "."
            print("  0x%04X [%2d]: [%s]  %s" % (did, len(data), _h(data), asc))
            hits[did] = data

        # Also check write-ability
        if resp and len(resp) >= 3 and resp[0] == 0x62:
            wresp = _diag(lin, 0x2E, [dh, dl] + list(data))
            if wresp and len(wresp) >= 1 and wresp[0] == 0x6E:
                print("         ^ WRITABLE (echo-write OK)")

        time.sleep_ms(5)
        if (did - start) % 128 == 127:
            print("  ... 0x%04X" % did)

    print("Found %d DIDs." % len(hits))
    return hits


# ================================================================
# Coding DID 0x0611
# ================================================================

CODING_DID = 0x0611


def read_coding():
    """Read current coding value from DID 0x0611."""
    data = read_did(CODING_DID)
    if data:
        print("Coding: [%s] = byte[0]=0x%02X byte[1]=0x%02X byte[2]=0x%02X" % (
            _h(data), data[0], data[1], data[2]))
    return data


def write_coding(data, verbose=True):
    """Write 3-byte coding to DID 0x0611."""
    lin = _get()
    dh = (CODING_DID >> 8) & 0xFF
    dl = CODING_DID & 0xFF
    resp = _diag(lin, 0x2E, [dh, dl] + list(data))
    if resp and len(resp) >= 1 and resp[0] == 0x6E:
        if verbose:
            print("Coding written: [%s]" % _h(data))
        return True
    else:
        if verbose:
            print("Write failed: %s" % (_h(resp) if resp else "no response"))
        return False


def restore_coding():
    """Restore original coding [02 00 5D]."""
    return write_coding([0x02, 0x00, 0x5D])


# ================================================================
# FIR check (more thorough)
# ================================================================

def _check_fir(lin, cmd_fid=0x20, cmd_pay=None, cycles=40):
    """
    Send master command for N cycles, poll 0x30 each time.
    Returns (active, rain_data) where active means non-zero non-default.
    """
    if cmd_pay is None:
        cmd_pay = [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00]

    for _ in range(cycles):
        lin.send(cmd_fid, cmd_pay)
        time.sleep_ms(5)
        d = lin.recv(0x30, tmo=40)
        if d:
            b0, b1 = d[0], d[1]
            if b0 != 0 and b0 < 0xFE and b1 != 0 and b1 < 0xFE:
                return True, d
        lin.recv(0x23, tmo=20)
        time.sleep_ms(5)
    return False, None


# ================================================================
# COMBO FUZZER: coding value × master command
# ================================================================

# Master command candidates (frame IDs that might carry wiper commands)
MASTER_FIDS = [0x20, 0x21, 0x22, 0x10, 0x30, 0x08, 0x18, 0x28, 0x38]

# Master command payload patterns
MASTER_PAYLOADS = [
    [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00],
    [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00],
    [0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00],
    [0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    [0xC1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
]

# Coding value candidates — systematic bit exploration
CODING_VALUES = [
    # Original value
    [0x02, 0x00, 0x5D],
    # Single-bit toggles on byte[0] (from original 0x02)
    [0x03, 0x00, 0x5D],
    [0x06, 0x00, 0x5D],
    [0x0A, 0x00, 0x5D],
    [0x12, 0x00, 0x5D],
    [0x22, 0x00, 0x5D],
    [0x42, 0x00, 0x5D],
    [0x82, 0x00, 0x5D],
    # Byte[0] common values
    [0x00, 0x00, 0x5D],
    [0x01, 0x00, 0x5D],
    [0x04, 0x00, 0x5D],
    [0x07, 0x00, 0x5D],
    [0x0F, 0x00, 0x5D],
    [0xFF, 0x00, 0x5D],
    # Byte[2] toggles (from original 0x5D = 0101_1101)
    [0x02, 0x00, 0x5F],  # bit1 set
    [0x02, 0x00, 0x5C],  # bit0 clear
    [0x02, 0x00, 0x7D],  # bit5 set
    [0x02, 0x00, 0xDD],  # bit7 set
    [0x02, 0x00, 0xFF],
    [0x02, 0x00, 0x00],
    # Byte[1] (from original 0x00)
    [0x02, 0x01, 0x5D],
    [0x02, 0x02, 0x5D],
    [0x02, 0x04, 0x5D],
    [0x02, 0x08, 0x5D],
    [0x02, 0x10, 0x5D],
    [0x02, 0x20, 0x5D],
    [0x02, 0x40, 0x5D],
    [0x02, 0x80, 0x5D],
    [0x02, 0xFF, 0x5D],
    # All-same patterns
    [0x00, 0x00, 0x00],
    [0x01, 0x01, 0x01],
    [0xFF, 0xFF, 0xFF],
    [0x07, 0x07, 0x07],
    # Known VAG coding patterns
    [0x00, 0x03, 0x4D],
    [0x03, 0x03, 0x4D],
    [0x01, 0x00, 0x00],
    [0x00, 0x00, 0x01],
]


def combo(verbose=True):
    """
    Main combo fuzzer: for each coding value, write it, then sweep
    multiple master command frame IDs and payloads, checking FIR after each.
    """
    lin = _get()

    print("\n" + "=" * 64)
    print(" COMBO FUZZER: coding x master command")
    print(" %d coding values x %d frame IDs x %d payloads" % (
        len(CODING_VALUES), len(MASTER_FIDS), len(MASTER_PAYLOADS)))
    print(" Total combos: %d" % (
        len(CODING_VALUES) * len(MASTER_FIDS) * len(MASTER_PAYLOADS)))
    print("=" * 64)

    hits = []
    tested = 0
    total = len(CODING_VALUES) * len(MASTER_FIDS) * len(MASTER_PAYLOADS)

    for ci, coding in enumerate(CODING_VALUES):
        # Write coding
        ok = write_coding(coding, verbose=False)
        if not ok:
            if verbose:
                print("[%d/%d] Write [%s] FAILED, skip" % (
                    ci + 1, len(CODING_VALUES), _h(coding)))
            continue

        # Verify it stuck
        time.sleep_ms(20)
        readback = read_did(CODING_DID, verbose=False)
        if readback and readback != coding:
            if verbose:
                print("[%d/%d] [%s] readback=[%s] MISMATCH" % (
                    ci + 1, len(CODING_VALUES), _h(coding), _h(readback)))

        # Sweep master commands
        for fid in MASTER_FIDS:
            for pay in MASTER_PAYLOADS:
                active, rd = _check_fir(lin, cmd_fid=fid, cmd_pay=pay,
                                        cycles=25)
                tested += 1

                if active:
                    print("*** HIT! coding=[%s] fid=0x%02X pay=[%s] rain=[%s]" % (
                        _h(coding), fid, _h(pay), _h(rd)))
                    hits.append((coding, fid, pay, rd))

        if verbose and (ci + 1) % 4 == 0:
            print("  Progress: %d/%d codings, %d/%d combos" % (
                ci + 1, len(CODING_VALUES), tested, total))

    # Restore original coding
    print("\nRestoring original coding [02 00 5D]...")
    write_coding([0x02, 0x00, 0x5D], verbose=False)

    print("\n" + "=" * 64)
    if hits:
        print(" *** %d HITS FOUND! ***" % len(hits))
        for coding, fid, pay, rd in hits:
            print("  coding=[%s] fid=0x%02X pay=[%s]" % (
                _h(coding), fid, _h(pay)))
            print("    rain=[%s]" % _h(rd))
    else:
        print(" No FIR activation found in %d combos." % tested)
    print("=" * 64)
    return hits


# ================================================================
# Byte-level brute force on coding
# ================================================================

def brute_byte0():
    """Sweep byte[0] of coding (0x00-0xFF), keep byte[1-2] original."""
    lin = _get()
    print("Brute-forcing byte[0] of coding (256 values)...")
    for b0 in range(0x100):
        coding = [b0, 0x00, 0x5D]
        write_coding(coding, verbose=False)
        time.sleep_ms(10)

        # Check with default master command
        active, rd = _check_fir(lin, cycles=15)
        if active:
            print("*** HIT byte[0]=0x%02X! rain=[%s]" % (b0, _h(rd)))
            return b0, rd

        # Also try with fid 0x21
        active, rd = _check_fir(lin, cmd_fid=0x21, cycles=10)
        if active:
            print("*** HIT byte[0]=0x%02X fid=0x21! rain=[%s]" % (b0, _h(rd)))
            return b0, rd

        if b0 % 32 == 31:
            print("  ... byte[0] 0x%02X" % b0)

    write_coding([0x02, 0x00, 0x5D], verbose=False)
    print("No hit. Restored original coding.")
    return None


def brute_byte2():
    """Sweep byte[2] of coding (0x00-0xFF), keep byte[0-1] original."""
    lin = _get()
    print("Brute-forcing byte[2] of coding (256 values)...")
    for b2 in range(0x100):
        coding = [0x02, 0x00, b2]
        write_coding(coding, verbose=False)
        time.sleep_ms(10)

        active, rd = _check_fir(lin, cycles=15)
        if active:
            print("*** HIT byte[2]=0x%02X! rain=[%s]" % (b2, _h(rd)))
            return b2, rd

        active, rd = _check_fir(lin, cmd_fid=0x21, cycles=10)
        if active:
            print("*** HIT byte[2]=0x%02X fid=0x21! rain=[%s]" % (b2, _h(rd)))
            return b2, rd

        if b2 % 32 == 31:
            print("  ... byte[2] 0x%02X" % b2)

    write_coding([0x02, 0x00, 0x5D], verbose=False)
    print("No hit. Restored original coding.")
    return None


# ================================================================
# Full run
# ================================================================

def run():
    """Full automated scan."""
    lin = _get()

    print("\n" + "=" * 64)
    print(" FULL DIAGNOSTIC SCAN + COMBO FUZZER")
    print("=" * 64)

    # Phase 1: Read all LIDs with multi-frame
    print("\nPHASE 1: Read LIDs (multi-frame)")
    lids = read_all_lids()

    # Phase 2: Fine-grained DID scan
    print("\nPHASE 2: Fine DID scan (0x0580-0x0800)")
    dids = scan_dids(0x0580, 0x0800)

    # Phase 3: Current coding state
    print("\nPHASE 3: Current coding")
    read_coding()

    # Phase 4: Combo fuzzer
    print("\nPHASE 4: Combo fuzzer (coding x master)")
    hits = combo()

    # Phase 5: If no combo hit, brute-force individual bytes
    if not hits:
        print("\nPHASE 5: Brute-force byte[0]")
        r = brute_byte0()
        if not r:
            print("\nPHASE 6: Brute-force byte[2]")
            brute_byte2()

    # Final: restore
    print("\nRestoring original coding...")
    restore_coding()
    read_coding()

    print("\n" + "=" * 64)
    print(" DONE")
    print("=" * 64)


if __name__ == "__main__":
    run()
