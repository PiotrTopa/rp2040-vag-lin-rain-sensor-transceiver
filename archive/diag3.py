"""
Targeted DID Scanner — 81A 955 555 A

The sensor supports:
  - SID 0x21 ReadDataByLocalIdentifier (multi-frame, need CF fix)
  - SID 0x22 ReadDataByIdentifier (needs PCI=3 + 2-byte DID)
  - SID 0x2E WriteDataByIdentifier (NRC 0x31 = wrong DID, service exists!)

This script:
  1. Fixes multi-frame: debug CF retrieval for SID 0x21
  2. Sweeps SID 0x22 with correct PCI=3 across all DIDs
  3. Sweeps SID 0x2E to find writable DIDs
  4. After finding writable DIDs, tries coding values + checks FIR

Usage:
    from diag3 import *
    debug_mf()         # debug multi-frame on LID 0x01
    read22(0xF190)     # read UDS DID
    scan22()           # sweep all DIDs via SID 0x22
    scan2e()           # find writable DIDs via SID 0x2E
    run()              # full automated scan
"""

import rp2
from machine import Pin, UART
import time


# ================================================================
# PIO + LIN (self-contained, identical to other scripts)
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
# Low-level diagnostic I/O
# ================================================================

def _h(data):
    return " ".join("%02X" % b for b in data)


def _send_diag(lin, payload):
    """Send 8-byte frame on 0x3C with classic checksum."""
    while len(payload) < 8:
        payload.append(0xFF)
    lin.send(0x3C, payload[:8], enhanced=False)


def _recv_diag(lin, tmo=100):
    """Poll 0x3D and read 8-byte response (classic checksum)."""
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
    if pay[-1] == lin.chk(data):  # classic checksum
        return data
    return None


def _recv_diag_raw(lin, tmo=100):
    """Poll 0x3D and return raw bytes after PID, no checksum validation."""
    p = lin.header(0x3D)
    raw = lin._rx(11, tmo)
    if not raw or len(raw) < 3:
        return None
    try:
        idx = raw.index(bytes([p]))
    except ValueError:
        return None
    pay = raw[idx + 1:]
    if len(pay) < 1:
        return None
    return list(pay)


# ================================================================
# Transport Protocol with multi-frame support
# ================================================================

def diag_request(lin, sid, data, nad=None):
    """
    Send diagnostic request, handle SF/FF/CF response.
    Returns response payload bytes (starting with response SID) or None.
    """
    if nad is None:
        nad = lin.NAD

    # Build and send Single Frame request
    pci = 1 + len(data)  # SID + data bytes
    payload = [nad, pci, sid] + list(data)
    _send_diag(lin, payload)
    time.sleep_ms(15)

    # Read response
    resp = _recv_diag(lin, tmo=100)
    if not resp:
        return None

    if resp[0] != nad:
        return None

    pci_type = (resp[1] >> 4) & 0x0F
    pci_len = resp[1] & 0x0F

    # Single Frame (PCI type 0)
    if pci_type == 0:
        return resp[2:2 + pci_len]

    # First Frame (PCI type 1) — multi-frame
    if pci_type == 1:
        total_len = ((resp[1] & 0x0F) << 8) | resp[2]
        collected = list(resp[3:])  # up to 5 data bytes from FF

        # Send Flow Control
        time.sleep_ms(2)
        fc = [nad, 0x30, 0x00, 0x00]  # CTS, BS=0, STmin=0
        _send_diag(lin, fc)
        time.sleep_ms(5)

        # Read Consecutive Frames
        retries = 0
        while len(collected) < total_len and retries < 30:
            cf = _recv_diag(lin, tmo=120)
            if not cf:
                # Try without checksum validation
                cf = _recv_diag_raw(lin, tmo=120)
                if cf and len(cf) >= 2:
                    # Strip checksum byte if present
                    cf = cf[:-1] if len(cf) > 8 else cf
                else:
                    retries += 1
                    time.sleep_ms(5)
                    continue

            if cf[0] != nad:
                retries += 1
                continue

            cf_type = (cf[1] >> 4) & 0x0F
            if cf_type != 2:
                retries += 1
                continue

            # CF data: bytes 2..7
            collected.extend(cf[2:])
            retries = 0
            time.sleep_ms(2)

        return collected[:total_len]

    return None


def diag_raw(lin, payload_8, tmo=100):
    """Send raw 8 bytes on 0x3C, read raw response from 0x3D."""
    _send_diag(lin, list(payload_8))
    time.sleep_ms(15)
    return _recv_diag(lin, tmo)


# ================================================================
# Helpers
# ================================================================

NRC_NAMES = {
    0x10: "generalReject", 0x11: "serviceNotSupported",
    0x12: "subFunctionNotSupported", 0x13: "incorrectMessageLength",
    0x14: "responseTooLong", 0x21: "busyRepeatRequest",
    0x22: "conditionsNotCorrect", 0x24: "requestSequenceError",
    0x31: "requestOutOfRange", 0x33: "securityAccessDenied",
    0x35: "invalidKey", 0x36: "exceededNumberOfAttempts",
    0x72: "generalProgrammingFailure", 0x78: "responsePending",
    0x7E: "subFuncNotSupportedInSession", 0x7F: "serviceNotSupportedInSession",
}

_lin = None


def _get_lin():
    global _lin
    if _lin is None:
        _lin = LIN()
        for _ in range(10):
            d, ok = _lin.recv(0x23, tmo=80)
            if d and ok:
                break
            time.sleep_ms(50)
    return _lin


def check_fir(lin=None, cmd_id=0x20, cycles=30):
    if lin is None:
        lin = _get_lin()
    pay = [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00]
    for _ in range(cycles):
        lin.send(cmd_id, pay)
        time.sleep_ms(5)
        d, ok = lin.recv(0x30, tmo=40)
        if d and ok:
            if d[0] != 0 and d[0] < 0xFE and d[1] != 0 and d[1] < 0xFE:
                return True, d
        lin.recv(0x23, tmo=30)
        time.sleep_ms(10)
    return False, None


# ================================================================
# Debug: Multi-frame investigation
# ================================================================

def debug_mf(lid=0x01):
    """Debug multi-frame response for SID 0x21 (ReadDataByLocalIdentifier)."""
    lin = _get_lin()
    nad = lin.NAD

    print("\n=== Multi-frame debug for LID 0x%02X ===" % lid)

    # Send request
    payload = [nad, 0x02, 0x21, lid]
    _send_diag(lin, payload)
    time.sleep_ms(15)

    # Read First Frame
    print("\n1. Reading 0x3D after request...")
    resp = _recv_diag_raw(lin, tmo=150)
    if resp:
        print("   Raw (incl chk): [%s]" % _h(resp))
    else:
        print("   No response!")
        return

    # Parse FF
    if resp and len(resp) >= 8:
        data = resp[:-1]  # strip checksum
        print("   Data (no chk):  [%s]" % _h(data))
        pci_type = (data[1] >> 4) & 0x0F
        if pci_type == 1:
            total = ((data[1] & 0x0F) << 8) | data[2]
            print("   First Frame: total_len=%d, data=[%s]" % (total, _h(data[3:])))
        elif pci_type == 0:
            pci_len = data[1] & 0x0F
            print("   Single Frame: len=%d, data=[%s]" % (pci_len, _h(data[2:2 + pci_len])))
            return
        else:
            print("   Unknown PCI type: %d" % pci_type)

    # Try Flow Control
    print("\n2. Sending Flow Control...")
    fc = [nad, 0x30, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF]
    _send_diag(lin, fc)
    time.sleep_ms(10)

    # Poll 0x3D multiple times for Consecutive Frames
    print("\n3. Polling 0x3D for Consecutive Frames...")
    for i in range(10):
        cf = _recv_diag_raw(lin, tmo=150)
        if cf:
            print("   Poll %d: [%s]" % (i, _h(cf)))
            if len(cf) >= 2:
                ct = (cf[1] >> 4) & 0x0F if len(cf) > 1 else -1
                print("           PCI type=%d" % ct)
        else:
            print("   Poll %d: no response" % i)
        time.sleep_ms(10)

    # Try WITHOUT Flow Control — maybe slave auto-sends CFs
    print("\n4. Retry: send request, skip FC, just poll 0x3D...")
    payload = [nad, 0x02, 0x21, lid]
    _send_diag(lin, payload)
    time.sleep_ms(15)

    for i in range(10):
        cf = _recv_diag_raw(lin, tmo=150)
        if cf:
            print("   Poll %d: [%s]" % (i, _h(cf)))
        else:
            print("   Poll %d: no response" % i)
        time.sleep_ms(5)

    # Try with enhanced checksum on FC
    print("\n5. Retry: FC with enhanced checksum...")
    payload = [nad, 0x02, 0x21, lid]
    _send_diag(lin, payload)
    time.sleep_ms(15)
    ff = _recv_diag_raw(lin, tmo=150)
    if ff:
        print("   FF: [%s]" % _h(ff))

    # Send FC with enhanced checksum (include PID in checksum)
    p3c = lin.pid(0x3C)
    fc_data = [nad, 0x30, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF]
    lin.header(0x3C)  # send break+sync+PID only
    # manually TX data + enhanced checksum
    for b in fc_data:
        lin._tx(b)
    lin._tx(lin.chk(fc_data, p3c))
    time.sleep_ms(10)

    for i in range(5):
        cf = _recv_diag_raw(lin, tmo=150)
        if cf:
            print("   CF %d: [%s]" % (i, _h(cf)))
        else:
            print("   CF %d: no response" % i)
        time.sleep_ms(5)


# ================================================================
# SID 0x22: ReadDataByIdentifier (proper 2-byte DID format)
# ================================================================

def read22(did, verbose=True):
    """Read UDS DID via SID 0x22. DID is 16-bit."""
    lin = _get_lin()
    dh = (did >> 8) & 0xFF
    dl = did & 0xFF
    resp = diag_request(lin, 0x22, [dh, dl])
    if resp and len(resp) >= 1 and resp[0] == 0x62:
        data = resp[3:] if len(resp) > 3 else resp[1:]
        if verbose:
            print("DID 0x%04X: [%s]" % (did, _h(data)))
        return data
    elif resp and len(resp) >= 3 and resp[0] == 0x7F:
        nrc = resp[2]
        if verbose:
            print("DID 0x%04X: NRC 0x%02X (%s)" % (
                did, nrc, NRC_NAMES.get(nrc, "?")))
        return None
    else:
        if verbose and resp:
            print("DID 0x%04X: [%s]" % (did, _h(resp)))
        return None


def scan22(start=0x0000, end=0xFFFF, step=1):
    """Sweep SID 0x22 across DID range. Only prints hits."""
    lin = _get_lin()
    hits = {}
    total = (end - start) // step + 1
    count = 0

    print("Scanning SID 0x22 DIDs 0x%04X-0x%04X (%d DIDs)..." % (
        start, end, total))

    did = start
    while did <= end:
        dh = (did >> 8) & 0xFF
        dl = did & 0xFF
        resp = diag_request(lin, 0x22, [dh, dl])

        if resp and len(resp) >= 1:
            if resp[0] == 0x62:  # positive
                data = resp[3:] if len(resp) > 3 else resp[1:]
                print("  HIT DID 0x%04X: [%s]" % (did, _h(data)))
                hits[did] = data
            elif resp[0] == 0x7F and len(resp) >= 3:
                nrc = resp[2]
                # Log interesting NRCs (not 0x31=outOfRange, 0x11=notSupported)
                if nrc not in (0x31, 0x11, 0x12):
                    print("  DID 0x%04X: NRC 0x%02X (%s)" % (
                        did, nrc, NRC_NAMES.get(nrc, "?")))
                    hits[did] = ("NRC", nrc)

        count += 1
        if count % 256 == 0:
            print("  ... %d/%d (0x%04X)" % (count, total, did))
        time.sleep_ms(5)
        did += step

    print("Done. %d hits." % len(hits))
    return hits


def scan22_fast():
    """Quick scan of common VAG DID ranges."""
    lin = _get_lin()
    hits = {}

    # VAG-typical DID ranges
    ranges = [
        (0x0100, 0x01FF, "identification"),
        (0x0200, 0x02FF, "coding"),
        (0x0300, 0x03FF, "coding2"),
        (0x0400, 0x04FF, "adaptation"),
        (0x0500, 0x05FF, "adaptation2"),
        (0x0600, 0x06FF, "measured"),
        (0x0700, 0x07FF, "measured2"),
        (0xF100, 0xF1FF, "UDS_ident"),
        (0xF000, 0xF0FF, "UDS_config"),
        (0x2000, 0x20FF, "IoControl"),
        (0xFD00, 0xFDFF, "VW_special"),
        (0xFE00, 0xFEFF, "VW_special2"),
    ]

    for start, end, name in ranges:
        print("\nRange 0x%04X-0x%04X (%s):" % (start, end, name))
        found = 0
        for did in range(start, end + 1):
            dh = (did >> 8) & 0xFF
            dl = did & 0xFF
            resp = diag_request(lin, 0x22, [dh, dl])
            if resp and len(resp) >= 1:
                if resp[0] == 0x62:
                    data = resp[3:] if len(resp) > 3 else resp[1:]
                    print("  HIT 0x%04X: [%s]" % (did, _h(data)))
                    hits[did] = data
                    found += 1
                elif resp[0] == 0x7F and len(resp) >= 3:
                    nrc = resp[2]
                    if nrc not in (0x31, 0x11, 0x12, 0x13):
                        print("  NRC 0x%04X: 0x%02X (%s)" % (
                            did, nrc, NRC_NAMES.get(nrc, "?")))
                        hits[did] = ("NRC", nrc)
                        found += 1
            time.sleep_ms(5)
        if found == 0:
            print("  (none)")

    print("\nTotal hits: %d" % len(hits))
    return hits


# ================================================================
# SID 0x2E: WriteDataByIdentifier (find writable DIDs)
# ================================================================

def write2e(did, data, verbose=True):
    """Write UDS DID via SID 0x2E."""
    lin = _get_lin()
    dh = (did >> 8) & 0xFF
    dl = did & 0xFF
    payload = [dh, dl] + list(data)
    resp = diag_request(lin, 0x2E, payload)
    if resp and len(resp) >= 1 and resp[0] == 0x6E:
        if verbose:
            print("WRITE OK DID 0x%04X data=[%s] resp=[%s]" % (
                did, _h(data), _h(resp)))
        return True
    elif resp and len(resp) >= 3 and resp[0] == 0x7F:
        nrc = resp[2]
        if verbose:
            print("WRITE FAIL DID 0x%04X: NRC 0x%02X (%s)" % (
                did, nrc, NRC_NAMES.get(nrc, "?")))
        return False
    else:
        if verbose:
            if resp:
                print("WRITE DID 0x%04X: [%s]" % (did, _h(resp)))
            else:
                print("WRITE DID 0x%04X: no response" % did)
        return False


def scan2e(start=0x0000, end=0xFFFF, step=1):
    """Find writable DIDs. Tries writing [0x00] to each DID.
    NRC 0x31 = wrong DID, skip. Anything else = interesting."""
    lin = _get_lin()
    hits = {}
    total = (end - start) // step + 1
    count = 0

    print("Scanning SID 0x2E writable DIDs 0x%04X-0x%04X..." % (start, end))

    did = start
    while did <= end:
        dh = (did >> 8) & 0xFF
        dl = did & 0xFF
        # Try writing single byte 0x00
        resp = diag_request(lin, 0x2E, [dh, dl, 0x00])

        if resp and len(resp) >= 1:
            if resp[0] == 0x6E:  # positive
                print("  WRITABLE! DID 0x%04X resp=[%s]" % (did, _h(resp)))
                hits[did] = ("WRITE_OK", resp)
            elif resp[0] == 0x7F and len(resp) >= 3:
                nrc = resp[2]
                if nrc == 0x31:
                    pass  # requestOutOfRange — DID doesn't exist
                elif nrc == 0x13:
                    # incorrectMessageLength — DID exists, wrong data length!
                    print("  DID 0x%04X: NRC 0x13 (wrong length) — EXISTS!" % did)
                    hits[did] = ("WRONG_LEN", nrc)
                elif nrc == 0x33:
                    # securityAccessDenied — DID exists, needs unlock!
                    print("  DID 0x%04X: NRC 0x33 (security) — EXISTS!" % did)
                    hits[did] = ("SECURITY", nrc)
                elif nrc == 0x22:
                    # conditionsNotCorrect
                    print("  DID 0x%04X: NRC 0x22 (conditions) — EXISTS!" % did)
                    hits[did] = ("CONDITIONS", nrc)
                elif nrc == 0x72:
                    # generalProgrammingFailure
                    print("  DID 0x%04X: NRC 0x72 (prog fail) — EXISTS!" % did)
                    hits[did] = ("PROG_FAIL", nrc)
                elif nrc not in (0x11, 0x12):
                    print("  DID 0x%04X: NRC 0x%02X (%s)" % (
                        did, nrc, NRC_NAMES.get(nrc, "?")))
                    hits[did] = ("NRC", nrc)

        count += 1
        if count % 256 == 0:
            print("  ... %d/%d (0x%04X)" % (count, total, did))
        time.sleep_ms(5)
        did += step

    print("Done. %d interesting DIDs." % len(hits))
    return hits


def scan2e_fast():
    """Quick scan of common VAG coding DIDs with various data lengths."""
    lin = _get_lin()
    hits = {}

    ranges = [
        (0x0100, 0x01FF, "identification"),
        (0x0200, 0x02FF, "coding"),
        (0x0300, 0x03FF, "coding2"),
        (0x0400, 0x04FF, "adaptation"),
        (0x0500, 0x05FF, "adaptation2"),
        (0x0600, 0x06FF, "measured"),
        (0xF100, 0xF1FF, "UDS_ident"),
        (0xF000, 0xF0FF, "UDS_config"),
        (0xFD00, 0xFDFF, "VW_special"),
        (0xFE00, 0xFEFF, "VW_special2"),
    ]

    data_patterns = [
        [0x00],
        [0x01],
        [0x00, 0x00],
        [0x00, 0x00, 0x00],
        [0x01, 0x00, 0x00],
    ]

    for start, end, name in ranges:
        print("\nRange 0x%04X-0x%04X (%s):" % (start, end, name))
        found = 0
        for did in range(start, end + 1):
            dh = (did >> 8) & 0xFF
            dl = did & 0xFF
            for pat in data_patterns:
                resp = diag_request(lin, 0x2E, [dh, dl] + pat)
                if resp and len(resp) >= 1:
                    if resp[0] == 0x6E:
                        print("  WRITE OK! DID 0x%04X data=[%s]" % (
                            did, _h(pat)))
                        hits[did] = ("WRITE_OK", pat, resp)
                        found += 1
                        # Check FIR!
                        active, rd = check_fir(lin, cycles=15)
                        if active:
                            print("  *** FIR ACTIVATED! DID=0x%04X ***" % did)
                        break  # found working length, move on
                    elif resp[0] == 0x7F and len(resp) >= 3:
                        nrc = resp[2]
                        if nrc not in (0x31, 0x11, 0x12):
                            if nrc == 0x13 and pat == data_patterns[0]:
                                # Wrong length on first try — continue to next length
                                continue
                            elif nrc == 0x13:
                                continue  # keep trying lengths
                            print("  DID 0x%04X: NRC 0x%02X (%s) data=[%s]" % (
                                did, nrc, NRC_NAMES.get(nrc, "?"), _h(pat)))
                            hits[did] = ("NRC", nrc, pat)
                            found += 1
                            break
                time.sleep_ms(3)
        if found == 0 and (start & 0xFF00) not in (0x0100, 0x0600):
            pass  # quiet for less interesting ranges

    print("\nTotal interesting: %d" % len(hits))
    return hits


# ================================================================
# Full automated scan
# ================================================================

def run():
    """Full automated diagnostic scan targeting 0x22/0x2E."""
    lin = _get_lin()

    print("\n" + "=" * 64)
    print(" TARGETED DID SCAN — 81A 955 555 A (NAD=0x02)")
    print("=" * 64)

    findings = []

    # ----------------------------------------------------------
    # PHASE 1: Debug multi-frame on SID 0x21
    # ----------------------------------------------------------
    print("\nPHASE 1: Multi-frame debug")
    debug_mf(0x01)

    # ----------------------------------------------------------
    # PHASE 2: Full DID read sweep via SID 0x22
    # ----------------------------------------------------------
    print("\nPHASE 2: SID 0x22 DID read sweep (common ranges)")
    read_hits = scan22_fast()
    for did, data in read_hits.items():
        findings.append(("READ_22", did, data))

    # If no fast hits, do full sweep
    if not read_hits:
        print("\n  No hits in common ranges. Full sweep 0x0000-0xFFFF...")
        read_hits = scan22(0x0000, 0xFFFF)
        for did, data in read_hits.items():
            findings.append(("READ_22_FULL", did, data))

    # ----------------------------------------------------------
    # PHASE 3: Find writable DIDs via SID 0x2E
    # ----------------------------------------------------------
    print("\nPHASE 3: SID 0x2E writable DID sweep (common ranges)")
    write_hits = scan2e_fast()
    for did, info in write_hits.items():
        findings.append(("WRITE_2E", did, info))

    # If no fast hits, sweep wider
    if not write_hits:
        print("\n  No hits in common ranges. Full sweep 0x0000-0xFFFF...")
        write_hits = scan2e(0x0000, 0xFFFF)
        for did, info in write_hits.items():
            findings.append(("WRITE_2E_FULL", did, info))

    # ----------------------------------------------------------
    # PHASE 4: If we found writable DIDs, try coding values + check FIR
    # ----------------------------------------------------------
    writable_dids = [did for did, info in write_hits.items()
                     if isinstance(info, tuple) and info[0] == "WRITE_OK"]

    if writable_dids:
        print("\nPHASE 4: Coding values on writable DIDs")
        coding_values = [
            [0x00], [0x01], [0x02], [0x03], [0x07], [0x0F],
            [0x1F], [0x3F], [0x7F], [0xFF],
            [0x80], [0x81], [0xFE],
            [0x00, 0x00], [0x00, 0x01], [0x01, 0x00], [0x01, 0x01],
            [0xFF, 0xFF], [0x03, 0x30], [0x03, 0x4D],
            [0x00, 0x00, 0x00], [0x01, 0x00, 0x00], [0xFF, 0xFF, 0xFF],
        ]

        for did in writable_dids:
            dh = (did >> 8) & 0xFF
            dl = did & 0xFF
            print("\n  DID 0x%04X:" % did)
            for val in coding_values:
                resp = diag_request(lin, 0x2E, [dh, dl] + val)
                if resp and len(resp) >= 1 and resp[0] == 0x6E:
                    print("    WROTE [%s]" % _h(val))
                    findings.append(("CODED", did, val))
                    # Check FIR
                    active, rd = check_fir(lin, cycles=20)
                    if active:
                        print("    *** FIR ACTIVATED! ***")
                        findings.append(("FIR_ACTIVE", did, val, rd))
                time.sleep_ms(5)

    # ----------------------------------------------------------
    # PHASE 5: Try different SID 0x2E framing on known-good NAD
    # For sensors that expect non-standard PCI
    # ----------------------------------------------------------
    if not writable_dids:
        print("\nPHASE 5: Alternative 0x2E framing attempts")
        nad = lin.NAD
        # Try various PCI values with DID 0x0330 (common VAG coding DID)
        test_dids = [0x0330, 0x0600, 0x0100, 0x0200, 0xF198, 0xF199]
        for did in test_dids:
            dh = (did >> 8) & 0xFF
            dl = did & 0xFF
            for pci in range(0x03, 0x08):
                for val in ([0x01], [0x00], [0xFF], [0x01, 0x00], [0x00, 0x01]):
                    data_field = [dh, dl] + val
                    # Manually construct frame with explicit PCI
                    payload = [nad, pci, 0x2E] + data_field
                    _send_diag(lin, payload)
                    time.sleep_ms(15)
                    resp = _recv_diag(lin, tmo=100)
                    if resp and resp[0] == nad:
                        inner = resp[2:]
                        if len(inner) >= 1 and inner[0] == 0x6E:
                            print("  WRITE OK! DID=0x%04X PCI=%d val=[%s]" % (
                                did, pci, _h(val)))
                            findings.append(("ALT_WRITE", did, pci, val, resp))
                            active, rd = check_fir(lin)
                            if active:
                                print("  *** FIR ACTIVATED! ***")
                        elif len(inner) >= 3 and inner[0] == 0x7F:
                            nrc = inner[2]
                            if nrc not in (0x31, 0x11, 0x12, 0x13):
                                print("  DID=0x%04X PCI=%d NRC=0x%02X" % (
                                    did, pci, nrc))
                    time.sleep_ms(3)

    # ----------------------------------------------------------
    # SUMMARY
    # ----------------------------------------------------------
    print("\n" + "=" * 64)
    print(" SCAN COMPLETE")
    print("=" * 64)

    print("\nAll findings (%d):" % len(findings))
    for f in findings:
        tag = f[0]
        if "FIR" in tag:
            print("  >>> %s: %s" % (tag, str(f[1:])))
        elif "WRITE" in tag or "CODE" in tag:
            print("  +++ %s: %s" % (tag, str(f[1:])))
        else:
            print("  ... %s: %s" % (tag, str(f[1:])))

    print("=" * 64)
    return findings


if __name__ == "__main__":
    run()
