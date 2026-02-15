"""
UDS Diagnostic Coding Fuzzer — 81A 955 555 A

Brute-forces LIN diagnostic frames (0x3C/0x3D) to find the correct
NAD, session, and coding to activate FIR rain detection.

Designed to run unattended for hours. Only prints discoveries.

Approach:
  Phase 1: Find NAD — ReadByIdentifier (B2) on all 127 NADs + wildcard
  Phase 2: Find session — DiagnosticSessionControl (10) with found NAD
  Phase 3: Try coding — WriteDataByIdentifier (2E) with common VAG DIDs
  Phase 4: Try AssignFrameIdRange (B7) to remap frame 0x30
  Phase 5: Brute-force coding bytes on promising DIDs
  After each attempt: check if 0x30 wakes up with master commands

Run:  just execute the file, or:
    from diag_fuzzer import *
    run_diag()
"""

import rp2
from machine import Pin, UART
import time


# ================================================================
# PIO + LIN (self-contained)
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
# Diagnostic Helpers
# ================================================================

def _h(data):
    return " ".join("%02X" % b for b in data)


def diag_send(lin, nad, pci, sid, data, pad=0xFF):
    """Send diagnostic master request on 0x3C (classic checksum)."""
    payload = [nad, pci, sid] + list(data)
    while len(payload) < 8:
        payload.append(pad)
    lin.send(0x3C, payload, enhanced=False)


def diag_recv(lin, tmo=100):
    """Read diagnostic slave response on 0x3D (classic checksum).
    Returns (raw_8_bytes, checksum_ok) or (None, False)."""
    p = lin.header(0x3D)
    raw = lin._rx(11, tmo)
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
    # 0x3D uses classic checksum
    ok = pay[-1] == lin.chk(data)
    return data, ok


def diag_request(lin, nad, pci, sid, data, pad=0xFF):
    """Send diagnostic request and read response. Returns response or None."""
    diag_send(lin, nad, pci, sid, data, pad)
    time.sleep_ms(20)
    resp, ok = diag_recv(lin, tmo=100)
    if resp and ok:
        return resp
    return None


def is_positive(resp, sid):
    """Check if diagnostic response is positive (SID + 0x40)."""
    if not resp or len(resp) < 3:
        return False
    return resp[2] == (sid + 0x40)


def is_negative(resp):
    """Check if diagnostic response is negative (0x7F)."""
    if not resp or len(resp) < 3:
        return False
    return resp[2] == 0x7F


def check_fir(lin, cmd_id=0x20, cycles=30):
    """
    Send master command for N cycles, check if 0x30 activates.
    Returns (active, rain_data).
    """
    pay = [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00]
    for _ in range(cycles):
        lin.send(cmd_id, pay)
        time.sleep_ms(5)
        d, ok = lin.recv(0x30, tmo=40)
        if d and ok:
            b0, b1 = d[0], d[1]
            if b0 != 0 and b0 < 0xFE and b1 != 0 and b1 < 0xFE:
                return True, d
        lin.recv(0x23, tmo=30)
        time.sleep_ms(10)
    d, ok = lin.recv(0x30, tmo=50)
    return False, d if d else None


def wake_sensor(lin):
    """Poll until sensor responds."""
    for _ in range(20):
        d, ok = lin.recv(0x23, tmo=80)
        if d and ok:
            return True
        lin.recv(0x29, tmo=40)
        lin.recv(0x30, tmo=40)
        time.sleep_ms(50)
    return False


# ================================================================
# Main Diagnostic Fuzzer
# ================================================================

def run_diag():
    lin = LIN()

    print("\n" + "=" * 64)
    print(" UDS DIAGNOSTIC FUZZER - 81A 955 555 A")
    print("=" * 64)

    if not wake_sensor(lin):
        print("Sensor not responding. Check wiring.")
        return

    print("Sensor alive.")

    # Collect all findings
    findings = []
    found_nads = []
    found_sessions = []
    fir_activated = False

    # ==============================================================
    # PHASE 1: Find NAD — sweep 0x01-0x7F with ReadByIdentifier
    # ==============================================================
    # LIN 2.x: NAD 0x7F = wildcard (all nodes respond)
    # SID B2 = ReadByIdentifier, ID=0 = LIN Product ID
    # PCI = 0x06 (6 bytes follow: SID + 5 data)
    print("\nPHASE 1: NAD discovery (ReadByIdentifier on 128 NADs)")

    # Try wildcard first
    for nad in [0x7F]:
        resp = diag_request(lin, nad, 0x06, 0xB2,
                            [0x00, 0xFF, 0x7F, 0xFF, 0x3F])
        if resp:
            print("  NAD 0x%02X RESPONDS: [%s]" % (nad, _h(resp)))
            findings.append(("NAD_WILDCARD", nad, resp))

    # Sweep all NADs
    for nad in range(0x01, 0x80):
        resp = diag_request(lin, nad, 0x06, 0xB2,
                            [0x00, 0xFF, 0x7F, 0xFF, 0x3F])
        if resp:
            print("  NAD 0x%02X RESPONDS: [%s]" % (nad, _h(resp)))
            found_nads.append(nad)
            findings.append(("NAD_FOUND", nad, resp))
        time.sleep_ms(10)

    # Also try with different SIDs to find ANY responding NAD
    if not found_nads:
        print("  No B2 response. Trying B0 (AssignNAD) probe...")
        for nad in range(0x01, 0x80):
            # SID B0 = AssignNAD, just see if anything comes back
            resp = diag_request(lin, nad, 0x06, 0xB0,
                                [0xFF, 0x7F, 0xFF, 0x3F, nad])
            if resp:
                print("  NAD 0x%02X responds to B0: [%s]" % (nad, _h(resp)))
                found_nads.append(nad)
                findings.append(("NAD_B0", nad, resp))
            time.sleep_ms(10)

    # Try node configuration SID B5 (ReadByIdentifier in node config)
    if not found_nads:
        print("  Trying raw SID sweep on NAD 0x01 and 0x7F...")
        for nad in (0x01, 0x7F, 0x22, 0x20, 0x30, 0x40, 0x10):
            for sid in (0x10, 0x22, 0x3E, 0xB2, 0xB5, 0xB7, 0x19, 0x27):
                resp = diag_request(lin, nad, 0x02, sid, [0x01])
                if resp:
                    print("  NAD=0x%02X SID=0x%02X: [%s]" % (
                        nad, sid, _h(resp)))
                    found_nads.append(nad)
                    findings.append(("NAD_SID_%02X" % sid, nad, resp))
                time.sleep_ms(10)

    if not found_nads:
        print("  No NAD found via standard methods.")
        print("  Trying broadcast raw frame on 0x3C...")
        # Last resort: send raw 8 bytes on 0x3C, see if 0x3D answers
        for b0 in range(0x01, 0x80):
            payload = [b0, 0x02, 0x10, 0x01, 0xFF, 0xFF, 0xFF, 0xFF]
            lin.send(0x3C, payload, enhanced=False)
            time.sleep_ms(15)
            resp, ok = diag_recv(lin, tmo=80)
            if resp and ok:
                print("  RAW b0=0x%02X: [%s]" % (b0, _h(resp)))
                found_nads.append(b0)
                findings.append(("NAD_RAW", b0, resp))
            time.sleep_ms(5)

    if found_nads:
        print("\n  Found NADs: %s" % (
            ", ".join("0x%02X" % n for n in found_nads)))
    else:
        print("\n  WARNING: No NAD found at all.")
        print("  Will try common NADs anyway...")
        found_nads = [0x01, 0x22, 0x7F, 0x20, 0x10, 0x30]

    # ==============================================================
    # PHASE 2: Session discovery
    # ==============================================================
    # SID 0x10 = DiagnosticSessionControl
    # Sub-functions: 01=default, 02=programming, 03=extended, 04+=vendor
    print("\nPHASE 2: Session discovery")

    for nad in found_nads:
        for session in range(0x01, 0x10):
            resp = diag_request(lin, nad, 0x02, 0x10, [session])
            if resp and is_positive(resp, 0x10):
                print("  NAD=0x%02X session=0x%02X: POSITIVE [%s]" % (
                    nad, session, _h(resp)))
                found_sessions.append((nad, session))
                findings.append(("SESSION", nad, session, resp))
            elif resp and is_negative(resp):
                nrc = resp[4] if len(resp) > 4 else 0
                # NRC means the NAD is alive! Just rejected that session.
                if nrc != 0x12:  # 0x12 = subFunctionNotSupported (normal)
                    print("  NAD=0x%02X session=0x%02X: NRC=0x%02X [%s]" % (
                        nad, session, nrc, _h(resp)))
                findings.append(("SESSION_NRC", nad, session, resp))
                if nad not in [n for n, _ in found_sessions]:
                    found_sessions.append((nad, 0x01))  # default works
            time.sleep_ms(10)

        # TesterPresent
        resp = diag_request(lin, nad, 0x02, 0x3E, [0x00])
        if resp:
            print("  NAD=0x%02X TesterPresent: [%s]" % (nad, _h(resp)))
            findings.append(("TESTER_PRESENT", nad, resp))
        time.sleep_ms(10)

    # ==============================================================
    # PHASE 3: Read all DIDs
    # ==============================================================
    # SID 0x22 = ReadDataByIdentifier  (PCI=0x03, data=[DID_H, DID_L])
    # SID 0xB2 = ReadByIdentifier (LIN-specific, ID 0-63)
    print("\nPHASE 3: DID discovery (ReadDataByIdentifier)")

    working_nads = list(set([n for n, _ in found_sessions])) if found_sessions else found_nads[:3]

    for nad in working_nads:
        # Enter extended session first
        diag_request(lin, nad, 0x02, 0x10, [0x03])
        time.sleep_ms(20)

        # Common VAG DIDs
        vag_dids = [
            0x0100, 0x0101, 0x0102, 0x0103, 0x0108, 0x0110,
            0xF100, 0xF101, 0xF102, 0xF110, 0xF111, 0xF150,
            0xF186, 0xF187, 0xF189, 0xF18A, 0xF18B, 0xF18C,
            0xF190, 0xF191, 0xF192, 0xF193, 0xF194, 0xF195,
            0xF1A0, 0xF1A1, 0xF1A2, 0xF1A3,
            0x0330, 0x034D, 0x0600, 0x0601,
            0x0200, 0x0201, 0x0202, 0x0203,
            0x0300, 0x0301, 0x0302, 0x0303,
        ]
        for did in vag_dids:
            dh = (did >> 8) & 0xFF
            dl = did & 0xFF
            resp = diag_request(lin, nad, 0x03, 0x22, [dh, dl])
            if resp and is_positive(resp, 0x22):
                print("  NAD=0x%02X DID=0x%04X: [%s]" % (
                    nad, did, _h(resp)))
                findings.append(("DID_READ", nad, did, resp))
            elif resp and is_negative(resp):
                nrc = resp[4] if len(resp) > 4 else 0
                if nrc not in (0x31, 0x12, 0x11):  # not "out of range"
                    print("  NAD=0x%02X DID=0x%04X NRC=0x%02X" % (
                        nad, did, nrc))
            time.sleep_ms(10)

            # Keep TesterPresent alive
            if vag_dids.index(did) % 10 == 9:
                diag_request(lin, nad, 0x02, 0x3E, [0x00])

    # ==============================================================
    # PHASE 4: Try coding / WriteDataByIdentifier
    # ==============================================================
    # SID 0x2E = WriteDataByIdentifier
    # Common coding DIDs for VAG RLS: 0x0330, 0x0600, 0x0100-0x0103
    print("\nPHASE 4: Coding attempts (WriteDataByIdentifier)")

    coding_dids = [
        0x0330, 0x034D, 0x0600, 0x0601,
        0x0100, 0x0101, 0x0102, 0x0103,
        0x0200, 0x0201, 0x0108, 0x0110,
        0xF100, 0xF101, 0xF110, 0xF198,
    ]

    coding_values = [
        [0x01],
        [0xFF],
        [0x03, 0x30, 0x4D],        # VAG standard coding
        [0x01, 0x00, 0x01],
        [0xFF, 0xFF, 0xFF],
        [0x00, 0x00, 0x01],
        [0x07],                      # all features enabled
        [0x0F],
        [0x80],
        [0x81],
        [0x01, 0x01],
        [0x01, 0x00],
        [0xFF, 0xFF],
    ]

    for nad in working_nads:
        for did in coding_dids:
            # Enter extended session
            diag_request(lin, nad, 0x02, 0x10, [0x03])
            time.sleep_ms(20)

            dh = (did >> 8) & 0xFF
            dl = did & 0xFF

            for val in coding_values:
                pci = len(val) + 3  # SID + DID_H + DID_L + data
                data = [dh, dl] + val
                resp = diag_request(lin, nad, pci, 0x2E, data)

                if resp and is_positive(resp, 0x2E):
                    print("  WRITE OK! NAD=0x%02X DID=0x%04X val=[%s] resp=[%s]" % (
                        nad, did, _h(val), _h(resp)))
                    findings.append(("WRITE_OK", nad, did, val, resp))

                    # Check if FIR activated!
                    time.sleep_ms(50)
                    active, rd = check_fir(lin)
                    if active:
                        print("  *** FIR ACTIVATED! DID=0x%04X val=[%s] ***" % (
                            did, _h(val)))
                        findings.append(("FIR_ACTIVE", nad, did, val, rd))
                        fir_activated = True

                elif resp and is_negative(resp):
                    nrc = resp[4] if len(resp) > 4 else 0
                    # Log interesting NRCs (not just "request out of range")
                    if nrc not in (0x31, 0x12, 0x11, 0x7F):
                        print("  NAD=0x%02X DID=0x%04X NRC=0x%02X (val=[%s])" % (
                            nad, did, nrc, _h(val)))
                        findings.append(("WRITE_NRC", nad, did, nrc, val))

                time.sleep_ms(10)

    # ==============================================================
    # PHASE 5: ECU Reset + master command check
    # ==============================================================
    print("\nPHASE 5: ECU Reset + master command re-check")

    for nad in working_nads:
        # Hard reset
        resp = diag_request(lin, nad, 0x02, 0x11, [0x01])
        if resp:
            print("  NAD=0x%02X ECUReset: [%s]" % (nad, _h(resp)))
            findings.append(("ECU_RESET", nad, resp))

    # Wait for sensor to restart
    time.sleep(2)
    wake_sensor(lin)

    # Try master commands on 0x20, 0x21, 0x22 after coding
    print("  Checking FIR after reset...")
    for cmd_id in (0x20, 0x21, 0x22):
        active, rd = check_fir(lin, cmd_id=cmd_id, cycles=50)
        if active:
            print("  *** FIR WORKS with CMD 0x%02X! rain=[%s] ***" % (
                cmd_id, _h(rd)))
            findings.append(("FIR_POST_RESET", cmd_id, rd))
            fir_activated = True

    # ==============================================================
    # PHASE 6: LIN-specific node configuration
    # ==============================================================
    # Try AssignFrameIdRange (B7), ConditionalChangeNAD (B3),
    # SaveConfiguration (B6)
    print("\nPHASE 6: LIN node configuration services")

    for nad in working_nads:
        # AssignFrameIdRange: start_index, PID0..PID3
        # Try assigning frame 0x30 to a different slot
        for start in range(0, 8):
            data = [start, 0xFF, 0xFF, 0xFF, 0xFF]
            resp = diag_request(lin, nad, 0x06, 0xB7, data)
            if resp:
                print("  NAD=0x%02X B7 start=%d: [%s]" % (
                    nad, start, _h(resp)))
                findings.append(("ASSIGN_FRAME", nad, start, resp))
            time.sleep_ms(10)

        # SaveConfiguration
        resp = diag_request(lin, nad, 0x01, 0xB6, [])
        if resp:
            print("  NAD=0x%02X SaveConfig: [%s]" % (nad, _h(resp)))
            findings.append(("SAVE_CONFIG", nad, resp))
        time.sleep_ms(10)

    # ==============================================================
    # PHASE 7: Brute-force raw 0x3C payloads (if still no FIR)
    # ==============================================================
    if not fir_activated:
        print("\nPHASE 7: Raw 0x3C brute-force (byte[0]=NAD, byte[1]=PCI sweep)")
        nads_to_try = working_nads if working_nads else [0x01, 0x7F, 0x22]

        for nad in nads_to_try:
            for pci in range(0x01, 0x08):
                for sid in range(0x00, 0x100):
                    payload = [nad, pci, sid, 0x01, 0xFF, 0xFF, 0xFF, 0xFF]
                    lin.send(0x3C, payload, enhanced=False)
                    time.sleep_ms(15)
                    resp, ok = diag_recv(lin, tmo=60)
                    if resp and ok:
                        # Only log if it's not a simple negative
                        if not is_negative(resp) or (len(resp) > 4 and resp[4] not in (0x11, 0x12, 0x31)):
                            print("  NAD=0x%02X PCI=%d SID=0x%02X: [%s]" % (
                                nad, pci, sid, _h(resp)))
                            findings.append(("RAW_RESP", nad, pci, sid, resp))
                    time.sleep_ms(5)

                # After each SID sweep, check FIR
                active, rd = check_fir(lin, cycles=10)
                if active:
                    print("  *** FIR ACTIVATED after NAD=0x%02X PCI=%d ***" % (
                        nad, pci))
                    findings.append(("FIR_RAW", nad, pci, rd))
                    fir_activated = True
                    break
            if fir_activated:
                break

    # ==============================================================
    # SUMMARY
    # ==============================================================
    print("\n" + "=" * 64)
    print(" DIAGNOSTIC FUZZER COMPLETE")
    print("=" * 64)

    if fir_activated:
        print("\n *** FIR RAIN DETECTION WAS ACTIVATED! ***")

    if findings:
        print("\n All findings (%d):" % len(findings))
        for f in findings:
            tag = f[0]
            rest = f[1:]
            if tag.startswith("FIR"):
                print("  >>> %s: %s" % (tag, str(rest)))
            elif tag.startswith("WRITE_OK"):
                print("  +++ %s: %s" % (tag, str(rest)))
            elif tag.startswith("NAD"):
                print("  NAD %s: %s" % (tag, str(rest)))
            elif tag.startswith("SESSION"):
                print("  SES %s: %s" % (tag, str(rest)))
            elif tag.startswith("DID"):
                print("  DID %s: %s" % (tag, str(rest)))
            else:
                print("  ... %s: %s" % (tag, str(rest)))
    else:
        print("\n  No diagnostic responses at all.")
        print("  The sensor may not support standard UDS-over-LIN,")
        print("  or uses a proprietary initialization protocol.")

    print("=" * 64)
    return findings


if __name__ == "__main__":
    run_diag()
