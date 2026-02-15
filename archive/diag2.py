"""
KWP2000-over-LIN Diagnostic Tool — 81A 955 555 A

The sensor uses KWP2000-style diagnostics (not modern UDS):
  - NAD = 0x02
  - SID 0x21 (ReadDataByLocalIdentifier) confirmed working
  - Multi-frame responses via LIN Transport Protocol
  - All standard UDS/LIN services return NRC 0x11

This tool:
  1. Reads all local identifiers (0x00-0xFF) with multi-frame support
  2. Tries WriteDataByLocalIdentifier (0x3B)
  3. Tries other KWP2000 services the sensor may support
  4. Checks FIR after each write attempt

Usage:
    from diag2 import *
    run()           # full automatic scan
    read(0x01)      # read single local ID
    read_all()      # read all local IDs
    sids()          # find all supported SIDs
    write(lid, data)# write local ID
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
    NAD = 0x02  # confirmed from diag_fuzzer

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

    def recv_raw(self, fid, tmo=50):
        """Receive with classic checksum (for 0x3D diagnostic responses)."""
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
        if pay[-1] == self.chk(data):  # classic checksum
            return data
        return None


# ================================================================
# LIN Transport Protocol (multi-frame)
# ================================================================

def _h(data):
    return " ".join("%02X" % b for b in data)


def _diag_send(lin, payload):
    """Send 8-byte diagnostic frame on 0x3C (classic checksum)."""
    while len(payload) < 8:
        payload.append(0xFF)
    lin.send(0x3C, payload[:8], enhanced=False)


def _diag_recv(lin, tmo=100):
    """Read 8-byte diagnostic frame from 0x3D (classic checksum)."""
    return lin.recv_raw(0x3D, tmo)


def _send_sf(lin, nad, sid, data):
    """Send Single Frame: PCI = length (SID + data)."""
    pci = 1 + len(data)  # SID + data bytes
    payload = [nad, pci, sid] + list(data)
    _diag_send(lin, payload)


def _send_fc(lin, nad, bs=0, stmin=0):
    """Send Flow Control frame so slave continues sending CFs."""
    payload = [nad, 0x30, bs, stmin]
    _diag_send(lin, payload)


def diag_request(lin, sid, data, nad=None):
    """
    Send a diagnostic request and handle multi-frame response.
    Returns full response data bytes or None.
    """
    if nad is None:
        nad = lin.NAD

    # Send single-frame request
    _send_sf(lin, nad, sid, data)
    time.sleep_ms(15)

    # Read response
    resp = _diag_recv(lin, tmo=100)
    if not resp:
        return None

    # Check NAD matches
    if resp[0] != nad:
        return None

    pci_type = (resp[1] >> 4) & 0x0F
    pci_len = resp[1] & 0x0F

    # Single Frame (PCI type 0)
    if pci_type == 0:
        return resp[2:2 + pci_len]

    # First Frame (PCI type 1) — multi-frame response
    if pci_type == 1:
        total_len = ((resp[1] & 0x0F) << 8) | resp[2]
        collected = resp[3:]  # first 5 data bytes from FF

        # Send Flow Control
        time.sleep_ms(5)
        _send_fc(lin, nad)
        time.sleep_ms(10)

        # Read Consecutive Frames
        seq = 1
        retries = 0
        while len(collected) < total_len and retries < 20:
            cf = _diag_recv(lin, tmo=100)
            if not cf:
                retries += 1
                # Re-poll 0x3D
                time.sleep_ms(10)
                continue

            if cf[0] != nad:
                retries += 1
                continue

            cf_pci = cf[1]
            cf_type = (cf_pci >> 4) & 0x0F
            cf_seq = cf_pci & 0x0F

            if cf_type != 2:
                retries += 1
                continue

            # Consecutive Frame data: bytes 2-7
            collected.extend(cf[2:])
            seq += 1
            retries = 0
            time.sleep_ms(5)

        return collected[:total_len]

    return None


def is_positive(resp, sid):
    if not resp or len(resp) < 1:
        return False
    return resp[0] == (sid + 0x40)


def is_negative(resp):
    if not resp or len(resp) < 1:
        return False
    return resp[0] == 0x7F


def nrc_name(nrc):
    names = {
        0x10: "generalReject",
        0x11: "serviceNotSupported",
        0x12: "subFunctionNotSupported",
        0x13: "incorrectMessageLength",
        0x14: "responseTooLong",
        0x21: "busyRepeatRequest",
        0x22: "conditionsNotCorrect",
        0x24: "requestSequenceError",
        0x25: "noResponseFromSubnet",
        0x31: "requestOutOfRange",
        0x33: "securityAccessDenied",
        0x35: "invalidKey",
        0x36: "exceededNumberOfAttempts",
        0x37: "requiredTimeDelayNotExpired",
        0x70: "uploadDownloadNotAccepted",
        0x72: "generalProgrammingFailure",
        0x73: "wrongBlockSequenceCounter",
        0x78: "requestCorrectlyReceivedResponsePending",
        0x7E: "subFunctionNotSupportedInActiveSession",
        0x7F: "serviceNotSupportedInActiveSession",
    }
    return names.get(nrc, "unknown_0x%02X" % nrc)


# ================================================================
# High-level diagnostic functions
# ================================================================

_lin = None


def _get_lin():
    global _lin
    if _lin is None:
        _lin = LIN()
        # Wake sensor
        for _ in range(10):
            d, ok = _lin.recv(0x23, tmo=80)
            if d and ok:
                break
            time.sleep_ms(50)
    return _lin


def read(lid, verbose=True):
    """Read a local identifier via SID 0x21. Returns data or None."""
    lin = _get_lin()
    resp = diag_request(lin, 0x21, [lid])
    if resp and is_positive(resp, 0x21):
        data = resp[2:]  # skip SID+0x40, echoed LID
        if verbose:
            print("LID 0x%02X (%d bytes): [%s]" % (lid, len(data), _h(data)))
        return data
    elif resp and is_negative(resp):
        sid_echo = resp[1] if len(resp) > 1 else 0
        nrc = resp[2] if len(resp) > 2 else 0
        if verbose:
            print("LID 0x%02X: NRC 0x%02X (%s)" % (lid, nrc, nrc_name(nrc)))
        return None
    else:
        if verbose:
            if resp:
                print("LID 0x%02X: unexpected [%s]" % (lid, _h(resp)))
            else:
                print("LID 0x%02X: no response" % lid)
        return None


def read_all():
    """Read all 256 local identifiers. Returns dict of lid→data."""
    lin = _get_lin()
    results = {}
    print("Reading all local identifiers (0x00-0xFF)...")
    for lid in range(0x100):
        resp = diag_request(lin, 0x21, [lid])
        if resp and is_positive(resp, 0x21):
            data = resp[2:]
            results[lid] = data
            print("  LID 0x%02X (%2d bytes): [%s]" % (lid, len(data), _h(data)))
        elif resp and is_negative(resp):
            nrc = resp[2] if len(resp) > 2 else 0
            if nrc not in (0x11, 0x12, 0x31):  # skip common "not supported"
                print("  LID 0x%02X: NRC 0x%02X (%s)" % (lid, nrc, nrc_name(nrc)))
        time.sleep_ms(10)

        if lid % 32 == 31:
            print("  ... scanned 0x%02X" % lid)

    print("\nFound %d readable local identifiers." % len(results))
    return results


def sids():
    """Find all supported SIDs by sending each with minimal data."""
    lin = _get_lin()
    supported = []
    print("Scanning all SIDs (0x00-0xFF)...")

    # KWP2000 / UDS SID table
    sid_names = {
        0x10: "DiagnosticSessionControl",
        0x11: "ECUReset",
        0x14: "ClearDTC",
        0x17: "ReadStatusOfDTC",
        0x18: "ReadDTCByStatus",
        0x19: "ReadDTCInformation",
        0x1A: "ReadECUIdentification",
        0x20: "StopDiagnosticSession",
        0x21: "ReadDataByLocalIdentifier",
        0x22: "ReadDataByIdentifier",
        0x23: "ReadMemoryByAddress",
        0x27: "SecurityAccess",
        0x28: "CommunicationControl",
        0x2E: "WriteDataByIdentifier",
        0x2F: "InputOutputControlByIdentifier",
        0x30: "InputOutputControlByLocalIdentifier",
        0x31: "RoutineControl",
        0x34: "RequestDownload",
        0x35: "RequestUpload",
        0x36: "TransferData",
        0x37: "RequestTransferExit",
        0x3B: "WriteDataByLocalIdentifier",
        0x3D: "WriteMemoryByAddress",
        0x3E: "TesterPresent",
        0x85: "ControlDTCSetting",
        0xB0: "AssignNAD",
        0xB2: "ReadByIdentifier",
        0xB5: "ConditionalChangeNAD",
        0xB6: "SaveConfiguration",
        0xB7: "AssignFrameIdRange",
    }

    for sid in range(0x100):
        # Try with PCI=2 (SID + 1 data byte) which worked for 0x21
        resp = diag_request(lin, sid, [0x01])
        tag = ""

        if resp and is_positive(resp, sid):
            name = sid_names.get(sid, "")
            print("  SID 0x%02X: POSITIVE [%s] %s" % (sid, _h(resp), name))
            supported.append((sid, "POSITIVE", resp))
        elif resp and is_negative(resp):
            nrc = resp[2] if len(resp) > 2 else 0
            if nrc != 0x11:  # 0x11 = serviceNotSupported → skip
                name = sid_names.get(sid, "")
                print("  SID 0x%02X: NRC=0x%02X (%s) %s" % (
                    sid, nrc, nrc_name(nrc), name))
                supported.append((sid, "NRC_0x%02X" % nrc, resp))

        time.sleep_ms(10)
        if sid % 32 == 31:
            print("  ... scanned 0x%02X" % sid)

    print("\nFound %d supported/known SIDs." % len(supported))
    return supported


def write(lid, data, verbose=True):
    """Write local identifier via SID 0x3B (WriteDataByLocalIdentifier)."""
    lin = _get_lin()
    payload = [lid] + list(data)
    resp = diag_request(lin, 0x3B, payload)
    if resp and is_positive(resp, 0x3B):
        if verbose:
            print("WRITE OK LID 0x%02X data=[%s] resp=[%s]" % (
                lid, _h(data), _h(resp)))
        return True
    elif resp and is_negative(resp):
        nrc = resp[2] if len(resp) > 2 else 0
        if verbose:
            print("WRITE FAIL LID 0x%02X: NRC 0x%02X (%s)" % (
                lid, nrc, nrc_name(nrc)))
        return False
    else:
        if verbose:
            print("WRITE LID 0x%02X: no response" % lid)
        return False


def io_control(lid, ctrl, data=None, verbose=True):
    """InputOutputControlByLocalIdentifier (SID 0x30)."""
    lin = _get_lin()
    payload = [lid, ctrl]
    if data:
        payload.extend(data)
    resp = diag_request(lin, 0x30, payload)
    if resp and is_positive(resp, 0x30):
        if verbose:
            print("IO_CTRL OK LID=0x%02X ctrl=0x%02X: [%s]" % (
                lid, ctrl, _h(resp)))
        return resp
    elif resp and is_negative(resp):
        nrc = resp[2] if len(resp) > 2 else 0
        if verbose:
            print("IO_CTRL FAIL LID=0x%02X: NRC 0x%02X (%s)" % (
                lid, nrc, nrc_name(nrc)))
    return None


def routine(rid, sub=0x01, data=None, verbose=True):
    """RoutineControl (SID 0x31)."""
    lin = _get_lin()
    payload = [sub, (rid >> 8) & 0xFF, rid & 0xFF]
    if data:
        payload.extend(data)
    resp = diag_request(lin, 0x31, payload)
    if resp and is_positive(resp, 0x31):
        if verbose:
            print("ROUTINE OK RID=0x%04X sub=0x%02X: [%s]" % (
                rid, sub, _h(resp)))
        return resp
    elif resp and is_negative(resp):
        nrc = resp[2] if len(resp) > 2 else 0
        if verbose:
            print("ROUTINE FAIL RID=0x%04X: NRC 0x%02X (%s)" % (
                rid, nrc, nrc_name(nrc)))
    return None


def security(level=0x01, verbose=True):
    """SecurityAccess (SID 0x27) — request seed."""
    lin = _get_lin()
    resp = diag_request(lin, 0x27, [level])
    if resp and is_positive(resp, 0x27):
        if verbose:
            print("SECURITY seed (level 0x%02X): [%s]" % (level, _h(resp)))
        return resp
    elif resp and is_negative(resp):
        nrc = resp[2] if len(resp) > 2 else 0
        if verbose:
            print("SECURITY FAIL: NRC 0x%02X (%s)" % (nrc, nrc_name(nrc)))
    return None


def ecu_id(sub=0x00, verbose=True):
    """ReadECUIdentification (SID 0x1A)."""
    lin = _get_lin()
    resp = diag_request(lin, 0x1A, [sub])
    if resp and is_positive(resp, 0x1A):
        if verbose:
            print("ECU_ID sub=0x%02X: [%s]" % (sub, _h(resp)))
        return resp
    elif resp and is_negative(resp):
        nrc = resp[2] if len(resp) > 2 else 0
        if verbose:
            print("ECU_ID FAIL: NRC 0x%02X (%s)" % (nrc, nrc_name(nrc)))
    return None


def check_fir(lin=None, cmd_id=0x20, cycles=30):
    """Send master commands, check if 0x30 wakes up."""
    if lin is None:
        lin = _get_lin()
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
    return False, None


# ================================================================
# Automated full scan
# ================================================================

def run():
    """Full automated KWP2000 diagnostic scan."""
    lin = _get_lin()

    print("\n" + "=" * 64)
    print(" KWP2000 DIAGNOSTIC SCAN — 81A 955 555 A (NAD=0x02)")
    print("=" * 64)

    findings = []
    fir_activated = False

    # ----------------------------------------------------------
    # PHASE 1: Find all supported SIDs
    # ----------------------------------------------------------
    print("\nPHASE 1: SID discovery")
    sid_results = sids()
    for sid, status, resp in sid_results:
        findings.append(("SID", sid, status, resp))

    # ----------------------------------------------------------
    # PHASE 2: Read all local identifiers
    # ----------------------------------------------------------
    print("\nPHASE 2: ReadDataByLocalIdentifier sweep")
    lid_data = read_all()
    for lid, data in lid_data.items():
        findings.append(("LID_READ", lid, data))

    # ----------------------------------------------------------
    # PHASE 3: ReadECUIdentification sweep
    # ----------------------------------------------------------
    print("\nPHASE 3: ReadECUIdentification (SID 0x1A)")
    for sub in range(0x100):
        resp = diag_request(lin, 0x1A, [sub])
        if resp and is_positive(resp, 0x1A):
            print("  ECU_ID sub=0x%02X: [%s]" % (sub, _h(resp)))
            findings.append(("ECU_ID", sub, resp))
        elif resp and is_negative(resp):
            nrc = resp[2] if len(resp) > 2 else 0
            if nrc not in (0x11, 0x12, 0x31):
                print("  ECU_ID sub=0x%02X: NRC=0x%02X" % (sub, nrc))
        time.sleep_ms(10)
        if sub % 64 == 63:
            print("  ... scanned 0x%02X" % sub)

    # ----------------------------------------------------------
    # PHASE 4: SecurityAccess
    # ----------------------------------------------------------
    print("\nPHASE 4: SecurityAccess (SID 0x27)")
    for level in range(0x01, 0x42, 2):  # odd levels = request seed
        resp = diag_request(lin, 0x27, [level])
        if resp and is_positive(resp, 0x27):
            print("  SEED level=0x%02X: [%s]" % (level, _h(resp)))
            findings.append(("SECURITY_SEED", level, resp))
        elif resp and is_negative(resp):
            nrc = resp[2] if len(resp) > 2 else 0
            if nrc not in (0x11, 0x12, 0x31):
                print("  SEC level=0x%02X: NRC=0x%02X (%s)" % (
                    level, nrc, nrc_name(nrc)))
                findings.append(("SECURITY_NRC", level, nrc))
        time.sleep_ms(10)

    # ----------------------------------------------------------
    # PHASE 5: WriteDataByLocalIdentifier on readable LIDs
    # ----------------------------------------------------------
    print("\nPHASE 5: WriteDataByLocalIdentifier (SID 0x3B)")

    # First try writing back the same data we read (echo test)
    for lid, orig_data in lid_data.items():
        ok = write(lid, orig_data, verbose=True)
        if ok:
            findings.append(("WRITE_ECHO_OK", lid, orig_data))
            # Check FIR
            active, rd = check_fir(lin)
            if active:
                print("  *** FIR ACTIVATED after writing LID 0x%02X! ***" % lid)
                findings.append(("FIR_ACTIVE", lid, orig_data, rd))
                fir_activated = True
        time.sleep_ms(10)

    # Try writing with modified bytes
    coding_patterns = [
        [0x01], [0xFF], [0x00],
        [0x01, 0x00], [0x01, 0x01], [0xFF, 0xFF],
        [0x03, 0x30, 0x4D],
        [0x07], [0x0F], [0x80], [0x81],
    ]

    for lid in range(0x100):
        for pat in coding_patterns:
            ok = write(lid, pat, verbose=False)
            if ok:
                print("  WRITE OK! LID=0x%02X data=[%s]" % (lid, _h(pat)))
                findings.append(("WRITE_OK", lid, pat))
                # Check FIR
                active, rd = check_fir(lin, cycles=20)
                if active:
                    print("  *** FIR ACTIVATED! LID=0x%02X data=[%s] ***" % (
                        lid, _h(pat)))
                    findings.append(("FIR_ACTIVE", lid, pat, rd))
                    fir_activated = True
            time.sleep_ms(5)
        if lid % 32 == 31:
            print("  ... write-scan 0x%02X" % lid)

    # ----------------------------------------------------------
    # PHASE 6: InputOutputControlByLocalIdentifier
    # ----------------------------------------------------------
    print("\nPHASE 6: InputOutputControl (SID 0x30)")
    # Control params: 0x00=returnControlToECU, 0x01=reportCurrentState,
    # 0x04=resetToDefault, 0x07=shortTermAdjustment, 0x08=freezeCurrentState
    for lid in range(0x100):
        for ctrl in (0x00, 0x01, 0x04, 0x07, 0x08):
            resp = diag_request(lin, 0x30, [lid, ctrl])
            if resp and is_positive(resp, 0x30):
                print("  IO_CTRL LID=0x%02X ctrl=0x%02X: [%s]" % (
                    lid, ctrl, _h(resp)))
                findings.append(("IO_CTRL_OK", lid, ctrl, resp))
                # Check FIR
                active, rd = check_fir(lin, cycles=15)
                if active:
                    print("  *** FIR via IO_CTRL! LID=0x%02X ***" % lid)
                    fir_activated = True
            time.sleep_ms(5)
        if lid % 64 == 63:
            print("  ... io-scan 0x%02X" % lid)

    # ----------------------------------------------------------
    # PHASE 7: RoutineControl
    # ----------------------------------------------------------
    print("\nPHASE 7: RoutineControl (SID 0x31)")
    for rid in range(0x100):
        for sub in (0x01, 0x02, 0x03):  # start, stop, requestResults
            resp = diag_request(lin, 0x31, [sub, 0x00, rid])
            if resp and is_positive(resp, 0x31):
                print("  ROUTINE RID=0x%04X sub=0x%02X: [%s]" % (
                    rid, sub, _h(resp)))
                findings.append(("ROUTINE_OK", rid, sub, resp))
            time.sleep_ms(5)
        if rid % 64 == 63:
            print("  ... routine-scan 0x%02X" % rid)

    # ----------------------------------------------------------
    # SUMMARY
    # ----------------------------------------------------------
    print("\n" + "=" * 64)
    print(" SCAN COMPLETE")
    print("=" * 64)

    if fir_activated:
        print("\n *** FIR RAIN DETECTION ACTIVATED! ***")

    pos_findings = [f for f in findings if "OK" in f[0] or "ACTIVE" in f[0]
                    or "SEED" in f[0] or "LID_READ" in f[0] or "ECU_ID" in f[0]]
    print("\nKey findings (%d):" % len(pos_findings))
    for f in pos_findings:
        tag = f[0]
        if tag == "LID_READ":
            lid, data = f[1], f[2]
            print("  READ  LID=0x%02X: [%s]" % (lid, _h(data)))
        elif tag == "ECU_ID":
            sub, resp = f[1], f[2]
            print("  ECUID sub=0x%02X: [%s]" % (sub, _h(resp)))
        elif "FIR" in tag:
            print("  >>> %s: %s" % (tag, str(f[1:])))
        else:
            print("  %s: %s" % (tag, str(f[1:])))

    print("\nSIDs with non-0x11 responses:")
    for f in findings:
        if f[0] == "SID":
            print("  SID 0x%02X: %s" % (f[1], f[2]))

    print("=" * 64)
    return findings


if __name__ == "__main__":
    run()
