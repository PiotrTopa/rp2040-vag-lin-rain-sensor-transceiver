"""
LIN 2.x Master Driver — RP2040 PIO + UART

Shared low-level LIN bus driver for all sensor tools.
PIO handles TX (break + byte framing), hardware UART handles RX.

Hardware: RP2040 + NPN transistor LIN transceiver
  TX = GPIO0 (PIO), RX = GPIO1 (UART0), 19200 baud

Usage:
    from lin import LIN
    bus = LIN()
    bus.send(0x20, [0x81, 0x04, 0x02, 0, 0, 0, 0, 0])
    data, ok = bus.recv(0x23)
"""

import rp2
from machine import Pin, UART
import time


# ================================================================
# PIO Program — Universal LIN TX
# ================================================================
# sm.put(word):  bit8=1 -> Break (13+ bit dominant)
#                bit8=0 -> byte (bits 0-7, pre-XOR 0xFF for NPN)
# NPN driver: Pin HIGH = dominant (bus LOW), Pin LOW = recessive (bus HIGH)

@rp2.asm_pio(out_init=rp2.PIO.OUT_LOW, set_init=rp2.PIO.OUT_LOW,
             out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def _pio_tx():
    wrap_target()
    label("entry")
    pull(block)
    out(y, 8)                       # byte -> Y
    out(x, 1)                       # cmd flag -> X
    jmp(not_x, "byte")
    # Break: 13+ bit times dominant
    set(pins, 1)           [7]
    set(x, 12)             [7]
    label("brk")
    nop()                  [7]
    jmp(x_dec, "brk")     [7]
    set(pins, 0)           [7]      # break delimiter (recessive)
    jmp("entry")
    # Byte: start + 8 data + stop
    label("byte")
    mov(osr, y)            [7]      # inter-byte gap
    set(pins, 1)           [7]      # start bit (dominant)
    set(x, 7)
    label("bit")
    out(pins, 1)           [6]
    jmp(x_dec, "bit")
    set(pins, 0)           [7]      # stop bit (recessive)
    wrap()


# ================================================================
# LIN Master
# ================================================================

class LIN:
    """LIN 2.x master. PIO for TX, hardware UART for RX."""

    _BRK = 1 << 8      # Break command word for PIO
    NAD = 0x02          # Slave Node Address (81A 955 555 A)

    def __init__(self, tx=0, rx=1, baud=19200):
        self.uart = UART(0, baudrate=baud, tx=Pin(12), rx=Pin(rx),
                         bits=8, parity=None, stop=1)
        self.sm = rp2.StateMachine(
            0, _pio_tx, freq=baud * 8,
            out_base=Pin(tx), set_base=Pin(tx))
        self.sm.active(1)
        print("LIN: TX=GPIO%d RX=GPIO%d %dbaud" % (tx, rx, baud))

    # --- Static helpers ---

    @staticmethod
    def pid(fid):
        """Protected Identifier from 6-bit frame ID."""
        b = [(fid >> i) & 1 for i in range(6)]
        p0 = b[0] ^ b[1] ^ b[2] ^ b[4]
        p1 = ~(b[1] ^ b[3] ^ b[4] ^ b[5]) & 1
        return (fid & 0x3F) | (p0 << 6) | (p1 << 7)

    @staticmethod
    def chk(data, pid=None):
        """Checksum. Enhanced if pid given, else Classic."""
        s = pid if pid is not None else 0
        for v in data:
            s += v
            if s > 255:
                s -= 255
        return (~s) & 0xFF

    # --- Low-level TX/RX ---

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

    # --- Frame-level ---

    def header(self, fid):
        """Send Break + Sync + PID. Returns PID."""
        p = self.pid(fid)
        self._flush()
        self._brk()
        self._tx(0x55)
        self._tx(p)
        return p

    def send(self, fid, data, enhanced=True):
        """Send master-request frame (header + data + checksum)."""
        p = self.header(fid)
        for b in data:
            self._tx(b)
        self._tx(self.chk(data, p if enhanced else None))
        time.sleep_us(300)

    def recv(self, fid, tmo=50):
        """Poll slave-response frame. Returns (data_list, ok)."""
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

    # --- Diagnostic transport (0x3C / 0x3D) ---

    def diag_send(self, payload):
        """Send diagnostic request on frame 0x3C (Classic checksum)."""
        while len(payload) < 8:
            payload.append(0xFF)
        self.send(0x3C, payload[:8], enhanced=False)

    def diag_recv(self, tmo=100):
        """Receive diagnostic response from frame 0x3D (Classic checksum)."""
        p = self.header(0x3D)
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
        if pay[-1] == self.chk(data):
            return data
        return None

    def diag(self, sid, data, nad=None):
        """
        Send diagnostic request + receive response.
        Handles single-frame and multi-frame (auto-CF, no FC needed).
        Returns response payload or None.
        """
        if nad is None:
            nad = self.NAD

        pci = 1 + len(data)
        self.diag_send([nad, pci, sid] + list(data))
        time.sleep_ms(15)

        resp = self.diag_recv(tmo=100)
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
                cf = self.diag_recv(tmo=120)
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


def hex_str(data):
    """Format bytes as hex string."""
    return " ".join("%02X" % b for b in data)
