"""
LIN Bus ID Scanner

Scans all 64 possible LIN frame IDs (0x00-0x3F) to discover which ones
the slave sensor responds to. Reports raw data and checksum type.

Self-contained: includes its own PIO program and LIN master logic.
No external dependencies beyond MicroPython builtins.
"""

import rp2
from machine import Pin, UART
import time


# ==========================================
# PIO Program for LIN Header TX
# ==========================================
# Inverted pin levels for NPN open-collector driver:
#   Pin LOW  -> Q1 OFF -> bus 12V (recessive)
#   Pin HIGH -> Q1 ON  -> bus 0V  (dominant)

@rp2.asm_pio(out_init=rp2.PIO.OUT_LOW, set_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def lin_header_tx():
    pull(block)
    mov(y, osr)

    # Break Field (13+ bits dominant)
    set(pins, 1)              [7]
    set(x, 5)                 [7]
    label("break_loop")
    nop()                     [7]
    jmp(x_dec, "break_loop")  [7]

    # Break Delimiter (1 bit recessive)
    set(pins, 0)              [7]

    # Sync Byte 0x55 (inverted for NPN, hardcoded)
    set(pins, 1)              [7]      # Start bit
    set(pins, 0)              [7]      # Bit 0
    set(pins, 1)              [7]      # Bit 1
    set(pins, 0)              [7]      # Bit 2
    set(pins, 1)              [7]      # Bit 3
    set(pins, 0)              [7]      # Bit 4
    set(pins, 1)              [7]      # Bit 5
    set(pins, 0)              [7]      # Bit 6
    set(pins, 1)              [7]      # Bit 7
    set(pins, 0)              [7]      # Stop bit

    # PID byte (from Y register, pre-inverted by Python)
    mov(osr, y)               [7]
    set(pins, 1)              [6]      # Start bit
    set(x, 7)
    label("pid_loop")
    out(pins, 1)              [6]
    jmp(x_dec, "pid_loop")
    set(pins, 0)              [7]      # Stop bit


class LINMaster:
    def __init__(self, tx_pin_num=0, rx_pin_num=1, baud=19200):
        self.baud = baud
        self.uart = UART(0, baudrate=baud, tx=Pin(12), rx=Pin(rx_pin_num),
                         bits=8, parity=None, stop=1)
        sm_freq = baud * 8
        self.sm = rp2.StateMachine(0, lin_header_tx, freq=sm_freq,
                                   out_base=Pin(tx_pin_num),
                                   set_base=Pin(tx_pin_num))
        self.sm.active(1)
        print(f"LIN Scanner: TX=GPIO{tx_pin_num}, RX=GPIO{rx_pin_num}, {baud} baud")

    def calculate_pid(self, frame_id):
        id0 = (frame_id >> 0) & 1
        id1 = (frame_id >> 1) & 1
        id2 = (frame_id >> 2) & 1
        id3 = (frame_id >> 3) & 1
        id4 = (frame_id >> 4) & 1
        id5 = (frame_id >> 5) & 1
        p0 = id0 ^ id1 ^ id2 ^ id4
        p1 = ~(id1 ^ id3 ^ id4 ^ id5) & 1
        return (frame_id & 0x3F) | (p0 << 6) | (p1 << 7)

    def calculate_classic_checksum(self, data):
        chk = 0
        for b in data:
            chk += b
            if chk > 255:
                chk -= 255
        return (~chk) & 0xFF

    def calculate_enhanced_checksum(self, pid, data):
        chk = pid
        for b in data:
            chk += b
            if chk > 255:
                chk -= 255
        return (~chk) & 0xFF

    def send_header(self, frame_id):
        pid = self.calculate_pid(frame_id)
        while self.uart.any():
            self.uart.read()
        self.sm.put(pid ^ 0xFF)
        return pid

    def read_response(self, length=11, timeout_ms=50):
        start = time.ticks_ms()
        data = b''
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.uart.any():
                chunk = self.uart.read()
                if chunk:
                    data += chunk
                if len(data) >= length:
                    break
        return data


# ==========================================
# Scanner
# ==========================================

def _hex_list(data):
    return "[" + ", ".join("0x%02X" % b for b in data) + "]"


lin = LINMaster(tx_pin_num=0, rx_pin_num=1)

print("\nScanning LIN IDs 0x00-0x3F...")
print("=" * 60)

while True:
    try:
        found_any = False
        for target_id in range(0x40):
            pid = lin.send_header(target_id)
            raw = lin.read_response(length=11, timeout_ms=50)

            if raw and len(raw) > 1:
                pid_byte = bytes([pid])
                try:
                    idx = raw.index(pid_byte)
                    payload = raw[idx + 1:]

                    if len(payload) >= 2:
                        data_bytes = list(payload[:-1])
                        received_chk = payload[-1]

                        exp_classic = lin.calculate_classic_checksum(data_bytes)
                        exp_enhanced = lin.calculate_enhanced_checksum(pid, data_bytes)

                        if received_chk == exp_classic:
                            chk_str = "OK (Classic LIN 1.x)"
                        elif received_chk == exp_enhanced:
                            chk_str = "OK (Enhanced LIN 2.x)"
                        else:
                            chk_str = "FAIL (classic=%s, enhanced=%s)" % (
                                "0x%02X" % exp_classic, "0x%02X" % exp_enhanced)

                        print("\n[FOUND] Response for ID 0x%02X (PID 0x%02X)" % (target_id, pid))
                        print("  Raw: %s" % _hex_list(raw))
                        print("  Checksum: %s" % chk_str)
                        found_any = True
                        time.sleep(0.5)

                except ValueError:
                    pass

            time.sleep(0.01)

        if not found_any:
            print(".", end="")

    except KeyboardInterrupt:
        break
