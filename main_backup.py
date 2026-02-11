import rp2
from machine import Pin, UART
import time

# ==========================================
# 1. PIO Configuration for LIN Header
# ==========================================
# This PIO program generates the Master Request Header with INVERTED
# pin levels to compensate for the NPN (BC547) open-collector driver:
#   Pin LOW  → Q1 OFF → bus 12V (recessive)
#   Pin HIGH → Q1 ON  → bus 0V  (dominant)
#
# Therefore all UART levels are flipped at the PIO level:
#   UART idle/stop (recessive) → pin LOW
#   UART start/break (dominant) → pin HIGH
#   Data bits: 0 → pin HIGH, 1 → pin LOW
#
# PID data must be bitwise-inverted (XOR 0xFF) before sm.put().
#
# IMPORTANT: sideset is NOT used because set_base, out_base and sideset_base
# would all target the same pin (GPIO0). Per the RP2040 datasheet, sideset
# takes priority over out — so implicit side(0) on instructions without
# .side() would force the pin LOW during PID data output, corrupting it.
# Instead, we rely on set(pins, ...) for explicit control and out(pins, ...)
# for data shifting. Pin values latch between instructions.

@rp2.asm_pio(out_init=rp2.PIO.OUT_LOW, set_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def lin_header_tx():
    # Wait for trigger — PIO stalls here, pin LOW → Q1 OFF → bus recessive (12V)
    pull(block)                        # Get pre-inverted PID value from Python
    mov(y, osr)                        # Save inverted PID in Y register for later
    
    # --- Step 1: Break Field (13+ bits: pin HIGH → Q1 ON → bus dominant 0V) ---
    set(pins, 1)              [7]      # Drive HIGH (dominant) — 8 cycles
    set(x, 5)                 [7]      # Loop counter — pin latched HIGH — 8 cycles
    label("break_loop")
    nop()                     [7]      # 8 cycles
    jmp(x_dec, "break_loop")  [7]      # 8 cycles → 6 iterations × 16 = 96 + 16 = 112 → total 14 bit times
    
    # --- Step 2: Break Delimiter (1 bit: pin LOW → Q1 OFF → bus recessive 12V) ---
    set(pins, 0)              [7]      # Drive LOW (recessive) — 8 cycles = 1 bit time
    
    # --- Step 3: Send Sync Byte (0x55) — inverted pin levels for NPN ---
    # 0x55 = 01010101, LSB first: 1, 0, 1, 0, 1, 0, 1, 0
    # Inverted on pin:           0, 1, 0, 1, 0, 1, 0, 1
    # Start bit (pin HIGH → bus dominant)
    set(pins, 1)              [7]
    # Data bits (inverted: data 1 → pin LOW, data 0 → pin HIGH)
    set(pins, 0)              [7]      # Bit 0 (data=1 → pin LOW)
    set(pins, 1)              [7]      # Bit 1 (data=0 → pin HIGH)
    set(pins, 0)              [7]      # Bit 2 (data=1 → pin LOW)
    set(pins, 1)              [7]      # Bit 3 (data=0 → pin HIGH)
    set(pins, 0)              [7]      # Bit 4 (data=1 → pin LOW)
    set(pins, 1)              [7]      # Bit 5 (data=0 → pin HIGH)
    set(pins, 0)              [7]      # Bit 6 (data=1 → pin LOW)
    set(pins, 1)              [7]      # Bit 7 (data=0 → pin HIGH)
    # Stop bit (pin LOW → bus recessive)
    set(pins, 0)              [7]
    
    # --- Step 4: Send PID (from Y register, pre-inverted by Python) ---
    # Inter-byte space (bus recessive from stop bit, pin stays LOW)
    mov(osr, y)               [7]      # Restore inverted PID to OSR — 8 cycles LOW (recessive)
    # Start bit (pin HIGH → bus dominant)
    set(pins, 1)              [6]      # 7 cycles HIGH
    set(x, 7)                          # 1 cycle HIGH — total start bit = 8 cycles
    # Data bits — out drives the pin with pre-inverted data
    label("pid_loop")
    out(pins, 1)              [6]      # Shift 1 bit out (LSB first) — 7 cycles
    jmp(x_dec, "pid_loop")             # 1 cycle (pin latched) — total per bit = 8 cycles
    # Stop bit (pin LOW → bus recessive)
    set(pins, 0)              [7]      # 8 cycles LOW
    
    # Wrap back to pull(block) — PIO stalls, pin LOW → Q1 OFF → bus recessive (12V)

class LINMaster:
    def __init__(self, tx_pin_num=0, rx_pin_num=1, baud=19200):
        self.baud = baud
        self.tx_pin_num = tx_pin_num
        self.rx_pin_num = rx_pin_num
        
        # 1. Setup UART for RX — before PIO, so PIO takes final
        #    control of GPIO0. We assign UART TX to GPIO12 (valid UART0 TX,
        #    unused) to avoid FUNCSEL conflict with PIO on GPIO0.
        self.uart = UART(0, baudrate=self.baud, tx=Pin(12), rx=Pin(rx_pin_num), bits=8, parity=None, stop=1)
        
        # 2. Setup PIO for TX — takes control of GPIO0
        #    NPN inversion is handled entirely in PIO logic (no OUTOVER needed).
        #    Pin idles LOW → Q1 OFF → bus recessive (12V).
        # Frequency: 8 PIO cycles per bit → SM clock = Baud × 8
        sm_freq = self.baud * 8
        self.sm = rp2.StateMachine(0, lin_header_tx, freq=sm_freq, 
                                   out_base=Pin(tx_pin_num), 
                                   set_base=Pin(tx_pin_num))
        self.sm.active(1)
        
        print(f"LIN Master Initialized on TX={tx_pin_num}, RX={rx_pin_num} @ {baud} baud")

    def calculate_pid(self, frame_id):
        """LIN 2.0 Protected Identifier calculation."""
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
        """Classic checksum (LIN 1.3): inverted sum of data bytes only (excludes PID)."""
        chk = 0
        for b in data:
            chk += b
            if chk > 255:
                chk -= 255
        return (~chk) & 0xFF

    def calculate_enhanced_checksum(self, pid, data):
        """Enhanced checksum (LIN 2.0): inverted sum of PID + data bytes."""
        chk = pid
        for b in data:
            chk += b
            if chk > 255:
                chk -= 255
        return (~chk) & 0xFF

    def send_header(self, frame_id):
        pid = self.calculate_pid(frame_id)
        
        # Clear UART RX buffer from previous noise / echo
        while self.uart.any():
            self.uart.read()
            
        # Trigger PIO — it's stalled at pull(block), this resumes it
        # Invert PID bits for NPN: PIO outputs inverted levels, so data
        # must be pre-inverted so the bus sees correct UART values.
        self.sm.put(pid ^ 0xFF)
        
        return pid

    def read_response(self, length, timeout_ms=100, verbose=False):
        start = time.ticks_ms()
        data = b''
        
        while (time.ticks_diff(time.ticks_ms(), start) < timeout_ms):
            avail = self.uart.any()
            if avail:
                chunk = self.uart.read()
                if chunk:
                    if verbose:
                        print(f"  RX [{time.ticks_diff(time.ticks_ms(), start)}ms]: {avail} byte(s) -> {[hex(x) for x in chunk]}")
                    data += chunk
                
                # If we have enough data (PID + expected length), we can stop early
                # But for scanning we don't know expected length usually.
                if len(data) >= length: 
                    break
        
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        if verbose:
            if not data:
                print(f"  RX: No data received (timeout {elapsed}ms)")
            else:
                print(f"  RX: Total {len(data)} byte(s) in {elapsed}ms")
        
        return data

# ==========================================
# Main Execution
# ==========================================

lin = LINMaster(tx_pin_num=0, rx_pin_num=1)

# Found sensor IDs
# ID 0x22 (PID 0xE2) - Valid Data Response (Classic Checksum)
# ID 0x15 (PID 0x55) - Often returns just 0x55 echo or empty, likely command/status frames or noise
SENSOR_ID_DATA = 0x22

print(f"Starting Sensor Polling (ID {hex(SENSOR_ID_DATA)})...")

while True:
    try:
        # 1. Send Header for Data Request
        pid = lin.send_header(SENSOR_ID_DATA)
        
        # 2. Wait for response (Echo PID + Data + Checksum)
        # Expected from Scanner: [0x55, 0xe2, 0x0, 0x21, 0x8, 0x0, 0xd6]
        # PID Echo (0xE2) + 4 Data Bytes + Checksum = 6 bytes after the break/sync
        
        raw_response = lin.read_response(length=7, timeout_ms=50) # Adjusted length
        
        if raw_response:
             # Look for PID byte (0xE2) to align frame
            pid_byte = bytes([pid])
            try:
                if pid_byte in raw_response:
                    pid_index = raw_response.index(pid_byte)
                    frame = raw_response[pid_index+1:]
                    
                    if len(frame) >= 2: # At least data + checksum
                        data_bytes = frame[:-1]
                        received_chk = frame[-1]
                        
                        # Validate Checksum (Classic 1.x)
                        exp_chk = lin.calculate_classic_checksum(data_bytes)
                        
                        if received_chk == exp_chk:
                            # Typical RLS Payload Structure (4 bytes)
                            # Byte 0: Rain Intensity (0x00 = Dry)
                            # Byte 1: Light Sensor LSB
                            # Byte 2: Light Sensor MSB
                            # Byte 3: Status / Checksum / Inverted Byte 0 (varies)
                            
                            light_val = (data_bytes[2] << 8) | data_bytes[1]
                            rain_val = data_bytes[0]
                            
                            print(f"ID {hex(SENSOR_ID_DATA)} valid:")
                            print(f"  Rain:  {hex(rain_val)} ({rain_val})")
                            print(f"  Light: {hex(light_val)} ({light_val})")
                            print(f"  Raw:   {[hex(b) for b in data_bytes]}")

                        else:
                            print(f"ID {hex(SENSOR_ID_DATA)}: Checksum Fail (Rx {hex(received_chk)} != Exp {hex(exp_chk)})")
                            print(f"  Raw Frame: {[hex(b) for b in frame]}")

            except ValueError:
                pass
                
        time.sleep(0.5)
        
    except KeyboardInterrupt:
        break
            
        # time.sleep(1.0) # Removed pause to scan continuously
        
    except KeyboardInterrupt:
        break