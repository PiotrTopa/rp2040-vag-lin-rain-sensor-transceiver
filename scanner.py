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
        
        # 1. Setup UART for RX — before PIO, so PIO takes final control.
        #    Assign UART TX to GPIO12 (unused) to avoid FUNCSEL conflict.
        self.uart = UART(0, baudrate=self.baud, tx=Pin(12), rx=Pin(rx_pin_num), bits=8, parity=None, stop=1)
        
        # 2. Setup PIO for TX — takes control of GPIO0
        sm_freq = self.baud * 8
        self.sm = rp2.StateMachine(0, lin_header_tx, freq=sm_freq, 
                                   out_base=Pin(tx_pin_num), 
                                   set_base=Pin(tx_pin_num))
        self.sm.active(1)
        
        print(f"LIN Master Scanner Initialized on TX={tx_pin_num}, RX={rx_pin_num} @ {baud} baud")

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
        """Classic checksum (LIN 1.3): inverted sum of data bytes only."""
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
            
        # Trigger PIO
        # Invert PID bits for NPN driver logic
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
# Main Execution (Scanner)
# ==========================================

lin = LINMaster(tx_pin_num=0, rx_pin_num=1)

# Target IDs to poll
# Scanning all possible IDs (0x00 - 0x3F) to find the sensor
target_ids = range(0x40)
print(f"Polling all IDs: 0x00 - 0x3F")

while True:
    try:
        found_any = False
        for target_id in target_ids:
            
            # 1. Send Header
            pid = lin.send_header(target_id)
            
            # 2. Wait for response
            raw_response = lin.read_response(length=9, timeout_ms=50)
            
            # Check if we got anything MORE than just the echo of the PID
            if raw_response and len(raw_response) > 1:
                # Look for PID byte in response (echo from transceiver loopback)
                pid_byte = bytes([pid])
                try:
                    pid_index = raw_response.index(pid_byte)
                    payload = raw_response[pid_index+1:]
                    
                    if len(payload) > 0:
                        print(f"\n[FOUND] Response for ID {hex(target_id)} (PID {hex(pid)})")
                        print(f"  Raw: {[hex(x) for x in raw_response]}")
                        
                        if len(payload) >= 2:
                            data_bytes = payload[:-1]
                            received_chk = payload[-1]
                            
                            exp_classic = lin.calculate_classic_checksum(data_bytes)
                            is_classic = (received_chk == exp_classic)
                            
                            exp_enhanced = lin.calculate_enhanced_checksum(pid, data_bytes)
                            is_enhanced = (received_chk == exp_enhanced)

                            if is_classic:
                                print(f"  Checksum: OK (Classic LIN 1.x)")
                            elif is_enhanced:
                                print(f"  Checksum: OK (Enhanced LIN 2.x)")
                            else:
                                print(f"  Checksum: FAIL (Exp Classic: {hex(exp_classic)}, Exp Enhanced: {hex(exp_enhanced)})")
                        
                        found_any = True
                        time.sleep(0.5) 
                        
                except ValueError:
                    pass
            
            time.sleep(0.01) # Small delay between IDs

        if not found_any:
            print(".", end="") 
                    
    except KeyboardInterrupt:
        break
