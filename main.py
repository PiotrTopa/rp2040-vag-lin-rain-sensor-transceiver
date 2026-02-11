import rp2
from machine import Pin, UART, mem32
import time

# ==========================================
# 1. PIO Configuration for LIN Header
# ==========================================
# This PIO program generates the Master Request Header:
# 1. Break Field (13+ bits LOW)
# 2. Delimiter (1+ bit HIGH)
# 3. Sync Byte (0x55) (Standard UART 8N1)
# 4. PID Byte (Protected Identifier) (Standard UART 8N1)
#
# IMPORTANT: sideset is NOT used because set_base, out_base and sideset_base
# would all target the same pin (GPIO0). Per the RP2040 datasheet, sideset
# takes priority over out — so implicit side(0) on instructions without
# .side() would force the pin LOW during PID data output, corrupting it.
# Instead, we rely on set(pins, ...) for explicit control and out(pins, ...)
# for data shifting. Pin values latch between instructions.

@rp2.asm_pio(out_init=rp2.PIO.OUT_HIGH, set_init=rp2.PIO.OUT_HIGH)
def lin_header_tx():
    # Wait for trigger — PIO stalls here, bus stays HIGH (recessive)
    pull(block)                        # Get PID value from Python
    mov(y, osr)                        # Save PID in Y register for later
    
    # --- Step 1: Break Field (13+ bits LOW) ---
    set(pins, 0)              [7]      # Drive LOW (dominant) — 8 cycles
    set(x, 5)                 [7]      # Loop counter — pin latched LOW — 8 cycles
    label("break_loop")
    nop()                     [7]      # 8 cycles
    jmp(x_dec, "break_loop")  [7]      # 8 cycles → 6 iterations × 16 = 96 + 16 = 112 → total 14 bit times
    
    # --- Step 2: Break Delimiter (1 bit HIGH) ---
    set(pins, 1)              [7]      # Drive HIGH (recessive) — 8 cycles = 1 bit time
    
    # --- Step 3: Send Sync Byte (0x55) ---
    # 0x55 = 01010101, LSB first: 1, 0, 1, 0, 1, 0, 1, 0
    # Start bit (0)
    set(pins, 0)              [7]
    # Data bits
    set(pins, 1)              [7]      # Bit 0 (1)
    set(pins, 0)              [7]      # Bit 1 (0)
    set(pins, 1)              [7]      # Bit 2 (1)
    set(pins, 0)              [7]      # Bit 3 (0)
    set(pins, 1)              [7]      # Bit 4 (1)
    set(pins, 0)              [7]      # Bit 5 (0)
    set(pins, 1)              [7]      # Bit 6 (1)
    set(pins, 0)              [7]      # Bit 7 (0)
    # Stop bit (1)
    set(pins, 1)              [7]
    
    # --- Step 4: Send PID (from Y register) ---
    # Inter-byte space (bus HIGH from sync stop bit)
    mov(osr, y)               [7]      # Restore PID to OSR — 8 cycles HIGH
    # Start bit (0)
    set(pins, 0)              [6]      # 7 cycles LOW
    set(x, 7)                          # 1 cycle LOW — total start bit = 8 cycles
    # Data bits — out drives the pin, value latches through jmp
    label("pid_loop")
    out(pins, 1)              [6]      # Shift 1 bit out (LSB first) — 7 cycles
    jmp(x_dec, "pid_loop")             # 1 cycle (pin latched) — total per bit = 8 cycles
    # Stop bit (1)
    set(pins, 1)              [7]      # 8 cycles HIGH
    
    # Wrap back to pull(block) — PIO stalls, bus stays HIGH (recessive)
    # This gives the slave time to respond before the next header

class LINMaster:
    def __init__(self, tx_pin_num=0, rx_pin_num=1, baud=19200):
        self.baud = baud
        self.tx_pin_num = tx_pin_num
        self.rx_pin_num = rx_pin_num
        
        # 1. Setup UART for RX FIRST — before PIO, so PIO takes final
        #    control of GPIO0. We assign UART TX to GPIO12 (valid UART0 TX,
        #    unused) to avoid FUNCSEL conflict with PIO on GPIO0.
        self.uart = UART(0, baudrate=self.baud, tx=Pin(12), rx=Pin(rx_pin_num), bits=8, parity=None, stop=1)
        
        # 2. Configure Hardware Inversion for BC547 (NPN)
        # OUTOVER bits 13:12 in IO_BANK0_GPIOx_CTRL register
        # Value 0x1 = Invert output at the pad level:
        #   PIO HIGH (idle) → pad LOW → NPN OFF → bus recessive (12V)
        #   PIO LOW (start) → pad HIGH → NPN ON → bus dominant (0V)
        GPIO_CTRL_BASE = 0x40014004
        reg_addr = GPIO_CTRL_BASE + (self.tx_pin_num * 8)
        mem32[reg_addr] = (mem32[reg_addr] & ~0x3000) | 0x1000
        
        # 3. Setup PIO for TX LAST — takes final control of GPIO0
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
        self.sm.put(pid)
        
        return pid

    def read_response(self, length, timeout_ms=100):
        start = time.ticks_ms()
        data = b''
        
        while (time.ticks_diff(time.ticks_ms(), start) < timeout_ms):
            avail = self.uart.any()
            if avail:
                chunk = self.uart.read()
                if chunk:
                    print(f"  RX [{time.ticks_diff(time.ticks_ms(), start)}ms]: {avail} byte(s) -> {[hex(x) for x in chunk]}")
                    data += chunk
                if len(data) >= length: 
                    break
        
        elapsed = time.ticks_diff(time.ticks_ms(), start)
        if not data:
            print(f"  RX: No data received (timeout {elapsed}ms)")
        else:
            print(f"  RX: Total {len(data)} byte(s) in {elapsed}ms")
        
        return data

# ==========================================
# Main Execution
# ==========================================

lin = LINMaster(tx_pin_num=0, rx_pin_num=1)

# Target IDs to poll
target_ids = [0x02, 0x09, 0x0B, 0x30]
print(f"Polling IDs: {[hex(i) for i in target_ids]}")

while True:
    try:
        for target_id in target_ids:
            print(f"\n--- Polling ID {hex(target_id)} ---")
            
            # 1. Send Header
            pid = lin.send_header(target_id)
            print(f"Header sent: PID={hex(pid)}")
            
            # 2. Wait for response
            raw_response = lin.read_response(length=6)
            
            if raw_response:
                print(f"Raw: {[hex(x) for x in raw_response]}")
                
                # Look for PID byte in response (echo from transceiver loopback)
                pid_byte = bytes([pid])
                try:
                    pid_index = raw_response.index(pid_byte)
                    payload = raw_response[pid_index+1:]
                    if len(payload) > 0:
                        # Validate classic checksum (last byte is checksum)
                        data_bytes = payload[:-1]
                        received_chk = payload[-1]
                        expected_chk = lin.calculate_classic_checksum(data_bytes)
                        chk_ok = "OK" if received_chk == expected_chk else f"FAIL (expected {hex(expected_chk)})"
                        print(f"Payload: {[hex(x) for x in data_bytes]}, Checksum: {hex(received_chk)} [{chk_ok}]")
                except ValueError:
                    print(f"  PID {hex(pid)} not found in response")
                    
            time.sleep(0.5) # Delay between IDs
        
        time.sleep(1.0) # Pause between full cycles
        
    except KeyboardInterrupt:
        break