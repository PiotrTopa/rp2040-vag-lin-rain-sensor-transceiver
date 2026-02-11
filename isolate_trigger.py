import rp2
from machine import Pin, UART
import time

# ==========================================
# 1. PIO DRIVER (Universal)
# ==========================================
@rp2.asm_pio(out_init=rp2.PIO.OUT_LOW, set_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def lin_master_tx():
    wrap_target()
    label("param_entry")
    pull(block)
    out(y, 8)
    out(x, 1)
    jmp(not_x, "send_byte")
    
    # Send Break
    set(pins, 1)       [7]
    set(x, 12)         [7]
    label("break_loop")
    nop()              [7]
    jmp(x_dec, "break_loop") [7]
    set(pins, 0)       [7]   
    jmp("param_entry")
    
    # Send Byte
    label("send_byte")
    mov(osr, y)        [7]
    set(pins, 1)       [7]
    set(x, 7)
    label("bit_loop")
    out(pins, 1)       [6]
    jmp(x_dec, "bit_loop")
    set(pins, 0)       [7]
    wrap()

class LINMaster:
    def __init__(self, tx_pin_num=0, rx_pin_num=1, baud=19200):
        self.baud = baud
        self.uart = UART(0, baudrate=self.baud, tx=Pin(12), rx=Pin(rx_pin_num), bits=8, parity=None, stop=1)
        sm_freq = self.baud * 8
        self.sm = rp2.StateMachine(0, lin_master_tx, freq=sm_freq, 
                                   out_base=Pin(tx_pin_num), 
                                   set_base=Pin(tx_pin_num))
        self.sm.active(1)

    def _send_break(self):
        self.sm.put( (1 << 8) | 0x00 )

    def _send_byte(self, b):
        self.sm.put( (0 << 8) | (b ^ 0xFF) )

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

    def calculate_checksum(self, data, pid=None):
        val = 0
        if pid is not None: val = pid
        for b in data:
            val += b
            if val > 255: val -= 255
        return (~val) & 0xFF

    def send_frame(self, frame_id, data_bytes, enhanced_checksum=False):
        pid = self.calculate_pid(frame_id)
        self._send_break()
        self._send_byte(0x55)
        self._send_byte(pid)
        for b in data_bytes: self._send_byte(b)
        
        chk_val = pid if enhanced_checksum else None
        chk = self.calculate_checksum(data_bytes, chk_val)
        self._send_byte(chk)

    def read_response(self, frame_id, length=5, timeout_ms=50):
        """Polls specific ID and asks for response"""
        while self.uart.any(): self.uart.read()
        
        pid = self.calculate_pid(frame_id)
        self._send_break()
        self._send_byte(0x55)
        self._send_byte(pid)
        
        start = time.ticks_ms()
        data = b''
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.uart.any():
                data += self.uart.read()
                if len(data) >= length: break
        return data

# ==========================================
# ISOLATION LOGIC
# ==========================================

def run_investigation():
    lin = LINMaster(tx_pin_num=0, rx_pin_num=1)
    
    print("\n=== TRIGGER ISOLATION TOOL ===")
    print("STEP 1: Reset Sensor (Unplug Power if needed, then press Enter)")
    print("If not plausible, we proceed assuming it's in default state.")
    # input("Press Enter to continue...") 
    time.sleep(2)
    
    print("\nSTEP 2: Establishing Baseline on ID 0x22")
    baseline = lin.read_response(0x22)
    print(f"Baseline Response: {[hex(x) for x in baseline]}")
    
    print("\nSTEP 3: Sending Global Config (0x3C)")
    # Using the payload that worked in scanner Phase 1
    # [0x22, 0x04, 0x2E, 0x03, 0x30, 0x4D, 0xFF, 0xFF]
    init_payload = [0x22, 0x04, 0x2E, 0x03, 0x30, 0x4D, 0xFF, 0xFF]
    lin.send_frame(0x3C, init_payload, enhanced_checksum=False)
    time.sleep(0.1)
    
    resp_after_init = lin.read_response(0x22)
    print(f"Post-Init Response: {[hex(x) for x in resp_after_init]}")
    
    print("\nSTEP 4: Finding the 'Live' Trigger")
    print("Replicating Fuzzy Scanner payload (0x03 0x30 0x4D).")
    print("Scanning IDs 0x00 to 0x25 to find what causes Byte 2 to jump to 0x8C.")
    
    # Payload from Fuzzy Scanner that caused the jump
    fuzzy_payload = [0x03, 0x30, 0x4D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF] 
    
    for test_id in range(0x00, 0x25):
        if test_id == 0x22: continue
        
        # Send the payload that worked in the scanner
        lin.send_frame(test_id, fuzzy_payload, enhanced_checksum=True)
        time.sleep(0.04) # Wait a bit more for processing
        
        # Check impact
        resp = lin.read_response(0x22, length=7, timeout_ms=80)
        
        hex_resp = [hex(x) for x in resp]
        print(f"ID {hex(test_id)} -> {hex_resp}")
        
        # logic to detect the 'jump'
        if len(resp) >= 5: # PID, D0, D1, D2, D3
             # Assuming resp[0]=0x55, resp[1]=PID, resp[2]=D0...
             # We want to check Data Byte 2 (Amb) which is at index 4 (0x55, PID, D0, D1, D2)
             # Wait, read_response returns everything read from UART.
             # If successful: 0x55, 0xE2, D0, D1, D2, D3, CHK
             # D2 is at index 4.
             
             if len(resp) > 4:
                 val_b2 = resp[4]
                 if val_b2 > 0x10: # If it jumps from ~0 or 8 to high value
                     print(f"!!! CHANGE DETECTED at ID {hex(test_id)} (B2={hex(val_b2)}) !!!")
                     
    print("\n=== ISOLATION COMPLETE ===")

if __name__ == "__main__":
    run_investigation()
