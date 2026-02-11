import rp2
from machine import Pin, UART
import time

# ==========================================
# UNIVERSAL LIN MASTER TX (PIO)
# ==========================================
@rp2.asm_pio(out_init=rp2.PIO.OUT_LOW, set_init=rp2.PIO.OUT_LOW, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def lin_master_tx():
    wrap_target()
    label("param_entry")
    pull(block)
    out(y, 8)
    out(x, 1)
    jmp(not_x, "send_byte")
    
    set(pins, 1)       [7]
    set(x, 12)         [7]
    label("break_loop")
    nop()              [7]
    jmp(x_dec, "break_loop") [7]
    
    set(pins, 0)       [7]   
    jmp("param_entry")
    
    label("send_byte")
    mov(osr, y)        [7]
    set(pins, 1)       [7]   
    set(x, 7)                
    label("bit_loop")
    out(pins, 1)       [6]   
    jmp(x_dec, "bit_loop")
    set(pins, 0)       [7]   
    wrap()

# ==========================================
# CORE LIN INTERFACE
# ==========================================
class LINMaster:
    def __init__(self, tx_pin=0, rx_pin=1, baud=19200):
        self.baud = baud
        self.uart = UART(0, baudrate=self.baud, tx=Pin(12), rx=Pin(rx_pin), bits=8, parity=None, stop=1)
        sm_freq = self.baud * 8
        self.sm = rp2.StateMachine(0, lin_master_tx, freq=sm_freq, out_base=Pin(tx_pin), set_base=Pin(tx_pin))
        self.sm.active(1)

    def send_break(self):
        self.sm.put((1 << 8) | 0x00)

    def send_byte(self, b):
        self.sm.put((0 << 8) | (b ^ 0xFF))

    def calculate_pid(self, fid):
        p0 = ((fid >> 0) ^ (fid >> 1) ^ (fid >> 2) ^ (fid >> 4)) & 1
        p1 = ~((fid >> 1) ^ (fid >> 3) ^ (fid >> 4) ^ (fid >> 5)) & 1
        return (fid & 0x3F) | (p0 << 6) | (p1 << 7)
    
    def calculate_checksum(self, data, pid=None):
        val = pid if pid is not None else 0
        for b in data:
            val += b
            if val > 255:
                val -= 255
        return (~val) & 0xFF

    def send_frame(self, frame_id, payload_bytes, use_enhanced_checksum=False):
        pid = self.calculate_pid(frame_id)
        self.send_break()
        self.send_byte(0x55)
        self.send_byte(pid)
        for b in payload_bytes:
            self.send_byte(b)
        chk = self.calculate_checksum(payload_bytes, pid if use_enhanced_checksum else None)
        self.send_byte(chk)

    def poll_slave(self, frame_id, response_length, timeout_ms=30):
        while self.uart.any(): self.uart.read()
        pid = self.calculate_pid(frame_id)
        self.send_break()
        self.send_byte(0x55)
        self.send_byte(pid)
        
        expected_bytes = 2 + response_length + 1
        start = time.ticks_ms()
        raw = b''
        while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
            if self.uart.any():
                raw += self.uart.read()
                if len(raw) >= expected_bytes:
                    break
        return raw

# ==========================================
# B2 & B3 EXHAUSTIVE SWEEP
# ==========================================
class RLSB2B3Scanner:
    def __init__(self, lin):
        self.lin = lin

    def init_coding(self):
        print("\n=== FAZA 1: JEDNORAZOWE KODOWANIE (0x3C) ===")
        coding_payload = [0x00, 0x06, 0xB0, 0x03, 0x50, 0x4D, 0xFF, 0xFF]
        for _ in range(3):
            self.lin.send_frame(0x3C, coding_payload, use_enhanced_checksum=False)
            time.sleep_ms(50)
        time.sleep(1)

    def parse_0x22(self, raw_data):
        if not raw_data or len(raw_data) < 7:
            return None
        if raw_data[0] != 0x55 or raw_data[1] != 0xE2: 
            idx = raw_data.find(b'\x55\xE2')
            if idx == -1 or len(raw_data) - idx < 7:
                return None
            raw_data = raw_data[idx:]
            
        cntr = raw_data[2] & 0x0F
        status = raw_data[3]
        fir_rain = raw_data[4]
        ambient = raw_data[5]
        return (cntr, status, fir_rain, ambient)

    def run_b2_b3_sweep(self):
        ignition_id = 0x00
        ignition_payload = [0x01, 0x00]
        wiper_id = 0x21
        
        print("\n=== FAZA 2: DEEP SWEEP (Bajt 2 & Bajt 3) ===")
        print("Zablokowano: B0=0x05, B1=0x00")
        print("Skanowanie 65 536 kombinacji... Oszczędne logowanie.")
        
        for b2 in range(256):
            print(f"  -> Postęp: B2 = {hex(b2)} (Sweep B3: 0x00-0xFF)")
            
            for b3 in range(256):
                payload = [0x05, 0x00, b2, b3]
                
                alive = True
                
                # Szybkie pompowanie przez 4 cykle (wiemy, że samo 0x05 tyle przeżyje)
                for _ in range(4):
                    self.lin.send_frame(ignition_id, ignition_payload, False)
                    time.sleep_ms(15)
                    self.lin.send_frame(wiper_id, payload, False)
                    time.sleep_ms(15)
                    self.lin.poll_slave(0x22, response_length=4)
                    time.sleep_ms(10)
                
                # Sprawdzamy cykl 5 i 6 (prawdziwa weryfikacja)
                for _ in range(2):
                    self.lin.send_frame(ignition_id, ignition_payload, False)
                    time.sleep_ms(15)
                    self.lin.send_frame(wiper_id, payload, False)
                    time.sleep_ms(15)
                    
                    resp = self.lin.poll_slave(0x22, response_length=4)
                    parsed = self.parse_0x22(resp)
                    
                    if not parsed or parsed[1] == 0x21:
                        alive = False
                        break
                    time.sleep_ms(10)
                    
                # Jeśli przeżył krytyczne cykle, uruchamiamy długi test (2 sekundy)
                if alive:
                    print(f"\n[?] KANDYDAT PRZEŻYŁ 6 CYKLI: {[hex(x) for x in payload]}. Głęboki test...")
                    deep_success = True
                    for _ in range(30):
                        self.lin.send_frame(ignition_id, ignition_payload, False)
                        time.sleep_ms(15)
                        self.lin.send_frame(wiper_id, payload, False)
                        time.sleep_ms(15)
                        
                        resp = self.lin.poll_slave(0x22, response_length=4)
                        parsed = self.parse_0x22(resp)
                        
                        if not parsed or parsed[1] == 0x21:
                            deep_success = False
                            print("   [-] Odrzucony. Wrócił do 0x21.")
                            break
                        time.sleep_ms(15)
                        
                    if deep_success:
                        print(f"\n[!!!] BINGO [!!!]")
                        print(f"Znaleziono stabilne sterowanie dla ID: 0x21")
                        print(f"Payload: {[hex(x) for x in payload]}")
                        
                        with open("bcm_hit.txt", "w") as f:
                            f.write(f"ID: {hex(wiper_id)}\nPayload: {[hex(x) for x in payload]}\n")
                            
                        return payload

        print("\n[-] Skanowanie B2 i B3 nie znalazło stabilnego payloadu.")
        return None

def main():
    lin = LINMaster(tx_pin=0, rx_pin=1, baud=19200)
    scanner = RLSB2B3Scanner(lin)
    
    scanner.init_coding()
    valid_wiper_payload = scanner.run_b2_b3_sweep()
    
    if valid_wiper_payload is not None:
        print("\n=== SYSTEM ONLINE ===")
        print("Pomiary optyczne działają. Testuj czujnik latarką i wodą.")
        while True:
            lin.send_frame(0x00, [0x01, 0x00], False)
            time.sleep_ms(15)
            lin.send_frame(0x21, valid_wiper_payload, False)
            time.sleep_ms(15)
            
            resp = lin.poll_slave(0x22, response_length=4)
            parsed = scanner.parse_0x22(resp)
            
            if parsed:
                cntr, stat, rain, ambient = parsed
                print(f"Cnt: {cntr:02X} | Stat: {stat:02X} | Rain/FIR: {rain:02X} | Amb: {ambient:02X}")
            time.sleep_ms(50)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nPrzerwano działanie skryptu.")