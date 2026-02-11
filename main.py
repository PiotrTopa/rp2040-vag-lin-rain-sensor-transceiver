import rp2
from machine import Pin, UART
import time

# ==========================================
# 1. LOWER LAYER: LIN DRIVER (PIO + UART)
# ==========================================
# This section handles the physical layer signal generation.
# It is specific to the RP2040 implementation.

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
        
        # 1. Setup UART for RX 
        self.uart = UART(0, baudrate=self.baud, tx=Pin(12), rx=Pin(rx_pin_num), bits=8, parity=None, stop=1)
        
        # 2. Setup PIO for TX
        sm_freq = self.baud * 8
        self.sm = rp2.StateMachine(0, lin_header_tx, freq=sm_freq, 
                                   out_base=Pin(tx_pin_num), 
                                   set_base=Pin(tx_pin_num))
        self.sm.active(1)
        print(f"LIN Master Initialized on TX={tx_pin_num}, RX={rx_pin_num} @ {baud} baud")

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

    def send_header(self, frame_id):
        pid = self.calculate_pid(frame_id)
        while self.uart.any(): self.uart.read() # Clear buffer
        self.sm.put(pid ^ 0xFF) # Trigger PIO
        return pid

    def read_response(self, length, timeout_ms=50):
        start = time.ticks_ms()
        data = b''
        while (time.ticks_diff(time.ticks_ms(), start) < timeout_ms):
            if self.uart.any():
                data += self.uart.read()
                if len(data) >= length: 
                    break
        return data

# ==========================================
# 2. MIDDLE LAYER: DECODER / PARSER
# ==========================================
# This layer decouples the specific sensor bytes from the application logic.
# If the sensor version changes, only this class needs to be updated.

class VAG_RLS_Parser:
    """
    Parser for VAG Rain-Light Sensor (Model 1K0 955 559 AH).
    Decodes the raw 4-byte LIN payload into meaningful properties.
    """
    def __init__(self):
        self.rain_intensity = 0
        self.light_forward = 0  # Tunnel detection
        self.light_ambient = 0  # Day/Night detection
        self.status = 0
        
    def parse(self, data_bytes):
        """
        Parses 4 bytes of data.
        Hypothesis:
        Byte 0: Rain (0-255)
        Byte 1: Forward Light (0-255)
        Byte 2: Ambient Light (0-255)
        Byte 3: Status check
        """
        if len(data_bytes) < 4:
            return False
            
        # ISOLATION: Bytes are treated independently to avoid the previous
        # bug where Byte 1 and 2 were merged into a 16-bit int.
        self.rain_intensity = data_bytes[0]
        self.light_forward  = data_bytes[1]
        self.light_ambient  = data_bytes[2]
        self.status         = data_bytes[3]
        return True

    def get_debug_string(self):
        """Formatted string for testing procedure."""
        return (f"B0[Rain]: {self.rain_intensity:3d} (0x{self.rain_intensity:02X}) | "
                f"B1[Fwd]: {self.light_forward:3d} (0x{self.light_forward:02X}) | "
                f"B2[Amb]: {self.light_ambient:3d} (0x{self.light_ambient:02X}) | "
                f"B3[St]: 0x{self.status:02X}")

# ==========================================
# 3. UPPER LAYER: DRL CONTROLLER FRAMEWORK
# ==========================================
# This logic is pure software and relies on the normalized data from the Parser.

class DRLController:
    """
    Manages the logic for Daytime Running Lights vs Low Beams.
    Includes Hysteresis and Timers.
    """
    STATE_DAY_DRL = "DAY_DRL"
    STATE_NIGHT_BEAM = "NIGHT_BEAM"
    
    def __init__(self):
        self.state = self.STATE_DAY_DRL
        self.last_switch_time = time.ticks_ms()
        
        # -- PARAMETERS (Tune these during testing) --
        # Ambient Light (Byte 2)
        self.THRES_NIGHT_ENTER = 20  # If Ambient < 20 -> Go Night
        self.THRES_NIGHT_LEAVE = 40  # If Ambient > 40 -> Go Day (Hysteresis)
        
        # Tunnel / Forward Light (Byte 1)
        self.THRES_TUNNEL_ENTER = 10 # If Forward < 10 -> Go Night
        
        # Timers (ms)
        self.DELAY_TO_NIGHT = 2000   # Delay before switching ON lights (avoid bridge shadows)
        self.DELAY_TO_DAY   = 5000   # Delay before switching OFF lights (safety)
        self.DELAY_TUNNEL   = 0      # Instant reaction for tunnel
        
        self.pending_state = None
        self.pending_start_time = 0

    def update(self, parser: VAG_RLS_Parser):
        """Run the state machine based on latest sensor data."""
        
        desired_state = self.state
        reaction_time = 0
        reason = ""

        # Logic 1: Ambient Light (Day/Night)
        if self.state == self.STATE_DAY_DRL:
            if parser.light_ambient < self.THRES_NIGHT_ENTER:
                desired_state = self.STATE_NIGHT_BEAM
                reaction_time = self.DELAY_TO_NIGHT
                reason = "Ambient Low (Dusk)"
                
            # Logic 2: Tunnel Override (Fast reaction)
            # Tunnel is detected when Forward light drops drastically
            if parser.light_forward < self.THRES_TUNNEL_ENTER:
                desired_state = self.STATE_NIGHT_BEAM
                reaction_time = self.DELAY_TUNNEL
                reason = "Forward Low (Tunnel)"
                
        elif self.state == self.STATE_NIGHT_BEAM:
            # Return to Day only if Ambient is bright enough
            if parser.light_ambient > self.THRES_NIGHT_LEAVE:
                desired_state = self.STATE_DAY_DRL
                reaction_time = self.DELAY_TO_DAY
                reason = "Ambient High (Dawn)"
                
            # Tunnel exit logic is implicitly handled by Ambient check 
            # (usually tunnels are dark globally), but sometimes Forward raises first.
            # Ideally we check if Forward is High AND Ambient is High.

        # Hysteresis / Timer Logic
        current_time = time.ticks_ms()
        
        if desired_state != self.state:
            if self.pending_state != desired_state:
                # Start timer
                self.pending_state = desired_state
                self.pending_start_time = current_time
            else:
                # Timer running
                elapsed = time.ticks_diff(current_time, self.pending_start_time)
                if elapsed > reaction_time:
                    self.state = desired_state
                    self.pending_state = None
                    print(f"!!! STATE CHANGE: {self.state} ({reason}) !!!")
        else:
            # Reset timer if condition cleared
            self.pending_state = None
            
        return self.state

# ==========================================
# 4. MAIN EXECUTION LOOP
# ==========================================

def run():
    lin = LINMaster(tx_pin_num=0, rx_pin_num=1)
    parser = VAG_RLS_Parser()
    controller = DRLController()
    
    SENSOR_ID = 0x22
    PID_ECHO = 0xE2
    
    print("\n" + "="*60)
    print(" VAG RLS TESTING FRAMEWORK STARTED")
    print("="*60)
    print(" 1. Cover entire sensor -> Check B2 (Ambient) drops to 0")
    print(" 2. Shine light into front eye -> Check B1 (Fwd) spikes")
    print(" 3. Spray sensor (on glass) -> Check B0 (Rain) increases")
    print("="*60)
    
    while True:
        try:
            # 1. Send Header
            pid = lin.send_header(SENSOR_ID)
            
            # 2. Read Response (PID + 4 Data + Checksum = 6 bytes)
            # We ask for a bit more (7) to be safe with timing/noise
            raw = lin.read_response(length=7, timeout_ms=60)
            
            if raw:
                # Find the echo PID (0xE2)
                pid_byte = bytes([pid])
                if pid_byte in raw:
                    idx = raw.index(pid_byte)
                    frame = raw[idx+1:]
                    
                    if len(frame) >= 5: # 4 Data + 1 Checksum
                        data_bytes = frame[0:4]
                        chk_recv = frame[4]
                        
                        # Validate Checksum (Classic)
                        chk_calc = lin.calculate_classic_checksum(data_bytes)
                        
                        if chk_recv == chk_calc:
                            # Valid Frame -> Parse it
                            parser.parse(data_bytes)
                            
                            # Run Logic
                            current_state = controller.update(parser)
                            
                            # Log output (Raw + Logic decision)
                            # Using \r to overwrite line for cleaner dashboard look in terminal
                            # remove end='\r' if using simple scrolling log
                            print(f"[{current_state}] {parser.get_debug_string()}")
                            
                        else:
                            # print(f"Checksum Error: Recv {hex(chk_recv)} != Exp {hex(chk_calc)}")
                            pass
            
            time.sleep(0.1) # loop delay
            
        except KeyboardInterrupt:
            print("\nStopped.")
            break

if __name__ == "__main__":
    run()
