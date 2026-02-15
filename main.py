"""
VAG Rain/Light/Humidity Sensor — BCM Emulator

81A 955 555 A (MQB RLS — G397_RLFSS)

LIN master schedule loop that keeps the sensor fully active.
Decodes: forward light (16-bit), temperature, humidity/dew point,
rain intensity. DRL switching logic included.

Hardware: RP2040 + NPN transistor LIN transceiver
  TX = GPIO0, RX = GPIO1, 19200 baud

Usage (REPL):
    from main import *
    run()                    # BCM emulation
    run(wiper=0)             # Wiper off
    run(sensitivity=4)       # Rain sensitivity 0-7
    run(verbose=True)        # Show raw frames
"""

from lin import LIN
import time


# ================================================================
# Configuration
# ================================================================

# Slave-response frame IDs (81A 955 555 A)
ID_LIGHT = 0x23     # Forward light sensor + rolling counter
ID_ENV   = 0x29     # Temperature, humidity, solar
ID_RAIN  = 0x30     # FIR rain detection (needs master cmd to activate)

# Master command frame ID
ID_CMD = 0x20


def _build_cmd(wiper=1, sensitivity=2, kl15=True):
    """Build 8-byte master command payload."""
    b0 = 0x80 if kl15 else 0x00     # bit7 = ignition ON
    if wiper:
        b0 |= 0x01                  # bit0 = wiper active
    modes = {0: 0x00, 1: 0x04, 2: 0x08, 3: 0x0C}
    b1 = modes.get(wiper, 0x04)
    b2 = min(max(sensitivity, 0), 7)
    return [b0, b1, b2, 0x00, 0x00, 0x00, 0x00, 0x00]


# ================================================================
# Sensor Decoder
# ================================================================

class Sensor:
    """Decoded readings from all three slave-response frames."""

    def __init__(self):
        # Frame 0x23 — Forward light sensor
        self.counter = 0          # 4-bit rolling (byte[0] low nibble)
        self.light = 0            # 16-bit forward ambient (byte[4:5] LE)

        # Frame 0x29 — Environmental
        self.solar = 0            # byte[0] raw — solar/ambient intensity
        self.temp_raw = 0         # byte[2] raw — temperature
        self.temp2_raw = 0        # byte[5] raw — secondary temp / dew point

        # Frame 0x30 — Rain / FIR
        self.rain0 = 0            # byte[0] — rain intensity
        self.rain1 = 0            # byte[1] — rain status/flags
        self.fir = False          # True when FIR LEDs active

        self._ok = [False] * 3   # [light, env, rain] received at least once

    def feed(self, fid, data):
        """Decode frame data by ID."""
        if not data:
            return
        if fid == ID_LIGHT and len(data) >= 6:
            self.counter = data[0] & 0x0F
            self.light = data[4] | (data[5] << 8)
            self._ok[0] = True
        elif fid == ID_ENV and len(data) >= 6:
            self.solar = data[0]
            self.temp_raw = data[2]
            self.temp2_raw = data[5]
            self._ok[1] = True
        elif fid == ID_RAIN and len(data) >= 2:
            self.rain0 = data[0]
            self.rain1 = data[1]
            self.fir = data[0] != 0 or data[1] != 0
            self._ok[2] = True

    @property
    def temp(self):
        """Temperature in C (raw * 0.5 - 40)."""
        return self.temp_raw * 0.5 - 40.0

    @property
    def temp2(self):
        """Dew point temperature in C (raw * 0.5 - 40)."""
        return self.temp2_raw * 0.5 - 40.0

    @property
    def light_pct(self):
        """Light as percentage: 0xEC00=0% (dark) .. 0xEFFF=100% (bright)."""
        lo, hi = 0xEC00, 0xEFFF
        if self.light <= lo:
            return 0
        if self.light >= hi:
            return 100
        return (self.light - lo) * 100 // (hi - lo)

    def line(self):
        """One-line status summary."""
        parts = []
        if self._ok[0]:
            pct = self.light_pct
            n = pct // 10
            bar = '#' * n + '-' * (10 - n)
            parts.append("Light %3d%% [%s] 0x%04X" % (pct, bar, self.light))
        if self._ok[1]:
            parts.append("%.1fC  Dew %.1fC" % (self.temp, self.temp2))
        if self._ok[2]:
            if self.fir:
                parts.append("Rain: ACTIVE(%02X,%02X)" % (self.rain0, self.rain1))
            else:
                parts.append("Rain: dry")
        return " | ".join(parts) if parts else "no data"


# ================================================================
# DRL Controller
# ================================================================

class DRL:
    """Day/night state machine for headlight control."""

    def __init__(self, dark=0xEA00, bright=0xEC00):
        self.state = "DAY"
        self.dark = dark
        self.bright = bright
        self._pend = None
        self._t0 = 0

    def update(self, light):
        want, delay = self.state, 0
        if self.state == "DAY" and light < self.dark:
            want, delay = "NIGHT", 3000
        elif self.state == "NIGHT" and light > self.bright:
            want, delay = "DAY", 5000
        now = time.ticks_ms()
        if want != self.state:
            if self._pend != want:
                self._pend, self._t0 = want, now
            elif time.ticks_diff(now, self._t0) >= delay:
                old = self.state
                self.state, self._pend = want, None
                print(">>> DRL: %s -> %s <<<" % (old, self.state))
        else:
            self._pend = None
        return self.state


# ================================================================
# BCM Emulator — Main Scheduler
# ================================================================

def run(cmd_id=None, wiper=1, sensitivity=2, verbose=False):
    """
    BCM emulator: LIN master schedule loop.

    Every cycle (~60ms):
      1. TX master command -> sensor
      2. RX 0x23 (forward light)
      3. RX 0x30 (rain / FIR)
      4. RX 0x29 (environment, every 5th cycle)
      5. Update DRL state
    """
    if cmd_id is None:
        cmd_id = ID_CMD

    lin = LIN()
    sensor = Sensor()
    drl = DRL()
    cmd_data = _build_cmd(wiper=wiper, sensitivity=sensitivity)

    print("\n" + "=" * 64)
    print(" VAG RLS 81A 955 555 A \u2014 BCM Emulator")
    print(" Light: 0xEC00 (dark) .. 0xEFFF (bright)")
    print(" Temp: raw*0.5-40  Dew: dew point  Rain: FIR active/dry")
    print("=" * 64)
    print("CMD: 0x%02X [%s]  Wiper=%d Sens=%d" % (
        cmd_id, " ".join("%02X" % b for b in cmd_data), wiper, sensitivity))
    print("Ctrl+C to stop\n")

    cycle = 0
    try:
        while True:
            lin.send(cmd_id, cmd_data)
            time.sleep_ms(5)

            for fid in (ID_LIGHT, ID_RAIN):
                data, ok = lin.recv(fid)
                if data and ok:
                    sensor.feed(fid, data)
                time.sleep_ms(2)

            if cycle % 5 == 0:
                data, ok = lin.recv(ID_ENV)
                if data and ok:
                    sensor.feed(ID_ENV, data)

            st = drl.update(sensor.light) if sensor._ok[0] else "???"

            if verbose:
                print("[%04d][%-5s] %s" % (cycle, st, sensor.line()))
            else:
                print("[%-5s] %s" % (st, sensor.line()))

            cycle += 1
            time.sleep_ms(50)
    except KeyboardInterrupt:
        print("\nStopped after %d cycles." % cycle)


if __name__ == "__main__":
    run()
