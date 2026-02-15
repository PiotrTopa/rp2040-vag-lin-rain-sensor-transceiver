"""
Live LID Chart — VAG Rain/Light Sensor

PC-side tool: connects to RP2040 serial, runs diag.py stream(),
and plots live channel data with matplotlib.

  ** Disconnect MicroPico REPL before running! **

Requirements:
    pip install pyserial matplotlib

Usage:
    python chart.py                   # Auto-detect port
    python chart.py COM5              # Specify port
    python chart.py --lids 02,04,07   # Custom LIDs
    python chart.py --list            # Show available ports
"""

import sys
import time
import argparse
import threading
from collections import deque

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Missing pyserial.  pip install pyserial")
    sys.exit(1)

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
except ImportError:
    print("Missing matplotlib.  pip install matplotlib")
    sys.exit(1)


# ================================================================
# Configuration
# ================================================================

WINDOW = 300          # rolling window size (samples)
UPDATE_MS = 400       # chart refresh interval (ms)
DEFAULT_LIDS = [0x02, 0x04, 0x05, 0x07, 0x08, 0x09]

COLORS = ['#1E88E5', '#F4511E', '#43A047', '#8E24AA']
CH_LABELS = ['ch0', 'ch1', 'ch2', 'ch3']


# ================================================================
# Port detection
# ================================================================

def list_ports():
    """Print all available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    for p in ports:
        vid = ("0x%04X" % p.vid) if p.vid else "----"
        print("  %-8s  VID=%s  %s" % (p.device, vid, p.description))


def find_pico_port():
    """Auto-detect RP2040 port (VID 0x2E8A)."""
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x2E8A:
            return p.device
    # Fallback: first available port
    ports = serial.tools.list_ports.comports()
    if ports:
        print("No RP2040 found, using first available port:")
        list_ports()
        return ports[0].device
    return None


# ================================================================
# Live chart
# ================================================================

class LiveChart:
    """Serial reader + matplotlib live plotter."""

    def __init__(self, port, lids):
        self.port = port
        self.lids = lids
        self.ser = None
        self.lock = threading.Lock()
        self.sweep = 0
        self.ms = 0

        # Per-LID, per-channel rolling data
        self.data = {}
        for lid in lids:
            self.data[lid] = [deque(maxlen=WINDOW) for _ in range(4)]

    # --- Serial / raw REPL ---

    def connect(self):
        """Open serial and start RP2040 stream via raw REPL."""
        self.ser = serial.Serial(self.port, 115200, timeout=0.5)
        time.sleep(0.5)

        # Interrupt any running code
        self.ser.write(b'\x03\x03')
        time.sleep(0.5)
        self.ser.reset_input_buffer()

        # Enter raw REPL (Ctrl+A) — no echo, clean output
        self.ser.write(b'\x01')
        time.sleep(0.3)
        self.ser.reset_input_buffer()

        # Build and send the stream command
        lid_list = ','.join('0x%02X' % l for l in self.lids)
        code = 'from diag import stream\nstream([%s])\n' % lid_list
        self.ser.write(code.encode())
        self.ser.write(b'\x04')   # Ctrl+D = execute
        time.sleep(1.0)
        self.ser.reset_input_buffer()

        print("Streaming %d LIDs from %s..." % (len(self.lids), self.port))

    def disconnect(self):
        """Stop stream and close serial."""
        if self.ser and self.ser.is_open:
            self.ser.write(b'\x03\x03')   # Ctrl+C = stop stream
            time.sleep(0.2)
            self.ser.write(b'\x02')        # Ctrl+B = exit raw REPL
            time.sleep(0.1)
            self.ser.close()
        print("Disconnected.")

    # --- Background reader ---

    def _read_loop(self):
        """Parse serial lines in background thread."""
        while True:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()

                if line.startswith('D:'):
                    # D:02:256,264,256,264
                    parts = line.split(':')
                    if len(parts) >= 3:
                        lid = int(parts[1], 16)
                        vals = [int(v) for v in parts[2].split(',')[:4]]
                        with self.lock:
                            if lid in self.data:
                                for i, v in enumerate(vals):
                                    self.data[lid][i].append(v)

                elif line.startswith('S:'):
                    # S:42:398
                    parts = line.split(':')
                    if len(parts) >= 3:
                        with self.lock:
                            self.sweep = int(parts[1])
                            self.ms = int(parts[2])

            except (ValueError, IndexError):
                pass
            except serial.SerialException:
                break
            except Exception:
                time.sleep(0.05)

    # --- Matplotlib setup ---

    def _setup_plot(self):
        n = len(self.lids)
        cols = min(n, 3)
        rows = (n + cols - 1) // cols

        self.fig, axes = plt.subplots(
            rows, cols, figsize=(5 * cols, 3 * rows), squeeze=False)
        self.fig.canvas.manager.set_window_title(
            'VAG RLS \u2014 Live LID Monitor')

        self.axes = {}
        self.lines = {}

        for idx, lid in enumerate(self.lids):
            r, c = divmod(idx, cols)
            ax = axes[r][c]
            ax.set_title('LID 0x%02X' % lid, fontsize=11, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.set_xlabel('sample')
            self.axes[lid] = ax
            self.lines[lid] = []

            for ch in range(4):
                line, = ax.plot([], [], color=COLORS[ch],
                                label=CH_LABELS[ch], linewidth=1.5, alpha=0.85)
                self.lines[lid].append(line)
            ax.legend(loc='upper left', fontsize=7, ncol=2)

        # Hide unused subplots
        for idx in range(n, rows * cols):
            r, c = divmod(idx, cols)
            axes[r][c].set_visible(False)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

    def _update(self, frame):
        """Animation callback."""
        with self.lock:
            sweep = self.sweep
            ms = self.ms

            for lid in self.lids:
                ax = self.axes[lid]
                latest = []

                for ch in range(4):
                    d = list(self.data[lid][ch])
                    if d:
                        self.lines[lid][ch].set_data(range(len(d)), d)
                        latest.append('%04X' % d[-1])
                    else:
                        latest.append('----')

                # Auto-scale axes
                all_v = []
                max_n = 0
                for ch in range(4):
                    d = self.data[lid][ch]
                    all_v.extend(d)
                    max_n = max(max_n, len(d))

                if all_v:
                    ax.set_xlim(0, max(max_n, 10))
                    vmin, vmax = min(all_v), max(all_v)
                    margin = max((vmax - vmin) * 0.1, 2)
                    ax.set_ylim(vmin - margin, vmax + margin)

                ax.set_title('LID 0x%02X  [%s]' % (lid, ' '.join(latest)),
                             fontsize=10, fontweight='bold')

        self.fig.suptitle('Sweep #%d  %dms/cycle' % (sweep, ms), fontsize=12)
        return []

    # --- Main entry ---

    def run(self):
        """Connect, stream, and show live chart."""
        self.connect()

        reader = threading.Thread(target=self._read_loop, daemon=True)
        reader.start()

        self._setup_plot()
        self.ani = animation.FuncAnimation(
            self.fig, self._update, interval=UPDATE_MS,
            blit=False, cache_frame_data=False)

        try:
            plt.show()
        finally:
            self.disconnect()


# ================================================================
# CLI
# ================================================================

def main():
    ap = argparse.ArgumentParser(
        description='Live LID chart for VAG rain/light sensor')
    ap.add_argument('port', nargs='?', help='Serial port (e.g. COM5)')
    ap.add_argument('--lids', help='Comma-separated hex LIDs (e.g. 02,04,07)')
    ap.add_argument('--list', action='store_true', help='List serial ports')
    args = ap.parse_args()

    if args.list:
        list_ports()
        return

    port = args.port or find_pico_port()
    if not port:
        print("No serial port found.")
        print("Usage: python chart.py COM5")
        print("       python chart.py --list")
        sys.exit(1)

    lids = list(DEFAULT_LIDS)
    if args.lids:
        lids = [int(x, 16) for x in args.lids.split(',')]

    print("Port: %s" % port)
    print("LIDs: %s" % ', '.join('0x%02X' % l for l in lids))
    print("Disconnect MicroPico REPL first if connected!")
    print()

    chart = LiveChart(port, lids)
    chart.run()


if __name__ == '__main__':
    main()
