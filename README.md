# RP2040 VAG LIN Rain/Light Sensor Transceiver

RP2040-based LIN bus master for interfacing with the **VAG Rain/Light/Humidity Sensor** — part number `81A 955 555 A` (MQB platform). The RP2040 simulates the car's Body Control Module (BCM) to read environmental data from the sensor over LIN bus.

## Target Sensor

| Property    | Value                                    |
|-------------|------------------------------------------|
| Part Number | `81A 955 555 A`                          |
| Platform    | MQB (Audi A3 8V, Q2, Skoda, Seat, VW)   |
| Protocol    | LIN 2.x (Enhanced Checksum)              |
| Baud Rate   | 19200 baud (8N1)                         |
| Supply      | 12V DC automotive                        |

## Features

- **PIO-based LIN master** — RP2040 Programmable I/O for precise Break + Sync + PID generation
- **NPN transistor TX driver** — no dedicated LIN transceiver IC needed
- **Frame scheduler** — proper LIN master schedule with master command TX + slave response polling
- **Master command profiles** — configurable payloads to activate FIR rain detection
- **Temperature decoding** — °C = raw × 0.5 − 40
- **DRL controller** — automatic daytime running light switching based on forward light sensor
- **Live development tools** — real-time monitor, injection, timing, statistics

## Discovered LIN Frames

| Frame ID | PID    | Checksum | Content                        | Status    |
|----------|--------|----------|--------------------------------|-----------|
| `0x23`   | `0xA3` | Enhanced | Light intensity + counter      | Working   |
| `0x29`   | `0xE9` | Enhanced | Environmental (temp, humidity) | Working   |
| `0x30`   | `0xF0` | Enhanced | Extended / rain data           | Dormant*  |

\* Frame 0x30 is static without master command — FIR needs periodic BCM commands to activate.

## Hardware

Single NPN transistor (BC547) open-collector driver with resistive voltage divider RX. Full circuit in [WIRING.md](WIRING.md).

| RP2040 Pin | Function | Description                   |
|------------|----------|-------------------------------|
| GPIO0      | LIN TX   | To NPN base via 1kΩ          |
| GPIO1      | LIN RX   | From voltage divider (33k/10k)|

## Project Structure

```
├── lin.py             # Shared LIN 2.x master driver (PIO + UART)
├── main.py            # BCM emulator — sensor scheduler + decoder + DRL
├── diag.py            # Diagnostic tool — LID/DID reader, coding
├── analyze_lids.py    # PC-side — LID data visualizer & change tracker
├── WIRING.md          # Circuit description and schematic
├── DISCOVERY.md       # Data field mapping notes
├── FINDINGS.md        # Detailed findings and byte layouts
├── archive/           # Old exploration scripts (superseded)
│   ├── scanner.py     # LIN bus ID scanner
│   ├── decoder.py     # Live monitor + injection tools
│   ├── diag_fuzzer.py # UDS/NAD discovery
│   ├── diag2.py       # KWP2000 measuring block reader
│   ├── diag3.py       # DID scanner
│   └── diag4.py       # Combo fuzzer (coding × master commands)
└── LICENSE
```

## Usage

### 1. BCM Emulator (main.py) — runs on RP2040

```python
from main import *
run()                    # Default: ignition ON, wiper auto, sensitivity 2
run(wiper=0)             # Wiper off
run(sensitivity=4)       # Rain sensitivity 0-7
run(verbose=True)        # Show cycle numbers
```

Schedule loop (~60ms per cycle):
1. **TX** master command frame → sensor
2. **RX** forward light (0x23) + rain (0x30) every cycle
3. **RX** environment (0x29) every 5th cycle
4. DRL state machine update

### 2. Diagnostics (diag.py) — runs on RP2040

```python
from diag import *
info()                   # Sensor identification + coding
run()                    # Full dump: info + all LIDs decoded
read_lid(0x04)           # Read single measuring block
read_all_lids()          # Read all 32 LIDs
read_coding()            # Read coding DID 0x0611
scan_dids()              # Scan DID range 0x0580-0x0800
```

### 3. LID Analyzer (analyze_lids.py) — runs on PC

```python
python analyze_lids.py                  # Analyze embedded sample data
python analyze_lids.py capture.txt      # Analyze from file
python analyze_lids.py c1.txt c2.txt    # Compare captures
```

Parses captured LID output, shows per-channel statistics:
- **Dynamic** channels highlighted with value range
- **Static** channels shown in compact table
- Capture-by-capture heatmap for change tracking

## LIN Protocol

| Parameter  | Value                                                  |
|------------|--------------------------------------------------------|
| Baud Rate  | 19200                                                  |
| Frame      | 8N1                                                    |
| Break      | ≥13 bit times dominant (PIO-generated)                 |
| Checksum   | Enhanced (LIN 2.x) for data frames, Classic for 0x3C/0x3D |
| Sync       | 0x55                                                   |

### Frame Structure

```
[ Break ≥13 bits ] [ Delimiter 1 bit ] [ Sync 0x55 ] [ PID ] [ Data 0-8 bytes ] [ Checksum ]
```

## BOM

| Qty | Component       | Value          | Purpose                    |
|-----|-----------------|----------------|----------------------------|
| 1   | NPN Transistor  | BC547 / 2N3904 | TX open-collector driver   |
| 1   | Signal Diode    | 1N4148         | Master termination         |
| 1   | Zener Diode     | 3.3V           | RX overvoltage clamp       |
| 1   | Resistor        | 1kΩ            | Master pull-up (termination)|
| 1   | Resistor        | 1kΩ            | TX base resistor           |
| 1   | Resistor        | 10kΩ           | TX base pull-down          |
| 1   | Resistor        | 33kΩ           | RX divider (top)           |
| 1   | Resistor        | 10kΩ           | RX divider (bottom)        |

## Previous Hardware

> An earlier iteration targeted a different sensor with LIN 1.x classic checksum. 
> It was abandoned due to an overly complex initialization protocol and inability 
> to reliably activate the FIR rain detection subsystem. This project was restarted 
> with the `81A 955 555 A` (MQB, LIN 2.x).

## References

- [LIN Specification 2.2A](https://www.lin-cia.org/)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf)

## License

See [LICENSE](LICENSE).
