# RP2040 VAG LIN Rain/Light Sensor Transceiver

An RP2040-based LIN bus master transceiver for interfacing with the **VW Rain/Light Sensor (G397)** — part number `1K0 955 559 AH` (manufactured by Hella). The RP2040 simulates the car's Body Control Module (BCM) to communicate with the sensor over the LIN bus using PIO-based UART.

## Overview

The VAG rain/light sensor uses the **LIN (Local Interconnect Network)** protocol to report rain intensity and ambient light levels to the car's BCM. This project replaces the BCM with an RP2040 microcontroller, allowing direct reading of the sensor data outside the vehicle environment.

### Key Features

- **RP2040 as LIN Master** — simulates the BCM role on the LIN bus
- **PIO-based LIN UART** — uses RP2040's Programmable I/O for precise LIN frame timing, including Break field generation
- **Discrete level-shifting** — single NPN transistor TX driver + resistive voltage divider RX, no dedicated LIN transceiver IC required
- **12V automotive bus** interfaced to 3.3V logic safely

## Target Sensor

| Property        | Value                          |
|-----------------|--------------------------------|
| Designation     | Rain/Light Sensor G397         |
| Part Number     | `1K0 955 559 AH`              |
| Manufacturer    | Hella (for VW)                 |
| Generation      | 1K0 (Golf Mk5, Passat B6 era) |
| Protocol        | LIN Bus (LIN 1.x / 2.0)       |
| Connector       | 3-pin                         |

### Sensor Connector Pinout

> **Verify pin numbers molded into the connector housing before applying power.**

| Pin | Name | Level    | Description                                         |
|-----|------|----------|-----------------------------------------------------|
| 1   | VCC  | +12V DC  | Terminal 15 or 30 (switched or constant). Automotive 12V required. |
| 2   | GND  | Ground   | Terminal 31. Common ground with RP2040 circuit.      |
| 3   | LIN  | Data     | LIN Bus I/O. Pull-up to 12V required (master termination). |

> **Important:** The sensor requires automotive voltage (~12V). It will **not** operate on 3.3V or 5V.

## LIN Protocol Parameters

| Parameter      | Value                                                    |
|----------------|----------------------------------------------------------|
| Baud Rate      | 19200 baud                                               |
| UART Config    | 8 data bits, No parity, 1 Stop bit (8N1)                |
| Break Field    | ≥ 13 bit times dominant (low) — generated via PIO        |
| Checksum        | Classic (LIN 1.x) or Enhanced (LIN 2.x) |
| Sync Byte      | `0x55`                                                   |

### LIN Frame Structure

```
[ Break (≥13 bits low) ] [ Break Delimiter (1 bit high) ] [ Sync 0x55 ] [ Protected ID ] [ Data 0..8 bytes ] [ Checksum ]
```

The RP2040 (master) sends the **header** (Break + Sync + PID). For slave-response frames, the sensor then responds with **data bytes + checksum**.

## Hardware Architecture

The transceiver uses a **single NPN transistor** design for TX, with logic inversion handled in hardware via the RP2040's PIO software logic.

See [WIRING.md](WIRING.md) for the full circuit description, schematic, and component values.

### RP2040 Pin Assignment

| RP2040 Pin | Function | Direction | Description                     |
|------------|----------|-----------|---------------------------------|
| GPIO0      | LIN TX   | Output    | To NPN base via 1kΩ resistor   |
| GPIO1      | LIN RX   | Input     | From voltage divider (shared bus, directly from LIN bus via divider) |

> **Note:** TX and RX use separate GPIOs. GPIO0 is the primary LIN bus TX GPIO.

### Signal Inversion (TX)

The single NPN common-emitter topology **inverts** the logic:

| RP2040 GPIO | Transistor | LIN Bus          | LIN State  |
|-------------|------------|------------------|------------|
| LOW (0V)    | OFF        | Pulled to 12V    | Recessive (Idle) |
| HIGH (3.3V) | ON         | Pulled to GND    | Dominant   |

This inversion is compensated by the PIO state machine program, which drives the pin with inverted logic compared to standard UART.

## Bill of Materials (BOM)

| Qty | Component                 | Value / Type     | Purpose                        |
|-----|---------------------------|------------------|--------------------------------|
| 1   | NPN Transistor            | BC547 / 2N3904   | TX open-collector driver       |
| 1   | Signal Diode              | 1N4148 / 1N4007  | Master termination backflow protection |
| 1   | Zener Diode               | 3.3V (BZX55C3V3) | RX GPIO overvoltage clamp      |
| 1   | Resistor                  | 1kΩ (1/4W)       | Master termination pull-up     |
| 1   | Resistor                  | 1kΩ              | TX base resistor               |
| 1   | Resistor                  | 10kΩ             | TX base pull-down (boot safety)|
| 1   | Resistor                  | 33kΩ*            | RX voltage divider (top)       |
| 1   | Resistor                  | 10kΩ*            | RX voltage divider (bottom)    |

> \* RX divider values of **33kΩ / 10kΩ** give ~3.3V at 14.4V input. Alternative safe combinations: **42kΩ / 12kΩ** (~3.2V @ 14.4V) or **45kΩ / 12kΩ** (~3.03V @ 14.4V). If using a lower ratio like 30kΩ / 12kΩ, a 3.3V Zener clamp is **mandatory**.

## How to use

### Scanner

To find the LIN ID of your sensor, run the scanner script:

1.  Connect the hardware as described in [WIRING.md](WIRING.md).
2.  Upload `scanner.py` to your RP2040.
3.  Run the script.
4.  It will cycle through all possible LIN IDs (0x00 - 0x3F).
5.  Watch the console for `[FOUND]` messages.

### Main Program

The `main.py` script contains the logic to communicate with the sensor. It is configured to poll the IDs found by the scanner.

1.  Configure the `target_ids` list in `main.py` with the IDs found by the scanner.
2.  Run `main.py`.
3.  The script will poll the sensor and decode the response.

## Software Implementation Notes

### PIO-Based LIN

The RP2040's PIO is used for LIN communication because:

1. **Break field generation** — standard UARTs cannot easily produce a 13+ bit dominant state. PIO can hold the pin low for the exact number of cycles required.
2. **Precise timing** — PIO runs independently of the CPU, ensuring accurate baud rate timing.
3. **Logic inversion** — The PIO program handles the logical inversion required by the NPN driver.

### Break Field Generation Alternatives

| Method                     | Description                                         |
|----------------------------|-----------------------------------------------------|
| **PIO (Recommended)**      | State machine pulls pin low for ≥13 bit periods, then shifts out Sync/ID |

## Power Requirements

- **12V DC supply** for the sensor (VCC) and LIN bus master termination pull-up
- **3.3V** for the RP2040 (from its onboard regulator or external)
- **Common GND** between 12V supply, sensor, and RP2040 is **critical**

## Project Structure

```
├── README.md          # This file — project overview
├── WIRING.md          # Circuit description, schematic, and wiring guide
└── LICENSE            # Project license
```

## References

- [LIN Bus Specification (LIN 2.0)](https://www.lin-cia.org/)
- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) — PIO, GPIO_CTRL
- [VW Rain/Light Sensor G397 — Ross-Tech Wiki](https://wiki.ross-tech.com/)
- [BC547 Datasheet](https://www.onsemi.com/pdf/datasheet/bc547-d.pdf)

## License

See [LICENSE](LICENSE) for details.
