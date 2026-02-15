# Discovery Notes — 81A 955 555 A

Working notes for reverse-engineering the sensor's LIN data fields.

## Sensor Info

| Property    | Value              |
|-------------|--------------------|
| Part Number | `81A 955 555 A`    |
| Platform    | MQB               |
| Protocol    | LIN 2.x (Enhanced) |
| Baud        | 19200              |

## Scanner Results

Three slave-response frames confirmed, all **Enhanced LIN 2.x** checksum:

| Frame ID | PID    | Raw Data (first reading)                        |
|----------|--------|-------------------------------------------------|
| `0x23`   | `0xA3` | `00 2C F0 00 24 EC FF FF`                       |
| `0x29`   | `0xE9` | `20 7E 76 FE 7E 75 FF FF`                       |
| `0x30`   | `0xF0` | `00 00 FF FF FF FF FF FF`                        |

## Initial Observations

- **Light readings present** (frame 0x23, bytes 4-5 react to light)
- **Environmental readings present** (frame 0x29, slow-changing values)
- **FIR not activated** — frame 0x30 is static (`00 00 FF FF...`), FIR diodes are not firing
- Sensor appears to be in **fallback/passive mode** without proper initialization from BCM

## Frame 0x23 — Light Sensor

From Phase 1 monitoring:

| Byte | Observed     | Behavior         | Hypothesis                    |
|------|-------------|------------------|-------------------------------|
| [0]  | 0x00–0x1F   | Low nibble cycles | 4-bit rolling counter         |
|      |             | High nibble       | Status nibble                 |
| [1]  | 0x2C        | Constant          | Config/status flags           |
| [2]  | 0xF0        | Constant          | Config/status flags           |
| [3]  | 0x00        | Constant          | Reserved                      |
| [4]  | variable    | Fast-changing     | Light intensity low (16-bit LE) |
| [5]  | 0xEC range  | Overflows from [4]| Light intensity high          |
| [6]  | 0xFF        | Constant          | Unused                        |
| [7]  | 0xFF        | Constant          | Unused                        |

**Light value** = `byte[5] << 8 | byte[4]` — 16-bit little-endian.
This is the **forward-facing ambient light sensor** (reacts to flashlight, used for
automatic headlight / DRL switching). Higher value = brighter.

## Frame 0x29 — Environmental

| Byte | Observed  | Behavior     | Decoded                               |
|------|-----------|--------------|-----------------------------------------|
| [0]  | 0x20 area | Very slow    | Solar/ambient intensity (raw)           |
| [1]  | 0x7E      | Constant     | Config/status                           |
| [2]  | 0x76 area | Slow drift   | **Temperature** (°C = raw × 0.5 − 40)  |
| [3]  | 0xFE      | Constant     | Config/status                           |
| [4]  | 0x7E area | Constant     | Config/status                           |
| [5]  | 0x75 area | Slow drift   | **Temp2 / dew point** (°C = raw × 0.5 − 40) |
| [6]  | 0xFF      | Constant     | Unused                                  |
| [7]  | 0xFF      | Constant     | Unused                                  |

**Temperature formula**: `raw_byte × 0.5 − 40` → °C.  
Example: `0x7A` (122) × 0.5 − 40 = **21.0 °C** (confirmed room temperature).

## Frame 0x30 — Extended (Dormant)

| Byte | Observed | Hypothesis                      |
|------|----------|---------------------------------|
| [0]  | 0x00     | Rain intensity (when activated) |
| [1]  | 0x00     | Rain status (when activated)    |
| [2-7]| 0xFF     | Unused / reserved               |

This frame likely carries rain data once FIR is initialized.

## TODO

- [ ] Find master command frame ID (BCM → sensor) — try continuous TX on 0x20, 0x21
- [ ] Find correct master command payload to activate FIR (use `scan_profiles()`)
- [ ] If profiles don't work, try diagnostic coding via 0x3C
- [ ] Verify temperature formula with controlled heat/cold test
- [ ] Determine byte[0] of 0x29 — solar radiation vs ambient light
- [ ] Determine byte[5] of 0x29 — second temperature or dew point?
- [ ] Long-duration monitoring to track environmental byte drift

## Key Insight: FIR Activation

The sensor FIR (rain) subsystem requires **periodic master command frames** from the
BCM to activate. Without them, the sensor stays passive (light + temp work, but FIR
LEDs stay off and frame 0x30 remains static `00 00 FF FF...`).

In a real car, the BCM sends a master-request frame (likely 0x20 or 0x21) every
schedule cycle containing:
- Ignition on/off status (KL15)
- Wiper mode (auto/manual/off)
- Rain sensor sensitivity setting

Use `scan_profiles()` in main.py or `inject()` in decoder.py to find the right
combination.

---

## Previous Hardware Note

> An earlier version of this project targeted a different sensor part number with a simpler 
> LIN 1.x classic-checksum protocol. That sensor was abandoned due to an overly complex 
> initialization protocol and difficulties getting the FIR rain detection subsystem to activate. 
> The project was restarted targeting `81A 955 555 A` (MQB platform, LIN 2.x).
