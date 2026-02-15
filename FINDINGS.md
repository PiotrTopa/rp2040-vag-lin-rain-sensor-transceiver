# Findings — 81A 955 555 A Rain/Light/Humidity Sensor (G397_RLFSS)

## Sensor Identity

- **Part Number:** 81A 955 555 A
- **Platform:** MQB (Audi Q2 / VW family)
- **LIN NAD:** `0x02`
- **Protocol:** KWP2000-style diagnostics over LIN (SID 0x21 measuring blocks, SID 0x22/0x2E DID read/write)
- **Diagnostic transport:** Multi-frame responses are automatic (sensor sends consecutive frames without Flow Control)
- **Checksum:** Enhanced checksum on data frames, Classic checksum on diagnostic frames (0x3C/0x3D)

## LIN Frame Map

| Frame ID | Direction      | Name  | Status     |
|----------|---------------|-------|------------|
| `0x20`   | Master → Slave | CMD   | Master command frame (BCM → sensor) |
| `0x23`   | Slave → Master | LIGHT | **Active** — forward light + ambient data |
| `0x29`   | Slave → Master | ENV   | **Active** — humidity / temperature / dew point |
| `0x30`   | Slave → Master | RAIN  | **Dormant** — always `[00 00 FF FF FF FF FF FF]` |

## LIGHT Frame 0x23 — Byte Layout

```
 b0       b1       b2       b3       b4       b5       b6   b7
[status]  [mode]   [fwd_hi] [flags]  [fwd_lo] [solar]  [FF] [FF]
```

| Byte | Field | Range | Notes |
|------|-------|-------|-------|
| b0 | `{status[7:4], alive[3:0]}` | — | Alive counter increments by +5 mod 16 each cycle |
| b1 | Mode / config flags | `0x2C` or `0x28` | bit6 changes with coding byte0 bit6 |
| b2 | Forward light HIGH | 176–240 | Normally ~240. Drops when b4 saturates (overflow from b4) |
| b3 | Flags | `0x00` | Usually zero; seen non-zero briefly after coding changes |
| b4 | **Forward light LOW** | 0–254 | **Main light sensor** — fast, dynamic, full 0–254 range |
| b5 | Solar / IR sensor | 236–239 | Slow-changing, small range; likely infrared/solar load |
| b6–b7 | Unused | `0xFF` | Padding |

### Forward Light Sensor (b2 + b4)

Bytes 2 and 4 form a combined forward light measurement. Under normal conditions b2 stays near 240 and b4 carries the 0–254 value. Under extremely intense light, b4 saturates at 253–254 and b2 drops (observed 176, 192, 208, 224), acting as an overflow/high-byte.

Observed behavior:
- **Dark / covered:** b4 = 0, b2 = 240
- **Ambient indoor light:** b4 = 0–10, b2 = 240
- **Flashlight / bright:** b4 oscillates rapidly (rise/decay saw-tooth)
- **Extreme light (saturation):** b4 = 253, b2 drops to 176–224

## ENV Frame 0x29 — Byte Layout

```
 b0       b1       b2       b3       b4       b5       b6   b7
[counter] [~0x7E]  [~0x6A]  [~0xFE]  [~0x7C]  [~0x68]  [FF] [FF]
```

| Byte | Typical Value | Notes |
|------|--------------|-------|
| b0 | 0x00–0x07 | Counter/status; first read often 0x00, then 0x04–0x07 |
| b1 | ~0x7E (126) | Slow-changing; likely humidity or temperature high byte |
| b2 | ~0x6A (106) | Slow-changing |
| b3 | ~0xFE (254) | Nearly constant |
| b4 | ~0x7C (124) | Slow-changing |
| b5 | ~0x68 (104) | Slow-changing |
| b6–b7 | 0xFF | Padding |

## RAIN Frame 0x30 — Dormant

Always returns `[00 00 FF FF FF FF FF FF]`. No coding value or master command activated this frame. Possibly:
- Requires a different sensor variant (with FIR piezo element)
- Needs specific hardware (rain sensor pad on windshield)
- Is a master-publish frame (BCM sends wiper feedback TO sensor)
- Feature not present on this particular 81A unit

## Diagnostics — DID Map

| DID | Access | Length | Content |
|-----|--------|--------|---------|
| `0x0641` | Read | 10 bytes | Part number: `"81A955555A"` |
| `0x0611` | **Read/Write** | 3 bytes | **Coding** — default `[02 00 5D]` |

## Coding DID 0x0611 — Effects

Default: `[02 00 5D]`

| Coding Byte | Bit | Effect on LIGHT frame |
|-------------|-----|----------------------|
| byte 0, bit 6 | `0x40` | Changes b1 from `0x2C` → `0x28` |
| byte 2 = `0x00` | low bits | Changes b5 (solar) and b4 briefly; also b5 `0xEC` → `0xED` |
| All others | — | Only affect b0 (alive counter phase) — no functional change |

Coding has **no effect** on RAIN frame 0x30.

## Measuring Blocks (SID 0x21)

32 LIDs (0x00–0x1F) readable. Each returns 4 channels of 3 bytes: `[formula=0x87, high_byte, low_byte]`.

Notable LIDs with non-zero data:
- LID 0x04–0x09: Sensor calibration / ADC values (varying)
- LID 0x0A: Contains 0x01A3 (419), likely a constant/config
- LID 0x0D: ch2 = 0x0150 (336) — possibly a threshold
- LID 0x10–0x15: Multiple varying values — sensor measurements
- LID 0x16: 0x01A1, 0x0180 — config/calibration
- LID 0x18: Contains 0x0128 (296), 0x0150 (336), 0x0130 (304)
- LID 0x19: All channels varying — active sensor data
- LID 0x1D–0x1E: Contains 0x0180, calibration data

## Current Tools

| File | Purpose |
|------|---------|
| `lin.py` | Shared LIN 2.x master driver (PIO + UART + diagnostic transport) |
| `main.py` | BCM emulator — keeps sensor active, decodes light/temp/rain, DRL switching |
| `diag.py` | Diagnostic tool — LID/DID reader, coding read/write |
| `analyze_lids.py` | PC-side LID data visualizer — multi-capture comparison, change tracking |

## Archived Tools (in `archive/`)

| File | Purpose |
|------|---------|
| `scanner.py` | Initial LIN bus ID scanner (found frames 0x23, 0x29, 0x30) |
| `diag_fuzzer.py` | UDS/NAD discovery (found NAD=0x02) |
| `diag2.py` | KWP2000 measuring block reader |
| `diag3.py` | DID scanner (found 0x0611, 0x0641) |
| `diag4.py` | Combo fuzzer: coding × master commands (4662 combos, no 0x30 activation) |
| `decoder.py` | Live frame monitor with change detection |

## Open Questions

1. **Is rain detection physically present?** The 81A 955 555 A may be a light+humidity-only variant without the piezoelectric rain element.
2. **What do ENV bytes 1–5 map to?** Need temperature/humidity reference to calibrate.
3. **What is the master command payload format?** Currently sending `[FF FF FF FF 00 00 00 00]` on 0x20 — the real BCM likely sends specific wiper/vehicle state.
4. **Is 0x30 a master-publish frame?** If the BCM publishes wiper feedback on 0x30, the sensor would read it rather than respond. Not tested with full BCM emulation yet.
