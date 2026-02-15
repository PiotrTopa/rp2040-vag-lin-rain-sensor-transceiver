# Wiring Guide — RP2040 LIN Transceiver

Complete circuit description for interfacing the RP2040 (3.3V) with the VAG Rain/Light/Humidity Sensor 81A 955 555 A (12V LIN bus) using discrete components.

## Block Diagram

```
                    +12V DC
                      │
                      ├──── Sensor Pin 1 (VCC)
                      │
                 ┌────┴────┐
                 │ 1N4148  │ (Anode → 12V, Cathode → R_term)
                 └────┬────┘
                      │
                   [1kΩ] R_term (Master Termination)
                      │
       LIN BUS ───────┼──────────────────── Sensor Pin 3 (LIN)
                      │
          ┌───────────┼───────────┐
          │           │           │
       TX Path     RX Path       │
          │           │           │
          │      [33kΩ] R1        │
          │           │           │
     ┌────┴────┐      ├───────── RP2040 GPIO1 (RX Input)
     │ BC547   │      │
     │  (NPN)  │   [10kΩ] R2    [3.3V Zener]
     │         │      │              │
     │ C ──────┘      │              │
     │ B ─[1kΩ]── RP2040 GPIO0 (TX) │
     │ E ──┐          │              │
     │     │          │              │
     └─────┼──────────┴──────────────┘
           │
          GND ────────────────────── Sensor Pin 2 (GND)
           │
      RP2040 GND
```

> **All grounds must be connected together** — RP2040 GND, sensor GND, and 12V supply GND.

---

## Circuit Blocks

### 1. Master Termination

Since the RP2040 acts as the LIN Master (BCM), it must provide the bus pull-up.

```
+12V ──►|── 1N4148 ──── [1kΩ] ──── LIN BUS
```

| Component | Value    | Purpose                                              |
|-----------|----------|------------------------------------------------------|
| D1        | 1N4148   | Prevents current backflow if 12V rail sags below bus |
| R_term    | 1kΩ 1/4W | Master pull-up (sets recessive/idle state at ~12V)   |

The sensor (slave) has its own internal ~30kΩ pull-up. The 1kΩ master termination dominates the bus idle level.

---

### 2. TX Path — RP2040 → LIN Bus (GPIO0)

Single NPN common-emitter open-collector driver. The transistor pulls the LIN bus LOW (dominant) when turned ON.

```
RP2040 GPIO0 (TX) ──── [1kΩ] ──── BC547 Base
                                       │
                                   [10kΩ] (Base to GND pull-down)
                                       │
                                      GND

BC547 Collector ──── LIN BUS
BC547 Emitter   ──── GND
```

| Component | Value     | Purpose                                                        |
|-----------|-----------|----------------------------------------------------------------|
| Q1        | BC547     | NPN transistor, open-collector LIN driver                      |
| R_base    | 1kΩ       | Limits base current from GPIO0 (~2.3mA @ 3.3V)                |
| R_pulldown| 10kΩ      | Keeps Q1 OFF during RP2040 boot/reset (bus stays recessive)    |

#### Logic Table (Raw — Before Inversion)

| GPIO0 Output | Q1 State | LIN Bus Level | LIN State           |
|:------------:|:--------:|:-------------:|:-------------------:|
| LOW  (0V)    | OFF      | 12V (via R_term) | Recessive (Idle) |
| HIGH (3.3V)  | ON       | ~0V (GND)     | Dominant            |

> **This is inverted relative to standard UART** (UART idle = high, but we need GPIO idle = low to keep Q1 off and bus recessive).

#### Handling Inversion

**Recommended approach** — Use the RP2040's pad-level output inversion:

```c
// In initialization code:
// Invert GPIO0 output at the pad — PIO writes standard UART levels,
// hardware flips them before reaching the transistor.
hw_write_masked(
    &iobank0_hw->io[0].ctrl,
    1 << IO_BANK0_GPIO0_CTRL_OUTOVER_LSB,
    IO_BANK0_GPIO0_CTRL_OUTOVER_BITS
);
```

With `OUTOVER = 0x1`:
- PIO outputs HIGH (idle) → pad drives LOW → Q1 OFF → bus recessive ✓
- PIO outputs LOW (start bit) → pad drives HIGH → Q1 ON → bus dominant ✓

---

### 3. RX Path — LIN Bus → RP2040 (GPIO1)

Resistive voltage divider with Zener clamp to shift 12V bus levels to 3.3V-safe logic.

```
LIN BUS ──── [33kΩ] R1 ──── RP2040 GPIO1 (RX)
                                   │
                              [10kΩ] R2
                                   │
                              [3.3V Zener] (cathode to GPIO1, anode to GND)
                                   │
                                  GND
```

| Component | Value          | Purpose                                    |
|-----------|----------------|--------------------------------------------|
| R1        | 33kΩ           | Top resistor of voltage divider            |
| R2        | 10kΩ           | Bottom resistor of voltage divider         |
| D_z       | 3.3V Zener     | Clamps GPIO1 voltage, protects RP2040      |

#### Voltage Levels

| Bus Voltage      | Condition            | GPIO1 Voltage               | Logic Level |
|:----------------:|:--------------------:|:---------------------------:|:-----------:|
| 12.0V            | Engine off           | 12 × 10/(33+10) = **2.79V** | HIGH       |
| 14.4V            | Alternator charging  | 14.4 × 10/(33+10) = **3.35V** | HIGH (clamped by Zener) |
| 0V               | Dominant             | **0V**                      | LOW        |
| 11.0V            | Low battery          | 11 × 10/(33+10) = **2.56V** | HIGH       |

> The RP2040 input high threshold (`V_IH`) is approximately **2.0V** (0.625 × 3.3V). All recessive voltages are safely above this.

> The 3.3V Zener provides **mandatory** overvoltage protection, especially during load dump transients.

#### Alternative Divider Values

If you don't have 33kΩ, these alternatives are safe:

| R1 (Top) | R2 (Bottom) | V @ 14.4V | V @ 11.0V | Notes               |
|:---------:|:-----------:|:---------:|:---------:|----------------------|
| 33kΩ      | 10kΩ        | 3.35V     | 2.56V     | Ideal with Zener     |
| 42kΩ      | 12kΩ        | 3.20V     | 2.44V     | Safe without Zener   |
| 45kΩ      | 12kΩ        | 3.03V     | 2.31V     | Most conservative    |
| 30kΩ      | 12kΩ        | **4.11V** | 3.14V     | **UNSAFE** — Zener mandatory |

---

## RP2040 GPIO Summary

| GPIO  | Function | Direction | Connected To                                |
|:-----:|:--------:|:---------:|---------------------------------------------|
| GPIO0 | LIN TX   | Output    | BC547 Base via 1kΩ (with 10kΩ pull-down)    |
| GPIO1 | LIN RX   | Input     | Voltage divider from LIN bus (33kΩ / 10kΩ)  |

> If using a **single GPIO** for half-duplex LIN (PIO bidirectional), connect both TX and RX paths to GPIO0 — the RX divider's high impedance (33kΩ) will not interfere with the TX driver.

---

## Full Wiring Summary

```
+12V DC SUPPLY
    │
    ├──────────────────────────────────── Sensor Pin 1 (VCC, +12V)
    │
    ├──►|── D1 (1N4148) ──[1kΩ R_term]──┐
    │                                    │
    │                              LIN BUS LINE
    │                    ┌───────────────┤
    │                    │               │
    │               Q1 (BC547)      [33kΩ R1]
    │               Collector            │
    │                    │          RP2040 GPIO1 ◄── RX Input
    │               Emitter              │
    │                    │          [10kΩ R2]
    │                   GND              │
    │                    │       [3.3V Zener D_z]
    │                    │              │
    │                   GND            GND
    │
    │            [1kΩ R_base]
    │                 │
    │            Q1 Base ←── RP2040 GPIO0 (TX Output)
    │                 │
    │            [10kΩ R_pulldown]
    │                 │
    │                GND
    │
    └── GND ─────────────────────────── Sensor Pin 2 (GND)
                │
           RP2040 GND


    Sensor Pin 3 (LIN) ──────────────── LIN BUS LINE
```

---

## Important Safety Notes

1. **Common ground is essential.** The 12V supply, sensor, and RP2040 must share the same ground reference. Floating grounds will cause communication failures or damage.

2. **Boot safety.** The 10kΩ base pull-down on Q1 ensures the transistor stays OFF (bus idle/recessive) while the RP2040 is booting or resetting. Without this resistor, the GPIO pin may float and randomly drive the bus dominant.

3. **Never connect LIN bus directly to RP2040 GPIO.** The 12V bus will destroy the 3.3V GPIO pins instantly. Always use the voltage divider + Zener clamp.

4. **Automotive transients.** In-vehicle environments can see voltage spikes up to 40V+ (load dump). For in-vehicle use, add TVS diodes and consider a proper LIN transceiver IC (MCP2003, TJA1021). The discrete circuit described here is intended for **bench/lab use**.

5. **Power supply.** The RP2040 board's 3.3V regulator is separate from the 12V sensor supply. Do **not** feed 12V into the RP2040's VSYS/VBUS pins without an appropriate step-down regulator.
