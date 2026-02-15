#!/usr/bin/env python3
"""
LID Data Analyzer — PC-side visualization tool

Parses captured LID output from diag.py / diag4.py, compares multiple
captures, and shows which channels change vs stay static.

Each LID has 12 bytes = 4 channels of [formula=0x87, high_byte, low_byte].
16-bit channel value = (high << 8) | low.

Usage:
    python analyze_lids.py                  # Analyze embedded sample data
    python analyze_lids.py capture.txt      # Analyze from file
    python analyze_lids.py c1.txt c2.txt    # Compare two captures
"""

import re
import sys
from collections import defaultdict

# ================================================================
# Parser
# ================================================================

# Matches: LID 0x0A [12]: [87 01 A3 87 01 02 87 01 04 87 01 80]
LID_RE = re.compile(
    r"LID\s+0x([0-9A-Fa-f]{2})\s+\[\s*(\d+)\]:\s+\[((?:[0-9A-Fa-f]{2}\s*)+)\]"
)


def parse_lid_line(line):
    """Parse one LID line. Returns (lid_num, [bytes]) or None."""
    m = LID_RE.search(line)
    if not m:
        return None
    lid = int(m.group(1), 16)
    raw = [int(x, 16) for x in m.group(3).split()]
    return lid, raw


def parse_capture(text):
    """Parse a full capture (one run of read_all_lids).
    Returns dict: {lid_num: [bytes]}."""
    result = {}
    for line in text.splitlines():
        parsed = parse_lid_line(line)
        if parsed:
            lid, data = parsed
            result[lid] = data
    return result


def parse_all_captures(text):
    """Split text into multiple captures (separated by 'Reading LIDs' or 'PHASE 1').
    Returns list of dicts."""
    # Split on "Reading LIDs" header or "PHASE 1" marker
    blocks = re.split(r"(?:Reading LIDs|PHASE 1)", text)
    captures = []
    for block in blocks:
        cap = parse_capture(block)
        if cap:
            captures.append(cap)
    return captures


# ================================================================
# Channel extraction
# ================================================================

def extract_channels(data):
    """Extract 4 channels from 12-byte LID data.
    Returns list of (formula, value_16bit) tuples."""
    channels = []
    for i in range(0, min(len(data), 12) - 2, 3):
        formula = data[i]
        val = (data[i + 1] << 8) | data[i + 2]
        channels.append((formula, val))
    return channels


# ================================================================
# Analysis
# ================================================================

def analyze_captures(captures):
    """Analyze multiple captures of the same LIDs.
    Returns per-LID, per-channel statistics."""

    all_lids = sorted(set().union(*[c.keys() for c in captures]))
    stats = {}

    for lid in all_lids:
        ch_values = defaultdict(list)  # ch_index -> [values across captures]
        ch_raw = defaultdict(list)     # ch_index -> [(hi, lo) across captures]

        for cap in captures:
            if lid not in cap:
                continue
            data = cap[lid]
            channels = extract_channels(data)
            for ci, (formula, val) in enumerate(channels):
                ch_values[ci].append(val)
                hi = data[ci * 3 + 1] if ci * 3 + 1 < len(data) else 0
                lo = data[ci * 3 + 2] if ci * 3 + 2 < len(data) else 0
                ch_raw[ci].append((hi, lo))

        lid_stats = {}
        for ci in sorted(ch_values.keys()):
            vals = ch_values[ci]
            raw = ch_raw[ci]
            unique = sorted(set(vals))
            lid_stats[ci] = {
                "values": vals,
                "raw": raw,
                "min": min(vals),
                "max": max(vals),
                "range": max(vals) - min(vals),
                "unique": unique,
                "static": len(unique) == 1,
                "n": len(vals),
            }
        stats[lid] = lid_stats

    return stats


# ================================================================
# Display
# ================================================================

BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
CYAN = "\033[96m"
RESET = "\033[0m"


def print_summary(stats, captures):
    """Print a summary table of all LIDs and their channels."""

    n_cap = len(captures)
    print()
    print(f"{BOLD}{'=' * 78}{RESET}")
    print(f"{BOLD} LID Channel Analysis — {n_cap} capture(s){RESET}")
    print(f"{BOLD}{'=' * 78}{RESET}")

    # Classification
    dynamic_lids = []
    static_lids = []

    for lid in sorted(stats.keys()):
        has_dynamic = any(not s["static"] for s in stats[lid].values())
        if has_dynamic:
            dynamic_lids.append(lid)
        else:
            static_lids.append(lid)

    # Dynamic LIDs — full detail
    if dynamic_lids:
        print(f"\n{BOLD}{YELLOW}--- DYNAMIC LIDs (values change across captures) ---{RESET}\n")
        print(f"  {'LID':>6}  {'Ch':>3}  {'Min':>7}  {'Max':>7}  {'Range':>6}  {'Uniq':>5}  {'Values'}")
        print(f"  {'─' * 6}  {'─' * 3}  {'─' * 7}  {'─' * 7}  {'─' * 6}  {'─' * 5}  {'─' * 30}")

        for lid in dynamic_lids:
            first = True
            for ci in sorted(stats[lid].keys()):
                s = stats[lid][ci]
                lid_str = "0x%02X" % lid if first else ""
                first = False

                if s["static"]:
                    color = DIM
                    tag = ""
                else:
                    color = RED if s["range"] > 0x20 else YELLOW
                    tag = " <<<"

                vals_str = ", ".join("0x%04X" % v for v in s["unique"][:8])
                if len(s["unique"]) > 8:
                    vals_str += "..."

                print(f"  {color}{lid_str:>6}  ch{ci:>1}  "
                      f"0x{s['min']:04X}  0x{s['max']:04X}  "
                      f"{s['range']:5d}  {len(s['unique']):5d}  "
                      f"{vals_str}{tag}{RESET}")
            print()

    # Static LIDs — compact
    if static_lids:
        print(f"\n{DIM}--- STATIC LIDs (no change across captures) ---{RESET}\n")
        print(f"  {DIM}{'LID':>6}  {'ch0':>7}  {'ch1':>7}  {'ch2':>7}  {'ch3':>7}{RESET}")
        print(f"  {DIM}{'─' * 6}  {'─' * 7}  {'─' * 7}  {'─' * 7}  {'─' * 7}{RESET}")

        for lid in static_lids:
            vals = []
            for ci in sorted(stats[lid].keys()):
                s = stats[lid][ci]
                vals.append("0x%04X" % s["min"])
            while len(vals) < 4:
                vals.append("     -")
            print(f"  {DIM}0x{lid:02X}    {'  '.join(vals)}{RESET}")

    print()


def print_heatmap(stats, captures):
    """Print a capture-by-capture heatmap for dynamic channels."""

    dynamic = {}
    for lid in sorted(stats.keys()):
        for ci, s in stats[lid].items():
            if not s["static"]:
                dynamic[(lid, ci)] = s

    if not dynamic:
        print(f"{DIM}No dynamic channels found.{RESET}")
        return

    print(f"\n{BOLD}{'=' * 78}{RESET}")
    print(f"{BOLD} Capture-by-Capture Heatmap (dynamic channels only){RESET}")
    print(f"{BOLD}{'=' * 78}{RESET}\n")

    # Header
    hdr = f"  {'LID':>6} {'Ch':>3} |"
    for i in range(len(captures)):
        hdr += f" Cap{i:>2} |"
    hdr += f"  {'Δ':>5}"
    print(hdr)
    print("  " + "─" * (len(hdr) - 2))

    for (lid, ci), s in sorted(dynamic.items()):
        row = f"  0x{lid:02X}  ch{ci} |"
        vals = s["values"]
        prev = None
        for v in vals:
            if prev is not None and v != prev:
                row += f" {RED}0x{v:04X}{RESET} |"
            else:
                row += f" 0x{v:04X} |"
            prev = v
        row += f"  {s['range']:5d}"
        print(row)

    print()


def print_interpreted(stats):
    """Try to interpret known LID mappings based on observed ranges."""

    print(f"\n{BOLD}{'=' * 78}{RESET}")
    print(f"{BOLD} Interpretation Hints{RESET}")
    print(f"{BOLD}{'=' * 78}{RESET}\n")

    hints = {
        0x00: "Rain counters / status",
        0x02: "Ambient light (low range)",
        0x04: "ADC channel A — sensor calibration",
        0x05: "ADC channel B — near saturation",
        0x06: "ADC channel C — saturated (0x03FF)",
        0x07: "ADC channel D — mid-range sensor",
        0x08: "ADC channel E — varies",
        0x09: "ADC channel F — varies",
        0x0A: "Config/ID constant (0x01A2-0x01A3)",
        0x0B: "Solar sensor? (0x07AD-0x07B0)",
        0x0C: "Threshold / config",
        0x0D: "Threshold (ch2=0x0150)",
        0x10: "Measurement A",
        0x11: "Measurement B",
        0x12: "Measurement C",
        0x13: "Measurement D",
        0x14: "Measurement E",
        0x15: "Measurement F",
        0x19: "Active sensor data",
    }

    for lid in sorted(stats.keys()):
        has_dynamic = any(not s["static"] for s in stats[lid].values())
        if not has_dynamic:
            continue
        hint = hints.get(lid, "unknown")
        print(f"  LID 0x{lid:02X}: {hint}")
        for ci, s in sorted(stats[lid].items()):
            if s["static"]:
                print(f"    ch{ci}: static = 0x{s['min']:04X} ({s['min']})")
            else:
                print(f"    ch{ci}: {s['min']:5d} - {s['max']:5d}  "
                      f"(0x{s['min']:04X}-0x{s['max']:04X})  "
                      f"Δ={s['range']}")
        print()


# ================================================================
# Main
# ================================================================

# Embedded sample data (from the captures in the user's session)
SAMPLE_DATA = """
LID 0x00 [12]: [87 01 00 87 01 00 87 01 00 87 01 02]
LID 0x01 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x02 [12]: [87 01 00 87 01 49 87 01 00 87 01 4B]
LID 0x03 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x04 [12]: [87 01 02 87 01 AF 87 01 03 87 01 23]
LID 0x05 [12]: [87 01 03 87 01 31 87 01 03 87 01 FF]
LID 0x06 [12]: [87 01 03 87 01 FF 87 01 03 87 01 FF]
LID 0x07 [12]: [87 01 02 87 01 FA 87 01 03 87 01 07]
LID 0x08 [12]: [87 01 02 87 01 B1 87 01 02 87 01 DD]
LID 0x09 [12]: [87 01 02 87 01 EC 87 01 01 87 01 FE]
LID 0x0A [12]: [87 01 A2 87 01 02 87 01 04 87 01 80]
LID 0x0B [12]: [87 01 07 87 01 AE 87 01 00 87 01 00]
LID 0x0C [12]: [87 01 00 87 01 8C 87 01 04 87 01 01]
LID 0x0D [12]: [87 01 00 87 01 00 87 01 50 87 01 00]
LID 0x0E [12]: [87 01 00 87 01 00 87 01 00 87 01 01]
LID 0x0F [12]: [87 01 00 87 01 00 87 01 03 87 01 01]
LID 0x10 [12]: [87 01 6E 87 01 01 87 01 A7 87 01 01]
LID 0x11 [12]: [87 01 A1 87 01 01 87 01 77 87 01 03]
LID 0x12 [12]: [87 01 12 87 01 01 87 01 92 87 01 02]
LID 0x13 [12]: [87 01 FF 87 01 02 87 01 FC 87 01 01]
LID 0x14 [12]: [87 01 90 87 01 01 87 01 85 87 01 03]
LID 0x15 [12]: [87 01 20 87 01 03 87 01 10 87 01 01]
LID 0x16 [12]: [87 01 A0 87 01 00 87 01 00 87 01 80]
LID 0x17 [12]: [87 01 04 87 01 01 87 01 51 87 01 00]
LID 0x18 [12]: [87 01 04 87 01 28 87 01 50 87 01 30]
LID 0x19 [12]: [87 01 A5 87 01 CA 87 01 48 87 01 F0]
---
LID 0x00 [12]: [87 01 00 87 01 02 87 01 00 87 01 02]
LID 0x01 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x02 [12]: [87 01 00 87 01 4A 87 01 00 87 01 4B]
LID 0x03 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x04 [12]: [87 01 02 87 01 AF 87 01 03 87 01 23]
LID 0x05 [12]: [87 01 03 87 01 31 87 01 03 87 01 FF]
LID 0x06 [12]: [87 01 03 87 01 FF 87 01 03 87 01 FF]
LID 0x07 [12]: [87 01 02 87 01 FA 87 01 03 87 01 07]
LID 0x08 [12]: [87 01 02 87 01 B7 87 01 02 87 01 DD]
LID 0x09 [12]: [87 01 02 87 01 ED 87 01 01 87 01 FE]
LID 0x0A [12]: [87 01 A3 87 01 02 87 01 05 87 01 80]
LID 0x0B [12]: [87 01 07 87 01 AF 87 01 00 87 01 00]
LID 0x0C [12]: [87 01 00 87 01 8C 87 01 04 87 01 01]
LID 0x0D [12]: [87 01 00 87 01 00 87 01 50 87 01 00]
LID 0x0E [12]: [87 01 00 87 01 00 87 01 00 87 01 01]
LID 0x0F [12]: [87 01 00 87 01 00 87 01 03 87 01 01]
LID 0x10 [12]: [87 01 6E 87 01 01 87 01 A7 87 01 01]
LID 0x11 [12]: [87 01 A1 87 01 01 87 01 76 87 01 03]
LID 0x12 [12]: [87 01 12 87 01 01 87 01 92 87 01 02]
LID 0x13 [12]: [87 01 FF 87 01 02 87 01 FC 87 01 01]
LID 0x14 [12]: [87 01 8F 87 01 01 87 01 85 87 01 03]
LID 0x15 [12]: [87 01 20 87 01 03 87 01 10 87 01 01]
LID 0x16 [12]: [87 01 A0 87 01 00 87 01 00 87 01 80]
LID 0x17 [12]: [87 01 04 87 01 01 87 01 51 87 01 00]
LID 0x18 [12]: [87 01 04 87 01 28 87 01 50 87 01 30]
LID 0x19 [12]: [87 01 A5 87 01 CA 87 01 48 87 01 F0]
LID 0x1A [12]: [87 01 05 87 01 02 87 01 00 87 01 01]
LID 0x1B [12]: [87 01 02 87 01 FF 87 01 00 87 01 00]
LID 0x1C [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x1D [12]: [87 01 00 87 01 00 87 01 80 87 01 36]
LID 0x1E [12]: [87 01 A2 87 01 40 87 01 08 87 01 01]
LID 0x1F [12]: [87 01 21 87 01 00 87 01 00 87 01 00]
---
LID 0x00 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x01 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x02 [12]: [87 01 00 87 01 08 87 01 00 87 01 08]
LID 0x03 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x04 [12]: [87 01 02 87 01 B5 87 01 03 87 01 7F]
LID 0x05 [12]: [87 01 03 87 01 8D 87 01 03 87 01 FF]
LID 0x06 [12]: [87 01 03 87 01 FF 87 01 03 87 01 FF]
LID 0x07 [12]: [87 01 03 87 01 0A 87 01 03 87 01 15]
LID 0x08 [12]: [87 01 02 87 01 B2 87 01 02 87 01 E5]
LID 0x09 [12]: [87 01 02 87 01 F2 87 01 01 87 01 FE]
LID 0x0A [12]: [87 01 A3 87 01 02 87 01 04 87 01 80]
LID 0x0B [12]: [87 01 07 87 01 AF 87 01 00 87 01 00]
LID 0x0C [12]: [87 01 00 87 01 8C 87 01 05 87 01 01]
LID 0x0D [12]: [87 01 00 87 01 00 87 01 50 87 01 00]
LID 0x0E [12]: [87 01 00 87 01 00 87 01 00 87 01 01]
LID 0x0F [12]: [87 01 00 87 01 00 87 01 03 87 01 01]
LID 0x10 [12]: [87 01 6E 87 01 01 87 01 A7 87 01 01]
LID 0x11 [12]: [87 01 A1 87 01 01 87 01 76 87 01 03]
LID 0x12 [12]: [87 01 12 87 01 01 87 01 92 87 01 02]
LID 0x13 [12]: [87 01 FF 87 01 02 87 01 FC 87 01 01]
LID 0x14 [12]: [87 01 8F 87 01 01 87 01 85 87 01 03]
LID 0x15 [12]: [87 01 20 87 01 03 87 01 10 87 01 01]
LID 0x16 [12]: [87 01 A0 87 01 00 87 01 00 87 01 80]
LID 0x17 [12]: [87 01 04 87 01 01 87 01 50 87 01 00]
LID 0x18 [12]: [87 01 05 87 01 28 87 01 50 87 01 30]
LID 0x19 [12]: [87 01 A5 87 01 CA 87 01 48 87 01 F0]
LID 0x1A [12]: [87 01 05 87 01 02 87 01 00 87 01 01]
LID 0x1B [12]: [87 01 02 87 01 FF 87 01 00 87 01 00]
LID 0x1C [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x1D [12]: [87 01 00 87 01 00 87 01 80 87 01 36]
LID 0x1E [12]: [87 01 A2 87 01 40 87 01 08 87 01 01]
LID 0x1F [12]: [87 01 21 87 01 00 87 01 00 87 01 00]
---
LID 0x00 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x01 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x02 [12]: [87 01 00 87 01 05 87 01 00 87 01 05]
LID 0x03 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x04 [12]: [87 01 02 87 01 C7 87 01 03 87 01 8A]
LID 0x05 [12]: [87 01 03 87 01 98 87 01 03 87 01 FF]
LID 0x06 [12]: [87 01 03 87 01 FF 87 01 03 87 01 FF]
LID 0x07 [12]: [87 01 03 87 01 11 87 01 03 87 01 1A]
LID 0x08 [12]: [87 01 02 87 01 CF 87 01 02 87 01 F9]
LID 0x09 [12]: [87 01 03 87 01 04 87 01 01 87 01 FE]
LID 0x0A [12]: [87 01 A3 87 01 02 87 01 04 87 01 80]
LID 0x0B [12]: [87 01 07 87 01 B0 87 01 00 87 01 00]
LID 0x0C [12]: [87 01 00 87 01 8C 87 01 04 87 01 01]
LID 0x0D [12]: [87 01 00 87 01 00 87 01 50 87 01 00]
LID 0x0E [12]: [87 01 00 87 01 00 87 01 00 87 01 01]
LID 0x0F [12]: [87 01 00 87 01 00 87 01 03 87 01 01]
LID 0x10 [12]: [87 01 6E 87 01 01 87 01 A6 87 01 01]
LID 0x11 [12]: [87 01 A1 87 01 01 87 01 76 87 01 03]
LID 0x12 [12]: [87 01 12 87 01 01 87 01 91 87 01 02]
LID 0x13 [12]: [87 01 FF 87 01 02 87 01 FC 87 01 01]
LID 0x14 [12]: [87 01 8F 87 01 01 87 01 85 87 01 03]
LID 0x15 [12]: [87 01 20 87 01 03 87 01 10 87 01 01]
LID 0x16 [12]: [87 01 A0 87 01 00 87 01 00 87 01 80]
LID 0x17 [12]: [87 01 04 87 01 01 87 01 50 87 01 00]
LID 0x18 [12]: [87 01 04 87 01 28 87 01 50 87 01 30]
LID 0x19 [12]: [87 01 A5 87 01 CA 87 01 48 87 01 F0]
LID 0x1A [12]: [87 01 05 87 01 02 87 01 00 87 01 01]
LID 0x1B [12]: [87 01 02 87 01 FF 87 01 00 87 01 00]
LID 0x1C [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x1D [12]: [87 01 00 87 01 00 87 01 80 87 01 36]
LID 0x1E [12]: [87 01 A2 87 01 40 87 01 08 87 01 01]
LID 0x1F [12]: [87 01 21 87 01 00 87 01 00 87 01 00]
---
LID 0x00 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x01 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x02 [12]: [87 01 00 87 01 04 87 01 00 87 01 04]
LID 0x03 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x04 [12]: [87 01 02 87 01 CC 87 01 03 87 01 8D]
LID 0x05 [12]: [87 01 03 87 01 9B 87 01 03 87 01 FF]
LID 0x06 [12]: [87 01 03 87 01 FF 87 01 03 87 01 FF]
LID 0x07 [12]: [87 01 03 87 01 14 87 01 03 87 01 1C]
LID 0x08 [12]: [87 01 02 87 01 D6 87 01 02 87 01 FE]
LID 0x09 [12]: [87 01 03 87 01 0A 87 01 01 87 01 FE]
LID 0x0A [12]: [87 01 A3 87 01 02 87 01 04 87 01 80]
LID 0x0B [12]: [87 01 07 87 01 AD 87 01 00 87 01 00]
LID 0x0C [12]: [87 01 00 87 01 8C 87 01 05 87 01 01]
LID 0x0D [12]: [87 01 00 87 01 00 87 01 50 87 01 00]
LID 0x0E [12]: [87 01 00 87 01 00 87 01 00 87 01 01]
LID 0x0F [12]: [87 01 00 87 01 00 87 01 03 87 01 01]
LID 0x10 [12]: [87 01 6E 87 01 01 87 01 A7 87 01 01]
LID 0x11 [12]: [87 01 A1 87 01 01 87 01 76 87 01 03]
LID 0x12 [12]: [87 01 12 87 01 01 87 01 91 87 01 02]
LID 0x13 [12]: [87 01 FF 87 01 02 87 01 FD 87 01 01]
LID 0x14 [12]: [87 01 8F 87 01 01 87 01 85 87 01 03]
LID 0x15 [12]: [87 01 20 87 01 03 87 01 10 87 01 01]
LID 0x16 [12]: [87 01 A0 87 01 00 87 01 00 87 01 80]
LID 0x17 [12]: [87 01 04 87 01 01 87 01 50 87 01 00]
LID 0x18 [12]: [87 01 05 87 01 28 87 01 50 87 01 30]
LID 0x19 [12]: [87 01 A5 87 01 CA 87 01 48 87 01 F0]
LID 0x1A [12]: [87 01 05 87 01 02 87 01 00 87 01 01]
LID 0x1B [12]: [87 01 02 87 01 FF 87 01 00 87 01 00]
LID 0x1C [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x1D [12]: [87 01 00 87 01 00 87 01 80 87 01 36]
LID 0x1E [12]: [87 01 A2 87 01 40 87 01 08 87 01 01]
LID 0x1F [12]: [87 01 21 87 01 00 87 01 00 87 01 00]
---
LID 0x00 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x01 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x02 [12]: [87 01 00 87 01 1F 87 01 00 87 01 1F]
LID 0x03 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x04 [12]: [87 01 02 87 01 C8 87 01 03 87 01 49]
LID 0x05 [12]: [87 01 03 87 01 57 87 01 03 87 01 FF]
LID 0x06 [12]: [87 01 03 87 01 FF 87 01 03 87 01 FF]
LID 0x07 [12]: [87 01 03 87 01 06 87 01 03 87 01 12]
LID 0x08 [12]: [87 01 02 87 01 D0 87 01 02 87 01 F9]
LID 0x09 [12]: [87 01 03 87 01 08 87 01 09 87 01 FE]
LID 0x0A [12]: [87 01 A2 87 01 02 87 01 04 87 01 80]
LID 0x0B [12]: [87 01 07 87 01 AD 87 01 00 87 01 00]
LID 0x0C [12]: [87 01 00 87 01 8C 87 01 05 87 01 01]
LID 0x0D [12]: [87 01 00 87 01 00 87 01 50 87 01 00]
LID 0x0E [12]: [87 01 00 87 01 00 87 01 00 87 01 03]
LID 0x0F [12]: [87 01 00 87 01 00 87 01 03 87 01 01]
LID 0x10 [12]: [87 01 6E 87 01 01 87 01 A6 87 01 01]
LID 0x11 [12]: [87 01 A1 87 01 01 87 01 76 87 01 03]
LID 0x12 [12]: [87 01 12 87 01 01 87 01 91 87 01 02]
LID 0x13 [12]: [87 01 FF 87 01 02 87 01 FD 87 01 01]
LID 0x14 [12]: [87 01 8F 87 01 01 87 01 84 87 01 03]
LID 0x15 [12]: [87 01 20 87 01 03 87 01 10 87 01 01]
LID 0x16 [12]: [87 01 9F 87 01 00 87 01 00 87 01 80]
LID 0x17 [12]: [87 01 00 87 01 01 87 01 50 87 01 00]
LID 0x18 [12]: [87 01 05 87 01 28 87 01 50 87 01 30]
LID 0x19 [12]: [87 01 A5 87 01 CA 87 01 48 87 01 F0]
LID 0x1A [12]: [87 01 05 87 01 02 87 01 00 87 01 01]
LID 0x1B [12]: [87 01 02 87 01 FF 87 01 00 87 01 00]
LID 0x1C [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x1D [12]: [87 01 00 87 01 00 87 01 80 87 01 36]
LID 0x1E [12]: [87 01 A2 87 01 40 87 01 08 87 01 01]
LID 0x1F [12]: [87 01 21 87 01 00 87 01 00 87 01 00]
---
LID 0x00 [12]: [87 01 00 87 01 04 87 01 00 87 01 06]
LID 0x01 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x02 [12]: [87 01 00 87 01 66 87 01 00 87 01 68]
LID 0x03 [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x04 [12]: [87 01 02 87 01 BF 87 01 03 87 01 14]
LID 0x05 [12]: [87 01 03 87 01 24 87 01 03 87 01 FF]
LID 0x06 [12]: [87 01 03 87 01 FF 87 01 03 87 01 FF]
LID 0x07 [12]: [87 01 02 87 01 F4 87 01 03 87 01 01]
LID 0x08 [12]: [87 01 02 87 01 C2 87 01 02 87 01 EF]
LID 0x09 [12]: [87 01 02 87 01 FE 87 01 01 87 01 FE]
LID 0x0A [12]: [87 01 A3 87 01 02 87 01 04 87 01 80]
LID 0x0B [12]: [87 01 07 87 01 AE 87 01 00 87 01 00]
LID 0x0C [12]: [87 01 00 87 01 8C 87 01 04 87 01 01]
LID 0x0D [12]: [87 01 00 87 01 00 87 01 50 87 01 00]
LID 0x0E [12]: [87 01 00 87 01 00 87 01 00 87 01 01]
LID 0x0F [12]: [87 01 00 87 01 00 87 01 03 87 01 01]
LID 0x10 [12]: [87 01 6E 87 01 01 87 01 A7 87 01 01]
LID 0x11 [12]: [87 01 A1 87 01 01 87 01 76 87 01 03]
LID 0x12 [12]: [87 01 12 87 01 01 87 01 91 87 01 02]
LID 0x13 [12]: [87 01 FF 87 01 02 87 01 FD 87 01 01]
LID 0x14 [12]: [87 01 8F 87 01 01 87 01 84 87 01 03]
LID 0x15 [12]: [87 01 21 87 01 03 87 01 10 87 01 01]
LID 0x16 [12]: [87 01 A0 87 01 00 87 01 00 87 01 80]
LID 0x17 [12]: [87 01 04 87 01 01 87 01 50 87 01 00]
LID 0x18 [12]: [87 01 04 87 01 28 87 01 50 87 01 30]
LID 0x19 [12]: [87 01 A5 87 01 CA 87 01 48 87 01 F0]
LID 0x1A [12]: [87 01 05 87 01 02 87 01 00 87 01 01]
LID 0x1B [12]: [87 01 02 87 01 FF 87 01 00 87 01 00]
LID 0x1C [12]: [87 01 00 87 01 00 87 01 00 87 01 00]
LID 0x1D [12]: [87 01 00 87 01 00 87 01 80 87 01 36]
LID 0x1E [12]: [87 01 A2 87 01 40 87 01 08 87 01 01]
LID 0x1F [12]: [87 01 21 87 01 00 87 01 00 87 01 00]
"""


def main():
    if len(sys.argv) > 1:
        # Read from file(s)
        text = ""
        for path in sys.argv[1:]:
            with open(path, "r") as f:
                text += f.read() + "\n---\n"
    else:
        text = SAMPLE_DATA

    # Parse captures (split on ---  separators)
    blocks = text.split("---")
    captures = []
    for block in blocks:
        cap = parse_capture(block)
        if cap:
            captures.append(cap)

    if not captures:
        # Try the automatic splitter
        captures = parse_all_captures(text)

    if not captures:
        print("No LID data found in input.")
        return

    print(f"Parsed {len(captures)} capture(s), "
          f"{len(set().union(*[c.keys() for c in captures]))} unique LIDs")

    stats = analyze_captures(captures)
    print_summary(stats, captures)
    print_heatmap(stats, captures)
    print_interpreted(stats)


if __name__ == "__main__":
    main()
