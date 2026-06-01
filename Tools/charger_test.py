#!/usr/bin/env python3
# Copyright 2026 Axel Johansson
# SPDX-License-Identifier: GPL-3.0-only
"""
charger_test.py - automated charger compatibility test for AxxPD sinks.

Walks every advertised PDO (SPR Fixed, EPR Fixed, PPS min/mid/max,
AVS min/mid/max), measures VBUS at each point, then runs random
voltage requests across the full range.  Prints live results and saves
a timestamped report file.

Usage:
    python charger_test.py                       # auto-detect AxxPD CDC port
    python charger_test.py --port COM10          # explicit port
    python charger_test.py --port /dev/ttyACM0
    python charger_test.py --random 20           # 20 random sweep steps (default 10)

Requires: pyserial  (pip install pyserial)
"""

import argparse
import random
import re
import sys
import time
from datetime import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial not installed. Run: pip install pyserial")

# ---------------------------------------------------------------------------
# Tolerances
# ---------------------------------------------------------------------------
TOL_FIXED_PCT = 0.05        # +/-5 % for Fixed PDOs
TOL_PPS_MV    = 200         # +/-200 mV for PPS
TOL_AVS_MV    = 500         # +/-500 mV for AVS

# Timing
CONTRACT_TIMEOUT_S = 8.0    # max wait for #EVT CONTRACT after a request
SETTLE_S           = 2.0    # extra settle after contract event
INTER_STEP_S       = 0.5    # pause between test steps


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------
def find_axxpd_port():
    """Find the AxxPD USB CDC port.  Prefer STM VCP over STLink."""
    stlink = None
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        mfr = (p.manufacturer or "").lower()
        # Prefer the CDC virtual COM port (non-STLink)
        if "stmicroelectronics" in mfr or "stm" in mfr:
            if "stlink" in desc or "st-link" in desc:
                stlink = p.device
            else:
                return p.device  # CDC port — preferred
    return stlink  # fallback to STLink if no CDC found


def pick_port(requested):
    """Return a port name -- auto-detect or let the user choose."""
    if requested:
        return requested
    port = find_axxpd_port()
    if port:
        return port
    ports = list_ports.comports()
    if not ports:
        sys.exit("no serial ports found")
    if len(ports) == 1:
        return ports[0].device
    print("Available COM ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    while True:
        try:
            idx = int(input("Select port number: "))
            if 0 <= idx < len(ports):
                return ports[idx].device
        except (ValueError, EOFError, KeyboardInterrupt):
            sys.exit("aborted")


def drain(ser):
    """Discard anything sitting in the RX buffer."""
    ser.reset_input_buffer()
    while ser.read(1024):
        pass


def send(ser, cmd):
    """Send a command and return immediately."""
    ser.write((cmd + "\r\n").encode("ascii"))


def read_lines_until(ser, stop_re, timeout_s):
    """Read lines until *stop_re* matches or timeout.  Returns (all_lines, matched_line_or_None)."""
    lines = []
    deadline = time.monotonic() + timeout_s
    buf = ""
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        buf += chunk.decode("ascii", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            lines.append(line)
            if stop_re and stop_re.search(line):
                return lines, line
    return lines, None


def read_for(ser, seconds):
    """Collect all output for a fixed duration."""
    lines = []
    deadline = time.monotonic() + seconds
    buf = ""
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if not chunk:
            continue
        buf += chunk.decode("ascii", errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            line = line.strip()
            if line:
                lines.append(line)
    return lines


def send_and_read(ser, cmd, wait=1.5):
    """Send command, wait, return all lines."""
    drain(ser)
    send(ser, cmd)
    return read_for(ser, wait)


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------
# PDO line: "1,FIXED,5.000,3.000,SPR" or "4,PPS,5.000-11.000,3.000,SPR"
PDO_RE = re.compile(
    r"^(\d+),"                          # position
    r"(FIXED|PPS|EPR_AVS|SPR_AVS),"    # type
    r"([\d.]+-?[\d.]*),"               # voltage or range
    r"([\d.]+[^,]*),"                   # current / power
    r"(SPR|EPR)$"                       # mode
)

CONTRACT_RE = re.compile(r"#EVT CONTRACT ")
MEAS_RE     = re.compile(r"V=([\d.]+)\s+I=([\d.]+)")
IDN_RE      = re.compile(r"AxxPD")


def parse_pdo_line(line):
    """Return dict or None."""
    m = PDO_RE.match(line)
    if not m:
        return None
    pos, ptype, vrange, ifield, mode = m.groups()
    pdo = {"pos": int(pos), "type": ptype, "mode": mode, "i_max": ifield}
    if "-" in vrange:
        lo, hi = vrange.split("-")
        pdo["v_min"] = float(lo)
        pdo["v_max"] = float(hi)
    else:
        pdo["v_nom"] = float(vrange)
    return pdo


def parse_meas(line):
    """Return (volts, amps) or None."""
    m = MEAS_RE.search(line)
    if m:
        return float(m.group(1)), float(m.group(2))
    return None


# ---------------------------------------------------------------------------
# Test execution helpers
# ---------------------------------------------------------------------------
def measure(ser):
    """Take a single measurement, return (volts, amps) or (None, None)."""
    drain(ser)
    send(ser, "meas")
    mlines = read_for(ser, 1.0)
    for ml in mlines:
        result = parse_meas(ml)
        if result:
            return result
    return None, None


def check_tolerance(v_meas, expected_v, tol_mode):
    """Return (passed, lo, hi)."""
    if tol_mode == "fixed":
        lo = expected_v * (1.0 - TOL_FIXED_PCT)
        hi = expected_v * (1.0 + TOL_FIXED_PCT)
    elif tol_mode == "pps":
        lo = expected_v - TOL_PPS_MV / 1000.0
        hi = expected_v + TOL_PPS_MV / 1000.0
    else:  # avs
        lo = expected_v - TOL_AVS_MV / 1000.0
        hi = expected_v + TOL_AVS_MV / 1000.0
    return lo <= v_meas <= hi, lo, hi


def request_and_measure(ser, cmd, expected_v, tol_mode):
    """Send *cmd*, wait for contract, settle, measure.
    Returns (measured_v, measured_i, pass_bool, detail)."""
    drain(ser)
    send(ser, cmd)

    # wait for contract event
    lines, match = read_lines_until(ser, CONTRACT_RE, CONTRACT_TIMEOUT_S)
    if not match:
        # May already be at this voltage — measure anyway
        time.sleep(SETTLE_S)
        v_meas, i_meas = measure(ser)
        if v_meas is None:
            return None, None, False, "timeout waiting for contract"
        passed, lo, hi = check_tolerance(v_meas, expected_v, tol_mode)
        if passed:
            return v_meas, i_meas, True, ""
        return v_meas, i_meas, False, f"expected {lo:.3f}-{hi:.3f}V (no contract event)"

    time.sleep(SETTLE_S)
    v_meas, i_meas = measure(ser)

    if v_meas is None:
        return None, None, False, "no measurement response"

    passed, lo, hi = check_tolerance(v_meas, expected_v, tol_mode)
    if passed:
        return v_meas, i_meas, True, ""
    return v_meas, i_meas, False, f"expected {lo:.3f}-{hi:.3f}V"


def fetch_pdos(ser, cmd="list", wait=3.0):
    """Send list command, return list of parsed PDO dicts."""
    drain(ser)
    send(ser, cmd)
    lines = read_for(ser, wait)
    pdos = []
    for ln in lines:
        p = parse_pdo_line(ln)
        if p:
            pdos.append(p)
    return pdos


# ---------------------------------------------------------------------------
# Test plan builders
# ---------------------------------------------------------------------------
def build_test_plan(spr_pdos, epr_pdos):
    """Return list of (label, command, expected_v, tol_mode) tuples."""
    steps = []

    # SPR Fixed
    for p in spr_pdos:
        if p["type"] == "FIXED":
            v = p["v_nom"]
            steps.append((f"PDO{p['pos']} Fixed {v:.3f}V",
                          f"setpdo {p['pos']}", v, "fixed"))

    # EPR Fixed (only those not already in SPR)
    spr_positions = {p["pos"] for p in spr_pdos}
    for p in epr_pdos:
        if p["type"] == "FIXED" and p["pos"] not in spr_positions:
            v = p["v_nom"]
            steps.append((f"PDO{p['pos']} Fixed {v:.3f}V EPR",
                          f"setpdo {p['pos']}", v, "fixed"))

    # PPS min/mid/max
    all_pdos = _merge_pdos(spr_pdos, epr_pdos)
    for p in all_pdos:
        if p["type"] == "PPS":
            lo, hi = p["v_min"], p["v_max"]
            mid = round((lo + hi) / 2, 1)
            for label_sfx, v in [("min", lo), ("mid", mid), ("max", hi)]:
                steps.append((f"PPS {lo:.1f}-{hi:.1f}V {label_sfx}={v:.3f}V",
                              f"setpps {v}", v, "pps"))

    # AVS min/mid/max
    for p in all_pdos:
        if p["type"] in ("EPR_AVS", "SPR_AVS"):
            lo, hi = p["v_min"], p["v_max"]
            mid = round((lo + hi) / 2, 1)
            tag = "EPR-AVS" if p["type"] == "EPR_AVS" else "SPR-AVS"
            for label_sfx, v in [("min", lo), ("mid", mid), ("max", hi)]:
                steps.append((f"{tag} {lo:.1f}-{hi:.1f}V {label_sfx}={v:.3f}V",
                              f"setavs {v}", v, "avs"))

    return steps


def build_random_steps(all_pdos, n=10):
    """Generate n random requests that exercise Fixed, PPS, and AVS PDOs.
    Guarantees at least one step per available PDO type."""
    fixed_pdos = [p for p in all_pdos if p["type"] == "FIXED"]
    pps_pdos   = [p for p in all_pdos if p["type"] == "PPS"]
    avs_pdos   = [p for p in all_pdos if p["type"] in ("EPR_AVS", "SPR_AVS")]

    steps = []

    def add_fixed():
        pdo = random.choice(fixed_pdos)
        v = pdo["v_nom"]
        steps.append((f"Random Fixed {v:.0f}V (PDO{pdo['pos']})",
                      f"setpdo {pdo['pos']}", v, "fixed"))

    def add_pps():
        pdo = random.choice(pps_pdos)
        lo_mv = int(pdo["v_min"] * 1000)
        hi_mv = int(pdo["v_max"] * 1000)
        v_mv = random.randrange(lo_mv, hi_mv + 1, 20)  # 20 mV resolution
        v = v_mv / 1000.0
        steps.append((f"Random PPS {v:.2f}V ({pdo['v_min']:.1f}-{pdo['v_max']:.1f})",
                      f"setpps {v}", v, "pps"))

    def add_avs():
        pdo = random.choice(avs_pdos)
        lo_mv = int(pdo["v_min"] * 1000)
        hi_mv = int(pdo["v_max"] * 1000)
        v_mv = random.randrange(lo_mv, hi_mv + 1, 100)  # 100 mV resolution
        v = v_mv / 1000.0
        steps.append((f"Random AVS {v:.1f}V ({pdo['v_min']:.1f}-{pdo['v_max']:.1f})",
                      f"setavs {v}", v, "avs"))

    # Guarantee at least one of each available type
    generators = []
    if fixed_pdos:
        add_fixed()
        generators.append(add_fixed)
    if pps_pdos:
        add_pps()
        generators.append(add_pps)
    if avs_pdos:
        add_avs()
        generators.append(add_avs)

    if not generators:
        return []

    # Fill remaining slots randomly
    while len(steps) < n:
        random.choice(generators)()

    # Shuffle so guaranteed-coverage steps aren't always first
    random.shuffle(steps)
    return steps


def _merge_pdos(spr_pdos, epr_pdos):
    """Merge SPR and EPR PDO lists, avoiding duplicates by position."""
    seen = set()
    merged = []
    for p in spr_pdos + epr_pdos:
        key = (p["pos"], p["type"])
        if key not in seen:
            seen.add(key)
            merged.append(p)
    return merged


# ---------------------------------------------------------------------------
# Pretty-print
# ---------------------------------------------------------------------------
def print_result(idx, total, label, v_meas, i_meas, passed, detail):
    tag = "PASS" if passed else "FAIL"
    v_str = f"V={v_meas:<7.3f}" if v_meas is not None else "V=???    "
    i_str = f"I={i_meas:.3f}" if i_meas is not None else "I=???  "
    fail_info = f"  ({detail})" if detail else ""
    print(f"  [{idx:>2d}/{total}] {label:<42s} {v_str} {i_str}  {tag}{fail_info}")


def pdo_summary_str(pdos):
    """Return a compact multi-line summary of PDO capabilities."""
    lines = []
    for p in pdos:
        if p["type"] == "FIXED":
            epr = " (EPR)" if p["mode"] == "EPR" else ""
            lines.append(f"    PDO{p['pos']:>2d}  Fixed  {p['v_nom']:>5.1f}V  {p['i_max']}A{epr}")
        elif p["type"] == "PPS":
            lines.append(f"    PDO{p['pos']:>2d}  PPS    {p['v_min']:.1f}-{p['v_max']:.1f}V  {p['i_max']}A")
        elif p["type"] in ("EPR_AVS", "SPR_AVS"):
            tag = "EPR" if p["type"] == "EPR_AVS" else "SPR"
            lines.append(f"    PDO{p['pos']:>2d}  AVS    {p['v_min']:.1f}-{p['v_max']:.1f}V  {p['i_max']}W  ({tag})")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", help="serial port (autodetect if omitted)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--random", type=int, default=10, metavar="N",
                    help="number of random sweep steps (default: 10)")
    args = ap.parse_args()

    port = pick_port(args.port)
    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"could not open {port}: {e}")

    print(f"AxxPD Charger Test  -  {port} @ {args.baud}")
    print("=" * 60)

    # drain boot banner
    time.sleep(0.5)
    drain(ser)

    # Query device identity
    idn_lines = send_and_read(ser, "*IDN?", 1.0)
    idn = ""
    for ln in idn_lines:
        if "AxxPD" in ln or "axxpd" in ln.lower():
            idn = ln
            break
    if idn:
        print(f"  Device: {idn}")

    results = []  # (label, v_meas, i_meas, passed, detail)

    # ==================================================================
    # PHASE 1 -- DISCOVERY
    # ==================================================================
    print()
    print("  PHASE 1: DISCOVERING CHARGER CAPABILITIES")
    print("  " + "-" * 50)

    # SPR PDOs
    print("  Querying SPR PDOs ...  ", end="", flush=True)
    spr_pdos = fetch_pdos(ser, "list", 3.0)
    if not spr_pdos:
        print("WARNING: no PDOs received")
        print("  Check connection and charger. Aborting.")
        ser.close()
        sys.exit(1)

    spr_fixed = [p for p in spr_pdos if p["type"] == "FIXED"]
    spr_pps   = [p for p in spr_pdos if p["type"] == "PPS"]
    spr_avs   = [p for p in spr_pdos if p["type"] in ("SPR_AVS",)]
    print(f"found {len(spr_pdos)} PDOs "
          f"({len(spr_fixed)} Fixed, {len(spr_pps)} PPS, {len(spr_avs)} AVS)")
    print(pdo_summary_str(spr_pdos))

    # EPR PDOs
    print()
    print("  Entering EPR mode ...  ", end="", flush=True)
    drain(ser)
    send(ser, "epr")
    time.sleep(2.0)
    epr_pdos_raw = fetch_pdos(ser, "list all", 5.0)
    if len(epr_pdos_raw) > len(spr_pdos):
        epr_pdos = epr_pdos_raw
        epr_only = [p for p in epr_pdos if p["mode"] == "EPR"]
        epr_fixed = [p for p in epr_only if p["type"] == "FIXED"]
        epr_avs   = [p for p in epr_only if p["type"] == "EPR_AVS"]
        print(f"EPR active, {len(epr_pdos)} total PDOs "
              f"(+{len(epr_only)} EPR: {len(epr_fixed)} Fixed, {len(epr_avs)} AVS)")
        for p in epr_only:
            if p["type"] == "FIXED":
                print(f"    PDO{p['pos']:>2d}  Fixed  {p['v_nom']:>5.1f}V  (EPR)")
            elif p["type"] == "EPR_AVS":
                print(f"    PDO{p['pos']:>2d}  AVS    {p['v_min']:.1f}-{p['v_max']:.1f}V  (EPR)")
    else:
        epr_pdos = spr_pdos
        print("charger does not advertise additional EPR PDOs")

    # Build test plan
    all_pdos = _merge_pdos(spr_pdos, epr_pdos)
    plan = build_test_plan(spr_pdos, epr_pdos)
    random_steps = build_random_steps(all_pdos, n=args.random)
    total = len(plan) + len(random_steps)

    has_pps = any(p["type"] == "PPS" for p in all_pdos)
    has_avs = any(p["type"] in ("EPR_AVS", "SPR_AVS") for p in all_pdos)

    print()
    print(f"  Test plan: {len(plan)} systematic + {len(random_steps)} random = {total} steps")
    if not has_pps:
        print("  Note: charger has no PPS APDOs -- PPS tests skipped")
    if not has_avs:
        print("  Note: charger has no AVS APDOs -- AVS tests skipped")

    # ==================================================================
    # PHASE 2 -- SYSTEMATIC TESTING
    # ==================================================================
    print()
    print("  PHASE 2: SYSTEMATIC VOLTAGE TESTING")
    print("  " + "-" * 50)

    # Reset to known state
    send(ser, "rst")
    time.sleep(3.0)
    drain(ser)

    # Re-enter EPR if needed
    has_epr_steps = any("EPR" in s[0] or s[3] in ("avs",) for s in plan)
    if has_epr_steps:
        send(ser, "epr")
        read_lines_until(ser, re.compile(r"#EVT PD_MODE EPR"), 6.0)
        time.sleep(2.0)

    # Enable output
    send(ser, "on")
    time.sleep(2.0)
    drain(ser)

    step_num = 0
    section = None
    for label, cmd, expected_v, tol_mode in plan:
        # Section headers
        if "Fixed" in label and "EPR" not in label and section != "spr":
            section = "spr"
            print()
            print("  SPR Fixed PDOs:")
        elif "Fixed" in label and "EPR" in label and section != "epr":
            section = "epr"
            print()
            print("  EPR Fixed PDOs:")
        elif "PPS" in label and section != "pps":
            section = "pps"
            print()
            print("  PPS (min / mid / max):")
        elif "AVS" in label and section != "avs":
            section = "avs"
            print()
            print("  AVS (min / mid / max):")

        step_num += 1
        v_meas, i_meas, passed, detail = request_and_measure(
            ser, cmd, expected_v, tol_mode)
        print_result(step_num, total, label, v_meas, i_meas, passed, detail)
        results.append((label, v_meas, i_meas, passed, detail))
        time.sleep(INTER_STEP_S)

    # ==================================================================
    # PHASE 3 -- RANDOM SWEEP
    # ==================================================================
    if random_steps:
        types_in_sweep = set()
        for s in random_steps:
            types_in_sweep.add(s[3])
        type_desc = "/".join(t.upper() for t in sorted(types_in_sweep))

        print()
        print(f"  PHASE 3: RANDOM SWEEP ({len(random_steps)} points across {type_desc})")
        print("  " + "-" * 50)
        for label, cmd, expected_v, tol_mode in random_steps:
            step_num += 1
            v_meas, i_meas, passed, detail = request_and_measure(
                ser, cmd, expected_v, tol_mode)
            print_result(step_num, total, label, v_meas, i_meas, passed, detail)
            results.append((label, v_meas, i_meas, passed, detail))
            time.sleep(INTER_STEP_S)

    # ==================================================================
    # PHASE 4 -- CLEANUP
    # ==================================================================
    print()
    print("  Cleanup: output OFF, returning to minimum PDO ... ", end="", flush=True)
    send(ser, "off")
    time.sleep(0.5)
    send(ser, "setpdo 1")
    time.sleep(3.0)
    drain(ser)
    print("done")
    ser.close()

    # ---- Summary ----------------------------------------------------------
    n_pass = sum(1 for r in results if r[3])
    n_fail = len(results) - n_pass

    print()
    print("=" * 60)
    if n_fail == 0:
        print(f"  ALL PASSED: {n_pass}/{len(results)} steps")
    else:
        print(f"  RESULTS: {n_pass}/{len(results)} passed, {n_fail} FAILED")
    print("=" * 60)

    if n_fail:
        print()
        print("  Failed steps:")
        for label, v_meas, i_meas, passed, detail in results:
            if not passed:
                v_str = f"{v_meas:.3f}V" if v_meas is not None else "N/A"
                print(f"    {label}  measured={v_str}  ({detail})")

    # ---- Save report ------------------------------------------------------
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_name = f"charger_report_{ts}.txt"
    with open(report_name, "w") as f:
        f.write("AxxPD Charger Compatibility Test Report\n")
        f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Port: {port}\n")
        if idn:
            f.write(f"Device: {idn}\n")
        f.write(f"{'=' * 60}\n\n")

        f.write("Charger Capabilities:\n")
        f.write(pdo_summary_str(all_pdos))
        f.write("\n\n")

        f.write(f"{'=' * 60}\n")
        f.write(f"Test Results ({n_pass}/{len(results)} passed, {n_fail} failed):\n")
        f.write(f"{'=' * 60}\n\n")

        for label, v_meas, i_meas, passed, detail in results:
            tag = "PASS" if passed else "FAIL"
            v_str = f"{v_meas:.3f}V" if v_meas is not None else "N/A"
            i_str = f"{i_meas:.3f}A" if i_meas is not None else "N/A"
            fail_info = f"  ({detail})" if detail else ""
            f.write(f"  {tag}  {label:<42s}  {v_str:<10s} {i_str:<10s}{fail_info}\n")

        f.write(f"\n{'=' * 60}\n")
        if n_fail == 0:
            f.write(f"ALL PASSED: {n_pass}/{len(results)} steps\n")
        else:
            f.write(f"RESULTS: {n_pass}/{len(results)} passed, {n_fail} FAILED\n")

    print(f"\n  Report saved to: {report_name}")

    sys.exit(0 if n_fail == 0 else 1)


if __name__ == "__main__":
    main()
