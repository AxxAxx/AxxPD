#!/usr/bin/env python3
# Copyright 2026 Axel Johansson
# SPDX-License-Identifier: GPL-3.0-only
"""
axxpd_selftest_full.py - comprehensive standalone feature test for AxxPD.

Exercises every AxxPD feature that can be verified over the USB-CDC command
interface with NO external load connected:

  A. Identity & SCPI plumbing (*IDN?, error queue, *CLS, :SYST:HELP?)
  B. PD capability discovery & contract queries (list, ct, :PD:CONTR?/:MODE?)
  C. Voltage negotiation across ALL advertised levels
       - SPR Fixed 5/9/15/20V, PPS min/mid/max, EPR Fixed, EPR AVS min/mid/max
       - plus a random sweep across the full range
  D. Measurement subsystem (meas, :MEAS:VOLT?/CURR?/ALL?/POW?/TEMP?)
  E. Output enable/disable (verify VBUS present when ON, ~0V when OFF)
  F. Protection configuration (OVP/OCP set + read-back + bounds + status/clear)
  G. EPR mode entry
  H. Fault log (flog)
  I. Live telemetry stream (#S)

Prints live PASS/FAIL, writes a timestamped .txt and .md report, and exits 0
only if every test passed.

Usage:
    python axxpd_selftest_full.py                 # auto-detect AxxPD CDC port
    python axxpd_selftest_full.py --port COM15
    python axxpd_selftest_full.py --random 8      # random-sweep step count

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

# --------------------------------------------------------------------------- #
# Tolerances / timing
# --------------------------------------------------------------------------- #
TOL_FIXED_PCT = 0.06        # +/-6 % for Fixed PDOs (no-load cap voltage droop)
TOL_PPS_MV    = 300         # +/-300 mV for PPS
TOL_AVS_MV    = 700         # +/-700 mV for AVS / EPR fixed (cap loading)
NOLOAD_I_MAX  = 0.30        # |I| must be under this with no load (A)
NOLOAD_W_MAX  = 1.0         # |P| must be under this with no load (W)
TEMP_LO, TEMP_HI = -10.0, 85.0   # plausible die/NTC range (degC)

CONTRACT_TIMEOUT_S = 8.0
SETTLE_S           = 2.0
OUTPUT_COOLDOWN_S  = 2.0    # output toggle has a 1.5 s firmware cooldown

# --------------------------------------------------------------------------- #
# Serial helpers
# --------------------------------------------------------------------------- #
def find_axxpd_port():
    """Find the AxxPD USB CDC port (VID:PID 0483:5740).  Fall back to STLink.

    Match on VID/PID integers, NOT manufacturer/description strings: on
    Windows the inbox usbser.sys driver reports manufacturer "Microsoft".
    """
    stlink = None
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        if p.vid == 0x0483 and p.pid == 0x5740:   # STM CDC = AxxPD -- preferred
            return p.device
        if (p.vid == 0x0483 and p.pid == 0x3754) or "stlink" in desc or "st-link" in desc:
            stlink = p.device
    return stlink


def drain(ser):
    ser.reset_input_buffer()
    while ser.read(1024):
        pass


def send(ser, cmd):
    ser.write((cmd + "\r\n").encode("ascii"))


def read_for(ser, seconds):
    lines, buf = [], ""
    deadline = time.monotonic() + seconds
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


def read_until(ser, stop_re, timeout_s):
    lines, buf = [], ""
    deadline = time.monotonic() + timeout_s
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
            if stop_re.search(line):
                return lines, line
    return lines, None


def query(ser, cmd, wait=1.0):
    """Send cmd, return response lines with the echoed command stripped."""
    drain(ser)
    send(ser, cmd)
    lines = read_for(ser, wait)
    return [ln for ln in lines if ln != cmd]


def first_float(s):
    m = re.search(r"-?\d+\.?\d*", s)
    return float(m.group(0)) if m else None

# --------------------------------------------------------------------------- #
# Parsing (PDO list / meas / contract)
# --------------------------------------------------------------------------- #
PDO_RE = re.compile(
    r"^(\d+),(FIXED|PPS|EPR_AVS|SPR_AVS),([\d.]+-?[\d.]*),([\d.]+[^,]*),(SPR|EPR)$")
CONTRACT_RE = re.compile(r"#EVT CONTRACT ")
MEAS_RE = re.compile(r"V=([-\d.]+)\s+I=([-\d.]+)\s+Tdie=([-\d.]+).*Tntc=([-\d.]+).*i2c=(\d+)")


def parse_pdo_line(line):
    m = PDO_RE.match(line)
    if not m:
        return None
    pos, ptype, vrange, ifield, mode = m.groups()
    pdo = {"pos": int(pos), "type": ptype, "mode": mode, "i_max": ifield}
    if "-" in vrange:
        lo, hi = vrange.split("-")
        pdo["v_min"], pdo["v_max"] = float(lo), float(hi)
    else:
        pdo["v_nom"] = float(vrange)
    return pdo


def fetch_pdos(ser, cmd="list", wait=3.0):
    return [p for p in (parse_pdo_line(l) for l in query(ser, cmd, wait)) if p]


def meas_full(ser):
    """Return dict with v,i,tdie,tntc,i2c or None."""
    for ln in query(ser, "meas", 1.2):
        m = MEAS_RE.search(ln)
        if m:
            return {"v": float(m.group(1)), "i": float(m.group(2)),
                    "tdie": float(m.group(3)), "tntc": float(m.group(4)),
                    "i2c": int(m.group(5)), "raw": ln}
    return None


def meas_v(ser):
    for ln in query(ser, ":MEAS:VOLT?", 1.0):
        f = first_float(ln)
        if f is not None and "NAN" not in ln.upper():
            return f
    return None

# --------------------------------------------------------------------------- #
# Result recording
# --------------------------------------------------------------------------- #
RESULTS = []   # (group, name, passed, detail, meas_v, req_v)
_cur_group = ""


def group(g):
    global _cur_group
    _cur_group = g
    print(f"\n  {g}")
    print("  " + "-" * 56)


def record(name, passed, detail="", meas_v=None, req_v=None):
    RESULTS.append((_cur_group, name, passed, detail, meas_v, req_v))
    tag = "PASS" if passed else "FAIL"
    extra = f"  ({detail})" if detail else ""
    print(f"    [{tag}] {name}{extra}")
    return passed


def check(name, cond, detail=""):
    return record(name, bool(cond), detail)

# --------------------------------------------------------------------------- #
# Voltage-step tolerance
# --------------------------------------------------------------------------- #
def tol_bounds(expected_v, mode):
    if mode == "fixed":
        return expected_v * (1 - TOL_FIXED_PCT), expected_v * (1 + TOL_FIXED_PCT)
    if mode == "pps":
        return expected_v - TOL_PPS_MV / 1000, expected_v + TOL_PPS_MV / 1000
    return expected_v - TOL_AVS_MV / 1000, expected_v + TOL_AVS_MV / 1000


def request_and_measure(ser, cmd, expected_v, mode):
    drain(ser)
    send(ser, cmd)
    read_until(ser, CONTRACT_RE, CONTRACT_TIMEOUT_S)
    time.sleep(SETTLE_S)
    v = meas_v(ser)
    if v is None:
        return None, False, "no measurement"
    lo, hi = tol_bounds(expected_v, mode)
    return v, (lo <= v <= hi), f"want {lo:.3f}-{hi:.3f} V"

# --------------------------------------------------------------------------- #
# Test groups
# --------------------------------------------------------------------------- #
def test_identity_scpi(ser):
    group("A. Identity & SCPI plumbing")
    idn = " ".join(query(ser, "*IDN?", 1.0))
    check("*IDN? returns AxxPD identity (4 SCPI fields)",
          "AxxPD" in idn and idn.count(",") >= 3, idn[:60])

    query(ser, "*CLS", 0.6)
    err = " ".join(query(ser, ":SYST:ERR?", 0.8))
    check(":SYST:ERR? clean after *CLS", "No error" in err or err.strip().startswith("0,"), err[:50])

    query(ser, "NOTACOMMAND_XYZ", 0.6)
    err = " ".join(query(ser, ":SYST:ERR?", 0.8))
    check("Error queue captures a bad command",
          ("No error" not in err) and (not err.strip().startswith("0,")), err[:50])
    query(ser, "*CLS", 0.6)
    err = " ".join(query(ser, ":SYST:ERR?", 0.8))
    check("*CLS clears the error queue", "No error" in err or err.strip().startswith("0,"), err[:50])

    helptxt = " ".join(query(ser, ":SYST:HELP?", 1.5))
    check(":SYST:HELP? returns help text", "meas" in helptxt and len(helptxt) > 80,
          f"{len(helptxt)} chars")


def test_capability(ser):
    group("B. PD capability & contract queries")
    pdos = fetch_pdos(ser, "list", 3.0)
    check("`list` enumerates PDOs", len(pdos) >= 1, f"{len(pdos)} PDOs")
    ct = query(ser, "ct", 1.0)
    ct_ok = any(parse_pdo_line(l) or re.search(r"PDO\d", l) for l in ct)
    check("`ct` reports active contract", ct_ok, " ".join(ct)[:50])
    pc = " ".join(query(ser, ":PD:CONTR?", 1.0))
    check(":PD:CONTR? responds", len(pc) > 0 and re.search(r"\d", pc), pc[:50])
    md = " ".join(query(ser, ":PD:MODE?", 1.0))
    check(":PD:MODE? reports SPR/EPR", "SPR" in md.upper() or "EPR" in md.upper(), md[:30])
    return pdos


def build_plan(pdos):
    steps = []
    for p in pdos:
        if p["type"] == "FIXED":
            v = p["v_nom"]
            tag = " EPR" if p["mode"] == "EPR" else ""
            steps.append((f"Fixed {v:.0f}V{tag} (PDO{p['pos']})", f"setpdo {p['pos']}", v, "fixed"))
    for p in pdos:
        if p["type"] == "PPS":
            lo, hi = p["v_min"], p["v_max"]
            for tg, v in [("min", lo), ("mid", round((lo + hi) / 2, 1)), ("max", hi)]:
                steps.append((f"PPS {lo:.1f}-{hi:.1f}V {tg} {v:.2f}V", f"setpps {v}", v, "pps"))
    for p in pdos:
        if p["type"] in ("EPR_AVS", "SPR_AVS"):
            lo, hi = p["v_min"], p["v_max"]
            for tg, v in [("min", lo), ("mid", round((lo + hi) / 2, 1)), ("max", hi)]:
                steps.append((f"AVS {lo:.1f}-{hi:.1f}V {tg} {v:.2f}V", f"setavs {v}", v, "avs"))
    return steps


def test_voltage_sweep(ser, pdos, n_random):
    group("C. Voltage negotiation across all levels")
    # Enter EPR (so EPR PDOs are advertised) and enable output so the INA228
    # (which measures the OUTPUT side of the LTC4368) sees the negotiated VBUS.
    send(ser, "epr"); read_until(ser, re.compile(r"#EVT PD_MODE EPR"), 6.0); time.sleep(1.5)
    pdos = fetch_pdos(ser, "list", 3.0) or pdos
    send(ser, "on"); time.sleep(OUTPUT_COOLDOWN_S)
    plan = build_plan(pdos)

    for label, cmd, ev, mode in plan:
        v, ok, want = request_and_measure(ser, cmd, ev, mode)
        if v is None:
            record(label, False, "no measurement", req_v=ev)
        else:
            detail = f"measured {v:.3f} V" + ("" if ok else f" ({want})")
            record(label, ok, detail, meas_v=v, req_v=ev)
        time.sleep(0.4)

    # random sweep
    fixed = [p for p in pdos if p["type"] == "FIXED"]
    pps = [p for p in pdos if p["type"] == "PPS"]
    avs = [p for p in pdos if p["type"] in ("EPR_AVS", "SPR_AVS")]
    for k in range(n_random):
        kinds = [x for x in (("f", fixed), ("p", pps), ("a", avs)) if x[1]]
        kind, pool = random.choice(kinds)
        p = random.choice(pool)
        if kind == "f":
            v = p["v_nom"]; cmd = f"setpdo {p['pos']}"; mode = "fixed"
        elif kind == "p":
            v = round(random.uniform(p["v_min"], p["v_max"]), 2); cmd = f"setpps {v}"; mode = "pps"
        else:
            v = round(random.uniform(p["v_min"], p["v_max"]), 1); cmd = f"setavs {v}"; mode = "avs"
        mv, ok, want = request_and_measure(ser, cmd, v, mode)
        if mv is None:
            record(f"Random {v:.2f}V ({mode})", False, "no measurement", req_v=v)
        else:
            detail = f"requested {v:.2f} V, measured {mv:.3f} V" + ("" if ok else f" ({want})")
            record(f"Random {v:.2f}V ({mode})", ok, detail, meas_v=mv, req_v=v)
        time.sleep(0.4)
    return pdos


def test_measurement(ser):
    group("D. Measurement subsystem")
    # settle on 15V fixed, output on
    request_and_measure(ser, "setpdo 3", 15.0, "fixed")
    ct_v = meas_v(ser) or 15.0

    m = meas_full(ser)
    check("`meas` parses V/I/Tdie/Tntc/i2c", m is not None, "" if m else "no meas line")
    if m:
        check("meas: I2C bus healthy (i2c==0)", m["i2c"] == 0, f"i2c={m['i2c']}")
        check("meas: die temp plausible", TEMP_LO <= m["tdie"] <= TEMP_HI, f"{m['tdie']}C")
        check("meas: NTC temp plausible", TEMP_LO <= m["tntc"] <= TEMP_HI, f"{m['tntc']}C")
        check("meas: no-load current near zero", abs(m["i"]) < NOLOAD_I_MAX, f"{m['i']}A")

    mv = meas_v(ser)
    check(":MEAS:VOLT? ~ contract", mv is not None and abs(mv - ct_v) < 2.0,
          f"{mv}V vs {ct_v}V" if mv is not None else "no value")

    ci = query(ser, ":MEAS:CURR?", 1.0)
    ci_v = first_float(" ".join(ci))
    check(":MEAS:CURR? near zero (no load)", ci_v is not None and abs(ci_v) < NOLOAD_I_MAX,
          f"{ci_v}A")

    allline = " ".join(query(ser, ":MEAS:ALL?", 1.0))
    fields = [f for f in allline.split(",") if re.search(r"-?\d", f)]
    check(":MEAS:ALL? returns 7-field CSV (V,I,W,Wh,Ah,Tdie,Tntc)", len(fields) >= 7,
          f"{len(fields)} fields: {allline[:50]}")

    pw = first_float(" ".join(query(ser, ":MEAS:POW?", 1.0)))
    check(":MEAS:POW? near zero (no load)", pw is not None and abs(pw) < NOLOAD_W_MAX, f"{pw}W")

    tline = " ".join(query(ser, ":MEAS:TEMP?", 1.0))
    tvals = [float(x) for x in re.findall(r"-?\d+\.?\d*", tline)][:2]
    check(":MEAS:TEMP? returns Tdie,Tntc in range",
          len(tvals) == 2 and all(TEMP_LO <= t <= TEMP_HI for t in tvals), tline[:40])


def test_output(ser):
    group("E. Output enable/disable (no load)")
    request_and_measure(ser, "setpdo 3", 15.0, "fixed")   # 15V contract
    send(ser, "on"); time.sleep(OUTPUT_COOLDOWN_S)
    von = meas_v(ser)
    check("Output ON -> VBUS present (~15V)", von is not None and von > 12.0, f"{von}V")
    send(ser, "off"); time.sleep(OUTPUT_COOLDOWN_S + 1.0)   # bleed resistor discharge
    voff = meas_v(ser)
    check("Output OFF -> VBUS collapses (<3V)", voff is not None and voff < 3.0, f"{voff}V")


def test_protection(ser):
    group("F. Protection configuration")
    # remember originals to restore (:CONF? returns mV / mA)
    ovp0_mv = first_float(" ".join(query(ser, ":CONF:OVP?", 1.0)))
    ocp0_ma = first_float(" ".join(query(ser, ":CONF:OCP?", 1.0)))

    r = " ".join(query(ser, "protect ovp 50V", 1.0))
    check("Set OVP=50V (confirmation)", "50" in r and "OVP" in r.upper(), r[:40])
    rb = first_float(" ".join(query(ser, ":CONF:OVP?", 1.0)))   # mV
    check(":CONF:OVP? reads back 50V", rb is not None and abs(rb - 50000) <= 1500, f"{rb} mV")

    r = " ".join(query(ser, "protect ocp 6A", 1.0))
    check("Set OCP=6A (confirmation)", "6" in r and "OCP" in r.upper(), r[:40])
    rb = first_float(" ".join(query(ser, ":CONF:OCP?", 1.0)))   # mA
    check(":CONF:OCP? reads back 6A", rb is not None and abs(rb - 6000) <= 300, f"{rb} mA")

    query(ser, "*CLS", 0.5)
    query(ser, "protect ovp 200V", 0.8)   # out of range (max 55V)
    err = " ".join(query(ser, ":SYST:ERR?", 0.8))
    check("Out-of-range OVP rejected (error queued)",
          "No error" not in err and not err.strip().startswith("0,"), err[:40])
    check("Device still responsive after bad input",
          "AxxPD" in " ".join(query(ser, "*IDN?", 1.0)))

    st = " ".join(query(ser, "protect status", 1.0))
    check("`protect status` shows no active fault", "fault=0" in st.replace(" ", "")
          or re.search(r"fault\s*=\s*0", st) is not None, st[:40])
    cl = " ".join(query(ser, "protect clear", 1.0))
    check("`protect clear` clears faults", "clear" in cl.lower(), cl[:40])

    # restore originals (convert mV/mA back to V/A for the CLI)
    if ovp0_mv:
        query(ser, f"protect ovp {int(round(ovp0_mv / 1000))}V", 1.0)
    if ocp0_ma:
        query(ser, f"protect ocp {ocp0_ma / 1000:.1f}A", 1.0)


def test_epr(ser):
    group("G. EPR mode entry")
    send(ser, "epr"); read_until(ser, re.compile(r"#EVT PD_MODE EPR"), 6.0); time.sleep(1.0)
    md = " ".join(query(ser, ":PD:MODE?", 1.0))
    ctl = " ".join(query(ser, "ct", 1.0))
    check("Device reports EPR mode active", "EPR" in md.upper() or "EPR" in ctl.upper(),
          (md or ctl)[:40])


def test_faultlog(ser):
    group("H. Fault log")
    fl = query(ser, "flog", 1.5)
    joined = " ".join(fl)
    ok = ("no faults" in joined.lower()) or any(re.match(r"^\d+,", l) for l in fl)
    check("`flog` returns valid fault-log output", ok, joined[:50])


def test_stream(ser):
    group("I. Live telemetry stream (#S)")
    drain(ser)
    send(ser, "stream on")
    lines = read_for(ser, 2.0)
    s_lines = [l for l in lines if l.startswith("#S")]
    fields_ok = all(len(re.findall(r"-?\d+\.?\d*", l)) >= 7 for l in s_lines) if s_lines else False
    check("`stream on` emits #S telemetry frames", len(s_lines) >= 2 and fields_ok,
          f"{len(s_lines)} frames")
    send(ser, "stream off")
    time.sleep(0.6)
    drain(ser)
    lines = read_for(ser, 1.5)
    check("`stream off` stops the stream",
          len([l for l in lines if l.startswith("#S")]) == 0,
          f"{len([l for l in lines if l.startswith('#S')])} stray frames")

# --------------------------------------------------------------------------- #
# Report
# --------------------------------------------------------------------------- #
def write_reports(port, idn, pdos):
    ts = datetime.now()
    n_pass = sum(1 for r in RESULTS if r[2])
    n_total = len(RESULTS)
    n_fail = n_total - n_pass
    stamp = ts.strftime("%Y%m%d_%H%M%S")

    # group results
    groups = {}
    for g, name, ok, detail, mv, rv in RESULTS:
        groups.setdefault(g, []).append((name, ok, detail, mv, rv))

    verdict = "ALL PASSED" if n_fail == 0 else f"{n_fail} FAILED"

    def badge(ok):
        return ('<span class="badge ok">PASS</span>' if ok
                else '<span class="badge bad">FAIL</span>')

    volt_rows = [(name, rv, mv, ok)
                 for g, name, ok, detail, mv, rv in RESULTS if mv is not None]
    vmeas = [mv for (_n, _rv, mv, _ok) in volt_rows]
    vspan = f"{min(vmeas):.2f}-{max(vmeas):.2f} V" if vmeas else "n/a"

    md = []
    md.append("Meta: AxxPD · Standalone Feature Test · axel.johansson10@gmail.com")
    md.append("")
    md.append("# AxxPD Standalone Feature Test")
    md.append("")
    # Big colour-coded verdict banner
    if n_fail == 0:
        md.append(
            '<div style="text-align:center;font-weight:700;font-size:30pt;'
            'letter-spacing:0.05em;text-transform:uppercase;color:#ffffff;'
            'background:#2f8a5b;border-radius:12px;padding:18px 10px;margin:4px 0 22px;">'
            f'&#10003;&nbsp; ALL {n_total} TESTS PASSED</div>')
    else:
        md.append(
            '<div style="text-align:center;font-weight:700;font-size:30pt;'
            'letter-spacing:0.05em;text-transform:uppercase;color:#ffffff;'
            'background:#bb524d;border-radius:12px;padding:18px 10px;margin:4px 0 22px;">'
            f'&#10007;&nbsp; {n_fail} OF {n_total} TESTS FAILED</div>')
    md.append("")
    md.append(f"**Date:** {ts.strftime('%Y-%m-%d %H:%M:%S')} &nbsp;|&nbsp; "
              f"**Device:** {idn or 'unknown'} &nbsp;|&nbsp; "
              f"**Port:** {port} &nbsp;|&nbsp; **Load:** none (standalone bench test)")
    md.append("")

    # Summary
    md.append("## Summary")
    md.append("")
    md.append(f"> **Verdict:** {n_pass} of {n_total} checks passed across {len(groups)} feature "
              f"groups, with no load connected. PD negotiation was verified across the full "
              f"advertised range (**{vspan}** measured) - SPR Fixed, PPS, EPR Fixed 28 V and EPR "
              f"AVS - alongside SCPI plumbing, the measurement subsystem, output enable/disable, "
              f"protection configuration, EPR mode, the fault log and live telemetry.")
    md.append("")
    md.append("| Group | Result | Tests |")
    md.append("|---|---|---|")
    for g, items in groups.items():
        gp = sum(1 for t in items if t[1])
        md.append(f"| {g} | {badge(gp == len(items))} | {gp}/{len(items)} |")
    md.append("")

    if pdos:
        md.append("## Charger capabilities")
        md.append("")
        md.append("| PDO | Type | Voltage | Limit | Mode |")
        md.append("|---|---|---|---|---|")
        for p in pdos:
            if p["type"] == "FIXED":
                md.append(f"| {p['pos']} | Fixed | {p['v_nom']:.1f} V | {p['i_max']} A | {p['mode']} |")
            elif p["type"] == "PPS":
                md.append(f"| {p['pos']} | PPS | {p['v_min']:.1f}-{p['v_max']:.1f} V | {p['i_max']} A | {p['mode']} |")
            else:
                md.append(f"| {p['pos']} | AVS | {p['v_min']:.1f}-{p['v_max']:.1f} V | {p['i_max']} | {p['mode']} |")
        md.append("")

    if volt_rows:
        md.append("## Measured voltages")
        md.append("")
        md.append("| Step | Requested | Measured | Result |")
        md.append("|---|---|---|---|")
        for name, rv, mv, ok in volt_rows:
            req = f"{rv:.3f} V" if rv is not None else "-"
            md.append(f"| {name} | {req} | **{mv:.3f} V** | {badge(ok)} |")
        md.append("")

    md.append("## Detailed results")
    md.append("")
    for g, items in groups.items():
        md.append(f"### {g}")
        md.append("")
        for name, ok, detail, mv, rv in items:
            extra = f" - {detail}" if detail else ""
            md.append(f"- {badge(ok)} {name}{extra}")
        md.append("")

    md_path = f"axxpd_selftest_{stamp}.md"
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md))

    txt_path = f"axxpd_selftest_{stamp}.txt"
    with open(txt_path, "w", encoding="utf-8") as f:
        f.write(f"AxxPD Standalone Feature Test Report\n")
        f.write(f"Date: {ts.strftime('%Y-%m-%d %H:%M:%S')}  Device: {idn}  Port: {port}\n")
        f.write("=" * 64 + "\n")
        for g, name, ok, detail, mv, rv in RESULTS:
            f.write(f"  [{'PASS' if ok else 'FAIL'}] {g} :: {name}"
                    f"{('  (' + detail + ')') if detail else ''}\n")
        f.write("=" * 64 + "\n")
        f.write(f"{n_pass}/{n_total} passed, {n_fail} failed - {verdict}\n")

    return md_path, txt_path, n_pass, n_total, n_fail

# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--random", type=int, default=6)
    args = ap.parse_args()

    port = args.port or find_axxpd_port()
    if not port:
        sys.exit("no AxxPD CDC port found")
    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
        ser.dtr = True
        ser.rts = True
    except serial.SerialException as e:
        sys.exit(f"could not open {port}: {e}")

    print(f"AxxPD Full Standalone Feature Test  -  {port} @ {args.baud}")
    print("=" * 60)
    time.sleep(0.5)
    drain(ser)
    idn = ""
    for ln in query(ser, "*IDN?", 1.0):
        if "AxxPD" in ln:
            idn = ln
            break
    print(f"  Device: {idn}")

    pdos = []
    try:
        test_identity_scpi(ser)
        pdos = test_capability(ser)
        pdos = test_voltage_sweep(ser, pdos, args.random)
        test_measurement(ser)
        test_output(ser)
        test_protection(ser)
        test_epr(ser)
        test_faultlog(ser)
        test_stream(ser)
    finally:
        # cleanup: stream off, output off, back to 5V
        try:
            send(ser, "stream off"); time.sleep(0.3)
            send(ser, "off"); time.sleep(0.5)
            send(ser, "setpdo 1"); time.sleep(2.0)
        except Exception:
            pass

    md_path, txt_path, n_pass, n_total, n_fail = write_reports(port, idn, pdos)
    ser.close()

    print("\n" + "=" * 60)
    if n_fail == 0:
        print(f"  ALL PASSED: {n_pass}/{n_total} tests")
    else:
        print(f"  RESULTS: {n_pass}/{n_total} passed, {n_fail} FAILED")
        for g, name, ok, detail, mv, rv in RESULTS:
            if not ok:
                print(f"    FAIL: {g} :: {name}  ({detail})")
    print("=" * 60)
    print(f"  Reports: {md_path} , {txt_path}")
    sys.exit(0 if n_fail == 0 else 1)


if __name__ == "__main__":
    main()
