#!/usr/bin/env python3
"""Command-line AxxPD firmware updater over USB CDC.

Drives the same bootloader protocol as the web dashboard's Firmware Update
panel (see Firmware/bootloader/bl_main.c), so firmware can be flashed from
scripts/CI without a browser:

    python axxpd_fwup.py [path\\to\\AxxPD.bin] [--port COMxx]

Defaults to ../Firmware/build/AxxPD.bin (relative to this script). The image
MUST be header-patched (make does this via tools/patch_app_header.py) — the
bootloader rejects unpatched uploads by design.

Flow:
  1. Find the AxxPD CDC port by VID:PID 0483:5740 (Windows usbser reports
     "Microsoft" as manufacturer, so never match on strings).
  2. If the app CLI answers, send 'fwup' — the app steps down to a 5 V
     contract first (flash hazard: never program during EPR), replies +FWUP
     and resets into the bootloader. If the device is already in the
     bootloader (HELLO answers), skip this step.
  3. Wait for CDC re-enumeration, then run the upload protocol:
     HELLO / START <size> <crc32> / ERASE / DATA chunks / VERIFY / BOOT.

Chunks are retried up to 3x on -DATA errors (the bootloader ACKs an already
programmed retransmit idempotently, so a lost ACK is safe).
"""

import argparse
import struct
import sys
import time
import zlib

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial is required:  pip install pyserial")
    sys.exit(2)

AXXPD_VID = 0x0483
AXXPD_PID = 0x5740

HDR_OFF = 0x200
HDR_MAGIC = 0x50585841  # "AXXP"
CHUNK = 512             # bootloader BL_CHUNK_MAX


def find_port():
    for p in list_ports.comports():
        if p.vid == AXXPD_VID and p.pid == AXXPD_PID:
            return p.device
    return None


def read_line(ser, timeout_s=3.0):
    """Read one CR/LF-terminated line; returns '' on timeout."""
    deadline = time.monotonic() + timeout_s
    buf = bytearray()
    while time.monotonic() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b in (b"\r", b"\n"):
            if buf:
                return buf.decode(errors="replace")
            continue
        buf += b
    return ""


def expect(ser, prefix, timeout_s=5.0, progress_prefix=None):
    """Read lines until one starts with prefix. Progress lines are printed.
    Raises on -"""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        line = read_line(ser, timeout_s=deadline - time.monotonic())
        if not line:
            break
        if progress_prefix and line.startswith(progress_prefix):
            print(f"  {line}")
            deadline = time.monotonic() + timeout_s  # activity: extend
            continue
        if line.startswith(prefix):
            return line
        if line.startswith("-"):
            raise RuntimeError(f"device error: {line}")
        # unrelated chatter (app banner etc.) — ignore
    raise RuntimeError(f"timeout waiting for {prefix!r}")


def open_port(port, timeout_s=15.0):
    """Open the CDC port, retrying while the device (re)enumerates."""
    deadline = time.monotonic() + timeout_s
    last_err = None
    while time.monotonic() < deadline:
        dev = port or find_port()
        if dev:
            try:
                s = serial.Serial(dev, 115200, timeout=0.1, write_timeout=2)
                s.dtr = True  # device output is DTR-gated
                time.sleep(0.2)
                return s
            except (OSError, serial.SerialException) as e:
                last_err = e
        time.sleep(0.3)
    raise RuntimeError(f"could not open AxxPD port ({last_err})")


def send_fwup(ser):
    """Send 'fwup' to the app CLI. Returns '+FWUP...', '-FWUP...' or '' if the
    port died (device already resetting into the bootloader)."""
    ser.reset_input_buffer()
    ser.write(b"fwup\n")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 15.0:
        try:
            line = read_line(ser, timeout_s=1.0)
        except (OSError, serial.SerialException):
            return ""  # port died: device reset into the bootloader
        if line.startswith("+FWUP") or line.startswith("-FWUP"):
            return line
    raise RuntimeError("no +FWUP/-FWUP reply (is the firmware too old for fwup?)")


def enter_bootloader(ser):
    """Returns an open port with the bootloader answering HELLO."""
    ser.reset_input_buffer()
    ser.write(b"\nHELLO\n")
    line = read_line(ser, timeout_s=1.0)
    got_bl = False
    t0 = time.monotonic()
    while line:
        if line.startswith("+HELLO AxxPD-BL"):
            got_bl = True
            break
        if time.monotonic() - t0 > 2.0:
            break
        line = read_line(ser, timeout_s=1.0)
    if got_bl:
        print("already in bootloader")
        return ser

    print("app CLI detected - sending 'fwup' (device steps down to 5 V first)")
    verdict = send_fwup(ser)

    if verdict.startswith("-FWUP"):
        # Typical cause: the sink is in EPR mode (even at a 5 V contract) and
        # pdsink has no sink-initiated EPR exit, so fwup can never step out of
        # it. Reboot the device instead — a cold boot lands in a plain 5 V SPR
        # contract (EPR auto-entry is disabled), from which fwup succeeds.
        print(f"  {verdict}")
        print("  falling back: rebooting device to leave EPR mode...")
        try:
            ser.write(b"reboot\n")
            ser.close()
        except (OSError, serial.SerialException):
            pass
        time.sleep(2.0)  # reset (+ possible source hard-reset VBUS cycle)
        ser = open_port(None, timeout_s=30.0)
        time.sleep(3.0)  # let boot-time PD negotiation settle at 5 V SPR
        print("  re-sending 'fwup'")
        verdict = send_fwup(ser)
        if verdict.startswith("-FWUP"):
            raise RuntimeError(f"{verdict} (replug the charger and retry)")

    if verdict:
        print(f"  {verdict}")
    try:
        ser.close()
    except (OSError, serial.SerialException):
        pass

    time.sleep(1.0)  # device resets; wait for re-enumeration
    ser = open_port(None, timeout_s=20.0)
    ser.reset_input_buffer()
    ser.write(b"\nHELLO\n")
    hello = expect(ser, "+HELLO AxxPD-BL", timeout_s=5.0)
    print(f"  {hello}")
    return ser


def upload(ser, image):
    size = len(image)
    (crc,) = struct.unpack_from("<I", image, HDR_OFF + 12)
    print(f"image: {size} bytes, crc32=0x{crc:08X}")

    ser.write(f"START {size} {crc:08x}\n".encode())
    expect(ser, "+START")

    print("erasing app region...")
    ser.write(b"ERASE\n")
    expect(ser, "+ERASE", timeout_s=60.0, progress_prefix="+ERASING")

    nchunks = (size + CHUNK - 1) // CHUNK
    for seq in range(nchunks):
        data = image[seq * CHUNK:(seq + 1) * CHUNK]
        ccrc = zlib.crc32(data) & 0xFFFFFFFF
        hdr = f"DATA {seq} {len(data)} {ccrc:08x}\n".encode()
        for attempt in range(3):
            ser.write(hdr + data)
            line = read_line(ser, timeout_s=5.0)
            # skip any leftover progress lines
            while line and not (line.startswith("+DATA") or line.startswith("-DATA")):
                line = read_line(ser, timeout_s=5.0)
            if line == f"+DATA {seq}":
                break
            print(f"  chunk {seq}: {line or 'timeout'} (attempt {attempt + 1}/3)")
            time.sleep(0.2)
        else:
            raise RuntimeError(f"chunk {seq} failed after 3 attempts")
        if seq % 32 == 0 or seq == nchunks - 1:
            pct = 100 * (seq + 1) // nchunks
            print(f"\r  {seq + 1}/{nchunks} chunks ({pct}%)", end="", flush=True)
    print()

    print("verifying...")
    ser.write(b"VERIFY\n")
    print(f"  {expect(ser, '+VERIFY', timeout_s=15.0)}")

    ser.write(b"BOOT\n")
    print(f"  {expect(ser, '+BOOT', timeout_s=5.0)}")
    print("device rebooting into the new firmware")


def main():
    ap = argparse.ArgumentParser(description="AxxPD USB firmware updater")
    ap.add_argument("image", nargs="?",
                    default=None, help="header-patched AxxPD.bin")
    ap.add_argument("--port", default=None, help="COM port (default: auto by VID:PID)")
    args = ap.parse_args()

    path = args.image
    if path is None:
        import os
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "Firmware", "build", "AxxPD.bin")
    with open(path, "rb") as f:
        image = f.read()

    if len(image) < HDR_OFF + 32 or len(image) % 8 != 0:
        raise SystemExit(f"{path}: not a padded AxxPD image (size {len(image)})")
    (magic,) = struct.unpack_from("<I", image, HDR_OFF)
    (hsize,) = struct.unpack_from("<I", image, HDR_OFF + 8)
    (hcrc,) = struct.unpack_from("<I", image, HDR_OFF + 12)
    if magic != HDR_MAGIC:
        raise SystemExit(f"{path}: app header magic missing at 0x{HDR_OFF:X}")
    if hsize != len(image) or hcrc == 0xFFFFFFFF:
        raise SystemExit(f"{path}: header not patched (run make / patch_app_header.py)")
    ver = image[HDR_OFF + 16:HDR_OFF + 32].split(b"\0")[0].decode(errors="replace")
    print(f"uploading AxxPD v{ver} from {path}")

    ser = open_port(args.port, timeout_s=10.0)
    print(f"port: {ser.port}")
    try:
        ser = enter_bootloader(ser)
        upload(ser, image)
    finally:
        try:
            ser.close()
        except (OSError, serial.SerialException):
            pass


if __name__ == "__main__":
    main()
