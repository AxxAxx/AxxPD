#!/usr/bin/env python3
"""Patch the AxxPD application header with image size and CRC32.

Usage: patch_app_header.py <AxxPD.bin>   (patched in place)

The app carries a bl_app_header_t at offset 0x200 (see
bootloader/bl_layout.h):

    0x200  u32 magic        "AXXP" (0x50585841 LE)  — must already be present
    0x204  u32 hdr_version
    0x208  u32 image_size   <- patched: padded file size
    0x20C  u32 image_crc    <- patched: zlib CRC32 of the whole padded image
                               with these 4 bytes read as zeros
    0x210  char fw_version[16]

The image is padded with 0xFF to a multiple of 8 bytes (STM32G4 flash
double-word) before computing size/CRC. The bootloader recomputes the same
CRC (hardware CRC unit, zlib-compatible configuration) at every boot and
after every upload.
"""

import struct
import sys
import zlib

HDR_OFF = 0x200
MAGIC = 0x50585841  # "AXXP"


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2

    path = sys.argv[1]
    with open(path, "rb") as f:
        data = bytearray(f.read())

    if len(data) < HDR_OFF + 32:
        print(f"patch_app_header: {path}: too small to contain a header")
        return 1

    (magic,) = struct.unpack_from("<I", data, HDR_OFF)
    if magic != MAGIC:
        print(f"patch_app_header: {path}: header magic not found at 0x{HDR_OFF:X} "
              f"(got 0x{magic:08X}) — is .app_header in the linker script?")
        return 1

    # Pad to a flash double-word multiple.
    if len(data) % 8:
        data += b"\xFF" * (8 - len(data) % 8)

    size = len(data)
    APP_MAX_SIZE = 235520  # bl_layout.h APP_MAX_SIZE — bootloader rejects larger
    if size > APP_MAX_SIZE:
        print(f"patch_app_header: {path}: image {size} B exceeds the app "
              f"region ({APP_MAX_SIZE} B) — it will not fit behind the bootloader")
        return 1
    struct.pack_into("<I", data, HDR_OFF + 8, size)
    struct.pack_into("<I", data, HDR_OFF + 12, 0)  # crc field zeroed for CRC
    crc = zlib.crc32(bytes(data)) & 0xFFFFFFFF
    struct.pack_into("<I", data, HDR_OFF + 12, crc)

    with open(path, "wb") as f:
        f.write(data)

    version = data[HDR_OFF + 16:HDR_OFF + 32].split(b"\0")[0].decode(errors="replace")
    print(f"patch_app_header: {path}: v{version} size={size} crc32=0x{crc:08X}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
