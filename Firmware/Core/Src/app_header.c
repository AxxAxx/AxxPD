/**
 * @file    app_header.c
 * @brief   Application header consumed by the bootloader (bl_main.c).
 *
 * Lives in the fixed .app_header section at APP_BASE + 0x200 (see the
 * linker script). image_size/image_crc are 0xFFFFFFFF placeholders here;
 * tools/patch_app_header.py fills them in after objcopy. An unpatched
 * (SWD-flashed dev) image is still bootable — the bootloader accepts the
 * placeholder pair after vector-table sanity checks.
 */

#include "bl_layout.h"
#include "fw_version.h"

__attribute__((section(".app_header"), used))
const bl_app_header_t g_app_header = {
    .magic       = APP_HDR_MAGIC,
    .hdr_version = 1U,
    .image_size  = APP_HDR_UNPATCHED,
    .image_crc   = APP_HDR_UNPATCHED,
    .fw_version  = FW_VERSION,
    .reserved    = { 0xFFFFFFFFU, 0xFFFFFFFFU, 0xFFFFFFFFU },
};
