/**
 * @file    bl_layout.h
 * @brief   Flash/RAM layout contract shared between the AxxPD bootloader and
 *          the application. Included by both builds — keep it C-compatible
 *          and free of any HAL dependency.
 *
 * Flash map (STM32G491CC, 256 KB, 2 KB pages):
 *
 *   0x08000000 .. 0x08005FFF   bootloader      (24 KB, pages 0-11, never
 *                                               erased by the bootloader)
 *   0x08006000 .. 0x0803F7FF   application     (230 KB, pages 12-126)
 *   0x0803F800 .. 0x0803FFFF   settings        (2 KB, page 127 — owned by
 *                                               settings.c, never touched
 *                                               by the bootloader)
 *
 * The application carries a header at APP_BASE + APP_HDR_OFFSET (0x200,
 * right after the 0x1D8-byte vector table). `image_size` and `image_crc`
 * are 0xFFFFFFFF placeholders in the compiled image and are patched by
 * tools/patch_app_header.py after objcopy. The CRC is a standard IEEE
 * CRC-32 (zlib crc32) computed over the whole padded image with the four
 * bytes of `image_crc` replaced by zeros. The image is padded with 0xFF to
 * a multiple of 8 bytes (flash double-word) before patching.
 *
 * A dev image flashed over SWD without the patch step keeps both fields at
 * 0xFFFFFFFF; the bootloader accepts it after vector-table sanity checks
 * (see bl_main.c) so SWD development never gets locked out.
 */
#ifndef BL_LAYOUT_H
#define BL_LAYOUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BL_FLASH_BASE       0x08000000UL
#define BL_FLASH_SIZE       0x00006000UL                     /* 24 KB       */
#define APP_BASE            (BL_FLASH_BASE + BL_FLASH_SIZE)  /* 0x08006000  */

/* Bootloader metadata page (page 11, last page of the BL region — the BL
 * linker script is capped at 22 KB so code can never grow into it). Holds
 * the "upload dirty" marker: the first doubleword is programmed to zero
 * when an upload starts erasing the app and the page is erased again only
 * after VERIFY passes. A clean (erased) marker lets the boot path skip the
 * full-image CRC — cold-boot time matters: some chargers cycle VBUS if the
 * sink's PD stack isn't up fast enough after attach (~35 ms of boot CRC
 * was enough to break one bench charger). */
#define BL_META_ADDR        0x08005800UL
#define BL_META_PAGE        11U
#define APP_END             0x0803F800UL                     /* excl. — settings page */
#define APP_MAX_SIZE        (APP_END - APP_BASE)             /* 235520 B    */
#define APP_FIRST_PAGE      12U                              /* 0x6000/2048 */
#define APP_PAGE_COUNT      115U                             /* pages 12-126 */
#define FLASH_PAGE_BYTES    2048U

#define APP_HDR_OFFSET      0x200UL
#define APP_HDR_ADDR        (APP_BASE + APP_HDR_OFFSET)
#define APP_HDR_MAGIC       0x50585841UL   /* "AXXP" (AxxPD) little-endian */
#define APP_HDR_UNPATCHED   0xFFFFFFFFUL   /* size/crc value in dev images */

/* Offset of the image_crc field within the image — the CRC is computed with
 * this word read as zero (must stay word-aligned). */
#define APP_HDR_CRC_OFFSET  (APP_HDR_OFFSET + 12UL)

typedef struct {
    uint32_t magic;          /* APP_HDR_MAGIC                                */
    uint32_t hdr_version;    /* 1                                            */
    uint32_t image_size;     /* padded image size in bytes (patched)         */
    uint32_t image_crc;      /* zlib CRC-32, crc field zeroed (patched)      */
    char     fw_version[16]; /* NUL-terminated, e.g. "0.2.0"                 */
    uint32_t reserved[3];    /* 0xFFFFFFFF — future use                      */
} bl_app_header_t;

/* Bootloader-request mailbox: last 16 bytes of SRAM, excluded from both
 * images' RAM regions (linker LENGTH = 112K - 16) so it survives reset and
 * is never zeroed by startup code. Word 0 = magic, word 1 = ~magic — both
 * must match, which makes a false positive from random power-up SRAM
 * content negligible. The bootloader clears both words on every boot. */
#define BL_REQ_ADDR         0x2001BFF0UL
#define BL_REQ_MAGIC        0xB007C0DEUL

#ifdef __cplusplus
}
#endif

#endif /* BL_LAYOUT_H */
