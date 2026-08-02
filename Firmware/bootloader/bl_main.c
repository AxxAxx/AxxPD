/**
 * @file    bl_main.c
 * @brief   AxxPD USB CDC bootloader.
 *
 * Design goals (in priority order):
 *   1. Never brick: the bootloader never erases itself or the settings page,
 *      only marks an upload good after full CRC verification, and always
 *      falls back to bootloader mode when the app image is invalid. All
 *      fault handlers and the IWDG reset the MCU instead of hanging.
 *   2. Keep USB-C power alive: the ROM bootloader kills the dead-battery
 *      Rd (it reconfigures PA9/PA10 = UCPD1_DBCC1/2 for USART1) which makes
 *      the source remove VBUS. This bootloader never touches PA9/PA10 and
 *      never sets PWR_CR3.UCPD_DBDIS, so the hardware dead-battery pull-
 *      downs keep the source at vSafe5V. It also stays on HSI16 (same as
 *      the app's cold-boot phase 1) — no PLL, no inrush surprises.
 *   3. Accept any AxxPD firmware: validation is magic + CRC only. No
 *      version comparison — downgrades are always allowed.
 *
 * Boot decision (runs before HAL init; ~1 ms on a clean boot — the full
 * CRC (~35 ms at 16 MHz) runs ONLY when the upload-dirty marker is set,
 * because impatient chargers cycle VBUS if the sink's PD stack isn't up
 * fast after attach — observed on the bench):
 *   - app requested bootloader via SRAM mailbox         -> bootloader mode
 *     (checked FIRST, without reading app flash — see the ECC note below)
 *   - app header/vectors invalid (quick check)          -> bootloader mode
 *   - upload-dirty marker set AND full CRC fails        -> bootloader mode
 *   - otherwise                                         -> jump to app
 * (No boot-time button check — see the note above bootloader_mode support:
 * GPIO sampling during a cold power ramp caused false bootloader entry.)
 *
 * Upload protocol (USB CDC, 512-byte chunks, see AxxPD_Command_Reference.md):
 *   host: HELLO                          dev: +HELLO AxxPD-BL 1 <appmax> <chunk>
 *   host: INFO                           dev: +INFO valid=<0|1> ver=<s> size=<n>
 *   host: START <size> <crc32hex>        dev: +START
 *   host: ERASE                          dev: +ERASING <n>.. then +ERASE
 *   host: DATA <seq> <len> <crc32hex>\n<len raw bytes>
 *                                        dev: +DATA <seq>  |  -DATA <seq> <why>
 *   host: VERIFY                         dev: +VERIFY  |  -VERIFY <why>
 *   host: BOOT                           dev: +BOOT (then resets into app)
 *   host: ABORT                          dev: +ABORT (transfer state cleared)
 *
 * A failed/interrupted upload leaves the app region partially written; the
 * next boot lands back here ready to retry — either because the image CRC
 * fails, or (torn doubleword) because reading it raises the flash ECC NMI,
 * whose handler (bl_it.c, mirrored in the app) arms the request mailbox so
 * the next boot skips app-flash reads entirely. Nothing short of corrupting
 * the bootloader itself (SWD only) can lock the device out.
 */

#include "stm32g4xx_hal.h"
#include "bl_layout.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define BL_VERSION_STR      "1"
#define BL_CHUNK_MAX        512U
#define BL_LINE_MAX         64U
#define BL_CHUNK_TIMEOUT_MS 2000U   /* max wait for a chunk's raw bytes      */
#define BL_IDLE_EXIT_MS     180000U /* auto-boot valid app after 3 min idle  */
#define IWDG_KICK()         (IWDG->KR = 0xAAAAU)

extern USBD_HandleTypeDef hUsbDeviceFS;

/* ---------------------------------------------------------------------------
 * CRC-32 (zlib-compatible) via the hardware CRC unit.
 * REV_IN by word + REV_OUT + init 0xFFFFFFFF + final invert == zlib crc32
 * over the little-endian byte stream. All images/chunks are padded to a
 * multiple of 4 (in fact 8), so word feeding is exact.
 * ------------------------------------------------------------------------ */
static void crc32_begin(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    (void)RCC->AHB1ENR;                       /* clock-enable delay          */
    CRC->CR = CRC_CR_REV_OUT | CRC_CR_REV_IN | CRC_CR_RESET;
}

static void crc32_feed_words(const uint32_t *p, uint32_t nwords)
{
    while (nwords--) {
        CRC->DR = *p++;
    }
}

static uint32_t crc32_end(void)
{
    uint32_t r = ~CRC->DR;
    RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
    return r;
}

static uint32_t crc32_buf(const uint8_t *buf, uint32_t len)
{
    crc32_begin();
    crc32_feed_words((const uint32_t *)(uintptr_t)buf, len / 4U);
    return crc32_end();
}

/** CRC of the app flash region [APP_BASE, APP_BASE+size) with the header's
 *  image_crc word substituted by zero (matching tools/patch_app_header.py). */
static uint32_t crc32_app_flash(uint32_t size)
{
    const uint32_t *p = (const uint32_t *)APP_BASE;
    uint32_t nwords   = size / 4U;
    uint32_t crc_word = APP_HDR_CRC_OFFSET / 4U;
    crc32_begin();
    for (uint32_t i = 0; i < nwords; i++) {
        CRC->DR = (i == crc_word) ? 0U : p[i];
        if ((i & 0x3FFFU) == 0U) {
            IWDG_KICK();                      /* no-op until IWDG started    */
        }
    }
    return crc32_end();
}

/* ---------------------------------------------------------------------------
 * App validation + jump
 * ------------------------------------------------------------------------ */
static const bl_app_header_t *app_header(void)
{
    return (const bl_app_header_t *)APP_HDR_ADDR;
}

static int app_vectors_sane(void)
{
    uint32_t sp = *(const uint32_t *)APP_BASE;
    uint32_t pc = *(const uint32_t *)(APP_BASE + 4U);
    if (sp < 0x20000000UL || sp > 0x2001C000UL) return 0;
    if (pc < APP_BASE || pc >= APP_END || (pc & 1U) == 0U) return 0;
    return 1;
}

/* Cheap structural checks (~µs): header magic/version + vector sanity.
 * Run on EVERY boot. */
static int app_quick_valid(void)
{
    const bl_app_header_t *h = app_header();
    if (h->magic != APP_HDR_MAGIC) return 0;
    if (h->hdr_version != 1U)      return 0;   /* also guards partial erase:
                                                  0x00000001 erases to FFs
                                                  almost immediately */
    if (!app_vectors_sane())       return 0;
    return 1;
}

/* Full-image CRC (~35 ms at 16 MHz). Run only when the upload-dirty marker
 * is set (interrupted update) and during VERIFY — NOT on clean boots: the
 * boot-to-PD-ready time is critical, some chargers cycle VBUS when the
 * sink stays silent too long after attach. */
static int app_crc_valid(void)
{
    const bl_app_header_t *h = app_header();
    if (h->image_size == APP_HDR_UNPATCHED && h->image_crc == APP_HDR_UNPATCHED) {
        /* Unpatched dev image (SWD-flashed straight from the compiler).
         * Accept on the quick checks so development never locks out.
         * Images uploaded through the bootloader always carry a real CRC. */
        return 1;
    }
    if (h->image_size < 1024UL || h->image_size > APP_MAX_SIZE) return 0;
    if ((h->image_size & 7U) != 0U)                             return 0;
    return crc32_app_flash(h->image_size) == h->image_crc;
}

/* ---------------------------------------------------------------------------
 * Upload-dirty marker (bootloader metadata page)
 * ------------------------------------------------------------------------ */
static int bl_meta_dirty(void)
{
    const uint32_t *m = (const uint32_t *)BL_META_ADDR;
    return (m[0] != 0xFFFFFFFFUL) || (m[1] != 0xFFFFFFFFUL);
}

static int bl_meta_erase(void)
{
    FLASH_EraseInitTypeDef er = {0};
    uint32_t bad_page = 0;
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.Banks     = FLASH_BANK_1;
    er.Page      = BL_META_PAGE;
    er.NbPages   = 1U;
    HAL_FLASH_Unlock();
    IWDG_KICK();
    HAL_StatusTypeDef rc = HAL_FLASHEx_Erase(&er, &bad_page);
    HAL_FLASH_Lock();
    return (rc == HAL_OK) ? 0 : -1;
}

static int bl_meta_mark_dirty(void)
{
    /* Erase first so a torn previous state can't cause a program error,
     * then program the first doubleword to zero. */
    if (bl_meta_erase() != 0) return -1;
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef rc =
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BL_META_ADDR, 0ULL);
    HAL_FLASH_Lock();
    return (rc == HAL_OK) ? 0 : -1;
}

static void jump_to_app(void)
{
    uint32_t sp = *(const uint32_t *)APP_BASE;
    uint32_t pc = *(const uint32_t *)(APP_BASE + 4U);
    /* Hand the app a reset-like machine: only the CRC clock was touched on
     * the fast path and crc32_end() already disabled it. */
    SCB->VTOR = APP_BASE;
    __DSB();
    __ISB();
    __set_MSP(sp);
    ((void (*)(void))pc)();
    /* not reached */
}

/* NOTE: an earlier revision sampled SELECT (PB12) here as a force-bootloader
 * escape hatch. REMOVED: the buttons are active-high with RC conditioning,
 * and during a cold VBUS/3.3V ramp the line can read high for the first few
 * ms — the bootloader then entered bootloader mode (a deaf PD sink), the
 * charger cycled VBUS, and the device attach-looped. Bootloader entry is
 * fwup (mailbox), an invalid app, or SWD — do NOT add early GPIO sampling
 * back without accounting for power-ramp transients. */

/* ---------------------------------------------------------------------------
 * Bootloader-mode support
 * ------------------------------------------------------------------------ */
static void bl_clock_config(void)
{
    /* SYSCLK stays on HSI16 (reset default — matches the app's cold-boot
     * phase 1, keeps inrush low). Enable HSI48 for USB; CCIPR CLK48 reset
     * selection is HSI48 already. */
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State     = RCC_HSI48_ON;
    osc.PLL.PLLState   = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        NVIC_SystemReset();
    }
}

static void iwdg_start(void)
{
    /* ~32 s at LSI 32 kHz /256. Backstop against any hang: a watchdog reset
     * re-runs the boot decision, which lands back in bootloader mode when
     * the app is invalid. Kicked from the main loop and slow flash loops. */
    IWDG->KR  = 0x5555U;
    IWDG->PR  = 7U;       /* /256 */
    IWDG->RLR = 0xFFFU;
    IWDG->KR  = 0xAAAAU;
    IWDG->KR  = 0xCCCCU;
}

static void out_str(const char *s)
{
    /* Short timeout: if the host vanished we must not stall the loop (the
     * IWDG is running). CDC_Transmit_Blocking is DTR-gated like in the app. */
    CDC_Transmit_Blocking((const uint8_t *)s, (uint16_t)strlen(s), 100U);
}

static void out_line(const char *fmt, unsigned a, unsigned b)
{
    char buf[BL_LINE_MAX];
    snprintf(buf, sizeof(buf), fmt, a, b);
    out_str(buf);
}

/* Transfer state */
static uint32_t xfer_size;      /* expected image size (0 = no START yet)    */
static uint32_t xfer_crc;       /* expected image CRC                        */
static uint32_t xfer_received;  /* bytes accepted so far                     */
static uint32_t xfer_next_seq;  /* next expected chunk sequence number       */
static uint8_t  xfer_erased;    /* app region erased for this transfer      */
static uint8_t  xfer_verified;  /* VERIFY passed — gates BOOT after upload  */

static void xfer_reset(void)
{
    xfer_size = xfer_crc = xfer_received = xfer_next_seq = 0U;
    xfer_erased = 0U;
    xfer_verified = 0U;
}

static int erase_app_region(void)
{
    HAL_FLASH_Unlock();
    for (uint32_t pg = 0; pg < APP_PAGE_COUNT; pg++) {
        FLASH_EraseInitTypeDef er = {0};
        uint32_t bad_page = 0;
        er.TypeErase = FLASH_TYPEERASE_PAGES;
        er.Banks     = FLASH_BANK_1;
        er.Page      = APP_FIRST_PAGE + pg;
        er.NbPages   = 1U;
        IWDG_KICK();
        if (HAL_FLASHEx_Erase(&er, &bad_page) != HAL_OK) {
            HAL_FLASH_Lock();
            return -1;
        }
        if ((pg & 15U) == 0U) {
            out_line("+ERASING %u/%u\r\n", (unsigned)pg, APP_PAGE_COUNT);
        }
    }
    HAL_FLASH_Lock();
    return 0;
}

static int program_chunk(uint32_t addr, const uint8_t *data, uint32_t len)
{
    /* Strict bounds: never below APP_BASE, never at/above the settings page. */
    if (addr < APP_BASE || addr + len > APP_END || (len & 7U) != 0U) {
        return -1;
    }
    HAL_FLASH_Unlock();
    for (uint32_t off = 0; off < len; off += 8U) {
        uint64_t dw;
        memcpy(&dw, data + off, 8U);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + off, dw)
                != HAL_OK) {
            HAL_FLASH_Lock();
            return -2;
        }
    }
    HAL_FLASH_Lock();
    /* Read-back verify */
    if (memcmp((const void *)(uintptr_t)addr, data, len) != 0) {
        return -3;
    }
    return 0;
}

/* Byte input with millisecond deadline. Returns -1 on timeout. */
static int get_byte(uint32_t deadline_ms)
{
    uint32_t t0 = HAL_GetTick();
    for (;;) {
        int c = CDC_RxGetByte();
        if (c >= 0) return c;
        if ((uint32_t)(HAL_GetTick() - t0) >= deadline_ms) return -1;
        IWDG_KICK();
    }
}

/* Drain any garbage until the line is idle — used to resync after a chunk
 * timeout so a half-received chunk can't be parsed as commands. */
static void drain_input(void)
{
    uint32_t idle_t0 = HAL_GetTick();
    for (;;) {
        if (CDC_RxGetByte() >= 0) {
            idle_t0 = HAL_GetTick();
        } else if ((uint32_t)(HAL_GetTick() - idle_t0) >= 100U) {
            return;
        }
        IWDG_KICK();
    }
}

static void handle_data(char *args)
{
    /* DATA <seq> <len> <crc32hex> */
    static uint8_t chunk[BL_CHUNK_MAX] __attribute__((aligned(8)));
    char *end;
    uint32_t seq = strtoul(args, &end, 10);
    uint32_t len = strtoul(end, &end, 10);
    uint32_t ccrc = strtoul(end, &end, 16);

    if (xfer_size == 0U || !xfer_erased) {
        out_str("-DATA nostart\r\n");
        drain_input();
        return;
    }
    if (len == 0U || len > BL_CHUNK_MAX || (len & 7U) != 0U) {
        out_str("-DATA badlen\r\n");
        drain_input();
        return;
    }

    /* Receive raw payload (always — keeps the stream in sync) */
    for (uint32_t i = 0; i < len; i++) {
        int c = get_byte(BL_CHUNK_TIMEOUT_MS);
        if (c < 0) {
            /* Drain BEFORE replying: the host reacts to the error line
             * instantly, and a retransmit arriving mid-drain would be
             * swallowed, burning a retry attempt. */
            drain_input();
            out_line("-DATA %u timeout\r\n", (unsigned)seq, 0U);
            return;
        }
        chunk[i] = (uint8_t)c;
    }

    if (xfer_next_seq != 0U && seq + 1U == xfer_next_seq) {
        /* Retransmit of the chunk we already ACKed (our ACK was lost) —
         * ACK again idempotently, do not rewrite flash. */
        out_line("+DATA %u\r\n", (unsigned)seq, 0U);
        return;
    }
    if (seq != xfer_next_seq) {
        out_line("-DATA %u seq\r\n", (unsigned)seq, 0U);
        return;
    }
    if (crc32_buf(chunk, len) != ccrc) {
        out_line("-DATA %u crc\r\n", (unsigned)seq, 0U);
        return;
    }
    if (xfer_received + len > xfer_size) {
        out_line("-DATA %u overflow\r\n", (unsigned)seq, 0U);
        return;
    }

    int rc = program_chunk(APP_BASE + xfer_received, chunk, len);
    if (rc != 0) {
        out_line("-DATA %u pgm%u\r\n", (unsigned)seq, (unsigned)(-rc));
        return;
    }
    xfer_received += len;
    xfer_next_seq++;
    out_line("+DATA %u\r\n", (unsigned)seq, 0U);
}

static void handle_verify(void)
{
    if (xfer_size == 0U || xfer_received != xfer_size) {
        out_str("-VERIFY incomplete\r\n");
        return;
    }
    if (crc32_app_flash(xfer_size) != xfer_crc) {
        out_str("-VERIFY crc\r\n");
        return;
    }
    const bl_app_header_t *h = app_header();
    if (h->magic != APP_HDR_MAGIC || h->image_size != xfer_size ||
        h->image_crc != xfer_crc || !app_vectors_sane()) {
        out_str("-VERIFY header\r\n");
        return;
    }
    /* Clear the upload-dirty marker — future boots take the fast path. If
     * this erase is torn by power loss, the next boot re-runs the full CRC
     * (image is already complete, so it passes) and self-heals the marker. */
    if (bl_meta_erase() != 0) {
        out_str("-VERIFY meta\r\n");
        return;
    }
    xfer_verified = 1U;
    out_str("+VERIFY\r\n");
}

static void bl_reset(void)
{
    HAL_Delay(50U);                    /* let the +BOOT ack flush            */
    USBD_Stop(&hUsbDeviceFS);
    USBD_DeInit(&hUsbDeviceFS);
    HAL_Delay(20U);                    /* host sees a clean disconnect       */
    NVIC_SystemReset();
}

static void handle_line(char *line, uint8_t app_was_valid)
{
    if (strcmp(line, "HELLO") == 0) {
        xfer_reset();
        out_line("+HELLO AxxPD-BL " BL_VERSION_STR " %u %u\r\n",
                 (unsigned)APP_MAX_SIZE, BL_CHUNK_MAX);
    } else if (strcmp(line, "INFO") == 0) {
        const bl_app_header_t *h = app_header();
        char buf[96];
        char ver[sizeof(h->fw_version) + 1];
        memcpy(ver, h->fw_version, sizeof(h->fw_version));
        ver[sizeof(h->fw_version)] = '\0';
        if (h->magic != APP_HDR_MAGIC) ver[0] = '\0';
        snprintf(buf, sizeof(buf), "+INFO valid=%u ver=%s size=%lu\r\n",
                 (unsigned)app_was_valid, ver,
                 (unsigned long)(h->magic == APP_HDR_MAGIC ? h->image_size : 0));
        out_str(buf);
    } else if (strncmp(line, "START ", 6) == 0) {
        char *end;
        uint32_t size = strtoul(line + 6, &end, 10);
        uint32_t crc  = strtoul(end, &end, 16);
        if (size < 1024UL || size > APP_MAX_SIZE || (size & 7U) != 0U) {
            out_str("-START size\r\n");
            return;
        }
        xfer_reset();
        xfer_size = size;
        xfer_crc  = crc;
        out_str("+START\r\n");
    } else if (strcmp(line, "ERASE") == 0) {
        if (xfer_size == 0U) {
            out_str("-ERASE nostart\r\n");
            return;
        }
        /* Mark the upload dirty BEFORE touching the app region — from here
         * until VERIFY clears the marker, every boot runs the full CRC. */
        if (bl_meta_mark_dirty() != 0) {
            out_str("-ERASE meta\r\n");
            return;
        }
        if (erase_app_region() != 0) {
            out_str("-ERASE flash\r\n");
            return;
        }
        xfer_erased = 1U;
        xfer_received = 0U;
        xfer_next_seq = 0U;
        out_str("+ERASE\r\n");
    } else if (strncmp(line, "DATA ", 5) == 0) {
        handle_data(line + 5);
    } else if (strcmp(line, "VERIFY") == 0) {
        handle_verify();
    } else if (strcmp(line, "BOOT") == 0) {
        /* Only boot a just-uploaded image after VERIFY passed. With no
         * transfer in progress, BOOT is a plain exit request (the next boot
         * re-validates the app anyway). */
        if (xfer_size != 0U && !xfer_verified) {
            out_str("-BOOT unverified\r\n");
            return;
        }
        out_str("+BOOT\r\n");
        bl_reset();
    } else if (strcmp(line, "ABORT") == 0) {
        xfer_reset();
        out_str("+ABORT\r\n");
    } else if (line[0] != '\0') {
        out_str("-ERR unknown\r\n");
    }
}

static void bootloader_mode(uint8_t app_was_valid) __attribute__((noreturn));
static void bootloader_mode(uint8_t app_was_valid)
{
    HAL_Init();
    bl_clock_config();
    MX_USB_Device_Init();
    iwdg_start();

    char     line[BL_LINE_MAX];
    uint32_t line_len = 0;
    uint32_t last_activity = HAL_GetTick();

    for (;;) {
        IWDG_KICK();

        int c = CDC_RxGetByte();
        if (c >= 0) {
            last_activity = HAL_GetTick();
            if (c == '\n' || c == '\r') {
                if (line_len > 0U) {
                    line[line_len] = '\0';
                    line_len = 0U;
                    handle_line(line, app_was_valid);
                }
            } else if (line_len < BL_LINE_MAX - 1U) {
                line[line_len++] = (uint8_t)c;
            } else {
                line_len = 0U;             /* oversized — discard the line   */
            }
        }

        /* If the app is valid and nobody is talking to us, go back to it —
         * the user probably aborted the update. Never auto-exit mid-
         * transfer or when there is no valid app to return to. */
        if (app_was_valid && xfer_size == 0U &&
            (uint32_t)(HAL_GetTick() - last_activity) >= BL_IDLE_EXIT_MS) {
            bl_reset();
        }
    }
}

/* ---------------------------------------------------------------------------
 * Entry
 * ------------------------------------------------------------------------ */
int main(void)
{
    /* Read + always clear the request mailbox (double-word match). */
    volatile uint32_t *req = (volatile uint32_t *)BL_REQ_ADDR;
    uint8_t requested = (req[0] == BL_REQ_MAGIC) && (req[1] == ~BL_REQ_MAGIC);
    req[0] = 0U;
    req[1] = 0U;

    /* A requested entry goes straight to bootloader mode WITHOUT touching
     * app flash: after a torn write, reading the app region raises the ECC
     * NMI (which is what armed the mailbox) — validating here would fault
     * again before USB ever came up. app_was_valid=0 only disables the
     * idle auto-exit, which is the safe direction; a power-cycle clears
     * the mailbox and boots a valid app normally. */
    if (requested) {
        bootloader_mode(0U);
    }

    /* Normal boot must be FAST (~1 ms): the app's PD stack has to answer
     * the source quickly after a cold attach or impatient chargers cycle
     * VBUS (observed on the bench: +35 ms of boot CRC caused a hard
     * attach/brown-out loop). Full CRC runs only when the upload-dirty
     * marker says the last update never completed. */
    uint8_t app_ok = app_quick_valid();

    if (app_ok && bl_meta_dirty()) {
        app_ok = app_crc_valid() ? 1U : 0U;
        if (app_ok) {
            /* Image is complete despite the dirty marker (power loss after
             * VERIFY's CRC but before its marker-clear, or an SWD reflash
             * over a torn update) — self-heal so the next boot is fast. */
            (void)bl_meta_erase();
        }
    }

    if (app_ok) {
        jump_to_app();
    }

    bootloader_mode(app_ok);
}

/* Referenced by usbd_conf.c — must never hang (IWDG may not be running yet
 * during USB init, so reset directly). */
void Error_Handler(void)
{
    NVIC_SystemReset();
}

/* Referenced by usbd_conf.c's USB resume/LPM callbacks: restore clocks after
 * a USB suspend. SYSCLK stays on HSI16 throughout; only HSI48 needs
 * re-enabling. */
void SystemClock_Config(void)
{
    bl_clock_config();
}
