/**
 * @file    settings.c
 * @brief   Flash-backed settings storage for AxxPD
 *
 * Stores settings in the last flash page of STM32G491CCT6 (256KB single-bank):
 *   Bank 1, page 127, address 0x0803F800
 *
 * Why the last page?  Firmware occupies bank 1 (and grows into bank 2 from
 * the low end), so the very last page is the least likely to be overwritten
 * by a future firmware update.  Using a fixed, well-known address also
 * avoids the need for a wear-levelling scheme.
 *
 * Write safety with USB PD EPR:
 *   Flash erase stalls the CPU for ~50-100 ms with interrupts blocked.
 *   Under EPR (>20 V), the PD stack must send KeepAlive every 500 ms or
 *   the source issues a Hard Reset (drops VBUS).  To avoid that, saves
 *   are deferred while EPR is active and only committed when the PD bus
 *   is idle — or after a 10 s forced timeout as a last resort.
 *
 * Uses hardware CRC-32 peripheral (direct register access, no HAL CRC module).
 */

#include "settings.h"
#include "stm32g4xx_hal.h"
#include <string.h>
#include <stddef.h>

/* ---------- Flash geometry ---------- */
/* STM32G491 single-bank: bank 1 = 0x0800_0000..0x0803_FFFF (256 KB, pages 0-127)
 * Each page = 2 KB.  We use the absolute last page so firmware growth from
 * the start of flash never collides with settings storage.                   */
#define SETTINGS_FLASH_ADDR   0x0803F800U   /* bank 1, page 127 start addr   */
#define SETTINGS_FLASH_PAGE   127U          /* page index within bank 1      */
#define SETTINGS_FLASH_BANK   FLASH_BANK_1

/* CRC is stored at the next 8-byte-aligned offset after Settings_t.
 * Alignment to 8 bytes is required because STM32G4 flash programming
 * operates in doubleword (64-bit) units — partial writes are not allowed. */
#define CRC_OFFSET  (((sizeof(Settings_t)) + 7U) & ~7U)

/* Total bytes to program: settings data + 4-byte CRC, rounded up to the
 * next doubleword boundary so the final HAL_FLASH_Program() is complete. */
#define TOTAL_WRITE_SIZE  (CRC_OFFSET + 8U)

/* ---------- RAM mirror ---------- */
static Settings_t settings;                   /* working copy; flash is only read at boot */
static volatile uint8_t settings_save_pending = 0;  /* 1 = save was deferred due to EPR */
static uint32_t settings_save_deferred_tick = 0;     /* tick when deferral started       */

/* Force-save timeout: if EPR stays active continuously for this long, we
 * save anyway.  The resulting ~100 ms CPU stall will miss one KeepAlive,
 * causing a Hard Reset + re-negotiation, but that is preferable to losing
 * settings if the device runs on EPR for hours and then loses power.      */
#define SETTINGS_DEFERRED_TIMEOUT_MS  10000U

/* Backing buffer for flash writes (settings + CRC, doubleword-aligned).
 * Must be 8-byte aligned for HAL_FLASH_Program(DOUBLEWORD, ...).         */
static uint8_t write_buf[TOTAL_WRITE_SIZE] __attribute__((aligned(8)));

/* ------------------------------------------------------------------ */
/*  Hardware CRC-32 helpers (no HAL, direct register access)          */
/* ------------------------------------------------------------------ */
static void CRC_Init(void)
{
    /* Enable CRC peripheral clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
    /* Small delay for clock to stabilize */
    __DSB();
}

/**
 * Compute CRC-32 over an arbitrary byte buffer using the hardware CRC unit.
 *
 * Interrupts are disabled for the entire feed sequence because the CRC
 * peripheral holds sequential state in a single data register.  If an ISR
 * (especially SysTick, which ticks the PD stack every 2 ms) were to use
 * the peripheral concurrently, it would silently corrupt the running CRC.
 * The critical section is very short (microseconds for our ~100-byte struct).
 */
static uint32_t CRC_Calculate(const void *data, uint32_t len_bytes)
{
    const uint32_t *p32 = (const uint32_t *)data;
    uint32_t words = len_bytes / 4U;
    uint32_t tail  = len_bytes % 4U;

    /* Save and disable interrupts to protect the shared CRC peripheral */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* Reset CRC unit — uses default poly 0x04C11DB7, init 0xFFFFFFFF */
    CRC->CR = CRC_CR_RESET;

    /* Feed full 32-bit words via the 32-bit data register */
    for (uint32_t i = 0; i < words; i++) {
        CRC->DR = p32[i];
    }

    /* Feed remaining 1-3 bytes via 8-bit aliased access to CRC->DR.
     * The cast chain (__IO uint8_t *) lets the hardware accept byte-wide writes. */
    if (tail) {
        const uint8_t *p8 = (const uint8_t *)&p32[words];
        for (uint32_t i = 0; i < tail; i++) {
            *(__IO uint8_t *)(__IO void *)&CRC->DR = p8[i];
        }
    }

    uint32_t result = CRC->DR;
    if (!primask) __enable_irq();   /* restore only if IRQs were enabled before */

    return result;
}

/* ------------------------------------------------------------------ */
void Settings_LoadDefaults(void)
{
    memset(&settings, 0, sizeof(settings));

    settings.magic = SETTINGS_MAGIC;

    /* Mode defaults */
    settings.remember_boot    = 0;  /* NO */
    settings.power_on_boot    = 0;  /* NO — do not auto-arm output at boot */
    settings.start_locked     = 0;  /* NO — boot unlocked */
    settings.boot_selector    = 1;  /* YES */
    settings.serial_terminal  = 1;  /* YES */
    settings.splash_screen    = 1;  /* YES */

    /* Sound defaults */
    settings.buzzer_enabled   = 1;  /* YES */
    settings.startup_beep     = 1;  /* YES */

    /* Display defaults */
    settings.temp_fahrenheit  = 0;  /* Celsius */

    /* Runtime defaults */
    settings.last_voltage_mv  = 5000;
    settings.last_current_ma  = 3000;
    settings.last_used_pdo    = 1;
    settings.preset_count     = 1;

    /* Protection threshold defaults */
    settings.ocp_ma       = 5500;   /* 5.5 A */
    settings.ovp_mv       = 55000;  /* 55 V  */
    settings.opp_100mw    = 0;      /* disabled — charger enforces power limits via PD contract */
    settings.timer_seconds = 0;     /* disabled */
    settings.ah_limit_mah  = 0;     /* disabled */
    settings.wh_limit_mwh  = 0;     /* disabled */
    settings.charge_complete_ma  = 0;   /* disabled by default */
    settings.charge_complete_sec = 30;  /* 30 seconds hold time */
    settings.ocp_retry     = 2;     /* retry 3x (handles hot-plug inrush) */

    /* Calibration offsets — zero by default (no adjustment) */
    settings.cal_v_offset_uv = 0;
    settings.cal_i_offset_ua = 0;

    /* Display — graph */
    settings.graph_window = 1;  /* 10s (default) */

    /* Default preset: 5V / 3A */
    settings.presets[0].voltage_mv = 5000;
    settings.presets[0].current_ma = 3000;
    memcpy(settings.presets[0].name, "5V/3A\0\0", 8);
}

/* ------------------------------------------------------------------ */
void Settings_Init(void)
{
    CRC_Init();

    /* Copy settings from flash into the RAM mirror */
    const uint8_t *flash = (const uint8_t *)SETTINGS_FLASH_ADDR;
    memcpy(&settings, flash, sizeof(Settings_t));

    /* Read the CRC that was written alongside the settings */
    uint32_t stored_crc;
    memcpy(&stored_crc, flash + CRC_OFFSET, sizeof(uint32_t));

    /* Recompute and compare — catches both data corruption and the
     * first-boot case where flash is erased (all 0xFF). */
    uint32_t calc_crc = CRC_Calculate(&settings, sizeof(Settings_t));

    if (calc_crc != stored_crc) {
        Settings_LoadDefaults();
    }

    /* Validate magic byte — catches flash that passes CRC by coincidence
     * (e.g. a struct layout change between firmware versions). */
    if (settings.magic != SETTINGS_MAGIC) {
        Settings_LoadDefaults();
    }

    /* Bounds-check fields that index into fixed-size arrays */
    if (settings.preset_count > PRESET_SLOTS) settings.preset_count = 0;
}

/* ------------------------------------------------------------------ */
#include "axxpd_main.h"

/**
 * Perform the actual flash erase + program cycle.
 *
 * This stalls the CPU for ~50-100 ms (erase time) during which no
 * interrupts are serviced by the flash controller.  Callers must
 * ensure that the PD bus can tolerate the stall (see Settings_Save).
 */
static void settings_do_flash_write(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0;

    /* Build the write image in RAM: settings struct + CRC.
     * Pre-fill with 0xFF so unused padding matches erased flash. */
    memset(write_buf, 0xFF, sizeof(write_buf));
    memcpy(write_buf, &settings, sizeof(Settings_t));

    uint32_t crc = CRC_Calculate(&settings, sizeof(Settings_t));
    memcpy(write_buf + CRC_OFFSET, &crc, sizeof(uint32_t));

    HAL_FLASH_Unlock();

    /* Erase the settings page.  The STM32G4 flash controller can
     * return transient errors when DMA (PD PHY) or SPI (display)
     * contend on the bus, so we retry once before giving up.      */
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = SETTINGS_FLASH_BANK;
    erase.Page      = SETTINGS_FLASH_PAGE;
    erase.NbPages   = 1;
    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        page_error = 0;
        if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
            HAL_FLASH_Lock();
            return;   /* erase failed twice — abort, settings stay in RAM only */
        }
    }

    /* Program the settings data (excluding the CRC doubleword) as a
     * sequence of 64-bit doublewords.  If any doubleword fails, skip
     * the CRC write so that the next boot detects the partial write
     * via CRC mismatch and loads safe defaults instead.              */
    const uint64_t *src = (const uint64_t *)write_buf;
    uint32_t addr       = SETTINGS_FLASH_ADDR;
    uint32_t data_dwords = CRC_OFFSET / 8U;      /* data only, no CRC */

    uint8_t write_ok = 1;
    for (uint32_t i = 0; i < data_dwords; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, src[i]) != HAL_OK) {
            write_ok = 0;
            break;   /* partial write — CRC intentionally not written */
        }
        addr += 8;
    }

    /* Only write the CRC doubleword if all data was successfully programmed.
     * On partial failure the missing CRC causes Settings_Init to fall back
     * to defaults on the next boot. */
    if (write_ok) {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                          SETTINGS_FLASH_ADDR + CRC_OFFSET,
                          src[CRC_OFFSET / 8U]);
    }

    HAL_FLASH_Lock();
    settings_save_pending = 0;
}

/**
 * Request a settings save.  If EPR is active the write is deferred to
 * avoid a ~100 ms CPU stall that would miss the 500 ms KeepAlive
 * deadline and trigger a Hard Reset (VBUS drop + re-negotiation).
 *
 * Deferred saves are committed later by Settings_ProcessDeferred().
 */
void Settings_Save(void)
{
    if (axxpd_is_epr_active()) {
        if (!settings_save_pending) {
            settings_save_pending = 1;
            settings_save_deferred_tick = HAL_GetTick();
        }
        return;   /* will be picked up by Settings_ProcessDeferred() */
    }
    settings_do_flash_write();
}

/**
 * Force an immediate flash write, bypassing the EPR deferral. The ~50-100 ms
 * erase stall will miss an EPR KeepAlive — only call this when that is
 * acceptable, e.g. right before a reboot (MI_SAVE_REBOOT), where deferring
 * would lose the change because the reset fires before the deferred write.
 */
void Settings_SaveImmediate(void)
{
    settings_do_flash_write();
}

/**
 * Called from the main super-loop.  Commits any deferred save when it is
 * safe to do so (EPR no longer active), or forces the write after 10 s
 * to avoid losing settings on an unexpected power loss.
 */
void Settings_ProcessDeferred(void)
{
    if (!settings_save_pending) return;

    /* Best case: EPR dropped since the save was requested — write now */
    if (!axxpd_is_epr_active()) {
        settings_do_flash_write();
        return;
    }

    /* Worst case: still in EPR.  After 10 s, force the write.  The flash
     * erase stall will miss one KeepAlive, causing a Hard Reset + automatic
     * re-negotiation, but settings are preserved across power loss. */
    if ((HAL_GetTick() - settings_save_deferred_tick) >= SETTINGS_DEFERRED_TIMEOUT_MS) {
        settings_do_flash_write();
    }
}

uint8_t Settings_IsSavePending(void)
{
    return settings_save_pending;
}

/* ------------------------------------------------------------------ */
/*  Bool settings accessors                                           */
/* ------------------------------------------------------------------ */
uint8_t Settings_GetRememberBoot(void)    { return settings.remember_boot;   }
uint8_t Settings_GetPowerOnBoot(void)     { return settings.power_on_boot;   }
uint8_t Settings_GetStartLocked(void)     { return settings.start_locked;    }
uint8_t Settings_GetBootSelector(void)    { return settings.boot_selector;   }
uint8_t Settings_GetSerialTerminal(void)  { return settings.serial_terminal; }
uint8_t Settings_GetSplashScreen(void)    { return settings.splash_screen;   }
uint8_t Settings_GetBuzzerEnabled(void)   { return settings.buzzer_enabled;  }
uint8_t Settings_GetStartupBeep(void)     { return settings.startup_beep;    }
uint8_t Settings_GetTempFahrenheit(void)  { return settings.temp_fahrenheit; }
void    Settings_SetTempFahrenheit(uint8_t val) { settings.temp_fahrenheit = val ? 1U : 0U; }
uint8_t Settings_GetGraphWindow(void)    { return settings.graph_window; }
void    Settings_SetGraphWindow(uint8_t val) { if (val > 3U) val = 3U; settings.graph_window = val; }

/** Set a boolean field by its byte offset (fi) within Settings_t.
 *  Used by the menu system to toggle settings generically without
 *  needing a separate setter for each field. */
void Settings_SetBool(uint8_t fi, uint8_t val)
{
    if (fi >= sizeof(Settings_t)) return;
    uint8_t *base = (uint8_t *)&settings;
    base[fi] = val ? 1U : 0U;
}

/** Read a boolean field by its byte offset (fi) within Settings_t. */
uint8_t Settings_GetBool(uint8_t fi)
{
    if (fi >= sizeof(Settings_t)) return 0;
    const uint8_t *base = (const uint8_t *)&settings;
    return base[fi];
}

/* ------------------------------------------------------------------ */
/*  Runtime state                                                     */
/* ------------------------------------------------------------------ */
uint32_t Settings_GetLastVoltage(void)    { return settings.last_voltage_mv; }
uint32_t Settings_GetLastCurrent(void)    { return settings.last_current_ma; }

/** Persist last-used voltage/current so "remember boot" can restore them.
 *  Skips the flash write if values are unchanged (avoids unnecessary wear). */
void Settings_SaveLastSettings(uint32_t mv, uint32_t ma)
{
    if (settings.last_voltage_mv == mv && settings.last_current_ma == ma) return;
    settings.last_voltage_mv = mv;
    settings.last_current_ma = ma;
    Settings_Save();
}

uint8_t Settings_GetLastUsedPdo(void)     { return settings.last_used_pdo; }

void Settings_SetLastUsedPdo(uint8_t pdo)
{
    settings.last_used_pdo = pdo;
    Settings_Save();
}

/* ------------------------------------------------------------------ */
/*  Preset accessors                                                  */
/* ------------------------------------------------------------------ */
Preset_t* Settings_GetPreset(uint8_t index)
{
    if (index >= settings.preset_count) return &settings.presets[0];
    return &settings.presets[index];
}

uint8_t Settings_PresetCount(void)
{
    return settings.preset_count;
}

uint8_t Settings_PresetIsEmpty(uint8_t index)
{
    if (index >= PRESET_SLOTS) return 1;
    if (index >= settings.preset_count) return 1;
    return (settings.presets[index].voltage_mv == 0U) ? 1U : 0U;
}

void Settings_PresetSet(uint8_t index, uint32_t mv, uint32_t ma, const char *name)
{
    if (index >= PRESET_SLOTS) return;
    settings.presets[index].voltage_mv = mv;
    settings.presets[index].current_ma = ma;
    strncpy(settings.presets[index].name, name, 7);
    settings.presets[index].name[7] = '\0';
    if (index >= settings.preset_count) {
        settings.preset_count = index + 1;
    }
    Settings_Save();
}

void Settings_PresetDelete(uint8_t index)
{
    if (index >= settings.preset_count || index >= PRESET_SLOTS) return;
    /* Shift remaining presets down */
    for (uint8_t i = index; i + 1 < settings.preset_count; i++) {
        settings.presets[i] = settings.presets[i + 1];
    }
    settings.preset_count--;
    memset(&settings.presets[settings.preset_count], 0, sizeof(Preset_t));
    Settings_Save();
}

/* ------------------------------------------------------------------ */
/*  Numeric protection threshold accessors                            */
/* ------------------------------------------------------------------ */
uint16_t Settings_GetOcpMa(void)       { return settings.ocp_ma;        }
uint16_t Settings_GetOvpMv(void)       { return settings.ovp_mv;        }
uint16_t Settings_GetOpp100mw(void)    { return settings.opp_100mw;     }
uint16_t Settings_GetTimerSeconds(void){ return settings.timer_seconds;  }
uint32_t Settings_GetAhLimitMah(void)  { return settings.ah_limit_mah;  }
uint32_t Settings_GetWhLimitMwh(void)  { return settings.wh_limit_mwh;  }
uint16_t Settings_GetChargeCompleteMa(void)  { return settings.charge_complete_ma; }
uint16_t Settings_GetChargeCompleteSec(void) { return settings.charge_complete_sec; }
uint8_t  Settings_GetOcpRetry(void)    { return settings.ocp_retry;     }
int16_t  Settings_GetCalVOffsetUv(void){ return settings.cal_v_offset_uv; }
int16_t  Settings_GetCalIOffsetUa(void){ return settings.cal_i_offset_ua; }

void Settings_SetOcpRetry(uint8_t val)
{
    if (val > 2U) val = 2U;
    settings.ocp_retry = val;
    Settings_Save();
}

/**
 * Get the current value of a numeric menu item as a uint32.
 * Returns 0 for unknown items.
 */
uint32_t Settings_GetNumericValue(uint16_t mi)
{
    switch (mi) {
        case MI_OCP_LIMIT: return settings.ocp_ma;
        case MI_OVP_LIMIT: return settings.ovp_mv;
        case MI_OPP_LIMIT: return settings.opp_100mw;
        case MI_TIMER:     return settings.timer_seconds;
        case MI_AH_LIMIT:  return settings.ah_limit_mah;
        case MI_WH_LIMIT:  return settings.wh_limit_mwh;
        case MI_CHARGE_COMPLETE_MA: return settings.charge_complete_ma;
        case MI_CHARGE_COMPLETE_SEC: return settings.charge_complete_sec;
        case MI_OCP_RETRY:        return settings.ocp_retry;
        case MI_CAL_V:  return (uint32_t)(int32_t)settings.cal_v_offset_uv;
        case MI_CAL_I:  return (uint32_t)(int32_t)settings.cal_i_offset_ua;
        case MI_GRAPH_WINDOW:     return settings.graph_window;
        default:                  return 0;
    }
}

/**
 * Adjust a numeric setting by delta (+1 or -1 step).
 * Each item has its own step size and min/max clamp.
 */
void Settings_SetNumeric(uint16_t mi, int32_t delta)
{
    switch (mi) {
        case MI_OCP_LIMIT: {
            /* Step 100 mA, range 100..6000 mA */
            int32_t v = (int32_t)settings.ocp_ma + delta * 100;
            if (v < 100)  v = 100;
            if (v > 6000) v = 6000;
            settings.ocp_ma = (uint16_t)v;
            break;
        }
        case MI_OVP_LIMIT: {
            /* Step 1000 mV, range 5000..55000 mV */
            int32_t v = (int32_t)settings.ovp_mv + delta * 1000;
            if (v < 5000)  v = 5000;
            if (v > 55000) v = 55000;
            settings.ovp_mv = (uint16_t)v;
            break;
        }
        case MI_OPP_LIMIT: {
            /* Step 10 (=1W), range 0..2400 (0=disabled, 1W..240W) */
            int32_t v = (int32_t)settings.opp_100mw + delta * 10;
            if (v < 0)    v = 0;
            if (v > 2400) v = 2400;
            settings.opp_100mw = (uint16_t)v;
            break;
        }
        case MI_TIMER: {
            /* Step 30 s, range 0..3600 s, 0=disabled */
            int32_t v = (int32_t)settings.timer_seconds + delta * 30;
            if (v < 0)    v = 0;
            if (v > 3600) v = 3600;
            settings.timer_seconds = (uint16_t)v;
            break;
        }
        case MI_AH_LIMIT: {
            /* Step 100 mAh, range 0..100000 mAh (100Ah), 0=disabled */
            int32_t v = (int32_t)settings.ah_limit_mah + delta * 100;
            if (v < 0)       v = 0;
            if (v > 100000)  v = 100000;
            settings.ah_limit_mah = (uint32_t)v;
            break;
        }
        case MI_WH_LIMIT: {
            /* Step 1000 mWh (=1Wh), range 0..500000 mWh (500Wh), 0=disabled */
            int32_t v = (int32_t)settings.wh_limit_mwh + delta * 1000;
            if (v < 0)       v = 0;
            if (v > 500000)  v = 500000;
            settings.wh_limit_mwh = (uint32_t)v;
            break;
        }
        case MI_CHARGE_COMPLETE_MA: {
            int32_t v = (int32_t)settings.charge_complete_ma + delta * 50;
            if (v < 0) v = 0;
            if (v > 5000) v = 5000;
            settings.charge_complete_ma = (uint16_t)v;
            break;
        }
        case MI_CHARGE_COMPLETE_SEC: {
            int32_t v = (int32_t)settings.charge_complete_sec + delta * 5;
            if (v < 5) v = 5;
            if (v > 300) v = 300;
            settings.charge_complete_sec = (uint16_t)v;
            break;
        }
        case MI_OCP_RETRY: {
            /* Cycle through 3 values: 0=latch, 1=retry-once, 2=retry-3x */
            int32_t v = (int32_t)settings.ocp_retry + delta;
            if (v < 0) v = 2;
            if (v > 2) v = 0;
            settings.ocp_retry = (uint8_t)v;
            break;
        }
        case MI_GRAPH_WINDOW: {
            /* Cycle through 4 values: 0=5s, 1=10s, 2=30s, 3=60s */
            int32_t v = (int32_t)settings.graph_window + delta;
            if (v < 0) v = 3;
            if (v > 3) v = 0;
            settings.graph_window = (uint8_t)v;
            break;
        }
        case MI_CAL_V: {
            /* Step 100 uV (0.1mV), range ±10000 uV (±10mV) */
            int32_t v = (int32_t)settings.cal_v_offset_uv + delta * 100;
            if (v < -10000) v = -10000;
            if (v >  10000) v =  10000;
            settings.cal_v_offset_uv = (int16_t)v;
            break;
        }
        case MI_CAL_I: {
            /* Step 100 uA (0.1mA), range ±10000 uA (±10mA) */
            int32_t v = (int32_t)settings.cal_i_offset_ua + delta * 100;
            if (v < -10000) v = -10000;
            if (v >  10000) v =  10000;
            settings.cal_i_offset_ua = (int16_t)v;
            break;
        }
        default:
            return;
    }
    Settings_Save();
}
