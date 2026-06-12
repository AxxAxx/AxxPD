// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

/**
 * @file    ina228.c
 * @brief   INA228 85V, 20-bit current/voltage/power/energy monitor driver
 *
 * Connected via I2C3 (400kHz), A0=GND, A1=GND -> addr 0x40.
 * Shunt: 6.8mOhm.  Bus voltage range: 0–85V.  ADCRANGE=0 (163.84mV).
 *
 * Register protocol: 8-bit register address, big-endian data (MSB first).
 */

#include "ina228.h"
#include "settings.h"
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Timeout for all I2C operations (ms)
 * ---------------------------------------------------------------------- */
#define INA228_I2C_TIMEOUT  50U

/* Idle callback — called between the five blocking I2C reads in ReadAll().
 * Each HAL_I2C_Mem_Read blocks for ~1-2ms, so the full burst takes ~9ms.
 * Without this callback the PD stack (which relies on sub-ms polling) would
 * stall and miss protocol deadlines.  Set to axxpd_run() at startup. */
static void (*ina228_idle_cb)(void) = NULL;

void INA228_SetIdleCallback(void (*cb)(void)) { ina228_idle_cb = cb; }

/* -------------------------------------------------------------------------
 * Manufacturer / device IDs
 * ---------------------------------------------------------------------- */
#define INA228_MFG_ID_TI    0x5449U   /* "TI" in ASCII */

/* -------------------------------------------------------------------------
 * CONFIG register bits
 * ---------------------------------------------------------------------- */
#define INA228_CONFIG_RST   (1U << 15)  /* Software reset */
#define INA228_CONFIG_RSTACC (1U << 14) /* Reset energy/charge accumulators */

/* -------------------------------------------------------------------------
 * ADC_CONFIG register (0x01) — INA228 datasheet Table 7-18
 *   Bit layout: MODE[15:12] | VBUSCT[11:9] | VSHCT[8:6] | VTCT[5:3] | AVG[2:0]
 *
 *   0xF002 = 1111 0000 0000 0010
 *     MODE   [15:12] = 1111 = continuous bus+shunt+temp
 *     VBUSCT [11:9]  = 000  = 50 us   (fast bus voltage)
 *     VSHCT  [8:6]   = 000  = 50 us   (fast shunt for OCP)
 *     VTCT   [5:3]   = 000  = 50 us   (die temperature)
 *     AVG    [2:0]   = 010  = 4x averaging
 * ---------------------------------------------------------------------- */
#define INA228_ADC_CONFIG_DEFAULT  0xF002U

/* -------------------------------------------------------------------------
 * DIAG_ALRT bits
 * ---------------------------------------------------------------------- */
#define INA228_DIAG_SHUNT_OVF   (1U << 6)   /* SHNTOL — Shunt Over-Limit alert enable, bit 6 of DIAG_ALRT */
#define INA228_DIAG_ALATCH      (1U << 15)  /* Alert latch enable */

/* =========================================================================
 * Private helpers
 * ====================================================================== */

/**
 * Write a 16-bit value to an INA228 register (big-endian).
 */
static HAL_StatusTypeDef ina228_write16(INA228_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFFU);
    return HAL_I2C_Mem_Write(dev->hi2c, INA228_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, buf, 2U, INA228_I2C_TIMEOUT);
}

/**
 * Read a 16-bit value from an INA228 register (big-endian).
 */
static HAL_StatusTypeDef ina228_read16(INA228_t *dev, uint8_t reg, uint16_t *out)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(dev->hi2c, INA228_ADDR, reg,
                                              I2C_MEMADD_SIZE_8BIT, buf, 2U, INA228_I2C_TIMEOUT);
    if (ret == HAL_OK) {
        *out = ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
    }
    return ret;
}

/**
 * Read 3 bytes (24-bit) from a register — used for VBUS, CURRENT, POWER.
 * Returns raw 24-bit value with byte[0] as MSB.
 */
static HAL_StatusTypeDef ina228_read24(INA228_t *dev, uint8_t reg, uint32_t *out)
{
    uint8_t buf[3];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(dev->hi2c, INA228_ADDR, reg,
                                              I2C_MEMADD_SIZE_8BIT, buf, 3U, INA228_I2C_TIMEOUT);
    if (ret == HAL_OK) {
        *out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];
    }
    return ret;
}

/**
 * Read 5 bytes (40-bit) from a register — used for ENERGY and CHARGE.
 */
static HAL_StatusTypeDef ina228_read40(INA228_t *dev, uint8_t reg, uint64_t *out)
{
    uint8_t buf[5];
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(dev->hi2c, INA228_ADDR, reg,
                                              I2C_MEMADD_SIZE_8BIT, buf, 5U, INA228_I2C_TIMEOUT);
    if (ret == HAL_OK) {
        *out = ((uint64_t)buf[0] << 32)
             | ((uint64_t)buf[1] << 24)
             | ((uint64_t)buf[2] << 16)
             | ((uint64_t)buf[3] <<  8)
             |  (uint64_t)buf[4];
    }
    return ret;
}

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief  Initialise the INA228.
 *
 * Verifies manufacturer ID, issues a soft reset, configures the ADC for
 * continuous bus+shunt+temp measurement with 1052 us conversion time and
 * 4x averaging, then programs SHUNT_CAL based on the supplied parameters.
 *
 * @param  dev          Pointer to uninitialised INA228_t instance.
 * @param  hi2c         I2C peripheral handle (I2C3 -> hi2c3).
 * @param  shunt_ohms   Shunt resistor value in Ohms (e.g. 0.0068 for 6.8 mOhm).
 * @param  max_current_a Maximum expected current in Amperes.
 * @retval HAL_OK on success, HAL_ERROR on ID mismatch or I2C failure.
 */
HAL_StatusTypeDef INA228_Init(INA228_t *dev, I2C_HandleTypeDef *hi2c,
                               float shunt_ohms, float max_current_a)
{
    HAL_StatusTypeDef ret;
    uint16_t mfg_id;

    dev->hi2c       = hi2c;
    dev->shunt_ohms = shunt_ohms;

    /* current_lsb = max_current / 2^19 */
    dev->current_lsb = max_current_a / 524288.0f;  /* 2^19 = 524288 */

    /* Verify manufacturer ID */
    ret = ina228_read16(dev, INA228_REG_MFG_ID, &mfg_id);
    if (ret != HAL_OK) {
        return ret;
    }
    if (mfg_id != INA228_MFG_ID_TI) {
        return HAL_ERROR;
    }

    /* Software reset */
    ret = ina228_write16(dev, INA228_REG_CONFIG, INA228_CONFIG_RST);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Wait for reset to complete */
    HAL_Delay(2);

    /* ADCRANGE=0 (default after reset, 163.84mV full scale).
     * 6.8mOhm shunt at 5A = 34mV, well within range.
     * Resolution: LSB = 5uV -> 0.74mA per count. */

    /* Configure ADC: continuous bus+shunt+temp, all 50us conversion,
     * 4x averaging. Total cycle ~200us for fast OCP detection.
     * Small delays between writes: the STM32G4 I2C peripheral can
     * issue back-to-back transactions faster than the INA228 releases
     * the bus, causing subsequent operations to fail silently. */
    ret = ina228_write16(dev, INA228_REG_ADC_CONFIG, INA228_ADC_CONFIG_DEFAULT);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Calculate and write SHUNT_CAL
     * SHUNT_CAL = 13107.2e6 * current_lsb * shunt_ohms
     * No *4 factor needed with ADCRANGE=0 (default).
     * Result must fit in a 15-bit unsigned value (bits [14:0] of the register).
     */
    float shunt_cal_f = 13107.2e6f * dev->current_lsb * dev->shunt_ohms;
    uint16_t shunt_cal = (uint16_t)(shunt_cal_f + 0.5f);  /* round to nearest */

    ret = ina228_write16(dev, INA228_REG_SHUNT_CAL, shunt_cal);
    if (ret != HAL_OK) {
        return ret;
    }
    HAL_Delay(2);

    return HAL_OK;
}

/**
 * @brief  Read bus voltage only.
 * @param  dev       INA228 device handle.
 * @param  voltage_v Output voltage in Volts.
 * @retval HAL status.
 */
HAL_StatusTypeDef INA228_ReadVoltage(INA228_t *dev, float *voltage_v)
{
    uint32_t raw;
    HAL_StatusTypeDef ret = ina228_read24(dev, INA228_REG_VBUS, &raw);
    if (ret != HAL_OK) {
        return ret;
    }

    /* VBUS reg 0x05: [23:4] = 20-bit unsigned, [3:0] = reserved.
     * LSB = 195.3125 uV.  Shift right 4, mask to 20 bits. */
    uint32_t vbus_counts = (raw >> 4) & 0x000FFFFFU;
    *voltage_v = (float)vbus_counts * 195.3125e-6f;

    return HAL_OK;
}

/**
 * @brief  Read current only.
 * @param  dev       INA228 device handle.
 * @param  current_a Output current in Amperes.
 * @retval HAL status.
 */
HAL_StatusTypeDef INA228_ReadCurrent(INA228_t *dev, float *current_a)
{
    uint32_t raw;
    HAL_StatusTypeDef ret = ina228_read24(dev, INA228_REG_CURRENT, &raw);
    if (ret != HAL_OK) {
        return ret;
    }

    /* CURRENT reg 0x07: [23:4] = 20-bit two's complement, [3:0] = reserved.
     * LSB = current_lsb (configured by SHUNT_CAL).
     * Bit 19 is the sign bit; sign-extend to 32 bits for negative values. */
    int32_t current_counts = (int32_t)(raw >> 4) & 0x000FFFFF;
    if (current_counts & 0x00080000) {
        current_counts |= (int32_t)0xFFF00000;
    }

    *current_a = (float)current_counts * dev->current_lsb;

    return HAL_OK;
}

/**
 * @brief  Read all measurements in a single burst.
 *
 * Reads voltage, current, power, energy, charge, and die temperature.
 *
 * @param  dev     INA228 device handle.
 * @param  reading Output structure populated with all measurements.
 * @retval HAL status (first failure aborts and returns that status).
 */
/* -------------------------------------------------------------------------
 * Moving-average filter for V and I (8-sample window = 0.8s at 100ms poll)
 *
 * The INA228's 4x hardware averaging already reduces ADC noise, but the
 * remaining jitter (a few LSBs) is still visible as flicker on a 3-digit
 * display.  This software filter smooths the output to produce stable,
 * readable values for the UI without adding significant latency.
 *
 * Implementation: circular buffer with running sum.  The sum is clamped
 * to >= 0 after each update to prevent float rounding drift from producing
 * tiny negative values (which would display as "-0.00").
 * ---------------------------------------------------------------------- */
#define MAVG_N  8U

typedef struct {
    float buf[MAVG_N];   /* circular sample buffer                      */
    float sum;           /* running sum of all samples in buf            */
    uint8_t idx;         /* write index (next slot to overwrite)         */
    uint8_t filled;      /* number of valid samples (ramps up to MAVG_N)*/
} mavg_t;

static mavg_t filt_v, filt_i, filt_p;

static float mavg_feed(mavg_t *f, float sample)
{
    f->sum -= f->buf[f->idx];   /* subtract oldest sample being evicted */
    f->buf[f->idx] = sample;
    f->sum += sample;            /* add new sample                      */
    f->idx = (f->idx + 1U) % MAVG_N;
    if (f->filled < MAVG_N) f->filled++;
    /* Every full rotation, recalculate sum from scratch to eliminate
     * accumulated float subtraction drift (~50-100mV over 24 hours). */
    if (f->idx == 0U && f->filled == MAVG_N) {
        float s = 0.0f;
        for (uint8_t i = 0; i < MAVG_N; i++) s += f->buf[i];
        f->sum = s;
    }
    if (f->sum < 0.0f) f->sum = 0.0f;
    return f->sum / (float)f->filled;  /* mean of valid samples */
}

HAL_StatusTypeDef INA228_ReadAll(INA228_t *dev, INA228_Reading_t *reading)
{
    HAL_StatusTypeDef ret;
    uint32_t raw24;
    uint64_t raw40;

    /* Clear any sticky HAL error from prior failed transactions so the
     * next Master_Transmit/Receive doesn't refuse to run. */
    if (dev->hi2c->ErrorCode != HAL_I2C_ERROR_NONE) {
        dev->hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
    }

    /* --- Bus Voltage (reg 0x05, 24-bit read) --------------------------------
     * Register layout: [23:4] = 20-bit unsigned VBUS, [3:0] = reserved.
     * LSB = 195.3125 uV.  Full scale = 2^20 * 195.3125 uV = 204.8 V.
     * Negative clamp: VBUS is unsigned so v < 0 shouldn't occur, but
     * clamping here is defensive and costs nothing. */
    ret = ina228_read24(dev, INA228_REG_VBUS, &raw24);
    if (ret != HAL_OK) { return ret; }
    uint32_t vbus_counts = (raw24 >> 4) & 0x000FFFFFU;
    float v = (float)vbus_counts * 195.3125e-6f;
    /* Apply voltage calibration offset (in microvolts) before moving average */
    float cal_v = (float)Settings_GetCalVOffsetUv() / 1e6f;
    v += cal_v;
    if (v < 0.0f) v = 0.0f;
    reading->voltage_v = mavg_feed(&filt_v, v);
    if (ina228_idle_cb) ina228_idle_cb();  /* let PD stack poll between reads */

    /* --- Current (reg 0x07, 24-bit read) ----------------------------------
     * Register layout: [23:4] = 20-bit two's complement, [3:0] = reserved.
     * LSB = current_lsb (set during calibration).
     * Bit 19 is the sign bit; sign-extend to 32 bits for correct negatives.
     *
     * Negative clamp (single source of truth): the shunt can read small
     * negative values due to PCB layout or ADC offset even with zero real
     * current.  Clamping to 0 HERE, before the moving-average filter, means
     * no downstream code (power calc, display, charge accumulator) ever sees
     * a negative current.  This is the sole place negatives are suppressed. */
    ret = ina228_read24(dev, INA228_REG_CURRENT, &raw24);
    if (ret != HAL_OK) { return ret; }
    int32_t current_counts = (int32_t)((raw24 >> 4) & 0x000FFFFFU);
    if (current_counts & 0x00080000) {       /* sign bit set? */
        current_counts |= (int32_t)0xFFF00000;  /* extend sign to bits [31:20] */
    }
    float i = (float)current_counts * dev->current_lsb;
    /* Apply current calibration offset (in microamps) before moving average */
    float cal_i = (float)Settings_GetCalIOffsetUa() / 1e6f;
    i += cal_i;
    if (i < 0.0f) i = 0.0f;                 /* clamp: sole negative gate */
    reading->current_a = mavg_feed(&filt_i, i);
    if (ina228_idle_cb) ina228_idle_cb();

    /* --- Power (computed from filtered V * I, not hardware register) --------
     * The INA228 has a hardware POWER register (reg 0x08), but we compute
     * power from the already-filtered and clamped V and I instead.  Reasons:
     *   1. Consistency: if filtered I = 0.00 A, power = 0.00 W.  The HW
     *      register has its own noise floor and would report non-zero power
     *      even when displayed current is zero.
     *   2. One fewer I2C read saves ~2ms per poll cycle.
     *   3. The HW register uses a different LSB scale that would need its
     *      own conversion and wouldn't benefit from our moving-average. */
    reading->power_w = reading->voltage_v * reading->current_a;

    /* --- Energy (reg 0x09, 40-bit read) ------------------------------------
     * Register: 40-bit unsigned accumulator.
     * LSB = 16 * 3.2 * current_lsb (Joules).  Divide by 3600 -> Wh.
     * Double intermediates prevent float32 truncation when the accumulator
     * reaches large values (energy grows without bound until reset). */
    ret = ina228_read40(dev, INA228_REG_ENERGY, &raw40);
    if (ret != HAL_OK) { return ret; }
    reading->energy_wh = (float)((double)raw40 * 16.0 * 3.2 * (double)dev->current_lsb / 3600.0);
    if (ina228_idle_cb) ina228_idle_cb();

    /* --- Charge (reg 0x0A, 40-bit read) ------------------------------------
     * Register: 40-bit two's complement accumulator.
     * LSB = current_lsb (As).  Divide by 3600 -> Ah.
     * Bit 39 is the sign bit; sign-extend to 64 bits.
     * Negative clamp: small ADC offset can cause tiny negative accumulation
     * even at zero current; clamp to 0 for display consistency. */
    ret = ina228_read40(dev, INA228_REG_CHARGE, &raw40);
    if (ret != HAL_OK) { return ret; }
    int64_t charge_counts = (int64_t)raw40;
    if (charge_counts & ((int64_t)1 << 39)) {
        charge_counts |= (int64_t)0xFFFFFF0000000000LL;  /* sign-extend [63:40] */
    }
    float charge = (float)((double)charge_counts * (double)dev->current_lsb / 3600.0);
    reading->charge_ah = (charge > 0.0f) ? charge : 0.0f;
    if (ina228_idle_cb) ina228_idle_cb();

    /* --- Die Temperature (reg 0x06, 16-bit read) ---------------------------
     * Register layout: [15:4] = 12-bit signed temperature, [3:0] = reserved.
     * LSB = 0.125°C per 12-bit count after >>4 shift.
     * Raw 0x10F0 at ~25°C ambient → 0x10F >> 0 = 271 × 0.125 = 33.9°C. */
    {
        uint16_t temp_raw;
        ret = ina228_read16(dev, INA228_REG_DIETEMP, &temp_raw);
        if (ret != HAL_OK) { return ret; }
        int16_t temp_signed = (int16_t)temp_raw >> 4;  /* bits [15:4] are the 12-bit signed value */
        reading->die_temp_c = (float)temp_signed * 0.125f;
    }

    return HAL_OK;
}

/**
 * @brief  Reset the energy and charge accumulators to zero.
 * @param  dev INA228 device handle.
 * @retval HAL status.
 */
HAL_StatusTypeDef INA228_ResetEnergy(INA228_t *dev)
{
    /* Read current CONFIG value, set RSTACC bit */
    uint16_t config;
    HAL_StatusTypeDef ret = ina228_read16(dev, INA228_REG_CONFIG, &config);
    if (ret != HAL_OK) {
        return ret;
    }
    config |= INA228_CONFIG_RSTACC;
    return ina228_write16(dev, INA228_REG_CONFIG, config);
}

/**
 * @brief  Configure over-current alert via the shunt overvoltage limit register.
 *
 * Converts the current threshold in Amps to a shunt voltage (using the
 * configured shunt resistance), then writes it to SOVL.  Also enables
 * SHUNT_OVF alert and latching in DIAG_ALRT.
 *
 * @param  dev   INA228 device handle.
 * @param  amps  Over-current threshold in Amperes.
 * @retval HAL status.
 */
HAL_StatusTypeDef INA228_SetAlertOverCurrent(INA228_t *dev, float amps)
{
    HAL_StatusTypeDef ret;

    /* Shunt voltage = I * R_shunt.  SOVL LSB = 5 uV (with ADCRANGE=0). */
    float vshunt_uv = amps * dev->shunt_ohms * 1e6f;  /* convert to uV */
    int16_t sovl = (int16_t)(vshunt_uv / 5.0f + 0.5f);

    ret = ina228_write16(dev, INA228_REG_SOVL, (uint16_t)sovl);
    if (ret != HAL_OK) {
        return ret;
    }

    /* Read-modify-write DIAG_ALRT: enable SHUNT_OVF + alert latch */
    uint16_t diag;
    ret = ina228_read16(dev, INA228_REG_DIAG_ALRT, &diag);
    if (ret != HAL_OK) {
        return ret;
    }
    diag |= INA228_DIAG_SHUNT_OVF | INA228_DIAG_ALATCH;
    ret = ina228_write16(dev, INA228_REG_DIAG_ALRT, diag);

    return ret;
}

/**
 * @brief  Clear the INA228 ALERT latch by reading the DIAG_ALRT register.
 *
 * After an OCP trip the ALERT pin stays latched low.  A read of DIAG_ALRT
 * clears all latched status bits and releases the ALERT pin.  Call this
 * whenever the user acknowledges / clears a fault.
 *
 * @param  dev INA228 device handle.
 * @retval HAL status.
 */
HAL_StatusTypeDef INA228_ClearAlertLatch(INA228_t *dev)
{
    uint16_t dummy;
    return ina228_read16(dev, INA228_REG_DIAG_ALRT, &dummy);
}
