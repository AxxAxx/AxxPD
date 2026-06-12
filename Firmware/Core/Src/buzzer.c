// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

/**
 * @file    buzzer.c
 * @brief   PWM buzzer driver on TIM8_CH1.
 *
 * TIM8 is pre-configured by CubeMX:
 *   - Prescaler = 127  -> timer clock = 128MHz / 128 = 1MHz
 *
 * All beeps are fully timer-driven: the repetition counter (RCR) counts
 * down PWM cycles and one-pulse mode (OPM) auto-stops the timer.
 * No polling or Buzzer_Update() needed for timing — the hardware does it.
 */

#include "buzzer.h"
#include "settings.h"

#define BUZZER_TIM_CLOCK_HZ  1000000UL
#define BUZZER_BASE_HZ       1000U

static TIM_HandleTypeDef *s_htim = NULL;

void Buzzer_Init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;
    /* Invert TIM8_CH1 output polarity (active-low) so the resting/stopped PWM
     * level is LOW. The 2N7002 buzzer FET conducts on a HIGH gate; with the
     * default active-high polarity the pin was left HIGH after a one-pulse beep
     * (counter rests at 0 = active), keeping the MOSFET permanently conducting.
     * Active-low makes idle LOW (FET off); the 50%-duty tone is unaffected. */
    htim->Instance->CCER |= TIM_CCER_CC1P;
    /* Drive PA15 low to ensure the 2N7002 buzzer MOSFET is fully off.
     * MX_TIM8_Init puts PA15 in AF mode, but TIM8 isn't started yet,
     * so the pin floats at an intermediate voltage → 2N7002 partially
     * on → linear mode → hot. Reconfigure as GPIO output low. */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}

void Buzzer_Beep(uint16_t freq_hz, uint16_t duration_ms)
{
    if (!Settings_GetBuzzerEnabled() || s_htim == NULL || freq_hz == 0U)
        return;

    /* Reconfigure PA15 back to TIM8_CH1 AF mode for PWM output */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM8;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Stop any in-progress beep */
    HAL_TIM_PWM_Stop(s_htim, TIM_CHANNEL_1);
    s_htim->Instance->CR1 &= ~TIM_CR1_OPM;

    uint32_t period = BUZZER_TIM_CLOCK_HZ / (uint32_t)freq_hz;
    if (period == 0U) period = 1U;

    /* Number of PWM cycles for requested duration */
    uint32_t cycles = ((uint32_t)freq_hz * (uint32_t)duration_ms + 500U) / 1000U;
    if (cycles < 1U) cycles = 1U;
    if (cycles > 65536U) cycles = 65536U;  /* RCR is 16-bit on TIM8: max 65536 cycles (RCR = cycles-1) */

    __HAL_TIM_SET_AUTORELOAD(s_htim, period - 1U);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, period / 2U);

    /* RCR = cycles-1: timer generates (RCR+1) PWM cycles then stops (OPM) */
    s_htim->Instance->RCR = (uint16_t)(cycles - 1U);

    /* One-pulse mode: timer stops automatically after RCR+1 overflows */
    s_htim->Instance->CR1 |= TIM_CR1_OPM;

    /* Force update to load RCR and ARR shadow registers immediately */
    s_htim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(s_htim, 0U);

    HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1);
}

void Buzzer_Update(void)
{
    /* No-op: timer auto-stops via one-pulse mode. Kept for API compat. */
}

void Buzzer_Off(void)
{
    if (s_htim != NULL) {
        HAL_TIM_PWM_Stop(s_htim, TIM_CHANNEL_1);
        s_htim->Instance->CR1 &= ~TIM_CR1_OPM;
        /* Return PA15 to GPIO low so 2N7002 is fully off */
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = GPIO_PIN_15;
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pull = GPIO_PULLDOWN;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &gpio);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    }
}

/* Named presets — duration in ms, timer handles exact cycle count.
 * Perceived loudness of the piezo scales with burst length: 10 ms (10
 * cycles at 1 kHz) barely rang up and clicks were much quieter than the
 * 60-120 ms ON/OFF beeps, so Click is 20 ms with Confirm above it. */
void Buzzer_Click(void)    { Buzzer_Beep(BUZZER_BASE_HZ, 20);  }
void Buzzer_Confirm(void)  { Buzzer_Beep(BUZZER_BASE_HZ, 35);  }
void Buzzer_Fault(void)    {
    /* Always play fault tone regardless of buzzer setting — safety critical */
    if (s_htim == NULL || BUZZER_BASE_HZ == 0U) return;
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF2_TIM8;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_TIM_PWM_Stop(s_htim, TIM_CHANNEL_1);
    s_htim->Instance->CR1 &= ~TIM_CR1_OPM;
    /* 2 kHz tone for 320 ms. Derive from the 1 MHz timer clock (TIM8 has a
     * /128 prescaler). The old code divided SystemCoreClock (128 MHz), which
     * overflowed the 16-bit ARR and produced an inaudible ~16 Hz "tone" — that
     * was why fault/rejection beeps couldn't be heard. */
    uint32_t fault_hz = 2000U;
    uint32_t arr = (BUZZER_TIM_CLOCK_HZ / fault_hz) - 1U;
    __HAL_TIM_SET_AUTORELOAD(s_htim, arr);
    __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, arr / 2U);
    uint32_t rep = (fault_hz * 320U) / 1000U;
    if (rep == 0U) rep = 1U;
    s_htim->Instance->RCR = (uint16_t)(rep - 1U);
    s_htim->Instance->CR1 |= TIM_CR1_OPM;
    __HAL_TIM_SET_COUNTER(s_htim, 0);
    HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1);
}
void Buzzer_Enable(void)   { Buzzer_Beep(1200U, 60);  }  /* high chirp for ON */
void Buzzer_Disable(void)  { Buzzer_Beep(BUZZER_BASE_HZ, 120); }
void Buzzer_Warn(void)     { Buzzer_Beep(BUZZER_BASE_HZ, 20);  }

void Buzzer_FreqSweep(void)
{
    /* Narrow sweep around 1000 Hz to find exact resonance peak */
    static const uint16_t freqs[] = {
        700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500
    };
    for (uint8_t i = 0; i < sizeof(freqs)/sizeof(freqs[0]); i++) {
        /* Bypass the buzzer-enabled setting for this test */
        if (s_htim == NULL) return;

        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = GPIO_PIN_15;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF2_TIM8;
        HAL_GPIO_Init(GPIOA, &gpio);

        HAL_TIM_PWM_Stop(s_htim, TIM_CHANNEL_1);
        s_htim->Instance->CR1 &= ~TIM_CR1_OPM;

        uint32_t period = BUZZER_TIM_CLOCK_HZ / (uint32_t)freqs[i];
        uint32_t cycles = ((uint32_t)freqs[i] * 250U + 500U) / 1000U;
        if (cycles > 256U) cycles = 256U;

        __HAL_TIM_SET_AUTORELOAD(s_htim, period - 1U);
        __HAL_TIM_SET_COMPARE(s_htim, TIM_CHANNEL_1, period / 2U);
        s_htim->Instance->RCR = (uint16_t)(cycles - 1U);
        s_htim->Instance->CR1 |= TIM_CR1_OPM;
        s_htim->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_SET_COUNTER(s_htim, 0U);

        HAL_TIM_PWM_Start(s_htim, TIM_CHANNEL_1);
        HAL_Delay(350);  /* 250ms tone + 100ms gap */
    }
    Buzzer_Off();
}
