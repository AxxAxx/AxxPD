// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32g4xx_hal.h"

void Buzzer_Init(TIM_HandleTypeDef *htim);
void Buzzer_Beep(uint16_t freq_hz, uint16_t duration_ms);
void Buzzer_Update(void);  /* call from main loop */
void Buzzer_Off(void);

/* Named beep presets — 1000 Hz base unless noted. */
void Buzzer_Click(void);       /* Button press: short tick, 20ms         */
void Buzzer_Confirm(void);     /* Selection confirm: 35ms                */
void Buzzer_Fault(void);       /* HW fault / thermal: loud 2000Hz 320ms  */
void Buzzer_Enable(void);      /* Output enable: high chirp 1200Hz 60ms  */
void Buzzer_Disable(void);     /* Output disable / emergency off: 120ms  */
void Buzzer_Warn(void);        /* Temperature warning: short chirp 20ms  */
void Buzzer_FreqSweep(void);   /* Test sweep 700-1500Hz to find resonance */

#endif /* __BUZZER_H */
