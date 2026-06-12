// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#ifndef __NTC_H
#define __NTC_H

#include "stm32g4xx_hal.h"

void NTC_Init(ADC_HandleTypeDef *hadc);
float NTC_ReadTemperature(void);  /* returns degrees C */

#endif /* __NTC_H */
