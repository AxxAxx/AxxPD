// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#ifndef __UI_H
#define __UI_H

#include "ina228.h"
#include "buttons.h"

typedef enum {
    UI_SCREEN_DASHBOARD,
    UI_SCREEN_PDOS,
    UI_SCREEN_GRAPH,
    UI_SCREEN_PRESETS,
    UI_SCREEN_ENERGY,
    UI_SCREEN_SETTINGS,
    UI_SCREEN_COUNT,
} UIScreen_t;

void UI_Init(void);
void UI_Update(INA228_Reading_t *reading, float ntc_temp, uint8_t output_on);
void UI_HandleButton(ButtonEvent_t event);
uint8_t UI_WantsPwrShort(void);  /* Returns 1 if UI should handle PWR_SHORT instead of output toggle */
UIScreen_t UI_GetScreen(void);
void UI_SetLocked(uint8_t locked);
uint8_t UI_IsLocked(void);
void UI_ToolTick(INA228_Reading_t *reading);  /* Call from main loop every iteration for tool state machines */

#endif /* __UI_H */
