// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#pragma once
// pdsink configuration for STM32G4 UCPD port

// Timer: 0 = millisecond resolution (HAL_GetTick returns ms).
// Non-zero would multiply all ms timeouts by 1000 for µs timers.
#define PD_TIMER_RESOLUTION_US 0

// Disable jetlog (we use UART printf directly)
#define PD_LOG_LEVEL 0
