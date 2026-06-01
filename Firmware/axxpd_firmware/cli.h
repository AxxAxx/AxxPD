// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#pragma once
// Interactive UART CLI for AxxPD.
//
// Call cli_init() once after axxpd_init(), passing the live Port / PE / DPM
// pointers that axxpd_main.cpp placement-new'd. Call cli_poll() every main
// loop iteration to service UART RX and emit async status changes.

#include "pd/port.h"
#include "pd/pe.h"
#include "app_dpm.h"

class UcpdDriver;  // forward

void cli_init(pd::Port* port, pd::PE* pe, AppDPM* dpm, UcpdDriver* driver);
void cli_poll();
void cli_set_epr_intent(bool enable);  // set user_wants_epr from outside CLI
