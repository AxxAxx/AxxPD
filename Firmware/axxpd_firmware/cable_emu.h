// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#pragma once
// Cable e-marker emulation — responds to SOP' Discover_Identity VDM requests
// with a passive EPR cable identity (50V/5A, PD 3.0+, USB 2.0).
//
// This is needed because the STM32 UCPD sits between source and cable on the
// bus, and for EPR entry the source must discover an EPR-capable cable.

#include <cstdint>

// Check if rx_buf contains a SOP' Discover_Identity request.
// If so, build the VDM ACK response into tx_buf and return the byte count.
// Returns 0 if the message is not a Discover_Identity request.
uint16_t cable_emu_build_response(const uint8_t* rx_buf, uint16_t rx_size,
                                   uint8_t* tx_buf, uint16_t tx_buf_size);

// Reset the cable message-ID counter (call on every Hard Reset).
void cable_emu_reset(void);
