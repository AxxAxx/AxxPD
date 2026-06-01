// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#pragma once
// AxxPD firmware entry points. Thin C-callable wrapper around the vendored
// pdsink PD stack. Call axxpd_init() once after HAL/peripheral init, then
// axxpd_run() from the main loop. Route UCPD1_IRQHandler to
// axxpd_ucpd_irq().
//
// Accessor functions expose live pdsink state to the C application layer
// (ui.c, graph.c, main.c). All reads are live — no sync tick, no cached
// copies.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Lifecycle */
void axxpd_init(void);
void axxpd_run(void);
void axxpd_tick_pd(void);   /* PD-only tick, safe to call from SysTick ISR */
void axxpd_enable_tick(void); /* Call after axxpd_init() to enable SysTick/callback ticks */
void axxpd_ucpd_irq(void);

/* PD contract state — live reads from pdsink */
float    axxpd_get_negotiated_v(void);
float    axxpd_get_negotiated_a(void);
uint8_t  axxpd_get_active_pdo_index(void);  /* 1-based, 0 = no contract */
uint8_t  axxpd_is_pps_active(void);
uint32_t axxpd_get_pps_mv(void);
uint32_t axxpd_get_pps_ma(void);
uint8_t  axxpd_is_epr_active(void);
uint8_t  axxpd_is_src_epr_capable(void);

/* Source PDOs — caller-owned buffer, returns count written */
uint8_t  axxpd_get_src_pdos(uint32_t *out, uint8_t max);
uint8_t  axxpd_get_epr_src_pdos(uint32_t *out, uint8_t max);

/* Cable info — will report "not detected" until VCONN hardware rev */
uint8_t  axxpd_get_cable_info(uint8_t *type, uint8_t *max_current,
                               uint8_t *max_voltage, uint8_t *usb_ss);

/* Control */
void     axxpd_request_voltage(uint32_t mv, uint32_t ma);
void     axxpd_request_pdo_position(uint8_t position);
void     axxpd_hard_reset(void);
void     axxpd_enable_epr(void);
void     axxpd_disable_epr_intent(void);
void     axxpd_enable_cable_emu(void);

/* Verify PD stack health — force HAS_EXPLICIT_CONTRACT if contract exists */
void     axxpd_ensure_contract_flag(void);

/* Operation complete — for *OPC? / *WAI */
uint8_t  axxpd_is_opc_done(void);

#ifdef __cplusplus
}
#endif
