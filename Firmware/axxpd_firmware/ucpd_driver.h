// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#pragma once
// UcpdDriver — pdsink IDriver implementation for STM32G4 UCPD peripheral.
// Uses LL API + DMA for PD messaging, with SOP' cable e-marker emulation.

#include <cstdint>
#include <cstring>
#include "pd/idriver.h"
#include "pd/port.h"
#include "pd/utils/spsc_overwrite_queue.h"
#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_hal.h"

class UcpdDriver : public pd::IDriver {
public:
    explicit UcpdDriver(pd::Port& port);

    // IDriver
    void setup() override;

    // ITCPC -- CC scanning
    void req_scan_cc() override;
    bool try_scan_cc_result(pd::TCPC_CC_LEVEL::Type& cc1,
                            pd::TCPC_CC_LEVEL::Type& cc2) override;
    void req_active_cc() override;
    bool try_active_cc_result(pd::TCPC_CC_LEVEL::Type& cc) override;

    // ITCPC -- polarity + RX enable
    bool is_vbus_ok() override;
    void req_set_polarity(pd::TCPC_POLARITY active_cc) override;
    bool is_set_polarity_done() override;
    void req_rx_enable(bool enable) override;
    bool is_rx_enable_done() override;

    // ITCPC -- TX + RX data
    bool fetch_rx_data() override;
    void req_transmit() override;

    // ITCPC -- BIST + hard reset
    void req_set_bist(pd::TCPC_BIST_MODE mode) override;
    bool is_set_bist_done() override;
    void req_hr_send() override;
    bool is_hr_send_done() override;
    auto get_hw_features() -> pd::TCPC_HW_FEATURES override;

    // ITimer
    TimeFunc get_time_func() const override;
    void rearm(uint32_t interval) override;
    bool is_rearm_supported() override;

    // ErrorRecovery: drop Rd for 200ms to force charger to see a disconnect,
    // then re-assert Rd.  Call from main loop after repeated EPR failures.
    void error_recovery();

    // Cable e-marker emulation control for adaptive probing.
    bool is_cable_emu_disabled() const { return cable_emu_disabled_; }
    void enable_cable_emu();

    // ISR callbacks (called from UCPD1_IRQHandler via axxpd_ucpd_irq)
    void irq_handle_rxmsgend();
    void irq_handle_txmsgsent();
    void irq_handle_txmsgdisc();
    void irq_handle_txmsgabt();
    void irq_handle_rxhrstdet();
    void irq_handle_hrstsent();
    void irq_handle_typec_event();

private:
    pd::Port& port_;
    UCPD_TypeDef* ucpd_ = UCPD1;
    pd::TCPC_POLARITY polarity_ = pd::TCPC_POLARITY::NONE;

    // CC scan results (filled synchronously from UCPD_SR)
    pd::TCPC_CC_LEVEL::Type cc1_level_ = pd::TCPC_CC_LEVEL::NONE;
    pd::TCPC_CC_LEVEL::Type cc2_level_ = pd::TCPC_CC_LEVEL::NONE;
    volatile bool cc_scan_done_ = false;   // written by ISR (typec_event), read by main loop

    // RX DMA staging size (file-static dma_buf_ in ucpd_driver.cpp).
    // 272 B so chunked extended messages (EPR_Source_Capabilities arrives as
    // ≤30-byte chunks) have comfortable headroom. Previous 30 B was exactly
    // at the chunk limit and risked silent overrun.
    static constexpr size_t RX_BUF_SIZE = 272;
    // RX queue: the ISR (single producer) pushes decoded SOP messages,
    // fetch_rx_data() (single consumer, PRL task tick) pops them. Replaces
    // the old single rx_buf_ slot, which the ISR could overwrite mid-copy
    // (tearing) and which silently dropped back-to-back messages that were
    // already GoodCRC-acknowledged. Depth 4 matches pdsink's FUSB302 driver.
    spsc_overwrite_queue<pd::PD_CHUNK, 4> rx_queue_{};
    volatile uint32_t rx_ordset_ = 0;   // last forwarded ordered set (debug only)

    // TX buffer (DMA source, persists until TX complete)
    uint8_t tx_buf_[30] __attribute__((aligned(4)));

    // TX state
    volatile bool tx_done_ = false;
    volatile bool tx_success_ = false;

    // Software GoodCRC verification for PRL TX. The G4 UCPD is a raw PHY:
    // TXMSGSENT only means "transmitted", NOT "acknowledged". TXMSGSENT arms
    // this state; a matching SOP GoodCRC in irq_handle_rxmsgend() reports
    // SUCCEEDED; fetch_rx_data() reports FAILED once tReceive expires so the
    // PRL retry machinery fires.
    volatile bool tx_awaiting_goodcrc_ = false;
    volatile uint8_t tx_goodcrc_msg_id_ = 0;
    volatile uint32_t tx_goodcrc_deadline_ = 0;   // HAL tick

    // RX enabled guard — prevents DMA restarts while already running.
    // Written from both ISR and main contexts.
    volatile bool rx_enabled_ = false;

    // Set when ISR sends GoodCRC — TXMSGSENT for GoodCRC must NOT update PRL status
    volatile bool goodcrc_tx_pending_ = false;

    // SOP' cable e-marker emulation state.
    // When a SOP' Discover_Identity arrives, the ISR sends SOP' GoodCRC and
    // then (on TXMSGSENT) the VDM ACK response.  Two flags track the two-phase TX.
    // cable_emu_disabled_ starts TRUE (assume cable has e-marker).
    // Cleared by ErrorRecovery after EPR failure (no e-marker → need emu).
    // Set back to true by TXMSGDISC (real e-marker detected, stop colliding).
    // Only fully reset on fresh CC attach (new cable).
    volatile bool cable_goodcrc_tx_pending_ = false;
    volatile bool cable_response_tx_pending_ = false;
    volatile bool cable_emu_disabled_ = true;   // optimistic: assume real e-marker
    uint8_t cable_tx_buf_[30] __attribute__((aligned(4)));
    uint16_t cable_tx_size_ = 0;

    // Deferred SOP' handling: when a SOP' Discover_Identity arrives while TX
    // is busy (e.g. sending SOP GoodCRC), the request is queued here instead
    // of being silently dropped.  Serviced from irq_handle_txmsgsent() once
    // the current TX completes.
    volatile bool sop1_deferred_pending_ = false;
    uint8_t  sop1_deferred_buf_[30] __attribute__((aligned(4)));
    uint16_t sop1_deferred_size_ = 0;
    uint16_t sop1_deferred_hdr_ = 0;

    // Hard reset
    volatile bool hr_sent_ = false;

    // Helpers
    static pd::TCPC_CC_LEVEL::Type decode_cc(uint32_t vstate);
    void setup_rx_dma();
    void setup_tx_dma(const uint8_t* data, uint16_t size);
};
