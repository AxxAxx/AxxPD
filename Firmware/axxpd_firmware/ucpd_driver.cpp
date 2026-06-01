// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

// UcpdDriver — pdsink IDriver for STM32G4 UCPD.
// Implements CC scanning, polarity selection, DMA-based RX/TX, hard reset,
// and SOP' cable e-marker emulation (cable_emu.{cpp,h}).

#include "ucpd_driver.h"
#include "cable_emu.h"
#include "pd/port.h"
#include "pd/messages.h"

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_hal.h"

#include <cstring>
#include <cstdio>

volatile uint8_t  g_stream_enabled = 0;
volatile uint32_t g_stream_interval_ms = 50;  // default 20 Hz; settable via CLI

// Double-buffer for RX DMA: DMA writes to dma_buf_, ISR copies to rx_buf_.
// Size must match UcpdDriver::RX_BUF_SIZE (272 B) so chunked extended
// messages (EPR_Source_Capabilities, up to 30 B per chunk on-wire) fit
// with headroom.
static uint8_t dma_buf_[272] __attribute__((aligned(4)));
static_assert(sizeof(dma_buf_) >= 30, "dma_buf_ must hold a full on-wire chunk");

// ---------------------------------------------------------------------------
// Debug helpers (UART2, blocking)
// ---------------------------------------------------------------------------
extern "C" UART_HandleTypeDef huart2;

// Gated by axxpd_low_trace so [CC]/[RX]/[TX]/[UCPD] lines don't clutter
// the interactive console. Toggle at runtime via the CLI 'trace on/off'
// command (or SCPI :SYST:TRACE:LOW ON|OFF).
volatile bool axxpd_low_trace = false;

static void dbg(const char* msg) {
    if (!axxpd_low_trace) return;
    // 100 ms timeout; see axxpd_main.cpp dbg() for rationale.
    HAL_UART_Transmit(&huart2, reinterpret_cast<const uint8_t*>(msg),
                      static_cast<uint16_t>(strlen(msg)), 100);
}

// ---------------------------------------------------------------------------
// Construction + setup
// ---------------------------------------------------------------------------

UcpdDriver::UcpdDriver(pd::Port& port) : port_(port) {}

void UcpdDriver::setup() {
    // Peripheral clocks (DMA/CRC already enabled by MX_DMA_Init, but be safe)
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    // Disable dead battery pull-downs — they conflict with UCPD active CC
    // control (double Rd causes wrong impedance).  The TPD4S480 dead-battery
    // Rd provides sink detection at cold boot; once UCPD takes over, MCU
    // DBCC must be off.
    LL_PWR_DisableUCPDDeadBattery();

    // UCPD1 peripheral init (CubeMX already did GPIO + DMA mux, but we
    // re-initialise the UCPD registers ourselves for full control)
    LL_UCPD_Disable(ucpd_);

    LL_UCPD_InitTypeDef cfg = {};
    LL_UCPD_StructInit(&cfg);
    LL_UCPD_Init(ucpd_, &cfg);

    // Accept SOP + SOP' + Hard Reset + Cable Reset ordered sets.
    // SOP' must be in the filter — some chargers (Anker A2697) require the
    // sink's UCPD to acknowledge SOP' at the PHY level for cable discovery
    // to succeed.  cable_emu starts DISABLED (cable_emu_disabled_=true) so
    // we receive SOP' but don't respond, letting the real e-marker handle it.
    // If there's no e-marker, Pattern B in cli_poll() enables cable_emu.
    LL_UCPD_SetRxOrderSet(ucpd_,
        LL_UCPD_ORDERSET_SOP | LL_UCPD_ORDERSET_SOP1 |
        LL_UCPD_ORDERSET_HARDRST | LL_UCPD_ORDERSET_CABLERST);

    // Enable DMA for RX and TX
    LL_UCPD_RxDMAEnable(ucpd_);
    LL_UCPD_TxDMAEnable(ucpd_);

    // Enable UCPD peripheral FIRST — CR config bits need UCPDEN=1
    LL_UCPD_Enable(ucpd_);

    // NOW set sink mode: Rd on both CC lines (must be after Enable)
    LL_UCPD_SetSNKRole(ucpd_);
    LL_UCPD_SetccEnable(ucpd_, LL_UCPD_CCENABLE_CC1CC2);

    // Configure NVIC + IMR BEFORE enabling RX — if a message arrives
    // between RxEnable and IMR setup, the RXMSGEND flag gets stuck.
    NVIC_SetPriority(UCPD1_IRQn, 0);
    NVIC_EnableIRQ(UCPD1_IRQn);

    ucpd_->IMR = UCPD_IMR_TYPECEVT1IE | UCPD_IMR_TYPECEVT2IE |
                 UCPD_IMR_RXMSGENDIE  | UCPD_IMR_RXHRSTDETIE  |
                 UCPD_IMR_RXOVRIE     | UCPD_IMR_RXORDDETIE    |
                 UCPD_IMR_TXMSGSENTIE |
                 UCPD_IMR_TXMSGDISCIE | UCPD_IMR_TXMSGABTIE    |
                 UCPD_IMR_HRSTSENTIE  | UCPD_IMR_HRSTDISCIE;

    // NOW enable RX + DMA (interrupts already configured)
    setup_rx_dma();
    LL_UCPD_RxEnable(ucpd_);
    rx_enabled_ = true;

    dbg("[UCPD] setup done\r\n");
}

// ---------------------------------------------------------------------------
// ErrorRecovery — USB Type-C Spec Section 4.5.x
// ---------------------------------------------------------------------------

void UcpdDriver::error_recovery() {
    // 1. Remove Rd from both CC lines — charger sees cable unplug.
    LL_UCPD_SetccEnable(ucpd_, LL_UCPD_CCENABLE_NONE);
    LL_UCPD_RxDisable(ucpd_);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);  // RX DMA
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);  // TX DMA
    rx_enabled_ = false;

    // Clear all pending TX/RX flags — nothing can be in flight now.
    goodcrc_tx_pending_ = false;
    cable_goodcrc_tx_pending_ = false;
    cable_response_tx_pending_ = false;
    rx_msg_pending_ = false;
    sop1_deferred_pending_ = false;

    // 2. Wait tErrorRecovery.  Spec minimum is 25 ms; we use 200 ms
    //    because some chargers need a longer CC-absent window to fully
    //    reset their PD state machine (observed on 240W EPR sources).
    uint32_t t0 = HAL_GetTick();
    while ((int32_t)(HAL_GetTick() - t0) < 200) { /* spin */ }

    // 3. Re-assert Rd on both CC lines — charger will re-detect us.
    LL_UCPD_SetSNKRole(ucpd_);
    LL_UCPD_SetccEnable(ucpd_, LL_UCPD_CCENABLE_CC1CC2);

    // 4. Reset cable_emu message ID.  Do NOT change cable_emu_disabled_ —
    //    if a real e-marker was detected (TXMSGDISC), re-enabling cable_emu
    //    here would cause a collision on the next SOP' exchange.  Only fresh
    //    CC attach (new cable via req_set_polarity) toggles cable_emu_disabled_.
    cable_emu_reset();

    // pdsink's TC layer will re-detect CC on the next tick() and
    // drive a fresh SPR negotiation → PE_SNK_Ready → EPR auto-entry.
}

void UcpdDriver::enable_cable_emu() {
    cable_emu_disabled_ = false;
    cable_emu_reset();
    // Add SOP' to the RX filter so the ISR can see Discover_Identity.
    LL_UCPD_SetRxOrderSet(ucpd_,
        LL_UCPD_ORDERSET_SOP | LL_UCPD_ORDERSET_SOP1 |
        LL_UCPD_ORDERSET_HARDRST | LL_UCPD_ORDERSET_CABLERST);
}

// ---------------------------------------------------------------------------
// CC scanning (synchronous — UCPD has analog comparators on-chip)
// ---------------------------------------------------------------------------

pd::TCPC_CC_LEVEL::Type UcpdDriver::decode_cc(uint32_t vstate) {
    // UCPD SR VSTATE for sink: 0=open, 1=default Rp, 2=1.5A, 3=3.0A
    switch (vstate) {
        case 0:  return pd::TCPC_CC_LEVEL::NONE;
        case 1:  return pd::TCPC_CC_LEVEL::RP_0_5;
        case 2:  return pd::TCPC_CC_LEVEL::RP_1_5;
        case 3:  return pd::TCPC_CC_LEVEL::RP_3_0;
        default: return pd::TCPC_CC_LEVEL::NONE;
    }
}

void UcpdDriver::req_scan_cc() {
    static uint32_t scan_count = 0;
    uint32_t sr = ucpd_->SR;
    cc1_level_ = decode_cc((sr >> UCPD_SR_TYPEC_VSTATE_CC1_Pos) & 0x3);
    cc2_level_ = decode_cc((sr >> UCPD_SR_TYPEC_VSTATE_CC2_Pos) & 0x3);
    cc_scan_done_ = true;
    if (++scan_count <= 5) {
        char b[40]; snprintf(b,sizeof(b),"[CC] scan#%lu cc1=%u cc2=%u\r\n",
            (unsigned long)scan_count, (unsigned)cc1_level_, (unsigned)cc2_level_);
        dbg(b);
    }
}

bool UcpdDriver::try_scan_cc_result(pd::TCPC_CC_LEVEL::Type& cc1,
                                     pd::TCPC_CC_LEVEL::Type& cc2) {
    if (!cc_scan_done_) return false;
    cc1 = cc1_level_;
    cc2 = cc2_level_;
    cc_scan_done_ = false;
    return true;
}

void UcpdDriver::req_active_cc() {
    req_scan_cc();  // Same as full scan for UCPD
}

bool UcpdDriver::try_active_cc_result(pd::TCPC_CC_LEVEL::Type& cc) {
    if (!cc_scan_done_) return false;
    cc = (polarity_ == pd::TCPC_POLARITY::CC1) ? cc1_level_ : cc2_level_;
    cc_scan_done_ = false;
    return true;
}

// ---------------------------------------------------------------------------
// VBUS + polarity
// ---------------------------------------------------------------------------

bool UcpdDriver::is_vbus_ok() {
    // No VBUS ADC on NUCLEO-G431KB — trust CC attachment
    return true;
}

void UcpdDriver::req_set_polarity(pd::TCPC_POLARITY active_cc) {
    polarity_ = active_cc;
    if (active_cc == pd::TCPC_POLARITY::NONE) {
        LL_UCPD_RxDisable(ucpd_);
        rx_enabled_ = false;
        return;
    }
    LL_UCPD_SetCCPin(ucpd_, (active_cc == pd::TCPC_POLARITY::CC1)
                     ? LL_UCPD_CCPIN_CC1 : LL_UCPD_CCPIN_CC2);
    // Enable RX + DMA only on first call; ISR handles DMA restarts after
    if (!rx_enabled_) {
        setup_rx_dma();
        LL_UCPD_RxEnable(ucpd_);
        rx_enabled_ = true;
    }
}

bool UcpdDriver::is_set_polarity_done() { return true; }

// ---------------------------------------------------------------------------
// RX path — DMA setup + enable/disable + fetch
// ---------------------------------------------------------------------------

volatile uint32_t ucpd_rxmsgend_count = 0;
volatile uint32_t ucpd_cable_emu_sent = 0;
volatile uint32_t ucpd_cable_emu_fail = 0;
volatile uint32_t ucpd_rxhrstdet_count = 0;
volatile uint32_t ucpd_rxovr_count = 0;
volatile uint32_t ucpd_rxerr_count = 0;
volatile uint32_t ucpd_goodcrc_filtered = 0;

// ISR event log for debugging. do0 holds the first data object
// (little-endian from dma_buf_[2..5]) if the message carries one; 0 otherwise.
struct RxLogEntry { uint32_t ordset; uint16_t size; uint16_t hdr; uint32_t do0; };
volatile RxLogEntry rx_log[16];
volatile uint32_t rx_log_idx = 0;

void UcpdDriver::setup_rx_dma() {
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    // Ensure correct DMA config: periph-to-memory, byte, memory increment
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1,
        reinterpret_cast<uint32_t>(&ucpd_->RXDR));
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1,
        reinterpret_cast<uint32_t>(dma_buf_));
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1,
        LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, RX_BUF_SIZE);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
}

void UcpdDriver::req_rx_enable(bool enable) {
    if (enable) {
        if (!rx_enabled_) {
            // First enable — start DMA and RX
            setup_rx_dma();
            LL_UCPD_RxEnable(ucpd_);
            rx_enabled_ = true;
            dbg("[UCPD] RX enabled\r\n");
        }
        // Subsequent calls are no-ops: UCPD RX + DMA are already running.
        // The ISR restarts DMA after each message; don't touch it here.
    } else {
        LL_UCPD_RxDisable(ucpd_);
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
        rx_enabled_ = false;
    }
}

bool UcpdDriver::is_rx_enable_done() { return true; }

bool UcpdDriver::fetch_rx_data() {
    if (!rx_msg_pending_) return false;
    rx_msg_pending_ = false;

    // Copy received bytes into port.rx_chunk
    auto& chunk = port_.rx_chunk;
    chunk.clear();

    if (rx_size_ >= 2) {
        // First 2 bytes = PD header (little-endian)
        chunk.header.raw_value = static_cast<uint16_t>(rx_buf_[0] | (rx_buf_[1] << 8));

        // Remaining bytes = data objects payload
        uint16_t payload_bytes = rx_size_ - 2;
        auto& buf = chunk.get_data();
        for (uint16_t i = 0; i < payload_bytes && buf.available() > 0; ++i) {
            buf.push_back(rx_buf_[2 + i]);
        }

        static uint32_t fetch_cnt = 0;
        if (++fetch_cnt <= 3) {
            char b[60];
            snprintf(b, sizeof(b), "[RX] hdr=%04X ndo=%u sz=%u ord=%lu\r\n",
                     chunk.header.raw_value,
                     (unsigned)chunk.header.data_obj_count,
                     (unsigned)rx_size_,
                     (unsigned long)rx_ordset_);
            dbg(b);
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// TX path — DMA setup + transmit + ISR callbacks
// ---------------------------------------------------------------------------

void UcpdDriver::setup_tx_dma(const uint8_t* data, uint16_t size) {
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
        reinterpret_cast<uint32_t>(data),
        reinterpret_cast<uint32_t>(&ucpd_->TXDR),
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, size);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);
}

volatile uint32_t ucpd_tx_count = 0;

void UcpdDriver::req_transmit() {
    // The ISR sends GoodCRC and cable_emu VDM responses on DMA channel 2.
    // The main loop can reach here in ~50-100 us — before the ISR's TX
    // finishes.  Spin until the ISR-initiated TX completes to prevent
    // DMA CH2 clobbering that corrupts both messages.
    // Timeout after 5 ms to avoid infinite hang if flags get stuck.
    {
        uint32_t t0 = HAL_GetTick();
        while (goodcrc_tx_pending_ || cable_goodcrc_tx_pending_ ||
               cable_response_tx_pending_) {
            if ((int32_t)(HAL_GetTick() - t0) >= 5) {
                // Timeout — force-clear flags and continue
                goodcrc_tx_pending_ = false;
                cable_goodcrc_tx_pending_ = false;
                cable_response_tx_pending_ = false;
                break;
            }
        }
    }

    auto& chunk = port_.tx_chunk;
    port_.tcpc_tx_status.store(pd::TCPC_TRANSMIT_STATUS::SENDING);

    tx_done_ = false;
    tx_success_ = false;

    // Build TX buffer: header (2 bytes LE) + data objects
    uint16_t payload = static_cast<uint16_t>(chunk.data_size());
    uint16_t total = 2 + payload;

    ucpd_tx_count++;
    // Reset ISR log on first TX so we capture messages around the Request
    if (ucpd_tx_count == 1) { rx_log_idx = 0; }
    tx_buf_[0] = chunk.header.raw_value & 0xFF;
    tx_buf_[1] = (chunk.header.raw_value >> 8) & 0xFF;
    if (payload > 0) {
        const auto& data = chunk.get_data();
        memcpy(&tx_buf_[2], data.data(), payload);
    }
    {
        static uint32_t tx_dbg = 0;
        if (++tx_dbg <= 5) {
            char b[70];
            uint32_t rdo = (payload >= 4) ? (tx_buf_[2] | (tx_buf_[3]<<8) | (tx_buf_[4]<<16) | (tx_buf_[5]<<24)) : 0;
            snprintf(b,sizeof(b),"[TX] hdr=%04X ndo=%u sz=%u rdo=%08lX\r\n",
                chunk.header.raw_value, (unsigned)chunk.header.data_obj_count, total, (unsigned long)rdo);
            dbg(b);
        }
    }

    // Mask UCPD IRQ while setting up DMA CH2 + firing TXSEND.
    // If a message arrives between setup_tx_dma and SendMessage, the ISR
    // would send a GoodCRC on the same DMA channel, clobbering our Request.
    NVIC_DisableIRQ(UCPD1_IRQn);
    LL_UCPD_WriteTxOrderSet(ucpd_, LL_UCPD_ORDERED_SET_SOP);
    LL_UCPD_SetTxMode(ucpd_, LL_UCPD_TXMODE_NORMAL);
    LL_UCPD_WriteTxPaySize(ucpd_, total);
    setup_tx_dma(tx_buf_, total);
    LL_UCPD_SendMessage(ucpd_);
    NVIC_EnableIRQ(UCPD1_IRQn);
}

// ---------------------------------------------------------------------------
// BIST + Hard Reset
// ---------------------------------------------------------------------------

void UcpdDriver::req_set_bist(pd::TCPC_BIST_MODE mode) {
    if (mode == pd::TCPC_BIST_MODE::Carrier) {
        LL_UCPD_SetTxMode(ucpd_, LL_UCPD_TXMODE_BIST_CARRIER2);
        LL_UCPD_SendMessage(ucpd_);
    }
    // Off / TestData: nothing extra needed
}

bool UcpdDriver::is_set_bist_done() { return true; }

void UcpdDriver::req_hr_send() {
    hr_sent_ = false;
    LL_UCPD_SendHardReset(ucpd_);
    dbg("[UCPD] HR send\r\n");
}

bool UcpdDriver::is_hr_send_done() { return hr_sent_; }

auto UcpdDriver::get_hw_features() -> pd::TCPC_HW_FEATURES {
    // STM32G4 UCPD capabilities:
    //   rx_auto_goodcrc_send = false — G4 has NO AUTOCRCRX; GoodCRC is sent
    //       manually from the ISR (irq_handle_rxmsgend fast-path).  PRL's
    //       PRL_Rx_Send_GoodCRC state is a pass-through regardless of this flag,
    //       so setting it false is safe and accurate.
    //   tx_auto_goodcrc_check = true — the UCPD peripheral waits for the
    //       partner's GoodCRC after TX and reports success/failure via
    //       TXMSGSENT/TXMSGDISC interrupts.
    //   tx_auto_retry = false — G4 UCPD has NO hardware auto-retry.  Setting
    //       this false enables PRL software retries (up to nRetryCount = 2),
    //       giving 3 total TX attempts instead of 1.  Previously true, which
    //       caused PRL to skip retries and immediately report Transmission_Error
    //       on any single TX failure — a likely contributor to EPR bootloops.
    return { false, true, false };
}

// ---------------------------------------------------------------------------
// ITimer
// ---------------------------------------------------------------------------

static uint32_t ucpd_get_tick() { return HAL_GetTick(); }

UcpdDriver::TimeFunc UcpdDriver::get_time_func() const {
    return &ucpd_get_tick;
}

void UcpdDriver::rearm(uint32_t /*interval*/) {
    // No-op: 1 ms SysTick provides the tick; pdsink polls via task.tick()
}

bool UcpdDriver::is_rearm_supported() { return false; }

// ---------------------------------------------------------------------------
// ISR callbacks — called from axxpd_ucpd_irq() in ISR context
// ---------------------------------------------------------------------------

void UcpdDriver::irq_handle_rxmsgend() {
    ucpd_rxmsgend_count++;
    LL_UCPD_ClearFlag_RxMsgEnd(ucpd_);

    // Check for CRC / RX errors
    if (ucpd_->SR & UCPD_SR_RXERR) {
        ucpd_rxerr_count++;
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, RX_BUF_SIZE);
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1,
                                reinterpret_cast<uint32_t>(dma_buf_));
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
        return;
    }

    // Read ordered set and size BEFORE restarting DMA
    uint32_t ordset = LL_UCPD_ReadRxOrderSet(ucpd_);
    uint16_t remaining = static_cast<uint16_t>(
        LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_1));
    uint16_t size = static_cast<uint16_t>(RX_BUF_SIZE - remaining);

    // Snapshot header from DMA buffer BEFORE any DMA restart
    uint16_t raw_hdr = 0;
    if (size >= 2) {
        raw_hdr = dma_buf_[0] | (dma_buf_[1] << 8);
    }

    // Classify message: is it SOP' or GoodCRC (will be filtered)?
    bool is_sop1 = (ordset == LL_UCPD_RXORDSET_SOP1);
    bool is_sop  = (ordset == LL_UCPD_RXORDSET_SOP);
    bool is_goodcrc = false;
    if (is_sop && size >= 2) {
        uint8_t msg_type = raw_hdr & 0x1F;
        uint8_t ndo = (raw_hdr >> 12) & 0x07;
        is_goodcrc = (msg_type == 1 && ndo == 0);
    }

    // ================================================================
    // FAST PATH: Send GoodCRC for SOP messages IMMEDIATELY.
    //
    // STM32G4 UCPD does NOT have auto-GoodCRC (AUTOCRCRX exists only on
    // G0/H5/U5).  We must manually send GoodCRC within tReceive (1.1 ms).
    //
    // Fire the GoodCRC DMA NOW — before memcpy, logging, and DMA restart.
    // This minimises the latency path to just: error-check → read
    // ordset/hdr → classify → GoodCRC-send (~20 instructions, <1 us at
    // 128 MHz).  All remaining bookkeeping runs while the UCPD PHY is
    // already transmitting the GoodCRC preamble on CC.
    // ================================================================
    if (is_sop && !is_goodcrc) {
        uint8_t msg_id_bits = (raw_hdr >> 9) & 0x07;
        uint16_t goodcrc_hdr = 0x0001            // msg_type = 1 (GoodCRC)
                             | (0 << 5)           // port_data_role = 0 (UFP/Sink)
                             | ((raw_hdr & 0x00C0)) // copy spec_revision bits [7:6]
                             | (0 << 8)           // port_power_role = 0 (Sink)
                             | (msg_id_bits << 9); // mirror msg_id

        static uint8_t gcrc_buf[2];
        gcrc_buf[0] = goodcrc_hdr & 0xFF;
        gcrc_buf[1] = (goodcrc_hdr >> 8) & 0xFF;

        // If cable_emu was mid-TX (VDM response), abort it — SOP GoodCRC
        // has absolute priority.
        cable_goodcrc_tx_pending_ = false;
        cable_response_tx_pending_ = false;

        // Setup TX DMA (channel 2) for GoodCRC
        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
            reinterpret_cast<uint32_t>(gcrc_buf),
            reinterpret_cast<uint32_t>(&ucpd_->TXDR),
            LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

        LL_UCPD_WriteTxOrderSet(ucpd_, LL_UCPD_ORDERED_SET_SOP);
        LL_UCPD_SetTxMode(ucpd_, LL_UCPD_TXMODE_NORMAL);
        LL_UCPD_WriteTxPaySize(ucpd_, 2);
        LL_UCPD_SendMessage(ucpd_);
        goodcrc_tx_pending_ = true;
    }

    // --- GoodCRC is now on the wire (or N/A for SOP'/GoodCRC). ----------
    // Remaining work is non-time-critical and runs while the UCPD PHY
    // transmits the preamble + GoodCRC body on CC (~213 us).

    // Copy data to rx_buf_ ONLY for messages that will be forwarded to PRL.
    // GoodCRC and SOP' must NOT touch rx_buf_ — they would contaminate
    // a pending SRC_CAPA or Accept that hasn't been fetched yet.
    if (is_sop && !is_goodcrc && size > 0 && size <= RX_BUF_SIZE) {
        memcpy(rx_buf_, dma_buf_, size);
        rx_ordset_ = ordset;
        rx_size_ = size;
    }

    // Restart RX DMA (writes to dma_buf_, not rx_buf_)
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, RX_BUF_SIZE);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1,
                            reinterpret_cast<uint32_t>(dma_buf_));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

    // Log ALL received messages to a circular ring buffer (using snapshot
    // header). Capture first data object (bytes 2..5) if present so EPRMDO
    // action codes and similar diagnostic payloads are visible post-hoc.
    // Circular so late messages (around Hard Reset) stay visible.
    {
        uint32_t i = (rx_log_idx++) % 16;
        rx_log[i].ordset = ordset;
        rx_log[i].size = size;
        rx_log[i].hdr = raw_hdr;
        if (size >= 6) {
            rx_log[i].do0 = (uint32_t)dma_buf_[2]
                          | ((uint32_t)dma_buf_[3] << 8)
                          | ((uint32_t)dma_buf_[4] << 16)
                          | ((uint32_t)dma_buf_[5] << 24);
        } else {
            rx_log[i].do0 = 0;
        }
    }

    // SOP' (cable e-marker emulation) — respond to Discover_Identity VDM
    // requests so the source sees an EPR-capable cable (50V/5A).  240W/48V
    // sources perform strict cable discovery; without this they Hard-Reset
    // right after EPR Enter_Succeeded.
    //
    // Two-phase ISR TX: (1) send SOP' GoodCRC now, (2) on TXMSGSENT send the
    // VDM ACK response.  See irq_handle_txmsgsent for phase 2.
    if (is_sop1) {
        // If a real cable e-marker was detected (TXMSGDISC on a previous
        // attempt), stay silent so we don't collide on the CC wire.
        if (cable_emu_disabled_) return;

        // If TX is busy (e.g. SOP GoodCRC is still being transmitted),
        // queue the SOP' request instead of silently dropping it.  The
        // queued request will be serviced from irq_handle_txmsgsent()
        // once the current TX completes.
        if (goodcrc_tx_pending_ || cable_goodcrc_tx_pending_ ||
            cable_response_tx_pending_) {
            // Save SOP' data for deferred processing after TX completes.
            if (!sop1_deferred_pending_) {
                sop1_deferred_size_ = size;
                sop1_deferred_hdr_ = raw_hdr;
                // cable_emu_build_response reads from dma_buf_ which will
                // be overwritten once RX DMA is re-armed. Copy the first
                // 30 bytes (max PD message) so we can build the response
                // later. dma_buf_ is still valid here because RX DMA
                // restart (above) doesn't clear the buffer contents.
                uint16_t copy_len = (size <= 30) ? size : 30;
                memcpy(sop1_deferred_buf_, dma_buf_, copy_len);
                sop1_deferred_pending_ = true;
            }
            return;
        }

        uint16_t resp_sz = cable_emu_build_response(
            dma_buf_, size, cable_tx_buf_, sizeof(cable_tx_buf_));
        if (resp_sz == 0) return;   // not Discover_Identity — drop

        cable_tx_size_ = resp_sz;

        // Phase 1: send SOP' GoodCRC (CablePlug=1 in bit 5).
        uint8_t msg_id_bits = (raw_hdr >> 9) & 0x07;
        uint16_t goodcrc_hdr = 0x0001            // msg_type = GoodCRC
                             | (1 << 5)           // CablePlug = 1
                             | (raw_hdr & 0x00C0) // spec_revision
                             | (msg_id_bits << 9);

        static uint8_t sop1_gcrc_buf[2];
        sop1_gcrc_buf[0] = goodcrc_hdr & 0xFF;
        sop1_gcrc_buf[1] = goodcrc_hdr >> 8;

        LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);
        LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
            reinterpret_cast<uint32_t>(sop1_gcrc_buf),
            reinterpret_cast<uint32_t>(&ucpd_->TXDR),
            LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

        LL_UCPD_WriteTxOrderSet(ucpd_, LL_UCPD_ORDERED_SET_SOP1);
        LL_UCPD_SetTxMode(ucpd_, LL_UCPD_TXMODE_NORMAL);
        LL_UCPD_WriteTxPaySize(ucpd_, 2);
        LL_UCPD_SendMessage(ucpd_);
        cable_goodcrc_tx_pending_ = true;
        return;
    }

    // GoodCRC — already handled by hardware TXMSGSENT
    if (is_goodcrc) { ucpd_goodcrc_filtered++; return; }

    // SOP message: GoodCRC was already sent in the fast-path above.
    // Mark as pending for PRL to pick up.
    if (is_sop) {
        rx_msg_pending_ = true;
    }
}

volatile uint32_t ucpd_tx_ok = 0, ucpd_tx_fail = 0;

void UcpdDriver::irq_handle_txmsgsent() {
    ucpd_tx_ok++;
    LL_UCPD_ClearFlag_TxMSGSENT(ucpd_);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

    // SOP GoodCRC — hot path, checked first (every SOP message hits this).
    if (goodcrc_tx_pending_) {
        goodcrc_tx_pending_ = false;

        // Service deferred SOP' cable_emu request that was queued because
        // TX was busy when the SOP' Discover_Identity arrived.
        if (sop1_deferred_pending_ && !cable_emu_disabled_) {
            sop1_deferred_pending_ = false;

            uint16_t resp_sz = cable_emu_build_response(
                sop1_deferred_buf_, sop1_deferred_size_,
                cable_tx_buf_, sizeof(cable_tx_buf_));
            if (resp_sz > 0) {
                cable_tx_size_ = resp_sz;

                // Send SOP' GoodCRC for the deferred Discover_Identity.
                uint8_t msg_id_bits = (sop1_deferred_hdr_ >> 9) & 0x07;
                uint16_t gcrc_hdr = 0x0001
                                  | (1 << 5)
                                  | (sop1_deferred_hdr_ & 0x00C0)
                                  | (msg_id_bits << 9);

                static uint8_t sop1_def_gcrc[2];
                sop1_def_gcrc[0] = gcrc_hdr & 0xFF;
                sop1_def_gcrc[1] = gcrc_hdr >> 8;

                LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
                    reinterpret_cast<uint32_t>(sop1_def_gcrc),
                    reinterpret_cast<uint32_t>(&ucpd_->TXDR),
                    LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
                LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, 2);
                LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

                LL_UCPD_WriteTxOrderSet(ucpd_, LL_UCPD_ORDERED_SET_SOP1);
                LL_UCPD_SetTxMode(ucpd_, LL_UCPD_TXMODE_NORMAL);
                LL_UCPD_WriteTxPaySize(ucpd_, 2);
                LL_UCPD_SendMessage(ucpd_);
                cable_goodcrc_tx_pending_ = true;
            }
        }
        return;
    }

    // Cable e-marker phase 1 complete: SOP' GoodCRC was sent.
    // Now fire phase 2: the actual Discover_Identity VDM ACK response.
    if (cable_goodcrc_tx_pending_) {
        cable_goodcrc_tx_pending_ = false;

        LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_2,
            reinterpret_cast<uint32_t>(cable_tx_buf_),
            reinterpret_cast<uint32_t>(&ucpd_->TXDR),
            LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
        LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, cable_tx_size_);
        LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

        LL_UCPD_WriteTxOrderSet(ucpd_, LL_UCPD_ORDERED_SET_SOP1);
        LL_UCPD_SetTxMode(ucpd_, LL_UCPD_TXMODE_NORMAL);
        LL_UCPD_WriteTxPaySize(ucpd_, cable_tx_size_);
        LL_UCPD_SendMessage(ucpd_);
        cable_response_tx_pending_ = true;
        return;
    }

    // Cable e-marker phase 2 complete: VDM response was sent.
    if (cable_response_tx_pending_) {
        cable_response_tx_pending_ = false;
        ucpd_cable_emu_sent++;
        return;
    }

    tx_done_ = true;
    tx_success_ = true;

    auto expected = pd::TCPC_TRANSMIT_STATUS::SENDING;
    port_.tcpc_tx_status.compare_exchange_strong(expected,
        pd::TCPC_TRANSMIT_STATUS::SUCCEEDED);
}

void UcpdDriver::irq_handle_txmsgdisc() {
    ucpd_tx_fail++;
    LL_UCPD_ClearFlag_TxMSGDISC(ucpd_);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

    if (goodcrc_tx_pending_) { goodcrc_tx_pending_ = false; return; }
    if (cable_goodcrc_tx_pending_ || cable_response_tx_pending_) {
        cable_goodcrc_tx_pending_ = false;
        cable_response_tx_pending_ = false;
        ucpd_cable_emu_fail++;
        cable_emu_disabled_ = true;
        return;
    }

    tx_done_ = true;
    tx_success_ = false;

    auto expected = pd::TCPC_TRANSMIT_STATUS::SENDING;
    port_.tcpc_tx_status.compare_exchange_strong(expected,
        pd::TCPC_TRANSMIT_STATUS::FAILED);
}

void UcpdDriver::irq_handle_txmsgabt() {
    ucpd_tx_fail++;
    LL_UCPD_ClearFlag_TxMSGABT(ucpd_);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_2);

    if (goodcrc_tx_pending_) { goodcrc_tx_pending_ = false; return; }
    if (cable_goodcrc_tx_pending_ || cable_response_tx_pending_) {
        cable_goodcrc_tx_pending_ = false;
        cable_response_tx_pending_ = false;
        ucpd_cable_emu_fail++;
        cable_emu_disabled_ = true;
        return;
    }

    tx_done_ = true;
    tx_success_ = false;

    auto expected = pd::TCPC_TRANSMIT_STATUS::SENDING;
    port_.tcpc_tx_status.compare_exchange_strong(expected,
        pd::TCPC_TRANSMIT_STATUS::FAILED);
}

void UcpdDriver::irq_handle_rxhrstdet() {
    ucpd_rxhrstdet_count++;
    LL_UCPD_ClearFlag_RxHRST(ucpd_);

    // Cancel any in-flight cable_emu TX and reset message ID counter.
    cable_goodcrc_tx_pending_ = false;
    cable_response_tx_pending_ = false;
    cable_emu_reset();
    // Do NOT clear cable_emu_disabled_ — same cable, same e-marker.

    // Clear pending RX/TX flags — hard reset invalidates all in-flight state.
    rx_msg_pending_ = false;
    sop1_deferred_pending_ = false;
    goodcrc_tx_pending_ = false;

    port_.notify_prl(pd::MsgToPrl_TcpcHardReset{});
}

void UcpdDriver::irq_handle_hrstsent() {
    LL_UCPD_ClearFlag_TxHRSTSENT(ucpd_);
    cable_goodcrc_tx_pending_ = false;
    cable_response_tx_pending_ = false;
    cable_emu_reset();

    // Clear pending RX/TX flags — hard reset invalidates all in-flight state.
    rx_msg_pending_ = false;
    sop1_deferred_pending_ = false;
    goodcrc_tx_pending_ = false;

    hr_sent_ = true;
}

void UcpdDriver::irq_handle_typec_event() {
    LL_UCPD_ClearFlag_TypeCEventCC1(ucpd_);
    LL_UCPD_ClearFlag_TypeCEventCC2(ucpd_);
    // TC layer will re-scan CC on next axxpd_run() tick
}
