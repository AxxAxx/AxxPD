// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

// AxxPD firmware — wires the vendored pdsink PD stack to the STM32G4
// UCPD peripheral.
//
// Memory model: every pdsink object lives in a placement-new'd aligned
// static buffer — no malloc, no heap, deterministic RAM layout. Each
// buffer is sized to its type and 8-byte aligned for ARM ABI compliance.
//
// Wiring order follows pdsink examples (each object captures references
// to the ones it depends on, so construction order matters):
//   Port -> Driver -> Task -> DPM -> PRL -> PE -> TC -> start()
//
// Two tick paths exist:
//   axxpd_run()     — full tick: PD state machine + CLI. Called from the
//                     main super-loop. Always active, even during the
//                     pre-enable_tick stabilization window at boot.
//   axxpd_tick_pd() — PD-only tick: no CLI, no CDC, no UART. Safe to
//                     call from SysTick ISR or blocking-wait idle
//                     callbacks (LCD DMA, INA228 I2C). Gated by
//                     s_pd_tick_enabled so it stays dormant until the
//                     5 V contract is confirmed and axxpd_enable_tick()
//                     is called.
// Both paths funnel into pd::Task::set_event(), which has an internal
// IS_IN_TICK reentrancy guard — concurrent calls are safe.

#include "axxpd_main.h"
#include "ucpd_driver.h"
#include "app_dpm.h"
#include "cli.h"

#include "pd/port.h"
#include "pd/task.h"
#include "pd/prl.h"
#include "pd/pe.h"
#include "pd/tc.h"
#include "pd/data_objects.h"
#include "pd/pe_defs.h"
#include "pd/utils/dobj_utils.h"

#include "stm32g4xx_ll_ucpd.h"
#include "stm32g4xx_hal.h"

#include <new>
#include <cstring>
#include <cstdio>

// USB CDC debug output
extern "C" {
#include "usbd_cdc_if.h"
}

static void dbg(const char* msg) {
    CDC_Transmit_Blocking(reinterpret_cast<const uint8_t*>(msg),
                          static_cast<uint16_t>(strlen(msg)), 100);
}

// ---------------------------------------------------------------------------
// Static object storage (placement-new, no heap)
// ---------------------------------------------------------------------------
// Each pdsink object gets an aligned byte buffer sized to its type, plus
// a typed pointer set during axxpd_init(). The pointers start null and
// serve as "initialized?" guards throughout the file.

static __attribute__((aligned(8))) uint8_t port_mem[sizeof(pd::Port)];
static pd::Port* s_port_ptr = nullptr;   // shared state: caps, RDO, flags

static __attribute__((aligned(8))) uint8_t driver_mem[sizeof(UcpdDriver)];
static __attribute__((aligned(8))) uint8_t task_mem[sizeof(pd::Task)];
static __attribute__((aligned(8))) uint8_t dpm_mem[sizeof(AppDPM)];
static __attribute__((aligned(8))) uint8_t prl_mem[sizeof(pd::PRL)];
static __attribute__((aligned(8))) uint8_t pe_mem[sizeof(pd::PE)];
static __attribute__((aligned(8))) uint8_t tc_mem[sizeof(pd::TC)];

static UcpdDriver* s_driver = nullptr;   // UCPD HAL / LL bridge
static pd::Task*   s_task   = nullptr;   // tick dispatcher (owns event loop)
static AppDPM*     s_dpm    = nullptr;   // device policy manager (voltage requests)
static pd::PRL*    s_prl    = nullptr;   // protocol layer (message framing)
static pd::PE*     s_pe     = nullptr;   // policy engine (PD state machine)
static pd::TC*     s_tc     = nullptr;   // type-C connection manager

// ---------------------------------------------------------------------------
// C-callable API
// ---------------------------------------------------------------------------

extern "C" void axxpd_init(void) {
    dbg("\r\n[AxxPD] boot\r\n");

    // --- Placement-new construction (order matters) ---
    // Port is the shared state bag; every other object takes a Port& ref.
    s_port_ptr = new (port_mem) pd::Port();
    pd::Port& s_port = *s_port_ptr;

    // Driver wraps STM32 UCPD LL calls; Task owns the event-dispatch loop.
    // DPM, PRL, PE, TC are the four pdsink "layers" — wired in dependency
    // order so each constructor receives valid references.
    s_driver = new (driver_mem) UcpdDriver(s_port);
    s_task   = new (task_mem)   pd::Task(s_port, *s_driver);
    s_dpm    = new (dpm_mem)    AppDPM(s_port);
    s_prl    = new (prl_mem)    pd::PRL(s_port, *s_driver);
    s_pe     = new (pe_mem)     pd::PE(s_port, *s_dpm, *s_prl, *s_driver);
    s_tc     = new (tc_mem)     pd::TC(s_port, *s_driver);

    // start() connects the layers inside Task and kicks the state machine.
    s_task->start(*s_tc, *s_dpm, *s_pe, *s_prl, *s_driver);

    // Suppress pdsink's automatic EPR entry.  PE_SNK_Ready::on_enter_state
    // auto-queues EPR_MODE_ENTRY on the very first Ready entry (~200 ms
    // after boot).  This premature attempt fails on chargers that need
    // settling time (Anker A2697), causing soft-resets that poison all
    // subsequent EPR retries.  We block it here and let the boot selector
    // (1 s timer) or main-loop fallback control EPR timing explicitly.
    s_port.pe_flags.set(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);

    // Boot at 5 V — safest voltage that establishes a contract without
    // imposing anything on VBUS.
    s_dpm->trigger_any(5000, 0);

    // Kick the task to trigger initial CC scan (charger may already be connected)
    s_port.wakeup();

    // Attach the interactive CLI. This prints the banner + prompt.
    cli_init(s_port_ptr, s_pe, s_dpm, s_driver);
}

// PRL debug counters — defined in prl.cpp, exposed here for the CLI
// `stats` command to read. Not used in normal operation.
namespace pd {
    extern volatile uint32_t prl_rx_dup;
    extern volatile uint32_t prl_rx_new;
    extern volatile int8_t prl_rx_last_stored;
    extern volatile int8_t prl_rx_last_incoming;
    extern volatile uint8_t prl_rx_last_msgtype;
}

// Gate for axxpd_tick_pd(). Stays false during the boot stabilization
// window (~500 ms) to prevent SysTick / idle-callback ticks from firing
// before the 5 V contract is confirmed and LCD init is complete.
// Set to true by axxpd_enable_tick() in main.c after stabilization.
static volatile bool s_pd_tick_enabled = false;

// OPC (Operation Complete) tracking — set when axxpd_request_voltage() is
// called, cleared once the contract lands within 15% of the target voltage.
static volatile uint8_t s_opc_pending = 0;
static uint32_t s_opc_target_mv = 0;

extern "C" void axxpd_tick_pd(void) {
    // PD-only tick — safe to call from SysTick ISR, LCD DMA idle
    // callback, or INA228 I2C idle callback. Only drives the PD state
    // machine (TC, PRL, PE). No CLI, no CDC, no UART — those peripherals
    // are not reentrant and must not run from interrupt context.
    //
    // pd::Task::set_event() has an IS_IN_TICK reentrancy guard, so
    // overlapping calls from ISR + main-loop axxpd_run() are safe:
    // the second caller sees the flag and returns immediately.
    if (s_pd_tick_enabled && s_task) {
        s_port_ptr->timers.set_time(HAL_GetTick());
        s_task->set_event(pd::Task::EVENT_TIMER_MSK | pd::Task::EVENT_WAKEUP_MSK);
    }
}

extern "C" void axxpd_enable_tick(void) {
    // Called once from main.c after the 5 V contract is confirmed.
    // Unlocks axxpd_tick_pd() for SysTick and idle-callback callers.
    s_pd_tick_enabled = true;
}

extern "C" void axxpd_run(void) {
    // Full tick: PD state machine + CLI servicing.
    // Called from the main super-loop (and during the boot stabilization
    // window). NOT gated by s_pd_tick_enabled — this is the primary tick
    // path and must work from the moment axxpd_init() returns, even
    // before axxpd_enable_tick() is called.
    if (s_task) {
        s_port_ptr->timers.set_time(HAL_GetTick());
        s_task->set_event(pd::Task::EVENT_TIMER_MSK | pd::Task::EVENT_WAKEUP_MSK);
    }

    // Interactive CLI: services USB CDC RX, emits async contract/EPR
    // events, and handles the opt-in heartbeat (`hb on`). Safe here
    // because the main loop is non-interrupt context.
    cli_poll();
}

extern "C" void axxpd_ucpd_irq(void) {
    // UCPD1 ISR — routed here from UCPD1_IRQHandler in stm32g4xx_it.c.
    // Dispatch order matters:
    //   1. TX completion — lets PRL retire the outgoing message first.
    //   2. Hard-reset RX/TX — must be handled before ordinary RX to avoid
    //      interpreting a post-reset message under stale PRL state.
    //   3. RX message complete — enqueues the message for PRL; comes after
    //      TX to prevent the RX handler from triggering a new TX while the
    //      previous TX completion is still pending (avoids reentrancy).
    //   4. Type-C CC events — lowest urgency; connection state changes.
    //   5. Housekeeping flags — RXORDDET, RXOVR, HRSTDISC, TXUND.
    if (!s_driver) return;

    uint32_t sr  = UCPD1->SR;
    uint32_t imr = UCPD1->IMR;
    uint32_t active = sr & imr;

    // 1) TX completion
    if (active & UCPD_SR_TXMSGSENT) s_driver->irq_handle_txmsgsent();
    if (active & UCPD_SR_TXMSGDISC) s_driver->irq_handle_txmsgdisc();
    if (active & UCPD_SR_TXMSGABT)  s_driver->irq_handle_txmsgabt();

    // 2) Hard reset
    if (active & UCPD_SR_RXHRSTDET) s_driver->irq_handle_rxhrstdet();
    if (active & UCPD_SR_HRSTSENT)  s_driver->irq_handle_hrstsent();

    // 3) RX message complete
    if (active & UCPD_SR_RXMSGEND)  s_driver->irq_handle_rxmsgend();

    // 4) Type-C CC state change
    if (active & (UCPD_SR_TYPECEVT1 | UCPD_SR_TYPECEVT2))
        s_driver->irq_handle_typec_event();

    // 5) Low-priority housekeeping flags
    if (active & UCPD_SR_RXORDDET)  LL_UCPD_ClearFlag_RxOrderSet(UCPD1);
    if (active & UCPD_SR_RXOVR) {
        extern volatile uint32_t ucpd_rxovr_count;
        ucpd_rxovr_count++;
        LL_UCPD_ClearFlag_RxOvr(UCPD1);
    }
    if (active & UCPD_SR_HRSTDISC) {
        LL_UCPD_ClearFlag_TxHRSTDISC(UCPD1);
        s_driver->irq_handle_hrstsent(); // Treat HR discard as completion
    }
    if (active & UCPD_SR_TXUND)     LL_UCPD_ClearFlag_TxUND(UCPD1);
}

// ---------------------------------------------------------------------------
// extern "C" accessor block — bridges pdsink C++ state to C callers
// (ui.c, graph.c, main.c). All reads are live from the pdsink objects.
// ---------------------------------------------------------------------------

extern "C" uint8_t axxpd_get_active_pdo_index(void) {
    if (!s_port_ptr) return 0;
    uint32_t rdo = s_port_ptr->rdo_contracted;
    if (rdo == 0) return 0;
    pd::RDO_ANY rdo_any{rdo};
    return rdo_any.obj_position; // 1-based
}

extern "C" float axxpd_get_negotiated_a(void) {
    if (!s_port_ptr) return 0.0f;
    uint32_t rdo = s_port_ptr->rdo_contracted;
    if (rdo == 0) return 0.0f;

    pd::RDO_ANY rdo_any{rdo};
    uint8_t pos = rdo_any.obj_position;
    if (pos == 0 || pos > s_port_ptr->source_caps.size()) return 0.0f;

    uint32_t pdo = s_port_ptr->source_caps[pos - 1];
    uint8_t pdo_type = static_cast<uint8_t>((pdo >> 30) & 0x3);

    if (pdo_type == pd::PDO_TYPE::FIXED) {
        pd::PDO_FIXED pdo_fixed{pdo};
        return static_cast<float>(pdo_fixed.max_current * 10u) / 1000.0f;
    }
    if (pdo_type == pd::PDO_TYPE::AUGMENTED) {
        uint8_t subtype = static_cast<uint8_t>((pdo >> 28) & 0x3);
        if (subtype == pd::PDO_AUGMENTED_SUBTYPE::SPR_PPS) {
            pd::RDO_PPS rdo_pps{rdo};
            return static_cast<float>(rdo_pps.operating_current * 50u) / 1000.0f;
        }
        if (subtype == pd::PDO_AUGMENTED_SUBTYPE::EPR_AVS) {
            pd::RDO_AVS rdo_avs{rdo};
            return static_cast<float>(rdo_avs.operating_current * 50u) / 1000.0f;
        }
    }
    return 0.0f;
}

extern "C" float axxpd_get_negotiated_v(void) {
    if (!s_port_ptr) return 0.0f;
    uint32_t rdo = s_port_ptr->rdo_contracted;
    if (rdo == 0) return 0.0f;

    pd::RDO_ANY rdo_any{rdo};
    uint8_t pos = rdo_any.obj_position; // 1-based
    if (pos == 0 || pos > s_port_ptr->source_caps.size()) return 0.0f;

    uint32_t pdo = s_port_ptr->source_caps[pos - 1];
    uint8_t pdo_type = static_cast<uint8_t>((pdo >> 30) & 0x3);

    if (pdo_type == pd::PDO_TYPE::FIXED) {
        // FIXED PDO: voltage is bits[19:10], 50 mV step
        pd::PDO_FIXED pdo_fixed{pdo};
        uint32_t mv = pdo_fixed.voltage * 50u;
        return static_cast<float>(mv) / 1000.0f;
    }

    if (pdo_type == pd::PDO_TYPE::AUGMENTED) {
        uint8_t subtype = static_cast<uint8_t>((pdo >> 28) & 0x3);
        if (subtype == pd::PDO_AUGMENTED_SUBTYPE::SPR_PPS) {
            // PPS RDO: output_voltage bits[20:9], 20 mV step
            pd::RDO_PPS rdo_pps{rdo};
            uint32_t mv = rdo_pps.output_voltage * 20u;
            return static_cast<float>(mv) / 1000.0f;
        }
        if (subtype == pd::PDO_AUGMENTED_SUBTYPE::EPR_AVS) {
            // EPR AVS RDO: output_voltage bits[20:9], 25 mV step
            // (least 2 bits are 0 per spec, giving effective 100 mV resolution)
            pd::RDO_AVS rdo_avs{rdo};
            uint32_t mv = rdo_avs.output_voltage * 25u;
            return static_cast<float>(mv) / 1000.0f;
        }
        // SPR_AVS: no direct voltage in RDO — fall back to PDO max voltage
        if (subtype == pd::PDO_AUGMENTED_SUBTYPE::SPR_AVS) {
            pd::PDO_SPR_AVS pdo_avs{pdo};
            uint32_t mv = (pdo_avs.max_current_20v > 0) ? 20000u : 15000u;
            return static_cast<float>(mv) / 1000.0f;
        }
    }

    return 0.0f;
}

extern "C" uint8_t axxpd_is_pps_active(void) {
    if (!s_pe) return 0;
    return s_pe->is_in_pps_contract() ? 1u : 0u;
}

extern "C" uint32_t axxpd_get_pps_mv(void) {
    if (!s_port_ptr || !s_pe) return 0;
    if (!s_pe->is_in_pps_contract()) return 0;
    uint32_t rdo = s_port_ptr->rdo_contracted;
    if (rdo == 0) return 0;
    // PPS RDO: output_voltage bits[20:9], 20 mV step
    pd::RDO_PPS rdo_pps{rdo};
    return rdo_pps.output_voltage * 20u;
}

extern "C" uint32_t axxpd_get_pps_ma(void) {
    if (!s_port_ptr || !s_pe) return 0;
    if (!s_pe->is_in_pps_contract()) return 0;
    uint32_t rdo = s_port_ptr->rdo_contracted;
    if (rdo == 0) return 0;
    // PPS RDO: operating_current bits[6:0], 50 mA step
    pd::RDO_PPS rdo_pps{rdo};
    return rdo_pps.operating_current * 50u;
}

extern "C" uint8_t axxpd_is_epr_active(void) {
    if (!s_pe) return 0;
    return s_pe->is_in_epr_mode() ? 1u : 0u;
}

extern "C" uint8_t axxpd_is_src_epr_capable(void) {
    if (!s_port_ptr || s_port_ptr->source_caps.size() == 0) return 0;
    return ((s_port_ptr->source_caps[0] >> 23) & 1) ? 1u : 0u;
}

extern "C" uint8_t axxpd_get_src_pdos(uint32_t *out, uint8_t max) {
    if (!s_port_ptr || !out || max == 0) return 0;
    const auto& caps = s_port_ptr->source_caps;
    uint8_t count = 0;
    for (size_t i = 0; i < caps.size() && count < max; ++i) {
        out[count++] = caps[i];
    }
    return count;
}

extern "C" uint8_t axxpd_get_epr_src_pdos(uint32_t *out, uint8_t max) {
    if (!s_port_ptr || !out || max == 0) return 0;
    const auto& caps = s_port_ptr->source_caps;
    // EPR PDOs occupy slots 8-11 (0-indexed: 7-10). Per spec and cli.cpp
    // convention: indices >= 7 are EPR entries.
    uint8_t count = 0;
    for (size_t i = 7; i < caps.size() && count < max; ++i) {
        out[count++] = caps[i];
    }
    return count;
}

extern "C" uint8_t axxpd_get_cable_info(uint8_t *type, uint8_t *max_current,
                                          uint8_t *max_voltage, uint8_t *usb_ss) {
    // Cable e-marker reading requires VCONN sourcing, which is not implemented
    // until the VCONN hardware revision. Return "not detected" for all fields.
    if (type)        *type        = 0;
    if (max_current) *max_current = 0;
    if (max_voltage) *max_voltage = 0;
    if (usb_ss)      *usb_ss      = 0;
    return 0; // 0 = cable info not available
}

extern "C" void axxpd_request_voltage(uint32_t mv, uint32_t ma) {
    if (!s_dpm) return;
    /* Suppress software fault detection for 1 s.  During a PD voltage
     * transition the VBUS rail swings through intermediate voltages and
     * may trigger the software OVP/UVP window in buttons.c / main.c.
     * The 1 s window covers: PD negotiation (~100 ms) + source VBUS
     * ramp (~30 ms per spec) + inrush / cap settling.
     * Hardware protection (LTC4368 OVP/OCP latch) is NOT affected and
     * remains active at all times. */
    extern volatile uint32_t g_fault_suppress_until;
    g_fault_suppress_until = HAL_GetTick() + 1000U;
    s_opc_pending  = 1;
    s_opc_target_mv = mv;
    if (s_port_ptr) s_port_ptr->pe_flags.set(pd::PE_FLAG::HAS_EXPLICIT_CONTRACT);

    /* Smart PDO routing: trigger_any handles Fixed + PPS but not AVS.
     * First check if PPS can cover this voltage. If not, try AVS. */
    bool use_avs = false;
    if (s_port_ptr) {
        /* Find PPS max voltage */
        uint32_t pps_max = 0;
        uint32_t avs_min = 0, avs_max = 0;
        for (size_t i = 0; i < s_port_ptr->source_caps.size(); i++) {
            uint32_t pdo = s_port_ptr->source_caps[i];
            if (pdo == 0) continue;
            uint32_t type = (pdo >> 30) & 0x3;
            uint32_t sub  = (pdo >> 28) & 0x3;
            if (type == 3 && sub == 0) {
                uint32_t hi = ((pdo >> 17) & 0xFF) * 100;
                if (hi > pps_max) pps_max = hi;
            } else if (type == 3 && sub == 1) {
                avs_min = ((pdo >> 8) & 0xFF) * 100;
                avs_max = ((pdo >> 17) & 0x1FF) * 100;
            }
        }
        /* Only use AVS if voltage exceeds PPS range AND fits AVS */
        if (mv > pps_max && avs_max > 0 && mv >= avs_min && mv <= avs_max) {
            s_dpm->trigger_variant(pd::PDO_VARIANT::APDO_EPR_AVS, mv, ma);
            if (s_pe && !s_pe->is_in_epr_mode()) {
                cli_set_epr_intent(true);
                s_port_ptr->pe_flags.clear(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
                s_port_ptr->dpm_requests.set(pd::DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
                s_port_ptr->wakeup();
            }
            use_avs = true;
        }
    }
    if (!use_avs) {
        s_dpm->trigger_any(mv, ma);
    }
}

extern "C" void axxpd_ensure_contract_flag(void) {
    if (!s_port_ptr) return;
    /* If we have a valid contract (rdo_contracted != 0) but HAS_EXPLICIT_CONTRACT
     * is not set, force-set it. This happens after EPR hard resets where the PE
     * re-negotiates a 5V contract but doesn't always set the flag. Without this
     * flag, trigger_by_position() silently drops all PDO switch requests. */
    if (s_port_ptr->rdo_contracted != 0 &&
        !s_port_ptr->pe_flags.test(pd::PE_FLAG::HAS_EXPLICIT_CONTRACT)) {
        s_port_ptr->pe_flags.set(pd::PE_FLAG::HAS_EXPLICIT_CONTRACT);
    }
}

extern "C" uint8_t axxpd_is_opc_done(void) {
    if (!s_opc_pending) return 1;
    // When target_mv==0 (position-based request), just check for any valid contract
    if (s_opc_target_mv == 0) {
        if (axxpd_get_active_pdo_index() > 0) {
            s_opc_pending = 0;
            return 1;
        }
        return 0;
    }
    // Check if a valid contract has been established close to the target
    if (axxpd_get_active_pdo_index() > 0) {
        float v = axxpd_get_negotiated_v();
        uint32_t actual_mv = static_cast<uint32_t>(v * 1000.0f + 0.5f);
        uint32_t target = s_opc_target_mv;
        // Within 15% of target
        uint32_t margin = target / 7;  // ~14.3%, close enough to 15%
        if (actual_mv >= target - margin && actual_mv <= target + margin) {
            s_opc_pending = 0;
            return 1;
        }
    }
    return 0;
}

extern "C" void axxpd_request_pdo_position(uint8_t position) {
    if (!s_dpm) return;
    extern volatile uint32_t g_fault_suppress_until;
    g_fault_suppress_until = HAL_GetTick() + 2000U;
    s_opc_pending = 1;
    s_opc_target_mv = 0;
    // Force-set HAS_EXPLICIT_CONTRACT — after failed EPR entry cycles,
    // clear_all() wipes this flag and request_new_power_level() silently
    // drops the request.  We know a contract exists because the MCU is
    // running (powered from VBUS via negotiated 5V).
    if (s_port_ptr) s_port_ptr->pe_flags.set(pd::PE_FLAG::HAS_EXPLICIT_CONTRACT);
    s_dpm->trigger_by_position(position);
    /* EPR slots (8+) need EPR mode active — trigger entry if not already */
    if (position >= 8 && s_pe && !s_pe->is_in_epr_mode()) {
        cli_set_epr_intent(true);
        s_port_ptr->pe_flags.clear(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
        s_port_ptr->dpm_requests.set(pd::DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
        s_port_ptr->wakeup();
    }
}

extern "C" void axxpd_enable_epr(void) {
    if (!s_port_ptr) return;
    // Only attempt EPR if source actually advertises EPR capability.
    // Without this check, the ErrorRecovery timeout loop fires every 30s
    // on non-EPR sources, causing CC resets that disrupt PDO switching.
    if (!axxpd_is_src_epr_capable()) return;
    cli_set_epr_intent(true);  // prevent cli_poll() from re-disabling EPR
    s_port_ptr->pe_flags.clear(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
    s_port_ptr->dpm_requests.set(pd::DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
    s_port_ptr->wakeup();
}

extern "C" void axxpd_disable_epr_intent(void) {
    cli_set_epr_intent(false);
}

extern "C" void axxpd_enable_cable_emu(void) {
    if (!s_driver) return;
    s_driver->enable_cable_emu();
}

extern "C" void axxpd_hard_reset(void) {
    // Trigger a PD hard reset by setting the PE flag that PE's state machine
    // checks. This mirrors the mechanism pdsink uses internally (pe.cpp:830).
    // Safe to call from main-loop context; the flag is atomic.
    if (!s_port_ptr) return;
    s_port_ptr->pe_flags.set(pd::PE_FLAG::PRL_HARD_RESET_PENDING);
    s_port_ptr->wakeup();
}
