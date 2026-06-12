// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

// SCPI-style command interface for AxxPD (STM32G4 UCPD USB-PD sink).
//
// Accepts IEEE 488.2 common commands (*IDN?, *RST, *CLS) plus a small set
// of SCPI subsystems:
//   :SOURce   — program the requested PD power level
//   :MEASure  — read actual VBUS voltage / current (stubbed to 0 for now)
//   :PD       — PD-specific: mode (SPR/EPR), PDO enumeration, contract info
//   :SYSTem   — help + error queue
//
// Commands are case-insensitive, accept both short and long forms (SCPI
// convention: short form = the uppercase letters in the canonical spelling),
// and numeric arguments may carry V / mV / A / mA suffixes (default is the
// SCPI base unit: volts or amps).

#include "cli.h"

extern "C" {
    void LCD_Fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t c);
    void LCD_DrawPixel(int16_t x, int16_t y, uint16_t color);
    void LCD_PutStr(uint16_t x, uint16_t y, const char *s, void *f, uint16_t fg, uint16_t bg);
    extern uint8_t FONT_arial_17X18[];
    /* Buttons_Update / Buttons_GetEvent declared via ui.h -> buttons.h */
    void Buzzer_Update(void);
    void Buzzer_FreqSweep(void);
}
#include "ucpd_driver.h"

#include "pd/data_objects.h"
#include "pd/pe_defs.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_ucpd.h"
#include "main.h"   /* BLEED_CTRL pin — selftest tracks VBUS down-steps */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>

extern "C" {
#include "ina228.h"
#include "main.h"
#include "settings.h"
extern INA228_t g_ina;
extern INA228_Reading_t g_ina_reading;
extern volatile float   g_ntc_temp;
extern volatile uint8_t g_hw_fault;
extern volatile uint8_t g_fault_source;
void OVP_SetThreshold(uint32_t vbus_max_mv);
}

#include "axxpd_main.h"

// Helper: convert Celsius to display temperature respecting the settings toggle
static inline float temp_display(float celsius) {
    if (Settings_GetTempFahrenheit())
        return celsius * 9.0f / 5.0f + 32.0f;
    return celsius;
}

// -----------------------------------------------------------------------------
// Identification / version
// -----------------------------------------------------------------------------
// Bump FW_VERSION when you cut a new build. It appears in:
//   - the boot banner next to "AxxPD"
//   - the help header
//   - the *IDN? 4th field (per SCPI: Manufacturer,Model,Serial,FirmwareVersion)

#define FW_VERSION "0.2.0"

/* SCPI: Manufacturer,Model,Serial,FirmwareVersion. The serial field is the
 * MCU's 96-bit unique device ID (UID base 0x1FFF7590 on STM32G4) so every
 * unit identifies uniquely in multi-device test rigs. Built in cli_init(). */
static char IDN_STRING[64] =
    "AxxPD,USBPD-Sink,0," FW_VERSION;

static void idn_build(void) {
    const volatile uint32_t* uid = reinterpret_cast<const volatile uint32_t*>(0x1FFF7590UL);
    snprintf(IDN_STRING, sizeof(IDN_STRING), "AxxPD,USBPD-Sink,%08lX%08lX%08lX,%s",
             (unsigned long)uid[2], (unsigned long)uid[1], (unsigned long)uid[0],
             FW_VERSION);
}

// -----------------------------------------------------------------------------
// USB CDC output (replaces UART)
// -----------------------------------------------------------------------------

extern "C" {
#include "usbd_cdc_if.h"
#include "uart.h"
#include "ui.h"
void Output_Enable(void);
void Output_Disable(void);
extern volatile uint8_t g_output_enabled;
extern volatile uint8_t g_hw_fault;
extern volatile uint8_t g_fault_source;
extern volatile uint32_t g_fault_suppress_until;
}

static void out(const char* s) {
    size_t len = strlen(s);
    CDC_Transmit_Blocking(reinterpret_cast<const uint8_t*>(s),
                          static_cast<uint16_t>(len), 1000);
    UART_SendString(s);
}
static void out_char(char c) {
    uint8_t b = static_cast<uint8_t>(c);
    CDC_Transmit_Blocking(&b, 1, 10);
    char tmp[2] = {c, 0};
    UART_SendString(tmp);
}
static void outln() { out("\r\n"); }

// -----------------------------------------------------------------------------
// Shared diagnostic types (must be at namespace scope for extern linkage)
// -----------------------------------------------------------------------------

struct RxLogEntry { uint32_t ordset; uint16_t size; uint16_t hdr; uint32_t do0; };
extern volatile RxLogEntry rx_log[16];
extern volatile uint32_t rx_log_idx;

// pdsink PRL rx counter (incremented per unique non-dup received message).
// Used by the deferred-setpdo handler to wait for visible link activity
// after EPR entry before firing trigger_by_position.
namespace pd {
    extern volatile uint32_t prl_rx_new;
}

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

static pd::Port*  s_port   = nullptr;
static pd::PE*    s_pe     = nullptr;
static AppDPM*    s_dpm    = nullptr;
static UcpdDriver* s_driver = nullptr;

// Line buffer (SCPI allows semicolon-chained commands, but for this first pass
// we execute one per newline).
static constexpr size_t CLI_LINE_MAX = 128;
static char     line_buf[CLI_LINE_MAX];
static size_t   line_len = 0;

// Programmed (target) values — the last values set via :SOUR:VOLT / :CURR.
// These feed pdsink's DPM trigger on every apply. Initialised to 5 V / 0 A so
// *RST and boot both land there.
static uint32_t target_mv = 5000;
static uint32_t target_ma = 0;

// :SOURce:MODE preference — AUTO lets the DPM match any PDO type; FIXED / PPS
// / AVS restrict the match. Mirrors pdsink's PDO_VARIANT.
enum class SourceMode { AUTO, FIXED, PPS, AVS };
static SourceMode source_mode = SourceMode::AUTO;

// EPR user intent. See comment on cli_poll() — pdsink clears pe_flags on every
// Hard Reset so the CLI must re-pin EPR_AUTO_ENTER_DISABLED every tick whenever
// the user hasn't explicitly enabled EPR.
static bool user_wants_epr = false;  /* EPR on demand — auto-entry conflicts with boot PDO change */

void cli_set_epr_intent(bool enable) { user_wants_epr = enable; }

// EPR failure tracking — counts rapid EPR-enter/lose cycles for ErrorRecovery.
static uint32_t epr_quick_fail_count = 0;

// NOTE: An older "deferred setpdo" mechanism lived here — it armed an EPR
// slot request and waited for IN_EPR_MODE to settle before firing. It was
// removed when setpdo moved to calling trigger_by_position() directly
// alongside EPR_MODE_ENTRY (see cli.cpp in git history, commit 48aaa1a and
// its predecessors, for the Anker A2697 PS_RDY-quirk context).

// Deferred 'list all': EPR PDOs (slots 8-11, 28/36/48 V) only arrive in
// source_caps after EPR entry completes and the EPR_Source_Capabilities
// extended message has been received. When 'list all' is issued from SPR on
// an EPR-capable source, we kick off EPR entry and park this flag. cli_poll
// prints the full cap set once source_caps has expanded to >7, or bails
// with whatever is current after PENDING_LIST_ALL_TIMEOUT_MS.
static bool     pending_list_all = false;
static uint32_t pending_list_all_ms = 0;
static constexpr uint32_t PENDING_LIST_ALL_TIMEOUT_MS = 5000;

// Async event tracking
static uint32_t last_rdo_contracted = 0;
static bool     last_in_epr = false;
static size_t   last_ncaps  = 0;
static int      last_pe_state = -99;
static bool     events_enabled = true;  // enable unsolicited "#EVT" notifications
static bool     state_trace = false;    // :SYST:TRACE — print every PE state change

// Driver-level trace flag lives in ucpd_driver.cpp; when true, [CC]/[RX]/
// [TX]/[UCPD] debug prints go to the UART. Default false for clean output.
extern volatile bool axxpd_low_trace;

// -----------------------------------------------------------------------------
// Sequence execution state (Task S7)
// -----------------------------------------------------------------------------

struct SeqStep { uint32_t mv; uint32_t ma; uint32_t dwell_ms; };
static SeqStep seq_steps[16];
static uint8_t seq_count  = 0;
static volatile uint8_t seq_stop_flag = 0;  // set by 'seq stop' to abort run

// Non-blocking sequence execution state machine (FIX #13).
// Instead of blocking the main loop with HAL_Delay(), do_seq_run() arms these
// variables and seq_tick() (called from cli_poll()) advances one step per
// dwell period, keeping the main loop responsive for UI, protection, and WDG.
static bool     seq_running      = false;
static uint8_t  seq_current_step = 0;
static uint32_t seq_step_t0      = 0;

// -----------------------------------------------------------------------------
// Self-test machine state
//
// 'selftest' iterates the source's advertised PDOs and a few PPS/AVS sample
// points, requesting each and checking that the resulting contract matches.
// Runs asynchronously in cli_poll() so we don't block the main loop or
// pdsink's state machine. State stored in this struct.
// -----------------------------------------------------------------------------

struct SelftestStep {
    enum class Kind { FIXED, PPS, AVS } kind;
    uint8_t  pdo_pos;       // 1-based source position (only used for FIXED)
    uint32_t target_mv;     // 0 if N/A (FIXED uses PDO's own voltage)
    uint32_t target_ma;     // 0 = use PDO's max
    uint32_t expect_pos;    // contract should land on this position
    uint32_t expect_mv;     // contract should report this voltage
    uint32_t meas_mv;       // actual INA228 bus voltage reading after settle
    char     label[32];     // "PDO5 PPS 11.000V (max)" needs >20
    enum class Result { PENDING, PASS, FAIL } result;
    char     fail_reason[40];
};

static constexpr size_t SELFTEST_MAX_STEPS = 40;
static constexpr size_t SELFTEST_RANDOM_STEPS = 5;
static SelftestStep selftest_steps[SELFTEST_MAX_STEPS];
static size_t   selftest_count = 0;
static size_t   selftest_idx = 0;

// Two-step confirmation gate: 'selftest' prints a safety warning and parks
// here; the very next line typed must be exactly "OK" (case-insensitive) to
// proceed. Any other input aborts. Prevents an accidental 'selftest' (or a
// pasted command stream) from sweeping VBUS up to 28 V into a live load.
static bool     selftest_pending_confirm = false;
static enum class SelftestState {
    IDLE,
    PRE_EPR,        // entering EPR before building steps so EPR slots are visible
    BUILD,          // build step list from current source_caps
    REQUEST,        // issue trigger for current step
    WAIT_CONTRACT,  // wait for contract to match expected
    NEXT,           // settle delay then advance
    REPORT,         // print summary + return to 5 V
    FALLBACK_5V     // wait for the post-test 5 V contract
} selftest_state = SelftestState::IDLE;
static uint32_t selftest_step_t0 = 0;
static constexpr uint32_t SELFTEST_STEP_TIMEOUT_MS = 6000; // generous - 28V can take 3+s
static uint32_t selftest_pre_rdo = 0;

// -----------------------------------------------------------------------------
// Error queue (SCPI-compliant: code + message, FIFO up to ERR_QUEUE_LEN)
// -----------------------------------------------------------------------------

struct ErrEntry { int code; const char* msg; };
static constexpr size_t ERR_QUEUE_LEN = 8;
static ErrEntry err_queue[ERR_QUEUE_LEN];
static size_t   err_head = 0;
static size_t   err_tail = 0;

static void err_push(int code, const char* msg) {
    size_t next = (err_tail + 1) % ERR_QUEUE_LEN;
    if (next == err_head) return;  // full, drop silently (SCPI: set queue overflow)
    err_queue[err_tail] = { code, msg };
    err_tail = next;
}
static bool err_pop(ErrEntry* e) {
    if (err_head == err_tail) return false;
    *e = err_queue[err_head];
    err_head = (err_head + 1) % ERR_QUEUE_LEN;
    return true;
}
static void err_clear() { err_head = err_tail = 0; }

// -----------------------------------------------------------------------------
// Parsing helpers
// -----------------------------------------------------------------------------

static char to_upper(char c) {
    return (c >= 'a' && c <= 'z') ? char(c - 'a' + 'A') : c;
}
static void uppercase(char* s) { for (; *s; ++s) *s = to_upper(*s); }

// Trim leading / trailing whitespace in place.
static char* trim(char* s) {
    while (*s == ' ' || *s == '\t') s++;
    char* end = s + strlen(s);
    while (end > s && (end[-1] == ' ' || end[-1] == '\t' ||
                       end[-1] == '\r' || end[-1] == '\n')) {
        end--;
    }
    *end = 0;
    return s;
}

// SCPI short/long matcher. Example: match(tok, "VOLT", "VOLTAGE") returns true
// for "VOLT" or "VOLTAGE" (case-insensitive — caller uppercases first).
static bool match(const char* tok, const char* shortf, const char* longf) {
    return strcmp(tok, shortf) == 0 || strcmp(tok, longf) == 0;
}
static bool match1(const char* tok, const char* only) {
    return strcmp(tok, only) == 0;
}

// Split a mnemonic path like ":SOUR:VOLT" into ["SOUR","VOLT"]. Returns count.
// Strips leading ':'. Upper-cases in place. Separates the query '?' suffix.
static int split_path(char* path, char* parts[6], bool* is_query) {
    *is_query = false;
    if (*path == ':') path++;
    size_t n = strlen(path);
    if (n > 0 && path[n-1] == '?') { *is_query = true; path[n-1] = 0; }
    int count = 0;
    char* p = path;
    while (*p && count < 6) {
        parts[count++] = p;
        while (*p && *p != ':') p++;
        if (*p == ':') { *p = 0; p++; }
    }
    for (int i = 0; i < count; i++) uppercase(parts[i]);
    return count;
}

// Parse a numeric value with optional unit suffix. Returns value in the base
// unit scaled by `base_scale` (e.g. 1000 if we want millivolts and input is V).
// Sets *out_scaled = value in the requested scale (mV or mA).
// Accepts: "28", "28.0", "28V", "0.5V", "500mV", "3A", "1500mA".
static bool parse_numeric_unit(const char* s, char base_letter /*'V' or 'A'*/,
                                uint32_t* out_milli) {
    if (!s || !*s) return false;
    char* end = nullptr;
    float val = strtof(s, &end);
    if (end == s) return false;
    while (*end == ' ' || *end == '\t') end++;
    float scale = 1000.0f;  // default: treat bare number as base unit (V or A) -> mV / mA
    if (*end) {
        // Optional 'm' prefix then unit letter
        bool milli = false;
        if (*end == 'm' || *end == 'M') { milli = true; end++; }
        char U = to_upper(*end);
        if (U != 0 && U != base_letter) return false;
        if (milli) scale = 1.0f;
    }
    float scaled = val * scale;
    /* Inverted comparison so NaN/inf fail: "set nanV" parses to NaN via
     * strtof, and NaN compares false against any bound, so the old
     * (scaled < 0 || scaled > 1e6) check let it through to an undefined
     * uint32_t cast that was used directly as a voltage target. */
    if (!(scaled >= 0.0f && scaled <= 1e6f)) return false;
    *out_milli = static_cast<uint32_t>(scaled + 0.5f);
    return true;
}

// Format mV / mA as SCPI-style base-unit value ("28.000" etc.).
static void fmt_base(char* buf, size_t n, uint32_t milli) {
    snprintf(buf, n, "%u.%03u", (unsigned)(milli / 1000), (unsigned)(milli % 1000));
}

// -----------------------------------------------------------------------------
// PDO / contract decoding (used by :PD queries)
// -----------------------------------------------------------------------------

static void decode_pdo_line(unsigned idx /*1-based*/, uint32_t pdo,
                            char* buf, size_t bufsz) {
    unsigned type = (pdo >> 30) & 0x3;
    bool is_epr = (idx >= 8);

    if (type == 0) {
        unsigned v  = ((pdo >> 10) & 0x3FF) * 50;
        unsigned im = (pdo & 0x3FF) * 10;
        snprintf(buf, bufsz,
                 "%u,FIXED,%u.%03u,%u.%03u,%s",
                 idx, v / 1000, v % 1000, im / 1000, im % 1000,
                 is_epr ? "EPR" : "SPR");
    } else if (type == 3) {
        unsigned apdo = (pdo >> 28) & 0x3;
        if (apdo == 0) {
            unsigned vmax = ((pdo >> 17) & 0xFF) * 100;
            unsigned vmin = ((pdo >> 8)  & 0xFF) * 100;
            unsigned imax = (pdo & 0x7F) * 50;
            snprintf(buf, bufsz,
                     "%u,PPS,%u.%03u-%u.%03u,%u.%03u,SPR",
                     idx, vmin / 1000, vmin % 1000,
                     vmax / 1000, vmax % 1000,
                     imax / 1000, imax % 1000);
        } else if (apdo == 1) {
            unsigned vmax = ((pdo >> 17) & 0x1FF) * 100;
            unsigned vmin = ((pdo >> 8)  & 0xFF)  * 100;
            unsigned w    = pdo & 0xFF;
            snprintf(buf, bufsz,
                     "%u,EPR_AVS,%u.%03u-%u.%03u,%uW,EPR",
                     idx, vmin / 1000, vmin % 1000,
                     vmax / 1000, vmax % 1000, w);
        } else if (apdo == 2) {
            unsigned i15 = ((pdo >> 10) & 0x3FF) * 10;
            unsigned i20 = (pdo & 0x3FF) * 10;
            snprintf(buf, bufsz,
                     "%u,SPR_AVS,9.000-20.000,%u.%03u/%u.%03u,SPR",
                     idx, i15 / 1000, i15 % 1000, i20 / 1000, i20 % 1000);
        } else {
            snprintf(buf, bufsz, "%u,APDO,raw,%08lX,?", idx, (unsigned long)pdo);
        }
    } else {
        snprintf(buf, bufsz, "%u,UNKNOWN,raw,%08lX,?", idx, (unsigned long)pdo);
    }
}

static void decode_contract(char* buf, size_t bufsz) {
    if (!s_port) { snprintf(buf, bufsz, "NONE"); return; }
    uint32_t rdo = s_port->rdo_contracted;
    if (rdo == 0) { snprintf(buf, bufsz, "NONE"); return; }
    unsigned pos = (rdo >> 28) & 0xF;
    if (pos == 0 || pos > s_port->source_caps.size()) {
        snprintf(buf, bufsz, "UNKNOWN,pos=%u,rdo=%08lX", pos, (unsigned long)rdo);
        return;
    }
    uint32_t pdo = s_port->source_caps[pos - 1];
    unsigned type = (pdo >> 30) & 0x3;

    if (type == 0) {
        unsigned v  = ((pdo >> 10) & 0x3FF) * 50;
        unsigned op = ((rdo >> 10) & 0x3FF) * 10;
        snprintf(buf, bufsz, "PDO%u,FIXED,%u.%03uV,%u.%03uA,%s",
                 pos, v / 1000, v % 1000,
                 op / 1000, op % 1000,
                 pos >= 8 ? "EPR" : "SPR");
    } else if (type == 3) {
        unsigned apdo = (pdo >> 28) & 0x3;
        if (apdo == 0) {
            unsigned ov = ((rdo >> 9) & 0xFFF) * 20;
            unsigned oi = (rdo & 0x7F) * 50;
            snprintf(buf, bufsz, "PDO%u,PPS,%u.%03uV,%u.%03uA,SPR",
                     pos, ov / 1000, ov % 1000, oi / 1000, oi % 1000);
        } else if (apdo == 1) {
            unsigned ov = ((rdo >> 9) & 0xFFF) * 25;
            unsigned oi = (rdo & 0x7F) * 50;
            snprintf(buf, bufsz, "PDO%u,EPR_AVS,%u.%03uV,%u.%03uA,EPR",
                     pos, ov / 1000, ov % 1000, oi / 1000, oi % 1000);
        } else {
            snprintf(buf, bufsz, "PDO%u,APDO%u,rdo=%08lX",
                     pos, apdo, (unsigned long)rdo);
        }
    } else {
        snprintf(buf, bufsz, "PDO%u,type%u,rdo=%08lX",
                 pos, type, (unsigned long)rdo);
    }
}

// -----------------------------------------------------------------------------
// Apply the current (target_mv, target_ma, source_mode) into pdsink's DPM.
// Called after SOUR:VOLT / SOUR:CURR / SOUR:MODE writes.
// -----------------------------------------------------------------------------

static void apply_source_target() {
    if (!s_dpm || !s_pe) return;
    /* Persist the user's voltage selection for "Restore last V/I". Done before
     * the trigger: a fresh 48V/EPR selection is still on the old SPR contract
     * here, so the flash write commits immediately rather than deferring under
     * EPR. Skipped automatically (unchanged-value guard) on re-applies. */
    if (target_mv >= 3300U) Settings_SaveLastSettings(target_mv, target_ma);
    switch (source_mode) {
        case SourceMode::AUTO:
            s_dpm->trigger_any(target_mv, target_ma);
            break;
        case SourceMode::FIXED:
            s_dpm->trigger_variant(pd::PDO_VARIANT::FIXED, target_mv, target_ma);
            break;
        case SourceMode::PPS:
            s_dpm->trigger_variant(pd::PDO_VARIANT::APDO_PPS, target_mv, target_ma);
            break;
        case SourceMode::AVS:
            // EPR AVS if we're in EPR mode; otherwise try SPR AVS (PD 3.2 sources).
            s_dpm->trigger_variant(s_pe->is_in_epr_mode()
                                       ? pd::PDO_VARIANT::APDO_EPR_AVS
                                       : pd::PDO_VARIANT::APDO_SPR_AVS,
                                   target_mv, target_ma);
            break;
    }
}

// -----------------------------------------------------------------------------
// Help text (returned by :SYST:HELP?)
// -----------------------------------------------------------------------------

static const char* HELP_TEXT =
    "\r\n"
    "== AxxPD v" FW_VERSION "  interactive command reference =========================\r\n"
    "SPR = Standard Power Range (USB-PD 3.0+ baseline, up to 20 V / 100 W, PDO 1-7).\r\n"
    "EPR = Extended Power Range (PD 3.1+ addition, up to 48 V / 240 W, PDO 8-11).\r\n"
    "EPR slots (28-48 V) hidden until 'list all' or 'epr'. Chain commands with ';'.\r\n"
    "Numeric args are V or A; accept V/mV/A/mA suffixes (e.g. 'setpps 7500mV 2A').\r\n"
    "\r\n"
    "-- State inspection ----------------------------------------------------\r\n"
    "  list       PDOs currently in the cap set (SPR only until EPR is entered;\r\n"
    "             appends a hint when the source advertises EPR)\r\n"
    "  list all   full advertised set incl. EPR (auto-enters EPR if capable)\r\n"
    "  ct         show the active contract (V / A / PDO / mode)\r\n"
    "  meas       measured V, I, W, Wh, Ah, temperatures on VBUS\r\n"
    "\r\n"
    "-- Mode ----------------------------------------------------------------\r\n"
    "  epr        enter EPR mode. Resets target to 5 V; use setpdo/setavs next.\r\n"
    "  spr        leave EPR (PDO list shrinks back to 1-7)\r\n"
    "  rst        renegotiate to 5 V PDO1 and clear any pending setpdo/EPR triggers.\r\n"
    "             Stays in EPR if already entered; use 'spr' to leave EPR.\r\n"
    "             (Not a UCPD Hard Reset - unplug the cable for a full link reset.)\r\n"
    "  trace on|off  diagnostic prints ([CC]/[RX]/[TX]/[UCPD] + PE state). Default OFF.\r\n"
    "  stream on|off [rate_hz]  CSV data streaming (default 20 Hz, e.g. 'stream on 100')\r\n"
    "  seq add <V> <ms>  add voltage sequence step\r\n"
    "  seq list|clear|run|stop  manage voltage sequence\r\n"
    "\r\n"
    "-- Self-test, reboot, DFU ----------------------------------------------\r\n"
    "  selftest   walk every advertised PDO + PPS/AVS min/mid/max, print PASS/FAIL\r\n"
    "             report. Auto-enters EPR if needed. Takes ~10-30 s; do not\r\n"
    "             have a load on VBUS during the run (rail jumps to all PDO\r\n"
    "             voltages in sequence).\r\n"
    "  reboot     full MCU reset (NVIC). Returns to fresh boot state.\r\n"
    "  dfu        jump to STM32 system memory bootloader (USB DFU / USART1).\r\n"
    "             Use STM32CubeProgrammer or dfu-util to flash new firmware.\r\n"
    "             Power-cycle or reset to return to AxxPD.\r\n"
    "\r\n"
    "-- Request a voltage ---------------------------------------------------\r\n"
    "  setpdo <N>          request Fixed PDO at slot N (1-11). Uses its advertised V/I.\r\n"
    "                      N >= 8 auto-enters EPR first.\r\n"
    "                      e.g.  setpdo 1   5 V\r\n"
    "                            setpdo 4   20 V\r\n"
    "                            setpdo 8   28 V EPR\r\n"
    "  setpps <V> [<I>]    SPR PPS at V volts, 20 mV step. Optional current, 50 mA step.\r\n"
    "                      e.g.  setpps 7.5V 2A    PPS 7.5 V / 2 A\r\n"
    "                            setpps 11V        PPS 11 V (unchanged current)\r\n"
    "  setavs <V>          EPR AVS at V volts, 100 mV effective step. Needs 'epr' first.\r\n"
    "                      e.g.  setavs 24V        EPR AVS 24 V\r\n"
    "                            setavs 27.5V      EPR AVS 27.5 V\r\n"
    "  set <V> [<I>]       generic auto-select: any PDO that provides V (filtered by 'mode').\r\n"
    "                      Prefer setpdo/setpps/setavs when you know the PDO type.\r\n"
    "  mode AUTO|FIX|PPS|AVS   filter used by 'set' when picking a PDO. Default AUTO.\r\n"
    "\r\n"
    "-- Typical session -----------------------------------------------------\r\n"
    "  list                                 -- inspect what's available\r\n"
    "  setpdo 3       # 15 V                -- pick SPR Fixed by slot\r\n"
    "  setpps 9V 3A   # 9 V / 3 A PPS       -- programmable SPR supply\r\n"
    "  epr            # enter EPR           -- list expands to 9+ PDOs\r\n"
    "  setpdo 8       # 28 V EPR Fixed\r\n"
    "  setavs 24V     # 24 V EPR AVS\r\n"
    "  rst            # renegotiate to 5 V (use 'spr' after if you want out of EPR)\r\n"
    "\r\n"
    "SCPI is also accepted (*IDN?, *RST, :SOUR:VOLT, :PD:CONTR?, etc.) - see\r\n"
    "the SCPI reference sheet for scripting.\r\n"
    "\r\n"
    "\r\n";

// -----------------------------------------------------------------------------
// Command handlers
// -----------------------------------------------------------------------------

// *IDN?
static void do_idn() { out(IDN_STRING); outln(); }

// reboot — full MCU reset via NVIC. Returns to clean boot state.
// SCPI alias: :SYST:REBOOT
static void do_reboot() {
    out("rebooting...\r\n");
    HAL_Delay(50);   // let the UART TX FIFO drain
    NVIC_SystemReset();
    while (1) {}     // unreachable
}

// dfu — jump to STM32G4 system memory bootloader for firmware update.
// The G4 bootloader supports USB DFU, USART, I2C, SPI, etc.
// After the jump, the MCU appears as a DFU device on USB; flash with
// dfu-util or STM32CubeProgrammer's DFU mode. Returns to user firmware
// after the bootloader exits or on power cycle.
//
// System memory base: 0x1FFF0000 on STM32G4.
// Procedure: deinit peripherals, set MSP from bootloader vector table,
// jump to bootloader's reset handler.
//
// SCPI alias: :SYST:DFU
static void do_dfu() {
    out("entering DFU bootloader (USB / USART1 / etc.) ...\r\n");
    out("Reset MCU or power cycle to return to AxxPD firmware.\r\n");

    /* Safely disable output before jumping to bootloader */
    if (g_output_enabled) {
        Output_Disable();
        g_output_enabled = 0;
    }

    /* Stretch IWDG to its hardware maximum (4095 * 256 / 32 kHz ~= 32.8 s)
     * so the ROM bootloader doesn't get reset mid-firmware-update. The IWDG
     * cannot be stopped once started, so this is the longest uninterrupted
     * window a DFU flash can take — a full-image update over slow USART can
     * approach it; USB DFU is comfortably inside. */
    IWDG->KR  = 0x5555U;           /* unlock */
    IWDG->PR  = 7U;                /* /256 prescaler */
    IWDG->RLR = 0xFFFU;            /* max reload (4095) */
    IWDG->KR  = 0xAAAAU;           /* refresh */

    HAL_Delay(50);

    // System memory bootloader base on STM32G4
    constexpr uint32_t BOOT_ADDR = 0x1FFF0000UL;

    // Tear down everything we set up
    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    __disable_irq();

    // Clear and disable all NVIC interrupts so the bootloader gets a
    // clean state. Cortex-M4 has 8 ICER/ICPR registers.
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFu;
        NVIC->ICPR[i] = 0xFFFFFFFFu;
    }

    // Remap system memory to address 0 (so vector table at 0x1FFF0000
    // is reachable as the boot region).
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    // Bootloader's vector table: [0]=initial MSP, [1]=reset handler.
    uint32_t msp = *reinterpret_cast<const uint32_t*>(BOOT_ADDR);
    // Validate MSP points to SRAM
    if ((msp & 0xFFF00000) != 0x20000000) {
        out("DFU: invalid bootloader vector table\r\n");
        return;
    }
    void (*reset_handler)(void) = reinterpret_cast<void(*)(void)>(
        *reinterpret_cast<const uint32_t*>(BOOT_ADDR + 4));

    __set_MSP(msp);
    __enable_irq();
    reset_handler();
    while (1) {}     // unreachable
}

// -----------------------------------------------------------------------------
// Self-test machine
//
// Build the step list from the live source_caps:
//   - one FIXED step per advertised Fixed PDO (positions 1..11)
//   - PPS min, mid, max for each PPS APDO (50 mA target current)
//   - AVS min, mid, max for each EPR/SPR AVS APDO
// EPR slots are tested only if the source advertises EPR (PDO1 bit 23 = 1)
// and we'll auto-enter EPR before they run.
//
// For each step, request the PDO and wait for #EVT CONTRACT to confirm.
// PASS if rdo_contracted lands on the expected position with the expected
// voltage; FAIL if it falls back to PDO1 5 V or times out.
// -----------------------------------------------------------------------------

static void selftest_label_fixed(SelftestStep& s, unsigned pos, uint32_t mv) {
    snprintf(s.label, sizeof(s.label), "PDO%u Fixed %lu.%luV",
             pos, (unsigned long)(mv / 1000), (unsigned long)((mv % 1000) / 100));
}
static void selftest_label_pps(SelftestStep& s, unsigned pos, uint32_t mv,
                                const char* tag) {
    snprintf(s.label, sizeof(s.label), "PDO%u PPS %lu.%luV (%s)",
             pos, (unsigned long)(mv / 1000), (unsigned long)((mv % 1000) / 100), tag);
}
static void selftest_label_avs(SelftestStep& s, unsigned pos, uint32_t mv,
                                const char* tag) {
    snprintf(s.label, sizeof(s.label), "PDO%u AVS %lu.%luV (%s)",
             pos, (unsigned long)(mv / 1000), (unsigned long)((mv % 1000) / 100), tag);
}

static void selftest_build_steps() {
    selftest_count = 0;
    if (!s_port) return;

    bool src_epr = (s_port->source_caps.size() > 0) &&
                   ((s_port->source_caps[0] >> 23) & 1);

    for (size_t i = 0; i < s_port->source_caps.size() &&
                       selftest_count < SELFTEST_MAX_STEPS; i++) {
        uint32_t pdo = s_port->source_caps[i];
        if (pdo == 0) continue;
        unsigned type = (pdo >> 30) & 0x3;
        unsigned pos = static_cast<unsigned>(i + 1);

        if (type == 0) {
            unsigned v_mv = ((pdo >> 10) & 0x3FF) * 50;
            SelftestStep& s = selftest_steps[selftest_count++];
            s.kind = SelftestStep::Kind::FIXED;
            s.pdo_pos = static_cast<uint8_t>(pos);
            s.target_mv = v_mv;
            s.target_ma = 0;
            s.expect_pos = pos;
            s.expect_mv = v_mv;
            s.result = SelftestStep::Result::PENDING;
            selftest_label_fixed(s, pos, v_mv);
        } else if (type == 3) {
            unsigned sub = (pdo >> 28) & 0x3;
            if (sub == 0) {  // SPR PPS
                unsigned vmin = ((pdo >> 8)  & 0xFF) * 100;
                unsigned vmax = ((pdo >> 17) & 0xFF) * 100;
                unsigned vmid = (vmin + vmax) / 2;
                // Round to 20 mV step
                vmid = (vmid / 20) * 20;
                const struct { unsigned v; const char* tag; } pts[3] = {
                    {vmin, "min"}, {vmid, "mid"}, {vmax, "max"}};
                for (auto& pt : pts) {
                    if (selftest_count >= SELFTEST_MAX_STEPS) break;
                    SelftestStep& s = selftest_steps[selftest_count++];
                    s.kind = SelftestStep::Kind::PPS;
                    s.pdo_pos = static_cast<uint8_t>(pos);
                    s.target_mv = pt.v;
                    s.target_ma = 0;
                    s.expect_pos = pos;
                    s.expect_mv = pt.v;
                    s.result = SelftestStep::Result::PENDING;
                    selftest_label_pps(s, pos, pt.v, pt.tag);
                }
            } else if (sub == 1) {  // EPR AVS
                if (!src_epr) continue;
                unsigned vmin = ((pdo >> 8)  & 0xFF)  * 100;
                unsigned vmax = ((pdo >> 17) & 0x1FF) * 100;
                unsigned vmid = ((vmin + vmax) / 2 / 100) * 100;
                const struct { unsigned v; const char* tag; } pts[3] = {
                    {vmin, "min"}, {vmid, "mid"}, {vmax, "max"}};
                for (auto& pt : pts) {
                    if (selftest_count >= SELFTEST_MAX_STEPS) break;
                    SelftestStep& s = selftest_steps[selftest_count++];
                    s.kind = SelftestStep::Kind::AVS;
                    s.pdo_pos = static_cast<uint8_t>(pos);
                    s.target_mv = pt.v;
                    s.target_ma = 0;
                    s.expect_pos = pos;
                    s.expect_mv = pt.v;
                    s.result = SelftestStep::Result::PENDING;
                    selftest_label_avs(s, pos, pt.v, pt.tag);
                }
            }
            // sub == 2 (SPR AVS, PD 3.2 only) skipped — Anker doesn't have it
        }
    }

    // Add random voltage steps from the available PDOs
    if (selftest_count > 0 && !s_port->source_caps.empty()) {
        // Simple xorshift32 PRNG seeded from tick counter
        uint32_t rng = HAL_GetTick() ^ 0xDEADBEEFU;
        auto rng_next = [&rng]() {
            rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
            return rng;
        };

        size_t n_caps = s_port->source_caps.size();
        for (size_t r = 0; r < SELFTEST_RANDOM_STEPS &&
                           selftest_count < SELFTEST_MAX_STEPS; r++) {
            // Pick a random non-zero PDO
            size_t tries = 0;
            size_t pick;
            uint32_t pdo;
            do {
                pick = rng_next() % n_caps;
                pdo = s_port->source_caps[pick];
            } while (pdo == 0 && ++tries < n_caps);
            if (pdo == 0) continue;

            unsigned type = (pdo >> 30) & 0x3;
            unsigned pos = static_cast<unsigned>(pick + 1);
            SelftestStep& s = selftest_steps[selftest_count++];
            s.result = SelftestStep::Result::PENDING;

            if (type == 0) {
                // Fixed — use the PDO's voltage
                unsigned v_mv = ((pdo >> 10) & 0x3FF) * 50;
                s.kind = SelftestStep::Kind::FIXED;
                s.pdo_pos = static_cast<uint8_t>(pos);
                s.target_mv = v_mv;
                s.target_ma = 0;
                s.expect_pos = pos;
                s.expect_mv = v_mv;
                snprintf(s.label, sizeof(s.label), "Rnd %lu.%luV Fix",
                         (unsigned long)(v_mv / 1000), (unsigned long)((v_mv % 1000) / 100));
            } else if (type == 3) {
                unsigned sub = (pdo >> 28) & 0x3;
                if (sub == 0) {
                    // PPS — random voltage in 20 mV steps
                    unsigned vmin = ((pdo >> 8) & 0xFF) * 100;
                    unsigned vmax = ((pdo >> 17) & 0xFF) * 100;
                    unsigned steps_n = (vmax > vmin) ? (vmax - vmin) / 20 : 0;
                    unsigned v_mv = vmin + (rng_next() % (steps_n + 1)) * 20;
                    s.kind = SelftestStep::Kind::PPS;
                    s.pdo_pos = static_cast<uint8_t>(pos);
                    s.target_mv = v_mv;
                    s.target_ma = 0;
                    s.expect_pos = pos;
                    s.expect_mv = v_mv;
                    snprintf(s.label, sizeof(s.label), "Rnd %lu.%luV PPS",
                             (unsigned long)(v_mv / 1000), (unsigned long)((v_mv % 1000) / 100));
                } else if (sub == 1) {
                    // AVS — random voltage in 100 mV steps
                    unsigned vmin = ((pdo >> 8) & 0xFF) * 100;
                    unsigned vmax = ((pdo >> 17) & 0x1FF) * 100;
                    unsigned steps_n = (vmax > vmin) ? (vmax - vmin) / 100 : 0;
                    unsigned v_mv = vmin + (rng_next() % (steps_n + 1)) * 100;
                    s.kind = SelftestStep::Kind::AVS;
                    s.pdo_pos = static_cast<uint8_t>(pos);
                    s.target_mv = v_mv;
                    s.target_ma = 0;
                    s.expect_pos = pos;
                    s.expect_mv = v_mv;
                    snprintf(s.label, sizeof(s.label), "Rnd %lu.%luV AVS",
                             (unsigned long)(v_mv / 1000), (unsigned long)((v_mv % 1000) / 100));
                } else {
                    selftest_count--;  // unsupported sub-type, drop it
                }
            } else {
                selftest_count--;  // unsupported PDO type, drop it
            }
        }
    }
}

// Internal: actually kick the test machine. Caller has already confirmed.
static void selftest_start_run() {
    if (s_port->source_caps.empty()) {
        out("no source capabilities yet - is a charger attached?\r\n");
        return;
    }

    // If the source advertises EPR, enter EPR FIRST so the EPR PDOs appear
    // in source_caps and get included in the test plan. We can't see slots
    // 8-11 from the SPR cap set alone.
    bool src_epr = ((s_port->source_caps[0] >> 23) & 1) != 0;
    if (src_epr && !s_pe->is_in_epr_mode()) {
        out("self-test: entering EPR mode first to enumerate full PDO set...\r\n");
        user_wants_epr = true;
        target_mv = 5000; target_ma = 0;
        source_mode = SourceMode::AUTO;
        s_dpm->trigger_any(target_mv, target_ma);
        s_port->pe_flags.clear(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
        s_port->dpm_requests.set(pd::DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
        selftest_state = SelftestState::PRE_EPR;
    } else {
        selftest_state = SelftestState::BUILD;
    }
    selftest_idx = 0;
    selftest_step_t0 = HAL_GetTick();
}

// User-facing 'selftest' / :SYST:TEST entry. Prints the safety warning and
// parks waiting for the user's "OK" confirmation before actually running.
static void do_selftest() {
    if (!s_port || !s_dpm || !s_pe) {
        err_push(-240, "Hardware missing"); return;
    }
    if (selftest_state != SelftestState::IDLE) {
        out("self-test already running\r\n"); return;
    }
    if (selftest_pending_confirm) {
        out("self-test: already waiting for OK confirmation\r\n");
        return;
    }
    out("\r\n"
        "!! WARNING - SELF-TEST WILL SWEEP VBUS THROUGH ALL PDO LEVELS !!\r\n"
        "\r\n"
        "The self-test steps through every advertised PDO (5 V up to 28 V\r\n"
        "Fixed, plus PPS/AVS min/mid/max sweeps). Voltage transitions on\r\n"
        "VBUS are abrupt and MAY DAMAGE connected components.\r\n"
        "\r\n"
        "Make sure the AxxPD output is COMPLETELY disconnected from any\r\n"
        "external device or load before continuing.\r\n"
        "\r\n"
        "Type 'OK' (uppercase or lowercase) to start.\r\n"
        "Anything else aborts.\r\n");
    selftest_pending_confirm = true;
}

// Returns true when the live contract already satisfies a step's expectation
// (right position + right voltage for PPS/AVS). Used to pass-through steps
// that would be no-ops (e.g. requesting PDO1 when we're already at PDO1).
static bool selftest_contract_matches(const SelftestStep& s, uint32_t rdo) {
    if (rdo == 0) return false;
    unsigned pos = (rdo >> 28) & 0xF;
    if (pos != s.expect_pos) return false;
    if (s.kind == SelftestStep::Kind::FIXED) return true;
    if (s.kind == SelftestStep::Kind::PPS) {
        unsigned ov = ((rdo >> 9) & 0xFFF) * 20;
        // Allow 20 mV tolerance (one PPS step)
        return (ov + 20 >= s.expect_mv) && (s.expect_mv + 20 >= ov);
    }
    if (s.kind == SelftestStep::Kind::AVS) {
        unsigned ov = ((rdo >> 9) & 0xFFF) * 25;
        // AVS step is 25 mV in RDO but advertised step is 100 mV
        return (ov + 100 >= s.expect_mv) && (s.expect_mv + 100 >= ov);
    }
    return false;
}

// Drive the self-test step machine. Called from cli_poll() each tick.
static void selftest_tick() {
    if (selftest_state == SelftestState::IDLE) return;
    uint32_t now = HAL_GetTick();

    switch (selftest_state) {
    case SelftestState::PRE_EPR: {
        // Wait for EPR mode to be active and stable, with caps populated to
        // the full EPR set (>7 PDOs), before building steps. Timeout after
        // 10 s and proceed anyway (we'll just have fewer steps to test).
        if (s_pe->is_in_epr_mode() &&
            (int32_t)(now - selftest_step_t0) > 3500 &&
            s_port->source_caps.size() > 7)
        {
            selftest_state = SelftestState::BUILD;
            selftest_step_t0 = now;
        } else if ((int32_t)(now - selftest_step_t0) > 10000) {
            out("self-test: EPR entry timed out, proceeding with SPR-only steps\r\n");
            selftest_state = SelftestState::BUILD;
            selftest_step_t0 = now;
        }
        break;
    }

    case SelftestState::BUILD: {
        selftest_build_steps();
        if (selftest_count == 0) {
            out("self-test: no testable PDOs\r\n");
            selftest_state = SelftestState::IDLE;
            return;
        }
        // Enable LTC4368 output so the INA228 can measure actual VBUS.
        // The selftest warning already told the user to disconnect loads.
        Output_Enable();
        // Bleed on for the whole run: the INA228 is on the OUTPUT side and
        // the LTC4368 blocks reverse current, so without the bleed the
        // output caps park at the old voltage on every downward step and
        // the report shows stale-high readings (10k bleed, tau ~0.5s).
        HAL_GPIO_WritePin(BLEED_CTRL_GPIO_Port, BLEED_CTRL_Pin, GPIO_PIN_SET);
        char b[64];
        snprintf(b, sizeof(b), "self-test: %u steps queued\r\n",
                 (unsigned)selftest_count);
        out(b);
        selftest_idx = 0;
        selftest_state = SelftestState::REQUEST;
        selftest_step_t0 = now;
        break;
    }

    case SelftestState::REQUEST: {
        SelftestStep& s = selftest_steps[selftest_idx];
        char b[80];
        snprintf(b, sizeof(b), "  [%2u/%2u] %-26s ... ",
                 (unsigned)(selftest_idx + 1), (unsigned)selftest_count, s.label);
        out(b);

        // If we're already at the target contract, no transition needed -
        // the request would be a no-op and we'd time out waiting for a
        // change that never happens. PASS immediately.
        selftest_pre_rdo = s_port->rdo_contracted;
        if (selftest_contract_matches(s, selftest_pre_rdo)) {
            s.result = SelftestStep::Result::PASS;
            selftest_state = SelftestState::NEXT;
            selftest_step_t0 = now;
            return;
        }

        // Suppress faults during PDO transition (inrush from voltage change)
        g_fault_suppress_until = HAL_GetTick() + 2000U;

        // Re-assert the bleed in case an OCP retry cleared it mid-run
        HAL_GPIO_WritePin(BLEED_CTRL_GPIO_Port, BLEED_CTRL_Pin, GPIO_PIN_SET);

        // Issue the request. trigger_*() calls request_new_power_level()
        // internally so NEW_POWER_LEVEL is set on the next PE tick.
        switch (s.kind) {
        case SelftestStep::Kind::FIXED:
            s_dpm->trigger_by_position(s.pdo_pos);
            break;
        case SelftestStep::Kind::PPS:
            s_dpm->trigger_variant(pd::PDO_VARIANT::APDO_PPS,
                                    s.target_mv, s.target_ma);
            break;
        case SelftestStep::Kind::AVS:
            s_dpm->trigger_variant(pd::PDO_VARIANT::APDO_EPR_AVS,
                                    s.target_mv, s.target_ma);
            break;
        }
        selftest_state = SelftestState::WAIT_CONTRACT;
        selftest_step_t0 = now;
        break;
    }

    case SelftestState::WAIT_CONTRACT: {
        SelftestStep& s = selftest_steps[selftest_idx];
        uint32_t rdo = s_port->rdo_contracted;
        if (rdo != selftest_pre_rdo && rdo != 0) {
            unsigned pos = (rdo >> 28) & 0xF;
            if (selftest_contract_matches(s, rdo)) {
                s.result = SelftestStep::Result::PASS;
            } else {
                s.result = SelftestStep::Result::FAIL;
                if (pos != s.expect_pos) {
                    snprintf(s.fail_reason, sizeof(s.fail_reason),
                             "got PDO%u, want PDO%u",
                             pos, (unsigned)s.expect_pos);
                } else {
                    // Right position, wrong voltage
                    unsigned ov = (s.kind == SelftestStep::Kind::PPS)
                                  ? ((rdo >> 9) & 0xFFF) * 20
                                  : ((rdo >> 9) & 0xFFF) * 25;
                    snprintf(s.fail_reason, sizeof(s.fail_reason),
                             "got %u.%03uV, want %lu.%03luV",
                             ov / 1000, ov % 1000,
                             (unsigned long)(s.expect_mv / 1000), (unsigned long)(s.expect_mv % 1000));
                }
            }
            selftest_state = SelftestState::NEXT;
            selftest_step_t0 = now;
            return;
        }
        if ((now - selftest_step_t0) > SELFTEST_STEP_TIMEOUT_MS) {
            s.result = SelftestStep::Result::FAIL;
            snprintf(s.fail_reason, sizeof(s.fail_reason),
                     "timeout (%lu ms)", (unsigned long)(now - selftest_step_t0));
            selftest_state = SelftestState::NEXT;
            selftest_step_t0 = now;
        }
        break;
    }

    case SelftestState::NEXT: {
        // Wait for VBUS to settle before measuring and advancing.  1.5 s
        // covers the bleed discharge on the worst down-step (28 V -> 3.3 V
        // is ~1.1 s at tau ~0.5 s); the LTC4368 clamps the output at VBUS
        // once the caps have bled down to it.
        if ((now - selftest_step_t0) < 1500) return;
        // Record INA228 measured voltage NOW, after the settle delay.
        {
            SelftestStep& s = selftest_steps[selftest_idx];
            s.meas_mv = (uint32_t)(g_ina_reading.voltage_v * 1000.0f + 0.5f);
            char mb[48];
            snprintf(mb, sizeof(mb), "%s (%lu.%03luV)\r\n",
                     s.result == SelftestStep::Result::PASS ? "OK" : "FAIL",
                     (unsigned long)(s.meas_mv / 1000),
                     (unsigned long)(s.meas_mv % 1000));
            out(mb);
        }
        selftest_idx++;
        if (selftest_idx >= selftest_count) {
            selftest_state = SelftestState::REPORT;
        } else {
            selftest_state = SelftestState::REQUEST;
            selftest_step_t0 = now;
        }
        break;
    }

    case SelftestState::REPORT: {
        unsigned pass = 0, fail = 0;
        for (size_t i = 0; i < selftest_count; i++) {
            if (selftest_steps[i].result == SelftestStep::Result::PASS) pass++;
            else fail++;
        }
        char b[140];
        out("\r\n-- self-test report ---------------------------------------\r\n");
        for (size_t i = 0; i < selftest_count; i++) {
            const auto& s = selftest_steps[i];
            const char* r = (s.result == SelftestStep::Result::PASS) ? "PASS"
                          : (s.result == SelftestStep::Result::FAIL) ? "FAIL"
                          :                                            "----";
            snprintf(b, sizeof(b), "  %-28s %lu.%03luV  %s%s%s\r\n",
                     s.label,
                     (unsigned long)(s.meas_mv / 1000),
                     (unsigned long)(s.meas_mv % 1000),
                     r,
                     s.result == SelftestStep::Result::FAIL ? " : " : "",
                     s.result == SelftestStep::Result::FAIL ? s.fail_reason : "");
            out(b);
        }
        snprintf(b, sizeof(b),
                 "-- summary: %u/%u passed, %u failed -----------------------\r\n",
                 pass, (unsigned)selftest_count, fail);
        out(b);

        // Disable output before returning to 5 V
        Output_Disable();

        // Return VBUS to a safe 5 V before idling - tests can leave us at
        // 28 V or wherever the last step landed.
        out("self-test: returning to 5 V SPR ...\r\n");
        target_mv = 5000; target_ma = 0;
        source_mode = SourceMode::AUTO;
        s_dpm->trigger_any(5000, 0);
        selftest_state = SelftestState::FALLBACK_5V;
        selftest_step_t0 = now;
        break;
    }

    case SelftestState::FALLBACK_5V: {
        // Wait for the contract to settle at PDO1 5 V (or just time out;
        // at that point the user can 'rst').
        uint32_t rdo = s_port->rdo_contracted;
        unsigned pos = (rdo >> 28) & 0xF;
        if (pos == 1 && rdo != 0) {
            out("self-test: done (contract back at 5 V)\r\n");
            selftest_state = SelftestState::IDLE;
            return;
        }
        if ((int32_t)(now - selftest_step_t0) > 5000) {
            out("self-test: 5 V fallback timed out (use 'rst' if needed)\r\n");
            selftest_state = SelftestState::IDLE;
        }
        break;
    }

    case SelftestState::IDLE: break;
    }
}

// 'rst' / *RST — reset target to 5 V and request a new power level.
// We deliberately do NOT call LL_UCPD_SendHardReset() here: a raw UCPD
// Hard Reset bypasses pdsink's PE state machine, which then has stale
// state vs. the physical link and issues its own Hard Reset in
// response to the resulting protocol mismatch. That caused the HR
// storm observed after set/v commands. If you really need a hardware
// reset, unplug and replug the cable.
static void do_rst() {
    Output_Disable();
    g_output_enabled = 0;
    target_mv = 5000; target_ma = 0;
    source_mode = SourceMode::AUTO;
    user_wants_epr = false;
    if (s_dpm) s_dpm->trigger_any(target_mv, target_ma);
    // Success is silent per SCPI convention.
}

static void do_cls() { err_clear(); }

// :SOUR:VOLT <v> | :SOUR:VOLT?
static void do_sour_volt(const char* arg, bool query) {
    if (query) {
        char b[16]; fmt_base(b, sizeof(b), target_mv);
        out(b); outln();
        return;
    }
    uint32_t v;
    if (!parse_numeric_unit(arg, 'V', &v)) { err_push(-222, "Data out of range"); return; }
    target_mv = v;
    apply_source_target();
}

// :SOUR:CURR <a> | :SOUR:CURR?
static void do_sour_curr(const char* arg, bool query) {
    if (query) {
        char b[16]; fmt_base(b, sizeof(b), target_ma);
        out(b); outln();
        return;
    }
    uint32_t v;
    if (!parse_numeric_unit(arg, 'A', &v)) { err_push(-222, "Data out of range"); return; }
    target_ma = v;
    apply_source_target();
}

// :SOUR:MODE AUTO|FIX|PPS|AVS
static void do_sour_mode(const char* arg, bool query) {
    if (query) {
        switch (source_mode) {
            case SourceMode::AUTO:  out("AUTO");  break;
            case SourceMode::FIXED: out("FIXED"); break;
            case SourceMode::PPS:   out("PPS");   break;
            case SourceMode::AVS:   out("AVS");   break;
        }
        outln(); return;
    }
    if (!arg) { err_push(-109, "Missing parameter"); return; }
    char tmp[8]; strncpy(tmp, arg, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0;
    uppercase(tmp);
    if      (match1(tmp, "AUTO"))                          source_mode = SourceMode::AUTO;
    else if (match(tmp, "FIX", "FIXED"))                   source_mode = SourceMode::FIXED;
    else if (match1(tmp, "PPS"))                           source_mode = SourceMode::PPS;
    else if (match1(tmp, "AVS"))                           source_mode = SourceMode::AVS;
    else { err_push(-224, "Illegal parameter value"); out("Error: expected AUTO|FIX|PPS|AVS\r\n"); return; }
    out("OK\r\n");
}

// :SOUR:APPL - re-apply the current target (e.g. after PD:MODE EPR expands caps)
static void do_sour_apply() { apply_source_target(); }

// :MEAS:VOLT? / :MEAS:CURR?
static void do_meas_volt() {
    float v = 0.0f;
    INA228_ReadVoltage(&g_ina, &v);
    char b[16]; snprintf(b, sizeof(b), "%.3f", (double)v);
    out(b); outln();
}
static void do_meas_curr() {
    float i = 0.0f;
    INA228_ReadCurrent(&g_ina, &i);
    char b[16]; snprintf(b, sizeof(b), "%.3f", (double)i);
    out(b); outln();
}

// do_meas_all() — fresh INA228 read, emit CSV: V,A,W,Wh,Ah,Tdie,Tntc
static void do_meas_all() {
    INA228_Reading_t rd = {};
    INA228_ReadAll(&g_ina, &rd);
    char b[128];
    snprintf(b, sizeof(b), "%.3f,%.3f,%.3f,%.4f,%.4f,%.1f,%.1f\r\n",
             (double)rd.voltage_v,  (double)rd.current_a,
             (double)rd.power_w,    (double)rd.energy_wh,
             (double)rd.charge_ah,  (double)temp_display(rd.die_temp_c),
             (double)temp_display(g_ntc_temp));
    out(b);
}

// do_meas_pow() — V*I from fresh reading
static void do_meas_pow() {
    INA228_Reading_t rd = {};
    INA228_ReadAll(&g_ina, &rd);
    char b[24]; snprintf(b, sizeof(b), "%.3f\r\n", (double)rd.power_w);
    out(b);
}

// do_meas_temp() — Tdie,Tntc
static void do_meas_temp() {
    INA228_Reading_t rd = {};
    INA228_ReadAll(&g_ina, &rd);
    char b[32]; snprintf(b, sizeof(b), "%.1f,%.1f\r\n",
                         (double)temp_display(rd.die_temp_c),
                         (double)temp_display(g_ntc_temp));
    out(b);
}

// do_meas_ener() — Wh,Ah
static void do_meas_ener() {
    INA228_Reading_t rd = {};
    INA228_ReadAll(&g_ina, &rd);
    char b[32]; snprintf(b, sizeof(b), "%.4f,%.4f\r\n",
                         (double)rd.energy_wh, (double)rd.charge_ah);
    out(b);
}

// do_protect_ocp() — set INA228 alert over-current threshold
static void do_protect_ocp(const char* arg) {
    if (!arg || !*arg) { err_push(-109, "Missing parameter"); return; }
    uint32_t ma = 0;
    if (!parse_numeric_unit(arg, 'A', &ma)) { err_push(-222, "bad current"); return; }
    if (ma < 100U) { err_push(-222, "OCP minimum is 0.1A"); return; }
    if (ma > 7000U) { err_push(-222, "OCP maximum is 7A"); return; }
    float amps = (float)ma / 1000.0f;
    INA228_SetAlertOverCurrent(&g_ina, amps);
    // Sync persistent settings so :CONF:OCP? reflects the new value
    int32_t diff = (int32_t)ma - (int32_t)Settings_GetOcpMa();
    int32_t steps = diff / 100;
    if (steps != 0) Settings_SetNumeric(MI_OCP_LIMIT, steps);
    char b[32]; snprintf(b, sizeof(b), "OCP set to %.1fA\r\n", (double)amps);
    out(b);
}

// do_protect_ovp() — set OVP threshold
static void do_protect_ovp(const char* arg) {
    if (!arg || !*arg) { err_push(-109, "Missing parameter"); return; }
    uint32_t mv = 0;
    if (!parse_numeric_unit(arg, 'V', &mv)) { err_push(-222, "bad voltage"); return; }
    OVP_SetThreshold(mv);
    char b[32]; snprintf(b, sizeof(b), "OVP set to %luV\r\n", (unsigned long)(mv / 1000));
    out(b);
}

// do_protect_status() — report active faults from g_fault_source
static void do_protect_status() {
    char b[64];
    snprintf(b, sizeof(b), "fault=%d src=%u", (int)g_hw_fault, (unsigned)g_fault_source);
    out(b);
    if (g_fault_source == FAULT_INA228_OCP)      out(" INA228_OCP");
    else if (g_fault_source == FAULT_COMP_OVP)   out(" COMP1_OVP");
    else if (g_fault_source == FAULT_LTC4368)    out(" LTC4368");
    else if (g_fault_source == FAULT_TPD4S480)   out(" TPD4S480");
    else if (g_fault_source == FAULT_LM5166_PGOOD) out(" LM5166_PG");
    else if (g_fault_source == FAULT_OPP)        out(" OPP");
    else if (g_fault_source == FAULT_TIMER)      out(" TIMER");
    else if (g_fault_source == FAULT_AH_LIMIT)   out(" AH_LIMIT");
    else if (g_fault_source == FAULT_WH_LIMIT)   out(" WH_LIMIT");
    else if (g_fault_source == FAULT_CHARGE_COMPLETE) out(" CHARGE_COMPLETE");
    else if (g_fault_source == FAULT_THERMAL)    out(" THERMAL");
    outln();
}

// do_protect_clear() — clear latched faults
static void do_protect_clear() {
    g_hw_fault    = 0;
    g_fault_source = 0;
    INA228_ClearAlertLatch(&g_ina);
    out("Faults cleared\r\n");
}

// do_seq_add() — append a step to seq_steps
// Accepts two forms:
//   2 args: seq add <V_with_unit> <dwell_ms>  — voltage via parse_numeric_unit, 0 mA
//   3 args: seq add <mV> <mA> <dwell_ms>      — legacy raw integer mode
static void do_seq_add(char* tok[], int nt) {
    if (nt < 3) { err_push(-109, "usage: seq add <V> <dwell_ms>  or  seq add <mV> <mA> <dwell_ms>"); return; }
    if (seq_count >= 16) { out("Sequence full (max 16 steps)\r\n"); return; }
    uint32_t mv, ma, dw;
    if (nt == 3) {
        // 2-arg form: seq add <V_with_unit> <dwell_ms>
        if (!parse_numeric_unit(tok[1], 'V', &mv)) { err_push(-222, "bad voltage"); return; }
        ma = 0;
        dw = static_cast<uint32_t>(strtoul(tok[2], nullptr, 10));
    } else {
        // 3-arg form (legacy): seq add <mV> <mA> <dwell_ms>
        mv = static_cast<uint32_t>(strtoul(tok[1], nullptr, 10));
        ma = static_cast<uint32_t>(strtoul(tok[2], nullptr, 10));
        dw = static_cast<uint32_t>(strtoul(tok[3], nullptr, 10));
    }
    if (mv < 3300U || mv > 48000U) { err_push(-222, "voltage out of range (3.3-48V)"); return; }
    if (dw == 0U) { err_push(-222, "dwell time must be > 0"); return; }
    seq_steps[seq_count] = { mv, ma, dw };
    char b[40]; snprintf(b, sizeof(b), "Step %u added\r\n", (unsigned)(seq_count + 1));
    seq_count++;
    out(b);
}

// do_seq_list() — print all steps
static void do_seq_list() {
    if (seq_count == 0) { out("Sequence empty\r\n"); return; }
    for (uint8_t i = 0; i < seq_count; i++) {
        char b[64];
        snprintf(b, sizeof(b), "  [%u] %lu mV  %lu mA  %lu ms\r\n",
                 (unsigned)(i + 1),
                 (unsigned long)seq_steps[i].mv,
                 (unsigned long)seq_steps[i].ma,
                 (unsigned long)seq_steps[i].dwell_ms);
        out(b);
    }
}

// do_seq_run() — kick off non-blocking sequence execution (FIX #13).
//
// Arms the state variables and requests the first step's voltage. The actual
// dwell timing and measurement happen in seq_tick(), called from cli_poll()
// every main-loop iteration, so UI, protection checks, and the watchdog all
// continue to run normally throughout the sequence.
static void do_seq_run() {
    if (seq_count == 0) { out("Sequence empty\r\n"); return; }
    if (seq_running) { out("Sequence already running\r\n"); return; }
    seq_stop_flag = 0;
    seq_current_step = 0;
    seq_running = true;

    // Print and request the first step
    char b[64];
    snprintf(b, sizeof(b), "[%u/%u] %lu mV %lu mA dwell=%lu ms\r\n",
             (unsigned)(seq_current_step + 1), (unsigned)seq_count,
             (unsigned long)seq_steps[0].mv,
             (unsigned long)seq_steps[0].ma,
             (unsigned long)seq_steps[0].dwell_ms);
    out(b);
    axxpd_request_voltage(seq_steps[0].mv, seq_steps[0].ma);
    seq_step_t0 = HAL_GetTick();
}

// seq_tick() — non-blocking sequence step machine, called from cli_poll().
//
// Waits for the current step's dwell time to elapse, reads and prints the
// measurement, then advances to the next step (or finishes). Because this
// runs inside the normal main-loop cadence, all other subsystems (display,
// INA228 polling, protection, buttons, watchdog) keep running.
static void seq_tick() {
    if (!seq_running) return;

    // Check for abort request
    if (seq_stop_flag) {
        out("Sequence aborted\r\n");
        seq_running = false;
        return;
    }

    // Wait for dwell time to elapse
    if ((HAL_GetTick() - seq_step_t0) < seq_steps[seq_current_step].dwell_ms)
        return;

    // Dwell elapsed — read and report measured values for this step
    INA228_Reading_t rd = {};
    INA228_ReadAll(&g_ina, &rd);
    char b[64];
    snprintf(b, sizeof(b), "  meas: %.3fV %.3fA %.3fW\r\n",
             (double)rd.voltage_v, (double)rd.current_a, (double)rd.power_w);
    out(b);

    // Advance to next step or finish
    seq_current_step++;
    if (seq_current_step >= seq_count) {
        out("Sequence complete\r\n");
        seq_running = false;
        return;
    }

    // Print and request the next step
    snprintf(b, sizeof(b), "[%u/%u] %lu mV %lu mA dwell=%lu ms\r\n",
             (unsigned)(seq_current_step + 1), (unsigned)seq_count,
             (unsigned long)seq_steps[seq_current_step].mv,
             (unsigned long)seq_steps[seq_current_step].ma,
             (unsigned long)seq_steps[seq_current_step].dwell_ms);
    out(b);
    axxpd_request_voltage(seq_steps[seq_current_step].mv,
                          seq_steps[seq_current_step].ma);
    seq_step_t0 = HAL_GetTick();
}

// Reset the EPR failure counter (called when user explicitly requests EPR
// so the background tracker starts fresh).
static void reset_epr_fail_tracker() {
    epr_quick_fail_count = 0;
}

// :PD:MODE SPR|EPR | :PD:MODE?
static void do_pd_mode(const char* arg, bool query) {
    if (!s_port || !s_pe) { err_push(-240, "Hardware missing"); return; }
    if (query) {
        out(s_pe->is_in_epr_mode() ? "EPR" : "SPR");
        outln(); return;
    }
    if (!arg) { err_push(-109, "Missing parameter"); return; }
    char tmp[8]; strncpy(tmp, arg, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0;
    uppercase(tmp);
    if (match1(tmp, "EPR")) {
        reset_epr_fail_tracker();
        user_wants_epr = true;
        // Re-prime the DPM target to a safe 5 V before EPR entry. Without
        // this, a stale trigger_position from a prior setpdo call (e.g.
        // setpdo 4 -> 20 V) would cause pdsink's DPM to select that same
        // PDO after EPR entry — and an intra-EPR jump to 20 V+ is exactly
        // what trips Anker A2697's PS_RDY quirk. 'epr' by itself just
        // enters EPR; the user can then issue setpdo/setpps/setavs for a
        // specific voltage.
        target_mv = 5000; target_ma = 0;
        source_mode = SourceMode::AUTO;
        s_dpm->trigger_any(target_mv, target_ma);
        s_port->pe_flags.clear(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
        s_port->dpm_requests.set(pd::DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
    } else if (match1(tmp, "SPR")) {
        user_wants_epr = false;
        s_port->pe_flags.set(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
        s_port->dpm_requests.clear(pd::DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
    } else {
        err_push(-224, "Illegal parameter value");
    }
}

// :PD:PDO:COUN?
static void do_pd_pdo_count() {
    if (!s_port) { out("0"); outln(); return; }
    char b[8]; snprintf(b, sizeof(b), "%u", (unsigned)s_port->source_caps.size());
    out(b); outln();
}

// :PD:PDO<n>?
static void do_pd_pdo_n(unsigned n) {
    if (!s_port || n == 0 || n > s_port->source_caps.size()) {
        err_push(-222, "Data out of range"); return;
    }
    uint32_t pdo = s_port->source_caps[n - 1];
    char b[80]; decode_pdo_line(n, pdo, b, sizeof(b));
    out(b); outln();
}

// Print one decoded line per populated PDO in source_caps. Assumes caller
// has verified s_port is non-null and source_caps is non-empty.
static void emit_pdo_lines() {
    char b[80];
    for (size_t i = 0; i < s_port->source_caps.size(); i++) {
        uint32_t pdo = s_port->source_caps[i];
        if (pdo == 0) continue;
        decode_pdo_line(static_cast<unsigned>(i + 1), pdo, b, sizeof(b));
        out(b); outln();
    }
}

// :PD:PDO:LIST? — dump the cap set as the sink currently holds it. Note
// that EPR PDOs (slots 8-11) are only present *after* EPR entry; before
// that, source_caps holds the 7-PDO SPR set only. When the source
// advertises EPR (PDO1 bit 23 = 1) but we're still in SPR, emit a hint
// pointing at 'list all' so the user knows 28/36/48 V slots exist.
static void do_pd_pdo_list() {
    if (!s_port || s_port->source_caps.empty()) { out("NONE"); outln(); return; }
    emit_pdo_lines();
    bool src_epr = ((s_port->source_caps[0] >> 23) & 1) != 0;
    if (s_pe && src_epr && !s_pe->is_in_epr_mode()) {
        out("(source advertises EPR - run 'list all' to include 28/36/48 V slots)");
        outln();
    }
}

// 'list all' / :PD:PDO:LIST? ALL — print the FULL advertised cap set.
// Behaviour:
//   - already in EPR    -> print current source_caps immediately
//   - SPR, EPR-capable  -> kick off EPR entry, park pending_list_all;
//                          cli_poll prints once source_caps expands
//   - SPR, not capable  -> print SPR cap set with a note (no EPR to show)
static void do_pd_pdo_list_all() {
    if (!s_port || !s_pe || !s_dpm) { err_push(-240, "Hardware missing"); return; }
    if (pending_list_all) {
        out("list all: already waiting for EPR enumeration\r\n");
        return;
    }
    if (s_pe->is_in_epr_mode()) {
        if (s_port->source_caps.empty()) { out("NONE"); outln(); return; }
        emit_pdo_lines();
        return;
    }
    bool src_epr = (s_port->source_caps.size() > 0) &&
                   ((s_port->source_caps[0] >> 23) & 1);
    if (!src_epr) {
        if (s_port->source_caps.empty()) { out("NONE"); outln(); return; }
        out("(source does not advertise EPR - SPR cap set only:)"); outln();
        emit_pdo_lines();
        return;
    }
    // SPR + EPR-capable: enter EPR at safe 5 V, defer the print. Same
    // priming as do_pd_mode("EPR"): target_mv=5000 avoids a stale
    // trigger_position from a prior setpdo selecting an EPR slot on entry.
    // ErrorRecovery first ensures the charger is in a clean state.
    reset_epr_fail_tracker();
    out("list all: entering EPR to enumerate full cap set...\r\n");
    user_wants_epr = true;
    target_mv = 5000;
    target_ma = 0;
    source_mode = SourceMode::AUTO;
    s_dpm->trigger_any(target_mv, target_ma);
    s_port->pe_flags.clear(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
    s_port->dpm_requests.set(pd::DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
    pending_list_all = true;
    pending_list_all_ms = HAL_GetTick();
}

// :PD:CONTR?
static void do_pd_contract() {
    char b[96]; decode_contract(b, sizeof(b));
    out(b); outln();
}

// :SYST:HELP?
static void do_syst_help() { out(HELP_TEXT); }

// :SYST:ERR?
static void do_syst_err() {
    ErrEntry e;
    if (err_pop(&e)) {
        char b[96]; snprintf(b, sizeof(b), "%d,\"%s\"", e.code, e.msg);
        out(b);
    } else {
        out("0,\"No error\"");
    }
    outln();
}

// :SYST:EVEN ON|OFF
static void do_syst_events(const char* arg, bool query) {
    if (query) { out(events_enabled ? "ON" : "OFF"); outln(); return; }
    if (!arg) { err_push(-109, "Missing parameter"); return; }
    char tmp[8]; strncpy(tmp, arg, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0; uppercase(tmp);
    if (match1(tmp, "ON") || match1(tmp, "1"))       events_enabled = true;
    else if (match1(tmp, "OFF") || match1(tmp, "0")) events_enabled = false;
    else err_push(-224, "Illegal parameter value");
}

// :SYST:TRACE ON|OFF — toggles both high-level (PE state changes) and
// low-level (driver [CC]/[RX]/[TX]/[UCPD]) debug prints at once. Default
// OFF keeps the interactive console clean.
static void do_syst_trace(const char* arg, bool query) {
    if (query) {
        out((state_trace || axxpd_low_trace) ? "ON" : "OFF"); outln();
        return;
    }
    if (!arg) { err_push(-109, "Missing parameter"); return; }
    char tmp[8]; strncpy(tmp, arg, sizeof(tmp)-1); tmp[sizeof(tmp)-1]=0; uppercase(tmp);
    if (match1(tmp, "ON") || match1(tmp, "1")) {
        state_trace = true; axxpd_low_trace = true;
    } else if (match1(tmp, "OFF") || match1(tmp, "0")) {
        state_trace = false; axxpd_low_trace = false;
    } else {
        err_push(-224, "Illegal parameter value");
    }
}

// -----------------------------------------------------------------------------
// Shortcut dispatch — human-friendly aliases on top of the SCPI handlers.
// Accepts lines that don't start with '*' or ':'. Returns true if the line
// was recognised and handled (whether the command succeeded or pushed an
// error); false to fall through to SCPI path parsing.
// -----------------------------------------------------------------------------

// Walk source_caps and report whether `mv` can be satisfied by any PDO of
// the types allowed by the current source_mode. Writes a human-readable
// reason to reason_buf on failure. Called by 'set V' / 'v V' handlers to
// avoid pdsink's silent fallback to PDO1 when no PDO matches the target.
static bool voltage_reachable(uint32_t mv, SourceMode mode,
                              char* reason_buf, size_t reason_n) {
    if (!s_port || s_port->source_caps.empty()) {
        snprintf(reason_buf, reason_n, "no source capabilities yet");
        return false;
    }
    bool allow_fixed = (mode == SourceMode::AUTO || mode == SourceMode::FIXED);
    bool allow_pps   = (mode == SourceMode::AUTO || mode == SourceMode::PPS);
    bool allow_avs   = (mode == SourceMode::AUTO || mode == SourceMode::AVS);
    for (size_t i = 0; i < s_port->source_caps.size(); i++) {
        uint32_t pdo = s_port->source_caps[i];
        if (pdo == 0) continue;
        unsigned type = (pdo >> 30) & 0x3;
        if (type == 0) {
            if (!allow_fixed) continue;
            unsigned v = ((pdo >> 10) & 0x3FF) * 50;
            if (v == mv) return true;
        } else if (type == 3) {
            unsigned sub = (pdo >> 28) & 0x3;
            if (sub == 0 && allow_pps) {
                unsigned vmin = ((pdo >> 8)  & 0xFF) * 100;
                unsigned vmax = ((pdo >> 17) & 0xFF) * 100;
                if (mv >= vmin && mv <= vmax) return true;
            } else if ((sub == 1 || sub == 2) && allow_avs) {
                unsigned vmin, vmax;
                if (sub == 1) { vmin = ((pdo >> 8) & 0xFF) * 100;
                                vmax = ((pdo >> 17) & 0x1FF) * 100; }
                else          { vmin = 9000; vmax = 20000; }
                if (mv >= vmin && mv <= vmax) return true;
            }
        }
    }
    const char* ms = (mode == SourceMode::AUTO)  ? "any advertised"
                   : (mode == SourceMode::FIXED) ? "any Fixed"
                   : (mode == SourceMode::PPS)   ? "any PPS"
                   :                                "any AVS";
    snprintf(reason_buf, reason_n,
             "%u mV not satisfied by %s PDO - use 'list' to see what's available",
             (unsigned)mv, ms);
    return false;
}

// Tokenize a line into up to 4 whitespace-separated tokens. Mutates the line.
static int tokenize4(char* line, char* tok[4]) {
    int n = 0;
    char* p = line;
    while (*p && n < 4) {
        while (*p == ' ' || *p == '\t') p++;
        if (!*p) break;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != '\t') p++;
        if (*p) { *p = 0; p++; }
    }
    return n;
}

static bool try_shortcut(char* line) {
    // Copy then tokenize so we can uppercase the command mnemonic without
    // breaking numeric args like "7.5V".
    char work[CLI_LINE_MAX];
    strncpy(work, line, sizeof(work) - 1);
    work[sizeof(work) - 1] = 0;
    char* tok[4] = {};
    int nt = tokenize4(work, tok);
    if (nt == 0) return false;

    char cmd[12];
    strncpy(cmd, tok[0], sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = 0;
    uppercase(cmd);

    // help / h / ?
    if (match(cmd, "H", "HELP") || match1(cmd, "?")) {
        do_syst_help(); return true;
    }

    // on / off — enable/disable output switch (LTC4368)
    if (match1(cmd, "ON")) {
        uint8_t r = Output_Enable_Guarded();
        if (r == 1) {
            out("Blocked: output toggle cooldown (min 1.5s between enables)\r\n");
        } else if (r == 2) {
            out("Blocked: board too hot - wait for thermal cooldown\r\n");
        } else {
            g_output_enabled = 1;
            out("Output ON\r\n");
        }
        return true;
    }
    if (match1(cmd, "OFF")) {
        Output_Disable();
        g_output_enabled = 0;
        out("Output OFF\r\n");
        return true;
    }

    // clear — acknowledge/clear any active fault
    if (match1(cmd, "CLEAR")) {
        g_hw_fault = 0;
        g_fault_source = 0;
        g_fault_suppress_until = 0;
        extern volatile uint8_t g_fault_pending_beep;
        g_fault_pending_beep = 0;
        INA228_ClearAlertLatch(&g_ina);
        out("Fault cleared\r\n");
        return true;
    }

    // lock / unlock — UI lock control
    if (match1(cmd, "LOCK")) {
        UI_SetLocked(1);
        out("UI locked\r\n");
        return true;
    }
    if (match1(cmd, "UNLOCK")) {
        UI_SetLocked(0);
        out("UI unlocked\r\n");
        return true;
    }

    // stream on|off [rate_hz] — configurable-rate CSV data streaming (FIX #14)
    //   stream on        → 20 Hz (default)
    //   stream on 100    → 100 Hz (10 ms interval)
    //   stream on 50     → 50 Hz  (20 ms interval)
    //   stream off       → disable
    //   stream           → toggle at current rate
    if (match1(cmd, "STREAM")) {
        extern volatile uint8_t  g_stream_enabled;
        extern volatile uint32_t g_stream_interval_ms;
        if (nt >= 2) {
            char arg[8]; strncpy(arg, tok[1], sizeof(arg)-1); arg[sizeof(arg)-1]=0; uppercase(arg);
            if (match1(arg, "ON") || match1(arg, "1")) {
                // Optional 3rd token: rate in Hz
                if (nt >= 3) {
                    uint32_t hz = (uint32_t)strtoul(tok[2], nullptr, 10);
                    if (hz >= 1 && hz <= 1000) {
                        g_stream_interval_ms = 1000 / hz;
                        if (g_stream_interval_ms == 0) g_stream_interval_ms = 1;
                    } else {
                        out("Rate out of range (1-1000 Hz), keeping current\r\n");
                    }
                } else {
                    g_stream_interval_ms = 50;  // default 20 Hz
                }
                g_stream_enabled = 1;
                char sb[48];
                snprintf(sb, sizeof(sb), "Stream ON (%lu Hz, #S prefix)\r\n",
                         (unsigned long)(1000 / g_stream_interval_ms));
                out(sb);
            } else {
                g_stream_enabled = 0;
                out("Stream OFF\r\n");
            }
        } else {
            g_stream_enabled = !g_stream_enabled;
            if (g_stream_enabled) {
                char sb[48];
                snprintf(sb, sizeof(sb), "Stream ON (%lu Hz, #S prefix)\r\n",
                         (unsigned long)(1000 / g_stream_interval_ms));
                out(sb);
            } else {
                out("Stream OFF\r\n");
            }
        }
        return true;
    }

    // rst
    if (match1(cmd, "RST")) {
        do_rst(); return true;
    }

    // stat — dump UCPD ISR counters for debugging
    if (match1(cmd, "STAT")) {
        extern volatile uint32_t ucpd_tx_ok, ucpd_tx_fail, ucpd_tx_count;
        extern volatile uint32_t ucpd_rxmsgend_count, ucpd_rxerr_count;
        extern volatile uint32_t ucpd_goodcrc_filtered, ucpd_rxovr_count;
        extern volatile uint32_t ucpd_rxhrstdet_count;
        char b[120];
        snprintf(b, sizeof(b), "TX: ok=%lu fail=%lu req=%lu\r\n",
            (unsigned long)ucpd_tx_ok, (unsigned long)ucpd_tx_fail,
            (unsigned long)ucpd_tx_count);
        out(b);
        snprintf(b, sizeof(b), "RX: msgend=%lu err=%lu ovr=%lu gcrc_filt=%lu hrst=%lu\r\n",
            (unsigned long)ucpd_rxmsgend_count, (unsigned long)ucpd_rxerr_count,
            (unsigned long)ucpd_rxovr_count, (unsigned long)ucpd_goodcrc_filtered,
            (unsigned long)ucpd_rxhrstdet_count);
        out(b);
        return true;
    }

    // colortest — draw labeled color swatches to calibrate panel
    if (match1(cmd, "COLORTEST")) {
        /* Standard RGB565 matching color565 in lcd.h */
        #define _c(r,g,b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))
        uint16_t W = _c(255,255,255), K = _c(0,0,0);
        LCD_Fill(0, 0, 319, 171, K);
        LCD_Fill(  0, 0,  52, 55, _c(255,0,0));
        LCD_PutStr(4, 58, (const char*)"RED", FONT_arial_17X18, W, K);
        LCD_Fill( 54, 0, 106, 55, _c(0,255,0));
        LCD_PutStr(58, 58, (const char*)"GREEN", FONT_arial_17X18, W, K);
        LCD_Fill(108, 0, 160, 55, _c(0,0,255));
        LCD_PutStr(112, 58, (const char*)"BLUE", FONT_arial_17X18, W, K);
        LCD_Fill(162, 0, 214, 55, _c(255,255,0));
        LCD_PutStr(166, 58, (const char*)"YELLOW", FONT_arial_17X18, W, K);
        LCD_Fill(216, 0, 268, 55, _c(20,160,20));
        LCD_PutStr(220, 58, (const char*)"DK GRN", FONT_arial_17X18, W, K);
        LCD_Fill(270, 0, 319, 55, _c(150,150,150));
        LCD_PutStr(274, 58, (const char*)"GREY", FONT_arial_17X18, W, K);
        #undef _c
        out("6 swatches shown - press any button to exit\r\n");
        /* Block here so UI_Update doesn't overwrite the test pattern */
        while (1) {
            IWDG->KR = 0xAAAAU;
            Buttons_Update();
            Buzzer_Update();
            if (Buttons_GetEvent() != 0) break;
            HAL_Delay(10);
        }
        /* Force full screen redraw on return */
        LCD_Fill(0, 0, 319, 171, (uint16_t)0);
        out("colortest done\r\n");
        return true;
    }

    // swaptest — empirical byte-order test: LCD_Fill vs LCD_DrawPixel side-by-side
    //
    // Draws four test colors, each as two adjacent rectangles:
    //   LEFT  = LCD_Fill (16-bit SPI path)
    //   RIGHT = LCD_DrawPixel loop (manual 8-bit path)
    //
    // If both rectangles in a row are the same color, the two paths agree.
    // If they differ, there is a byte-swap bug in the 16-bit SPI path.
    //
    // Test colors are chosen so that byte-swapping produces a visibly
    // different color (no palindromes, no pure single-channel values).
    if (match1(cmd, "SWAPTEST")) {
        extern SPI_HandleTypeDef hspi1;
        char b[96];

        // Report SPI hardware register state for diagnosis
        snprintf(b, sizeof(b),
                 "SPI1 CR1=0x%04lX CR2=0x%04lX (DS bits[11:8]=0x%lX = %lu-bit)\r\n",
                 (unsigned long)hspi1.Instance->CR1,
                 (unsigned long)hspi1.Instance->CR2,
                 (unsigned long)((hspi1.Instance->CR2 >> 8) & 0xF),
                 (unsigned long)(((hspi1.Instance->CR2 >> 8) & 0xF) + 1));
        out(b);

        // Raw RGB565 test colors — bypass color565() macro entirely.
        // Each is chosen so high byte != low byte (swap is visible).
        //
        // Standard RGB565 encoding: [15:11]=R, [10:5]=G, [4:0]=B
        //   0xFBE0 = R:31 G:31 B:0  = yellow/orange
        //   0x07FF = R:0  G:31 B:31  = cyan
        //   0xF81F = R:31 G:0  B:31  = magenta
        //   0xA145 = R:20 G:10 B:5   = muted brown (arbitrary asymmetric)
        //
        // NOTE: On the BRG-remapped panel these won't look like standard
        // colors, but that is IRRELEVANT — we only care whether the LEFT
        // and RIGHT rectangles in each row are identical.
        struct {
            uint16_t    val;
            const char *label;
        } tests[] = {
            { 0xFBE0, "0xFBE0" },   // swap -> 0xE0FB (very different)
            { 0x07FF, "0x07FF" },   // swap -> 0xFF07 (very different)
            { 0xF81F, "0xF81F" },   // swap -> 0x1FF8 (very different)
            { 0xA145, "0xA145" },   // swap -> 0x45A1 (very different)
        };
        const int N = 4;

        // Layout constants  (320x172 screen)
        const uint16_t ROW_H   = 30;    // height of each color row
        const uint16_t RECT_W  = 100;   // width of each rectangle
        const uint16_t GAP     = 6;     // gap between FILL and PIXEL rects
        const uint16_t X_FILL  = 6;     // left edge of FILL rect
        const uint16_t X_PIX   = X_FILL + RECT_W + GAP;  // = 112
        const uint16_t Y_START = 2;     // top of first row
        const uint16_t LBL_X   = X_PIX + RECT_W + 4;     // = 216

        // Black background
        LCD_Fill(0, 0, 319, 171, (uint16_t)0x0000);

        // Column headers
        uint16_t white = 0xFFFF;
        LCD_PutStr(X_FILL, Y_START, (const char*)"FILL",  FONT_arial_17X18, white, 0);
        LCD_PutStr(X_PIX,  Y_START, (const char*)"PIXEL", FONT_arial_17X18, white, 0);
        LCD_PutStr(LBL_X,  Y_START, (const char*)"VALUE", FONT_arial_17X18, white, 0);

        for (int i = 0; i < N; i++) {
            uint16_t color = tests[i].val;
            uint16_t y0 = Y_START + 20 + i * (ROW_H + 4);
            uint16_t y1 = y0 + ROW_H - 1;

            // LEFT rectangle: LCD_Fill (uses 16-bit SPI path)
            LCD_Fill(X_FILL, y0, X_FILL + RECT_W - 1, y1, color);

            // RIGHT rectangle: LCD_DrawPixel (uses manual 8-bit path)
            for (uint16_t py = y0; py <= y1; py++) {
                for (uint16_t px = X_PIX; px < X_PIX + RECT_W; px++) {
                    LCD_DrawPixel((int16_t)px, (int16_t)py, color);
                }
            }

            // Label: hex value
            LCD_PutStr(LBL_X, y0 + 8, (const char*)tests[i].label,
                       FONT_arial_17X18, white, 0);

            // Print to serial too
            snprintf(b, sizeof(b),
                     "Row %d: color=0x%04X  swapped=0x%04X  FILL=left  PIXEL=right\r\n",
                     i, (unsigned)color,
                     (unsigned)((color >> 8) | ((color & 0xFF) << 8)));
            out(b);

            // Feed watchdog — pixel loop is slow
            IWDG->KR = 0xAAAAU;
        }

        out("\r\n");
        out("RESULT: If FILL and PIXEL columns look IDENTICAL, byte order is correct.\r\n");
        out("        If they look DIFFERENT, LCD_Fill has a byte-swap bug.\r\n");
        out("\r\nPress any button to exit.\r\n");

        // Block so UI_Update doesn't overwrite the test pattern
        while (1) {
            IWDG->KR = 0xAAAAU;
            Buttons_Update();
            Buzzer_Update();
            if (Buttons_GetEvent() != 0) break;
            HAL_Delay(10);
        }

        // Force full screen redraw on return
        LCD_Fill(0, 0, 319, 171, (uint16_t)0);
        out("swaptest done\r\n");
        return true;
    }

    // gpio — read button GPIO pins for wiring debug
    if (match1(cmd, "GPIO")) {
        char b[80];
        snprintf(b, sizeof(b), "PB1(DEC)=%u PB2(INC)=%u PB10(SEL)=%u PB11(PWR)=%u\r\n",
            (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1),
            (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2),
            (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10),
            (unsigned)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11));
        out(b);
        return true;
    }

    // list [all|epr] / pdos [all|epr]
    if (match1(cmd, "LIST") || match1(cmd, "PDOS")) {
        if (nt >= 2) {
            char a[8]; strncpy(a, tok[1], sizeof(a)-1); a[sizeof(a)-1]=0;
            uppercase(a);
            if (match1(a, "ALL") || match1(a, "EPR")) {
                do_pd_pdo_list_all(); return true;
            }
        }
        do_pd_pdo_list(); return true;
    }

    // ct / contract
    if (match1(cmd, "CT") || match(cmd, "CONTR", "CONTRACT")) {
        do_pd_contract(); return true;
    }

    // fault — show fault status
    if (match(cmd, "FAULT", "FAULTS")) {
        do_protect_status();
        return true;
    }

    /* stream command handled earlier (with on/off args + toggle) */

    // meas
    if (match(cmd, "MEAS", "MEASURE")) {
        extern volatile float g_ntc_temp;
        extern I2C_HandleTypeDef hi2c3;
        INA228_Reading_t rd = {};
        HAL_StatusTypeDef rc = INA228_ReadAll(&g_ina, &rd);
        char b[96];
        const char *tunit = Settings_GetTempFahrenheit() ? "F" : "C";
        snprintf(b, sizeof(b), "V=%.3f I=%.3f Tdie=%.1f%s Tntc=%.1f%s i2c=%d s=%lu e=0x%lX",
                 (double)rd.voltage_v, (double)rd.current_a,
                 (double)temp_display(rd.die_temp_c), tunit,
                 (double)temp_display(g_ntc_temp), tunit,
                 (int)rc, (unsigned long)hi2c3.State,
                 (unsigned long)hi2c3.ErrorCode);
        out(b); outln();
        return true;
    }

    // i2cscan - scan I2C3 bus for responding devices
    if (match1(cmd, "I2CSCAN")) {
        extern I2C_HandleTypeDef hi2c3;
        out("Scanning I2C3 bus...\r\n");
        int found = 0;
        for (uint8_t addr = 1; addr < 127; addr++) {
            if (HAL_I2C_IsDeviceReady(&hi2c3, addr << 1, 1, 10) == HAL_OK) {
                char b[32];
                snprintf(b, sizeof(b), "  0x%02X ACK\r\n", addr);
                out(b);
                found++;
            }
        }
        if (found == 0) out("  No devices found!\r\n");
        char b[32]; snprintf(b, sizeof(b), "Done (%d device%s)\r\n", found, found==1?"":"s");
        out(b);
        return true;
    }

    // ina228diag - read manufacturer ID and report init status
    if (match(cmd, "INA228", "INA228DIAG")) {
        extern I2C_HandleTypeDef hi2c3;
        uint8_t rx[2] = {0};
        char b[80];

        // Report I2C state before recovery
        snprintf(b, sizeof(b), "I2C3 state=%lu err=0x%04lX\r\n",
                 (unsigned long)hi2c3.State, (unsigned long)hi2c3.ErrorCode);
        out(b);

        // Force full I2C reinit to clear any lingering errors
        out("Resetting I2C3...\r\n");
        HAL_I2C_DeInit(&hi2c3);
        HAL_Delay(10);
        HAL_I2C_Init(&hi2c3);
        HAL_Delay(10);
        snprintf(b, sizeof(b), "After reset: state=%lu err=0x%04lX\r\n",
                 (unsigned long)hi2c3.State, (unsigned long)hi2c3.ErrorCode);
        out(b);

        // Probe address first
        HAL_StatusTypeDef probe = HAL_I2C_IsDeviceReady(&hi2c3, 0x40<<1, 3, 100);
        snprintf(b, sizeof(b), "Address probe 0x40: %s\r\n",
                 probe==HAL_OK?"ACK":"NACK");
        out(b);

        // Pre-init register read test
        uint8_t t2[2];
        HAL_StatusTypeDef r;
        r = HAL_I2C_Mem_Read(&hi2c3, 0x40<<1, 0x01, I2C_MEMADD_SIZE_8BIT, t2, 2, 50);
        snprintf(b, sizeof(b), "Pre-init ADC_CFG(0x01): %s 0x%02X%02X\r\n",
                 r==HAL_OK?"OK":"FAIL", t2[0], t2[1]); out(b);
        r = HAL_I2C_Mem_Read(&hi2c3, 0x40<<1, 0x06, I2C_MEMADD_SIZE_8BIT, t2, 2, 50);
        snprintf(b, sizeof(b), "Pre-init DIETEMP(0x06): %s 0x%02X%02X\r\n",
                 r==HAL_OK?"OK":"FAIL", t2[0], t2[1]); out(b);

        // Read manufacturer ID
        HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c3, 0x40<<1, 0x3E,
                                                  I2C_MEMADD_SIZE_8BIT,
                                                  rx, 2, 50);
        uint16_t mfg_id = (rx[0] << 8) | rx[1];
        snprintf(b, sizeof(b), "MFG_ID read: %s  val=0x%04X (expect 0x5449)\r\n",
                 ret==HAL_OK?"OK":"FAIL", mfg_id);
        out(b);

        // If INA228 found, do full init + read
        if (mfg_id == 0x5449) {
            ret = INA228_Init(&g_ina, &hi2c3, 0.0068f, 6.0f);
            snprintf(b, sizeof(b), "INA228_Init: %s\r\n", ret==HAL_OK?"OK":"FAIL");
            out(b);
            if (ret == HAL_OK) {
                INA228_SetAlertOverCurrent(&g_ina, 5.5f);
                HAL_Delay(100);  // let ADC do first conversion

                // Read back registers using Master_Transmit/Receive
                // (same method the driver now uses)
                uint8_t reg, r2[2];

                reg = 0x01;
                ret = HAL_I2C_Master_Transmit(&hi2c3, 0x80, &reg, 1, 50);
                if (ret == HAL_OK) ret = HAL_I2C_Master_Receive(&hi2c3, 0x80, r2, 2, 50);
                snprintf(b, sizeof(b), "ADC_CFG(0x01): %s 0x%02X%02X (expect 0xF0AA)\r\n",
                         ret==HAL_OK?"OK":"FAIL", r2[0], r2[1]); out(b);

                reg = 0x02;
                ret = HAL_I2C_Master_Transmit(&hi2c3, 0x80, &reg, 1, 50);
                if (ret == HAL_OK) ret = HAL_I2C_Master_Receive(&hi2c3, 0x80, r2, 2, 50);
                snprintf(b, sizeof(b), "SHUNT_CAL(0x02): %s 0x%02X%02X\r\n",
                         ret==HAL_OK?"OK":"FAIL", r2[0], r2[1]); out(b);

                reg = 0x06;
                ret = HAL_I2C_Master_Transmit(&hi2c3, 0x80, &reg, 1, 50);
                if (ret == HAL_OK) ret = HAL_I2C_Master_Receive(&hi2c3, 0x80, r2, 2, 50);
                int16_t raw_t = (int16_t)((r2[0]<<8)|r2[1]) >> 4;
                float t = raw_t * 7.8125e-3f;
                snprintf(b, sizeof(b), "DIETEMP(0x06): %s 0x%02X%02X (%.1f C)\r\n",
                         ret==HAL_OK?"OK":"FAIL", r2[0], r2[1], (double)t); out(b);

                // Full read via driver
                INA228_Reading_t rd = {};
                ret = INA228_ReadAll(&g_ina, &rd);
                snprintf(b, sizeof(b), "ReadAll: %s V=%.3f I=%.3f T=%.1f C\r\n",
                         ret==HAL_OK?"OK":"FAIL",
                         (double)rd.voltage_v, (double)rd.current_a,
                         (double)rd.die_temp_c);
                out(b);
            }
        }

        return true;
    }

    // btntest — read raw button GPIO state
    if (match(cmd, "BTNT", "BTNTEST")) {
        char b[80];
        out("Reading buttons for 5 seconds... press buttons now\r\n");
        for (int i = 0; i < 50; i++) {
            uint8_t dec = HAL_GPIO_ReadPin(BUTTON_DECREASE_GPIO_Port, BUTTON_DECREASE_Pin) == GPIO_PIN_SET;
            uint8_t inc = HAL_GPIO_ReadPin(BUTTON_INCREASE_GPIO_Port, BUTTON_INCREASE_Pin) == GPIO_PIN_SET;
            uint8_t sel = HAL_GPIO_ReadPin(BUTTON_SELECT_GPIO_Port, BUTTON_SELECT_Pin) == GPIO_PIN_SET;
            uint8_t pwr = HAL_GPIO_ReadPin(BUTTON_POWER_GPIO_Port, BUTTON_POWER_Pin) == GPIO_PIN_SET;
            if (dec || inc || sel || pwr) {
                snprintf(b, sizeof(b), "DEC=%d INC=%d SEL=%d PWR=%d\r\n", dec, inc, sel, pwr);
                out(b);
            }
            HAL_Delay(100);
            IWDG->KR = 0xAAAAU;
        }
        out("Done\r\n");
        return true;
    }

    // lcdtest — bare-bones SPI test to the display
    if (match(cmd, "LCDT", "LCDTEST")) {
        extern SPI_HandleTypeDef hspi1;
        char b[64];

        // Report SPI state
        snprintf(b, sizeof(b), "SPI1 state=%lu err=0x%lX CR1=0x%04lX CR2=0x%04lX\r\n",
                 (unsigned long)hspi1.State, (unsigned long)hspi1.ErrorCode,
                 (unsigned long)hspi1.Instance->CR1, (unsigned long)hspi1.Instance->CR2);
        out(b);

        // Hardware reset the display
        HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_RESET);
        HAL_Delay(10);
        HAL_GPIO_WritePin(TFT_RESET_GPIO_Port, TFT_RESET_Pin, GPIO_PIN_SET);
        HAL_Delay(200);

        // Send SLPOUT command (0x11) — DC=low for command
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
        uint8_t cmd_slpout = 0x11;
        HAL_StatusTypeDef rc = HAL_SPI_Transmit(&hspi1, &cmd_slpout, 1, 100);
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);
        snprintf(b, sizeof(b), "SLPOUT: %s\r\n", rc==HAL_OK?"OK":"FAIL"); out(b);
        HAL_Delay(120);

        // Send COLMOD (0x3A, 0x55) — 16bit color
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
        uint8_t cmd_colmod = 0x3A;
        rc = HAL_SPI_Transmit(&hspi1, &cmd_colmod, 1, 100);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
        uint8_t colmod_arg = 0x55;
        rc = HAL_SPI_Transmit(&hspi1, &colmod_arg, 1, 100);
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

        // Send INVON (0x21)
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
        uint8_t cmd_invon = 0x21;
        HAL_SPI_Transmit(&hspi1, &cmd_invon, 1, 100);
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

        // Send DISPON (0x29)
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
        uint8_t cmd_dispon = 0x29;
        HAL_SPI_Transmit(&hspi1, &cmd_dispon, 1, 100);
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

        // Fill screen with red: set address window then send pixel data
        // CASET (0x2A) + RASET (0x2B) + RAMWR (0x2C)
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
        uint8_t cmd_caset = 0x2A;
        HAL_SPI_Transmit(&hspi1, &cmd_caset, 1, 100);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
        uint8_t caset_args[] = {0x00, 0x00, 0x00, 0xEF}; // 0-239
        HAL_SPI_Transmit(&hspi1, caset_args, 4, 100);
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
        uint8_t cmd_raset = 0x2B;
        HAL_SPI_Transmit(&hspi1, &cmd_raset, 1, 100);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
        uint8_t raset_args[] = {0x00, 0x22, 0x01, 0x53}; // 34-339 (172 rows with offset 34)
        HAL_SPI_Transmit(&hspi1, raset_args, 4, 100);
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

        // RAMWR + red pixels
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
        uint8_t cmd_ramwr = 0x2C;
        HAL_SPI_Transmit(&hspi1, &cmd_ramwr, 1, 100);
        HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_SET);
        // Send 100 red pixels (0xF800 in 565)
        uint8_t red_pixel[] = {0xF8, 0x00};
        for (int i = 0; i < 100; i++) {
            HAL_SPI_Transmit(&hspi1, red_pixel, 2, 100);
        }
        HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

        out("Done. Check screen for red pixels.\r\n");
        return true;
    }

    // epr / spr
    if (match1(cmd, "EPR")) { do_pd_mode("EPR", false); return true; }
    if (match1(cmd, "SPR")) { do_pd_mode("SPR", false); return true; }

    // trace on|off — driver-level + PE-state diagnostic prints.
    // Default OFF; enable when diagnosing HR loops or unexpected behaviour.
    if (match1(cmd, "TRACE")) {
        if (nt < 2) { do_syst_trace(nullptr, true); return true; }
        do_syst_trace(tok[1], false); return true;
    }

    // selftest — walk every advertised PDO + PPS/AVS sample points and report
    if (match1(cmd, "SELFTEST") || match1(cmd, "TEST")) {
        do_selftest();
        return true;
    }

    // buzzer frequency sweep — test to find resonant frequency
    if (match1(cmd, "BUZZSWEEP")) {
        out("buzzer sweep 800-5000 Hz...\r\n");
        Buzzer_FreqSweep();
        out("done\r\n");
        return true;
    }

    // reboot — full MCU reset
    if (match1(cmd, "REBOOT")) { do_reboot(); return true; }

    // dfu — jump to STM32 system memory bootloader
    if (match1(cmd, "DFU") || match1(cmd, "FWUPD") || match1(cmd, "BOOTLOADER")) {
        do_dfu();
        return true;
    }

    // mode AUTO|FIX|PPS|AVS — filter that 'set' uses to pick a PDO variant
    if (match1(cmd, "MODE")) {
        if (nt < 2) { do_sour_mode(nullptr, true); return true; }
        do_sour_mode(tok[1], false); return true;
    }

    // set <V> [<I>] — generic auto-select by voltage. Filtered by 'mode'
    // (default AUTO = any PDO type). For discrete-PDO or PPS/AVS specifics,
    // use setpdo / setpps / setavs.
    if (match1(cmd, "SET")) {
        if (!s_port || !s_pe) { err_push(-240, "Hardware missing"); return true; }
        if (nt < 2) { err_push(-109, "Missing parameter"); out("Usage: set <V> [<A>]\r\n"); return true; }
        uint32_t mv = 0;
        if (!parse_numeric_unit(tok[1], 'V', &mv) || mv == 0) { err_push(-222, "bad voltage"); out("Invalid voltage\r\n"); return true; }

        // Auto-enter EPR if the requested voltage is in the EPR range and
        // we're not already there. SPR caps only show up to 20-21 V, so a
        // 'set 28' from SPR would otherwise just print "not satisfied".
        // Match the convenience behaviour of 'setpdo 8'.
        bool src_epr_capable = (s_port->source_caps.size() > 0) &&
                               ((s_port->source_caps[0] >> 23) & 1);
        if (mv > 21000 && src_epr_capable && !s_pe->is_in_epr_mode()) {
            reset_epr_fail_tracker();
            user_wants_epr = true;
            target_mv = mv;
            target_ma = 0;
            if (nt >= 3) { uint32_t a; if (parse_numeric_unit(tok[2], 'A', &a)) target_ma = a; }
            // source_mode left at whatever the user picked; AUTO by default.
            s_dpm->trigger_any(target_mv, target_ma);  // pre-arm for after entry
            s_port->pe_flags.clear(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
            s_port->dpm_requests.set(pd::DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
            char b[96];
            snprintf(b, sizeof(b),
                     "entering EPR first (%u mV is above SPR 21 V ceiling)\r\n",
                     (unsigned)mv);
            out(b);
            return true;
        }

        char why[96];
        if (!voltage_reachable(mv, source_mode, why, sizeof(why))) {
            out(why); out("\r\n"); err_push(-222, "voltage out of range");
            return true;
        }
        do_sour_volt(tok[1], false);
        if (nt >= 3) do_sour_curr(tok[2], false);
        return true;
    }

    // setpdo <N> — request Fixed PDO by its position (1-based). Reads the
    // advertised voltage/current from the PDO itself. Rejects if PDO N is
    // not a Fixed type (use setpps / setavs for APDOs instead).
    if (match1(cmd, "SETPDO")) {
        if (!s_port || !s_dpm) { err_push(-240, "Hardware missing"); return true; }
        if (nt < 2) { err_push(-109, "usage: setpdo N"); out("Usage: setpdo <N> (1-11)\r\n"); return true; }
        char* endp = nullptr;
        unsigned long nl = strtoul(tok[1], &endp, 10);
        if (endp == tok[1] || nl == 0 || nl > 11) {
            err_push(-222, "PDO position out of range (1..11)");
            out("Invalid PDO index (1-11)\r\n");
            return true;
        }
        uint32_t n = static_cast<uint32_t>(nl);
        bool src_epr_capable = (s_port->source_caps.size() > 0) &&
                               ((s_port->source_caps[0] >> 23) & 1);
        bool want_epr_slot = (n >= 8);

        // EPR slots (8..11) only appear in source_caps after EPR entry.
        //
        // Key observation: Anker A2697 doesn't complete intra-EPR voltage
        // jumps cleanly (e.g. 5 V EPR -> 28 V EPR times out at PS_RDY with
        // Hard Reset). But it DOES complete a fresh EPR entry with the
        // initial EPR_Request already pointed at the desired high PDO.
        //
        // So: pre-arm the DPM with trigger_by_position(N) BEFORE setting
        // EPR_MODE_ENTRY. pdsink's EPR-entry path leads to
        // PE_SNK_Wait_for_Capabilities -> Evaluate_Capability, where the
        // DPM's trigger_position selects PDO N directly. The first
        // EPR_Request after Enter_Succeeded is therefore for PDO N, no
        // intermediate 5 V hop.
        if (want_epr_slot) {
            if (!src_epr_capable) {
                err_push(-221, "source does not advertise EPR capability");
                return true;
            }
            // If we're already in EPR mode, the full 11-PDO cap set is live.
            // Check that position N is actually populated (many sources pad
            // unused EPR slots to 0 - Anker A2697 uses slots 8 and 9 only,
            // leaving 10 and 11 as padding). Without this check, requesting
            // a padded slot falls back to PDO1 5 V silently.
            if (s_pe->is_in_epr_mode()) {
                if (n > s_port->source_caps.size() ||
                    s_port->source_caps[n - 1] == 0)
                {
                    char b[72];
                    snprintf(b, sizeof(b),
                             "PDO %u not advertised by source in EPR caps\r\n",
                             (unsigned)n);
                    out(b);
                    err_push(-222, "EPR slot not populated");
                    return true;
                }
            }
            // Arm trigger first (this also sets NEW_POWER_LEVEL - harmless;
            // the EPR_MODE_ENTRY request is processed first in SNK_Ready).
            s_dpm->trigger_by_position(static_cast<uint8_t>(n));
            g_fault_suppress_until = HAL_GetTick() + 2000U;
            if (!s_pe->is_in_epr_mode()) {
                reset_epr_fail_tracker();
                user_wants_epr = true;
                s_port->pe_flags.clear(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
                s_port->dpm_requests.set(pd::DPM_REQUEST_FLAG::EPR_MODE_ENTRY);
            }
            char b[56];
            snprintf(b, sizeof(b), "Requesting EPR PDO %u\r\n", (unsigned)n);
            out(b);
            return true;
        }

        // SPR slot (1..7): validate it's advertised + is Fixed.
        if (n > s_port->source_caps.size()) {
            err_push(-222, "position not advertised by source");
            return true;
        }
        uint32_t pdo = s_port->source_caps[n - 1];
        unsigned type = (pdo >> 30) & 0x3;
        if (type != 0) {
            err_push(-221, "PDO is not Fixed - use setpps / setavs instead");
            return true;
        }
        s_dpm->trigger_by_position(static_cast<uint8_t>(n));
        /* Suppress fault detection during VBUS transition (inrush trips LTC4368) */
        g_fault_suppress_until = HAL_GetTick() + 2000U;
        // Restore default OCP (PPS CC override no longer active)
        INA228_SetAlertOverCurrent(&g_ina, (float)Settings_GetOcpMa() / 1000.0f);
        char b[56]; snprintf(b, sizeof(b), "Requesting Fixed PDO %u\r\n", (unsigned)n);
        out(b);
        return true;
    }

    // setpps <V> [<I>] — SPR PPS target (20 mV step, 50 mA step)
    if (match1(cmd, "SETPPS")) {
        if (!s_dpm || !s_port) { err_push(-240, "Hardware missing"); return true; }
        if (nt < 2) { err_push(-109, "usage: setpps V [I]"); out("Usage: setpps <V> [<A>]\r\n"); return true; }
        uint32_t mv = 0, ma = 0;
        if (!parse_numeric_unit(tok[1], 'V', &mv)) { err_push(-222, "bad voltage"); out("Invalid voltage\r\n"); return true; }
        if (nt >= 3 && !parse_numeric_unit(tok[2], 'A', &ma)) { err_push(-222, "bad current"); out("Invalid current\r\n"); return true; }

        // Walk source_caps for any PPS APDO (subtype 0) whose [vmin, vmax]
        // contains the requested voltage. If the user supplied a current,
        // also check it doesn't exceed the PDO's max_current. pdsink's DPM
        // silently falls back to PDO1 5 V on no-match.
        unsigned best_min = 0, best_max = 0;
        bool have_pps = false;
        for (size_t i = 0; i < s_port->source_caps.size(); i++) {
            uint32_t pdo = s_port->source_caps[i];
            if (((pdo >> 30) & 0x3) != 3) continue;           // must be APDO
            if (((pdo >> 28) & 0x3) != 0) continue;           // must be PPS subtype
            unsigned vmax = ((pdo >> 17) & 0xFF) * 100;
            unsigned vmin = ((pdo >> 8)  & 0xFF) * 100;
            unsigned imax = (pdo & 0x7F) * 50;
            if (!have_pps) { best_min = vmin; best_max = vmax; have_pps = true; }
            if (mv < vmin || mv > vmax) continue;
            if (ma > 0 && ma > imax) {
                char b[96];
                snprintf(b, sizeof(b),
                         "%u mA exceeds PPS PDO%u max %u.%03u A\r\n",
                         (unsigned)ma, (unsigned)(i+1),
                         imax / 1000, imax % 1000);
                out(b);
                err_push(-222, "PPS current out of range");
                return true;
            }
            target_mv = mv; target_ma = ma; source_mode = SourceMode::PPS;
            s_dpm->trigger_variant(pd::PDO_VARIANT::APDO_PPS, mv, ma);
            // If user specified a CC current limit, set INA228 OCP to enforce
            // it locally in case the charger doesn't honour the operating
            // current in the PPS RDO.  Restore the default OCP when the
            // current argument is omitted (ma == 0).
            if (ma > 0) {
                float cc_amps = (float)ma / 1000.0f * 1.15f; // 15% margin
                INA228_SetAlertOverCurrent(&g_ina, cc_amps);
            } else {
                float def_amps = (float)Settings_GetOcpMa() / 1000.0f;
                INA228_SetAlertOverCurrent(&g_ina, def_amps);
            }
            char b[64];
            snprintf(b, sizeof(b), "Requesting PPS %u mV / %u mA\r\n",
                     (unsigned)mv, (unsigned)ma);
            out(b);
            return true;
        }

        char b[96];
        if (have_pps) {
            snprintf(b, sizeof(b),
                     "%u mV outside advertised PPS range "
                     "(%u.%03u - %u.%03u V)\r\n",
                     (unsigned)mv,
                     best_min / 1000, best_min % 1000,
                     best_max / 1000, best_max % 1000);
        } else {
            snprintf(b, sizeof(b), "source advertises no PPS PDO\r\n");
        }
        out(b);
        err_push(-222, "PPS voltage out of range");
        return true;
    }

    // setavs <V> — EPR AVS (if in EPR), otherwise SPR AVS (PD 3.2 sources).
    // Validate V is within the source's advertised AVS range first — pdsink's
    // DPM would otherwise silently fall back to PDO1 5 V on no-match, which
    // looks like "the command just didn't work".
    if (match1(cmd, "SETAVS")) {
        if (!s_dpm || !s_pe || !s_port) { err_push(-240, "Hardware missing"); return true; }
        if (nt < 2) { err_push(-109, "usage: setavs V"); out("Usage: setavs <V>\r\n"); return true; }
        uint32_t mv = 0;
        if (!parse_numeric_unit(tok[1], 'V', &mv)) { err_push(-222, "bad voltage"); out("Invalid voltage\r\n"); return true; }

        // Walk source_caps for any AVS APDO whose range contains mv.
        unsigned best_min = 0, best_max = 0;
        bool have_avs = false;
        for (size_t i = 0; i < s_port->source_caps.size(); i++) {
            uint32_t pdo = s_port->source_caps[i];
            if (((pdo >> 30) & 0x3) != 3) continue;          // must be APDO
            unsigned sub = (pdo >> 28) & 0x3;
            unsigned vmin = 0, vmax = 0;
            pd::PDO_VARIANT variant = pd::PDO_VARIANT::UNKNOWN;
            if (sub == 1) {                                   // EPR AVS
                vmin = ((pdo >> 8)  & 0xFF)  * 100;
                vmax = ((pdo >> 17) & 0x1FF) * 100;
                variant = pd::PDO_VARIANT::APDO_EPR_AVS;
            } else if (sub == 2) {                            // SPR AVS (PD 3.2)
                vmin = 9000; vmax = 20000;
                variant = pd::PDO_VARIANT::APDO_SPR_AVS;
            } else {
                continue;
            }
            if (!have_avs) { best_min = vmin; best_max = vmax; have_avs = true; }
            if (mv >= vmin && mv <= vmax) {
                target_mv = mv; target_ma = 0; source_mode = SourceMode::AVS;
                s_dpm->trigger_variant(variant, mv, 0);
                char b[64];
                snprintf(b, sizeof(b), "Requesting %s AVS %u mV\r\n",
                         (sub == 1) ? "EPR" : "SPR", (unsigned)mv);
                out(b);
                return true;
            }
        }

        char b[96];
        if (have_avs) {
            snprintf(b, sizeof(b),
                     "%u mV outside advertised AVS range "
                     "(%u.%03u - %u.%03u V)\r\n",
                     (unsigned)mv,
                     best_min / 1000, best_min % 1000,
                     best_max / 1000, best_max % 1000);
        } else {
            snprintf(b, sizeof(b),
                     "source advertises no AVS PDO%s\r\n",
                     s_pe->is_in_epr_mode() ? "" : " - try 'epr' first");
        }
        out(b);
        err_push(-222, "AVS voltage out of range");
        return true;
    }

    // protect ocp|ovp|status?|clear
    if (match(cmd, "PROT", "PROTECT")) {
        if (nt < 2) {
            do_protect_status(); return true;
        }
        char sub[12]; strncpy(sub, tok[1], sizeof(sub)-1); sub[sizeof(sub)-1]=0; uppercase(sub);
        if (match1(sub, "OCP")) {
            do_protect_ocp(nt >= 3 ? tok[2] : nullptr); return true;
        }
        if (match1(sub, "OVP")) {
            do_protect_ovp(nt >= 3 ? tok[2] : nullptr); return true;
        }
        if (match1(sub, "STATUS") || match1(sub, "STATUS?") || match1(sub, "STAT")) {
            do_protect_status(); return true;
        }
        if (match1(sub, "CLEAR") || match1(sub, "CLR")) {
            do_protect_clear(); return true;
        }
        err_push(-113, "Undefined protect sub-command"); return true;
    }

    // seq add|clear|list|run|stop
    if (match1(cmd, "SEQ")) {
        if (nt < 2) { out("usage: seq add|clear|list|run|stop\r\n"); return true; }
        char sub[8]; strncpy(sub, tok[1], sizeof(sub)-1); sub[sizeof(sub)-1]=0; uppercase(sub);
        if (match1(sub, "ADD")) {
            // pass tok[1..3] as tok[1..3] but shift: tok[0]=cmd, tok[1]="add", tok[2]=mv,tok[3]=ma,tok[4]=dwell
            // re-tokenize with 5 tokens
            char work2[CLI_LINE_MAX];
            strncpy(work2, line, sizeof(work2)-1); work2[sizeof(work2)-1]=0;
            char* tok5[5] = {};
            int nt5 = 0;
            char* p = work2;
            while (*p && nt5 < 5) {
                while (*p == ' ' || *p == '\t') p++;
                if (!*p) break;
                tok5[nt5++] = p;
                while (*p && *p != ' ' && *p != '\t') p++;
                if (*p) { *p = 0; p++; }
            }
            // After re-tokenizing, verify we have enough parameters
            // nt5 includes "SEQ" and "ADD", so need at least 4 (seq add <V> <dwell>)
            if (nt5 < 4) {
                out("usage: seq add <voltage> <dwell_ms>\r\n");
                return true;
            }
            // tok5[0]=SEQ tok5[1]=ADD tok5[2]=mv tok5[3]=ma tok5[4]=dwell
            do_seq_add(tok5 + 1, nt5 - 1); return true;
        }
        if (match1(sub, "CLEAR") || match1(sub, "CLR")) {
            if (seq_running) { out("Cannot clear while sequence is running\r\n"); return true; }
            seq_count = 0; out("Sequence cleared\r\n"); return true;
        }
        if (match1(sub, "LIST")) { do_seq_list(); return true; }
        if (match1(sub, "RUN"))  { do_seq_run();  return true; }
        if (match1(sub, "STOP")) { seq_stop_flag = 1; out("Sequence stop requested\r\n"); return true; }
        err_push(-113, "Undefined seq sub-command"); return true;
    }

    return false;  // fall through to SCPI path
}

// -----------------------------------------------------------------------------
// Dispatcher
// -----------------------------------------------------------------------------

static void dispatch_one(char* line);

static void dispatch(char* line) {
    // SCPI allows ';'-chained commands on one line. Split and dispatch each.
    // Works for shortcuts too (e.g. 'mode avs; set 24V').
    while (*line) {
        char* semi = line;
        while (*semi && *semi != ';') semi++;
        bool more = (*semi == ';');
        if (more) *semi = 0;
        dispatch_one(line);
        if (!more) break;
        line = semi + 1;
    }
}

static void dispatch_one(char* line) {
    line = trim(line);
    if (!*line) return;

    // Self-test confirmation gate: the prompt printed by do_selftest() asks
    // the user to type 'OK' to proceed. The very next non-empty line is
    // consumed here as the answer, regardless of what it is.
    if (selftest_pending_confirm) {
        selftest_pending_confirm = false;
        char ans[4]; strncpy(ans, line, 3); ans[3] = 0; uppercase(ans);
        if (strcmp(ans, "OK") == 0) {
            out("Confirmed - starting self-test.\r\n");
            selftest_start_run();
        } else {
            out("Self-test ABORTED (no 'OK' received).\r\n");
            err_push(-200, "Self-test aborted by user");
        }
        return;
    }

    // Try the interactive shortcut layer first; SCPI lines (starting with
    // '*' or ':') fall through.
    if (line[0] != '*' && line[0] != ':') {
        if (try_shortcut(line)) return;
    }

    // Split into path + argument at the first whitespace after the mnemonic.
    char* arg = nullptr;
    for (char* p = line; *p; p++) {
        if (*p == ' ' || *p == '\t') {
            *p = 0; arg = trim(p + 1); break;
        }
    }

    // Common commands ("*IDN?", etc.)
    if (line[0] == '*') {
        char cmd[8]; strncpy(cmd, line + 1, sizeof(cmd) - 1); cmd[sizeof(cmd)-1] = 0;
        bool q = false; size_t n = strlen(cmd);
        if (n > 0 && cmd[n-1] == '?') { q = true; cmd[n-1] = 0; }
        uppercase(cmd);
        if      (match1(cmd, "IDN") && q)   do_idn();
        else if (match1(cmd, "RST") && !q)  do_rst();
        else if (match1(cmd, "CLS") && !q)  do_cls();
        else if (match1(cmd, "OPC") && q) {
            // *OPC? — returns "1" when the last requested voltage has settled
            out(axxpd_is_opc_done() ? "1" : "0"); outln();
        }
        else if (match1(cmd, "WAI") && !q) {
            // *WAI — block until operation complete or 5 s timeout
            uint32_t t0 = HAL_GetTick();
            while (!axxpd_is_opc_done()) {
                IWDG->KR = 0xAAAAU;
                axxpd_run();
                if ((uint32_t)(HAL_GetTick() - t0) >= 5000U) {
                    out("TIMEOUT\r\n"); return;
                }
            }
            out("OK\r\n");
        }
        else err_push(-113, "Undefined header");
        return;
    }

    // SCPI path commands
    char* parts[6] = {};
    bool query = false;
    int np = split_path(line, parts, &query);
    if (np == 0) { err_push(-100, "Command error"); return; }

    // :OUTPut[:STATe] {ON|OFF|1|0} / :OUTPut?
    if (match(parts[0], "OUTP", "OUTPUT")) {
        if (query) {
            out(g_output_enabled ? "1\r\n" : "0\r\n");
        } else if (arg) {
            char a[4]; strncpy(a, arg, 3); a[3] = 0; uppercase(a);
            if (match1(a, "ON") || match1(a, "1")) {
                uint8_t r = Output_Enable_Guarded();
                if (r == 1) {
                    err_push(-200, "Output toggle cooldown active");
                    out("Error: toggle cooldown (min 1.5s between enables)\r\n");
                } else if (r == 2) {
                    err_push(-200, "Thermal cooldown active");
                    out("Error: board too hot - thermal cooldown\r\n");
                } else {
                    g_output_enabled = 1;
                    out("OK\r\n");
                }
            } else if (match1(a, "OFF") || match1(a, "0")) {
                Output_Disable(); g_output_enabled = 0;
                out("OK\r\n");
            } else {
                err_push(-222, "Expected ON|OFF|1|0");
                out("Error: expected ON|OFF|1|0\r\n");
            }
        } else {
            err_push(-109, "Missing parameter");
            out("Usage: :OUTP ON|OFF\r\n");
        }
        return;
    }

    // :SYSTem:*
    if (match(parts[0], "SYST", "SYSTEM")) {
        if (np >= 2 && match(parts[1], "HELP", "HELP") && query) { do_syst_help(); return; }
        if (np >= 2 && match(parts[1], "ERR", "ERROR") && query) { do_syst_err(); return; }
        if (np >= 2 && match(parts[1], "EVEN", "EVENT"))         { do_syst_events(arg, query); return; }
        if (np >= 2 && match(parts[1], "TRAC", "TRACE"))         { do_syst_trace(arg, query); return; }
        if (np >= 2 && match(parts[1], "REB", "REBOOT") && !query) { do_reboot(); return; }
        if (np >= 2 && match(parts[1], "DFU", "DFU") && !query)  { do_dfu(); return; }
        if (np >= 2 && match(parts[1], "TEST", "TEST") && !query) { do_selftest(); return; }
        if (np >= 2 && match(parts[1], "LOCK", "LOCK")) {
            if (query) {
                out(UI_IsLocked() ? "1\r\n" : "0\r\n");
            } else if (arg) {
                char a[4]; strncpy(a, arg, 3); a[3] = 0; uppercase(a);
                if (match1(a, "ON") || match1(a, "1")) { UI_SetLocked(1); out("OK\r\n"); }
                else if (match1(a, "OFF") || match1(a, "0")) { UI_SetLocked(0); out("OK\r\n"); }
                else { err_push(-222, "Expected ON|OFF|1|0"); out("Error: expected ON|OFF|1|0\r\n"); }
            } else {
                err_push(-109, "Missing parameter");
            }
            return;
        }
        err_push(-113, "Undefined header"); out("Unknown command\r\n"); return;
    }

    // :SOURce:*
    if (match(parts[0], "SOUR", "SOURCE")) {
        if (np >= 2 && match(parts[1], "VOLT", "VOLTAGE")) { do_sour_volt(arg, query); return; }
        if (np >= 2 && match(parts[1], "CURR", "CURRENT")) { do_sour_curr(arg, query); return; }
        if (np >= 2 && match(parts[1], "MODE", "MODE"))    { do_sour_mode(arg, query); return; }
        if (np >= 2 && match(parts[1], "APPL", "APPLY"))   { do_sour_apply(); return; }
        err_push(-113, "Undefined header"); out("Unknown command\r\n"); return;
    }

    // :MEASure:*
    if (match(parts[0], "MEAS", "MEASURE")) {
        if (np >= 2 && match(parts[1], "VOLT", "VOLTAGE") && query) { do_meas_volt(); return; }
        if (np >= 2 && match(parts[1], "CURR", "CURRENT") && query) { do_meas_curr(); return; }
        if (np >= 2 && match(parts[1], "ALL",  "ALL")     && query) { do_meas_all();  return; }
        if (np >= 2 && match(parts[1], "POW",  "POWER")   && query) { do_meas_pow();  return; }
        if (np >= 2 && match(parts[1], "TEMP", "TEMP")    && query) { do_meas_temp(); return; }
        if (np >= 2 && match(parts[1], "ENER", "ENERGY")) {
            // :MEAS:ENER:RES[ET] — reset energy counters
            if (np >= 3 && match(parts[2], "RES", "RESET") && !query) {
                INA228_ResetEnergy(&g_ina);
                Energy_SessionReset();
                out("Energy counters reset\r\n");
                return;
            }
            if (query) { do_meas_ener(); return; }
        }
        err_push(-113, "Undefined header"); out("Unknown command\r\n"); return;
    }

    // :PD:*
    if (match1(parts[0], "PD")) {
        if (np >= 2 && match(parts[1], "MODE", "MODE"))             { do_pd_mode(arg, query); return; }
        if (np >= 2 && match(parts[1], "CONTR", "CONTRACT") && query){ do_pd_contract(); return; }
        if (np >= 2 && !strncmp(parts[1], "PDO", 3)) {
            // :PD:PDO?                     -> list
            // :PD:PDO:COUN?                -> count
            // :PD:PDO:LIST?                -> list (explicit)
            // :PD:PDO<n>?                  -> single (n embedded in header per SCPI)
            const char* suffix = parts[1] + 3;
            // Helper: arg == "ALL" or "EPR" => full enumeration (auto-EPR).
            auto arg_is_all = [&]() -> bool {
                if (!arg) return false;
                char a[8]; strncpy(a, arg, sizeof(a)-1); a[sizeof(a)-1]=0;
                uppercase(a);
                return match1(a, "ALL") || match1(a, "EPR");
            };
            if (*suffix == 0 && np == 2 && query) {
                if (arg_is_all()) { do_pd_pdo_list_all(); return; }
                do_pd_pdo_list(); return;
            }
            if (*suffix == 0 && np >= 3 && query) {
                if (match(parts[2], "COUN", "COUNT")) { do_pd_pdo_count(); return; }
                if (match1(parts[2], "LIST")) {
                    if (arg_is_all()) { do_pd_pdo_list_all(); return; }
                    do_pd_pdo_list();  return;
                }
            }
            if (*suffix != 0 && query) {
                unsigned n = static_cast<unsigned>(strtoul(suffix, nullptr, 10));
                do_pd_pdo_n(n); return;
            }
        }
        err_push(-113, "Undefined header"); out("Unknown command\r\n"); return;
    }

    // :CONFigure:* — protection and limit settings
    if (match(parts[0], "CONF", "CONFIGURE")) {
        if (np < 2) { err_push(-109, "usage: :CONF:<param> [value]"); return; }
        // OCP — over-current protection limit (mA)
        if (match(parts[1], "OCP", "OCP")) {
            if (query) {
                char b[32]; snprintf(b, sizeof(b), "%u\r\n", (unsigned)Settings_GetOcpMa());
                out(b);
            } else if (arg) {
                uint32_t ma = 0;
                if (parse_numeric_unit(arg, 'A', &ma)) {
                    // Settings_SetNumeric uses delta steps of 100 mA
                    int32_t diff = (int32_t)ma - (int32_t)Settings_GetOcpMa();
                    int32_t steps = diff / 100;
                    if (steps != 0) Settings_SetNumeric(MI_OCP_LIMIT, steps);
                    out("OK\r\n");
                } else {
                    err_push(-222, "bad current value");
                }
            }
            return;
        }
        // OVP — over-voltage protection limit (mV)
        if (match(parts[1], "OVP", "OVP")) {
            if (query) {
                char b[32]; snprintf(b, sizeof(b), "%u\r\n", (unsigned)Settings_GetOvpMv());
                out(b);
            } else if (arg) {
                uint32_t mv = 0;
                if (parse_numeric_unit(arg, 'V', &mv)) {
                    // Settings_SetNumeric uses delta steps of 1000 mV
                    int32_t diff = (int32_t)mv - (int32_t)Settings_GetOvpMv();
                    int32_t steps = diff / 1000;
                    if (steps != 0) Settings_SetNumeric(MI_OVP_LIMIT, steps);
                    out("OK\r\n");
                } else {
                    err_push(-222, "bad voltage value");
                }
            }
            return;
        }
        // WH — energy limit (mWh)
        if (match(parts[1], "WH", "WH")) {
            if (query) {
                char b[32]; snprintf(b, sizeof(b), "%u\r\n", (unsigned)Settings_GetWhLimitMwh());
                out(b);
            }
            return;
        }
        // AH — charge limit (mAh)
        if (match(parts[1], "AH", "AH")) {
            if (query) {
                char b[32]; snprintf(b, sizeof(b), "%u\r\n", (unsigned)Settings_GetAhLimitMah());
                out(b);
            }
            return;
        }
        // CC — charge-complete detection (mA threshold, seconds hold)
        if (match(parts[1], "CC", "CC")) {
            if (query) {
                char b[48]; snprintf(b, sizeof(b), "%u,%u\r\n",
                    (unsigned)Settings_GetChargeCompleteMa(),
                    (unsigned)Settings_GetChargeCompleteSec());
                out(b);
            }
            return;
        }
        // OPP — over-power protection (100 mW units)
        if (match(parts[1], "OPP", "OPP")) {
            if (query) {
                char b[32]; snprintf(b, sizeof(b), "%u\r\n", (unsigned)Settings_GetOpp100mw());
                out(b);
            }
            return;
        }
        err_push(-113, "unknown :CONF sub-command"); out("Unknown command\r\n"); return;
    }

    err_push(-113, "Undefined header");
    out("Unknown command\r\n");
}

// -----------------------------------------------------------------------------
// Async event emission
// -----------------------------------------------------------------------------

static void maybe_emit_events() {
    if (!events_enabled || !s_pe || !s_port) return;

    // PE state change trace — on by default while debugging HR loops.
    if (state_trace) {
        int st = s_pe->get_state_id();
        if (st != last_pe_state) {
            last_pe_state = st;
            char b[40];
            snprintf(b, sizeof(b), "#EVT PE_STATE %d\r\n", st);
            out(b);
            // When HR is entered, dump the last 16 received message headers so
            // we can see what arrived right before HR.
            if (st == 9) {  // PE_SNK_Hard_Reset
                out("#DBG RXLOG:\r\n");
                uint32_t n = rx_log_idx;
                uint32_t start_i = (n > 16) ? (n - 16) : 0;
                for (uint32_t i = start_i; i < n; i++) {
                    uint32_t slot = i % 16;
                    char ln[72];
                    snprintf(ln, sizeof(ln),
                             "#DBG   [%lu] ord=%lu sz=%u hdr=%04X do0=%08lX\r\n",
                             (unsigned long)i,
                             (unsigned long)rx_log[slot].ordset,
                             (unsigned)rx_log[slot].size,
                             (unsigned)rx_log[slot].hdr,
                             (unsigned long)rx_log[slot].do0);
                    out(ln);
                }
            }
        }
    }

    // Caps size changed (EPR entry expanded the list, HR shrank it).
    size_t ncaps = s_port->source_caps.size();
    if (ncaps != last_ncaps) {
        bool first_time = (last_ncaps == 0);
        last_ncaps = ncaps;
        char b[48];
        snprintf(b, sizeof(b), "#EVT PDO_COUNT %u\r\n", (unsigned)ncaps);
        out(b);
        // Full PDO list is ~540 B and takes ~47 ms to TX at 115200 baud.
        // That blocks the main loop long enough to miss the PS_RDY that
        // the source sends mid-transition (rx_buf_ can be overwritten by
        // the next RX). So only print the full list ONCE, at first
        // reception. Subsequent caps changes (e.g. SPR -> EPR expansion)
        // just emit PDO_COUNT — use `list` to see the full set.
        if (first_time) {
            // Batch every PDO line (plus the EPR_CAPABLE summary) into one
            // UART write so HAL_UART_Transmit is called exactly once.
            // One out() per line previously produced visible gaps between
            // some lines — probably UART / terminal re-synchronisation at
            // line boundaries — fixed by emitting the whole block atomically.
            char block[800];
            size_t off = 0;
            for (size_t i = 0; i < ncaps && off < sizeof(block) - 96; i++) {
                uint32_t pdo = s_port->source_caps[i];
                if (pdo == 0) continue;
                char line[96];
                decode_pdo_line(static_cast<unsigned>(i + 1), pdo, line, sizeof(line));
                int w = snprintf(block + off, sizeof(block) - off,
                                 "#EVT PDO %s\r\n", line);
                if (w < 0) break;
                off += static_cast<size_t>(w);
            }
            if (ncaps > 0 && off < sizeof(block) - 32) {
                bool epr_cap = ((s_port->source_caps[0] >> 23) & 1) != 0;
                int w = snprintf(block + off, sizeof(block) - off,
                                 "#EVT EPR_CAPABLE %d\r\n", epr_cap ? 1 : 0);
                if (w > 0) off += static_cast<size_t>(w);
            }
            block[off] = 0;
            out(block);
        }
    }

    // EPR mode toggle.
    bool in_epr = s_pe->is_in_epr_mode();
    if (in_epr != last_in_epr) {
        last_in_epr = in_epr;
        out(in_epr ? "#EVT PD_MODE EPR\r\n" : "#EVT PD_MODE SPR\r\n");
    }

    // Contract change.
    uint32_t rdo = s_port->rdo_contracted;
    if (rdo != last_rdo_contracted) {
        last_rdo_contracted = rdo;
        if (rdo != 0) {
            char b[96];
            decode_contract(b, sizeof(b));
            char line[128];
            snprintf(line, sizeof(line), "#EVT CONTRACT %s\r\n", b);
            out(line);
        } else {
            out("#EVT CONTRACT NONE\r\n");
        }
    }
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

void cli_init(pd::Port* port, pd::PE* pe, AppDPM* dpm, UcpdDriver* driver) {
    s_port   = port;
    s_pe     = pe;
    s_dpm    = dpm;
    s_driver = driver;
    line_len = 0;

    target_mv = 5000;
    target_ma = 0;
    source_mode = SourceMode::AUTO;
    user_wants_epr = false;
    events_enabled = true;
    idn_build();
    err_clear();
    last_rdo_contracted = 0;
    last_in_epr = false;
    last_ncaps = 0;

    // Initial trigger so boot lands at 5 V once attached.
    if (s_dpm) s_dpm->trigger_any(target_mv, target_ma);

    out("\r\nAxxPD v" FW_VERSION " ready. Type 'help' for commands.\r\n");
}

void cli_poll() {
    // Reentrancy guard — cli_poll can be called from idle callbacks
    // (INA228, LCD DMA) while a previous invocation is still in progress.
    static volatile uint8_t in_cli_poll = 0;
    if (in_cli_poll) return;
    in_cli_poll = 1;

    // Re-pin EPR_AUTO_ENTER_DISABLED every tick when the user hasn't enabled
    // EPR, because pdsink's Hard-Reset path (pe.cpp:854) clears pe_flags and
    // would re-enable auto-entry, causing the EPR-enter/HR/exit loop on
    // strict sources when the DPM picks an SPR-slot PDO while in EPR mode.
    if (s_port && !user_wants_epr) {
        s_port->pe_flags.set(pd::PE_FLAG::EPR_AUTO_ENTER_DISABLED);
    }

    // --- EPR failure tracking + ErrorRecovery ---
    {
        static constexpr uint32_t EPR_QUICK_FAIL_WINDOW_MS = 5000;
        static constexpr uint32_t EPR_QUICK_FAIL_THRESHOLD = 2;
        static constexpr uint32_t EPR_STUCK_TIMEOUT_MS = 30000;  // Pattern B timeout
        static bool     epr_tracker_prev = false;
        static uint32_t epr_tracker_enter_ms = 0;
        static uint32_t epr_want_since_ms = 0;   // when user_wants_epr became true
        static bool     epr_want_armed = false;   // tracking Pattern B

        if (s_pe && s_driver && user_wants_epr) {
            bool in_epr = s_pe->is_in_epr_mode();

            if (in_epr && !epr_tracker_prev) {
                // Just entered EPR — start the stability timer.
                epr_tracker_enter_ms = HAL_GetTick();
            }

            if (!in_epr && epr_tracker_prev) {
                // Just lost EPR.  If it lasted < 5 s, this is a "quick failure"
                // (charger hard-reset right after Enter_Succeeded).
                uint32_t held_ms = HAL_GetTick() - epr_tracker_enter_ms;
                if (held_ms < EPR_QUICK_FAIL_WINDOW_MS) {
                    epr_quick_fail_count++;
                    if (epr_quick_fail_count >= EPR_QUICK_FAIL_THRESHOLD) {
                        epr_quick_fail_count = 0;
                        // EPR entered but lost quickly — cable discovery may
                        // have failed.  Enable cable_emu + ErrorRecovery so
                        // the next attempt can emulate the e-marker.
                        if (s_driver->is_cable_emu_disabled()) {
                            out("#DBG EPR quick-fail: enabling cable_emu + CC reset\r\n");
                            s_driver->enable_cable_emu();
                        } else {
                            out("#DBG EPR quick-fail: CC reset (cable_emu already on)\r\n");
                        }
                        s_driver->error_recovery();
                    }
                }
            }

            // EPR stable for > window → reset failure count + stuck timer.
            if (in_epr && (int32_t)(HAL_GetTick() - epr_tracker_enter_ms) >
                          (int32_t)EPR_QUICK_FAIL_WINDOW_MS) {
                epr_quick_fail_count = 0;
                epr_want_armed = false;  // success — stop Pattern B tracking
            }

            epr_tracker_prev = in_epr;

            // Pattern B: user_wants_epr is true but EPR never enters.
            // This catches sources that ignore EPR_Mode(Enter) because cable
            // discovery fails (no e-marker and cable_emu is disabled).
            // Strategy: first timeout → enable cable_emu (no CC reset).
            //           second timeout → full ErrorRecovery (CC reset).
            if (!epr_want_armed) {
                epr_want_armed = true;
                epr_want_since_ms = HAL_GetTick();
            }
            if (!in_epr && (int32_t)(HAL_GetTick() - epr_want_since_ms) >
                           (int32_t)EPR_STUCK_TIMEOUT_MS) {
                epr_want_armed = false;  // restart timer for next round
                if (s_driver->is_cable_emu_disabled()) {
                    // First stuck: try enabling cable_emu (cable has no e-marker)
                    out("#DBG EPR stuck: enabling cable_emu\r\n");
                    s_driver->enable_cable_emu();
                } else {
                    // Second stuck: cable_emu was already on, do full CC reset
                    out("#DBG ErrorRecovery: EPR stuck with cable_emu on\r\n");
                    s_driver->error_recovery();
                }
            }
        } else {
            epr_want_armed = false;
        }
    }

    // Deferred 'list all': fires once EPR is entered and the full
    // EPR_Source_Capabilities has been parsed (source_caps.size() > 7).
    // Bails after PENDING_LIST_ALL_TIMEOUT_MS with whatever caps exist —
    // on a strict source that refuses EPR, we at least print the SPR set.
    //
    // RACE CONDITION NOTE: source_caps can be mutated by the PD stack
    // (pe.cpp) at any time during a Hard Reset or new Source_Capabilities
    // message — it gets cleared and repopulated.  Reading source_caps
    // here in the main loop while the ISR/PD-tick path is writing it
    // could yield a partially-filled array.  In practice this is benign
    // (we'd print stale or incomplete PDO data, which self-corrects on
    // the next poll), but a proper fix would snapshot source_caps under
    // a critical section or use a double-buffer scheme.
    if (pending_list_all && s_pe && s_port) {
        uint32_t now = HAL_GetTick();
        bool ready = s_pe->is_in_epr_mode() && s_port->source_caps.size() > 7;
        bool timeout = (int32_t)(now - pending_list_all_ms) >
                       (int32_t)PENDING_LIST_ALL_TIMEOUT_MS;
        if (ready || timeout) {
            pending_list_all = false;
            outln();
            if (!ready && timeout) {
                out("list all: EPR enumeration timed out, current caps:\r\n");
            }
            if (s_port->source_caps.empty()) { out("NONE"); outln(); }
            else { emit_pdo_lines(); }
        }
    }

    selftest_tick();
    seq_tick();
    maybe_emit_events();

    // Drain CDC RX ring buffer byte-by-byte.
    int16_t rx;
    while ((rx = CDC_RxGetByte()) >= 0) {
        char c = static_cast<char>(rx);
        if (c == '\r' || c == '\n') {
            outln();
            if (line_len > 0) {
                line_buf[line_len] = 0;
                dispatch(line_buf);
                line_len = 0;
            }
        } else if (c == 0x08 || c == 0x7F) {       // BS / DEL
            if (line_len > 0) { line_len--; out("\b \b"); }
        } else if (c == 0x03) {                     // Ctrl-C: abort line
            line_len = 0; out("^C\r\n");
        } else if (c >= 32 && c < 127 && line_len + 1 < CLI_LINE_MAX) {
            line_buf[line_len++] = c;
            out_char(c);
        }
    }

    // Drain UART RX (line-buffered by uart.c)
    if (UART_HasLine()) {
        char uart_line[CLI_LINE_MAX];
        uint16_t len = UART_GetLine(uart_line, CLI_LINE_MAX);
        if (len > 0) {
            outln();
            dispatch(uart_line);
        }
    }

    in_cli_poll = 0;
}
