// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only

/**
 * @file    boot_selector.c
 * @brief   Boot-time PDO selector screen for AxxPD.
 *
 * Two-state machine driven by BootSelector_Run():
 *
 *   WAIT_FOR_CAPS  --[PDOs received]--> SELECTING --[confirm/timeout]--> return
 *       ^                                  |
 *       +------[source disconnects]--------+
 *
 * WAIT_FOR_CAPS: shows splash logo + "Connecting..." animation while polling
 *   for source capabilities. Transitions once SPR PDOs arrive and the
 *   splash minimum time (500 ms) has elapsed.
 *
 * SELECTING: scrollable PDO list with cursor. EPR entry is attempted at the
 *   1-second mark (cable_emu enabled first to answer SOP' if VCONN is absent).
 *   The PDO list updates dynamically as EPR PDOs arrive after the SPR contract.
 *   Auto-selects the last-used PDO after 10 s unless the user presses a button.
 *
 * Returns: 1-based PDO index on selection, or 0 if the feature is disabled.
 *
 * The caller must have already initialised the LCD, buttons and PD stack.
 * This function blocks and internally ticks axxpd_run() / Buttons_Update().
 */

#include "boot_selector.h"
#include "lcd.h"
#include "buttons.h"
#include "settings.h"
#include "buzzer.h"
extern uint8_t CDC_Transmit_Blocking(const uint8_t*, uint16_t, uint32_t);
#include "axxpd_main.h"
#include "splash_logo.h"
#include "stm32g4xx_hal.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* -------------------------------------------------------------------------
 * Fonts (defined in Drivers/UGUI/Fonts/)
 * ---------------------------------------------------------------------- */
extern UG_FONT FONT_arial_17X18[];

#define FONT_SM   FONT_arial_17X18   /* 17 px wide, 18 px tall */

/* Character cell metrics for FONT_SM */
#define FONT_SM_W   17
#define FONT_SM_H   18

/* -------------------------------------------------------------------------
 * Colors
 * ---------------------------------------------------------------------- */
#define COL_BG        RGB(0,   0,   0  )
#define COL_WHITE     RGB(255, 255, 255)
#define COL_GREY      LEGACY(150, 150, 150)
#define COL_ACCENT    RGB(255, 234, 0)           /* AxxPD brand yellow */
#define COL_SEL_BG    RGB(232, 64, 8)              /* red-orange selection bg (matches original) */
#define COL_TITLE_BG  RGB(122, 122, 122)         /* grey bar bg */

/* -------------------------------------------------------------------------
 * Layout constants (320 x 172 landscape)
 * ---------------------------------------------------------------------- */
#define SCREEN_W    320
#define SCREEN_H    172

/* Title bar — starts at top edge (bar fills to edges); text is inset */
#define TITLE_Y     0
#define TITLE_H     30
#define TITLE_TY    (TITLE_Y + (TITLE_H - 18) / 2 + 5)   /* text y: +5px down */
#define FOOTER_TY   (FOOTER_Y + (FOOTER_H - 18) / 2 - 5) /* text y: 5px up */

/* List area — sits directly below the title bar */
#define LIST_Y      (TITLE_Y + TITLE_H)
#define LIST_ROW_H  20
#define LIST_ROWS   5                          /* max visible rows (112px / 20px) */
#define LIST_H      (LIST_ROWS * LIST_ROW_H)   /* 120 px                  */

/* Scrollbar — 4 px wide on the right edge, spans the list area */
#define SCROLL_W    4
#define SCROLL_X    (SCREEN_W - SCROLL_W)
#define SCROLL_Y    LIST_Y
#define SCROLL_H    LIST_H

/* Footer — 30 px bar anchored to bottom edge */
#define FOOTER_H    30
#define FOOTER_Y    (SCREEN_H - FOOTER_H)       /* 142 */

/* Usable width for list text (excluding scrollbar) */
#define LIST_TEXT_W (SCREEN_W - SCROLL_W)

/* -------------------------------------------------------------------------
 * PDO decode constants (USB-PD spec)
 * ---------------------------------------------------------------------- */
#define PDO_TYPE_FIXED  0U
#define PDO_TYPE_APDO   3U
#define APDO_SUB_PPS    0U
#define APDO_SUB_AVS    1U

#define MAX_PDOS  14U   /* 7 SPR + 7 EPR, generous upper bound */

/**
 * Fetch all available PDOs (SPR + EPR) into buf[MAX_PDOS]. Returns count.
 * Three-pass sort ensures a stable display order: Fixed first, then PPS, then AVS.
 * In EPR mode, source_caps already contains both SPR and EPR entries
 * (EPR PDOs appear after the SPR→EPR mode transition).
 */
static uint8_t fetch_all_pdos(uint32_t *buf)
{
    uint32_t tmp[MAX_PDOS];
    uint8_t raw = axxpd_get_src_pdos(tmp, MAX_PDOS);
    uint8_t n = 0;
    /* Pass 1: Fixed PDOs (type bits [31:30] == 0b00) */
    for (uint8_t i = 0; i < raw; i++) {
        if (tmp[i] == 0U) continue;
        if (((tmp[i] >> 30U) & 0x3U) == PDO_TYPE_FIXED) buf[n++] = tmp[i];
    }
    /* Pass 2: PPS APDOs (type 0b11, subtype [29:28] == 0b00) */
    for (uint8_t i = 0; i < raw; i++) {
        if (tmp[i] == 0U) continue;
        uint32_t t = (tmp[i] >> 30U) & 0x3U;
        uint32_t s = (tmp[i] >> 28U) & 0x3U;
        if (t == PDO_TYPE_APDO && s == APDO_SUB_PPS) buf[n++] = tmp[i];
    }
    /* Pass 3: AVS APDOs (type 0b11, subtype [29:28] == 0b01) — EPR only */
    for (uint8_t i = 0; i < raw; i++) {
        if (tmp[i] == 0U) continue;
        uint32_t t = (tmp[i] >> 30U) & 0x3U;
        uint32_t s = (tmp[i] >> 28U) & 0x3U;
        if (t == PDO_TYPE_APDO && s == APDO_SUB_AVS) buf[n++] = tmp[i];
    }
    return n;
}

/* -------------------------------------------------------------------------
 * Auto-select timeout — if user doesn't press any button within this period
 * after entering SELECTING state, the last-used PDO is auto-confirmed.
 * ---------------------------------------------------------------------- */
#define AUTO_SELECT_MS   10000U

/* -------------------------------------------------------------------------
 * State machine
 * ---------------------------------------------------------------------- */
typedef enum {
    STATE_WAIT_FOR_CAPS = 0,
    STATE_SELECTING,
} BootSel_State_t;

/* -------------------------------------------------------------------------
 * Static helpers — PDO formatting
 * ---------------------------------------------------------------------- */

/**
 * Format a single PDO raw word into a human-readable string.
 * idx is the 1-based display index written at the start.
 * buf must be at least 32 bytes.
 */
static void format_pdo(char *buf, uint8_t idx, uint32_t pdo)
{
    uint32_t type    = (pdo >> 30U) & 0x3U;
    uint32_t subtype = (pdo >> 28U) & 0x3U;

    if (type == PDO_TYPE_FIXED) {
        /* Fixed supply: voltage in 50 mV units [21:10], current in 10 mA units [9:0] */
        uint32_t mv = ((pdo >> 10U) & 0x3FFU) * 50U;
        uint32_t ma = ((pdo >>  0U) & 0x3FFU) * 10U;
        snprintf(buf, 32, "%u: FIXED %2lu.%03luv %lu.%luA",
                 (unsigned)idx,
                 (unsigned long)(mv / 1000U),
                 (unsigned long)(mv % 1000U),
                 (unsigned long)(ma / 1000U),
                 (unsigned long)((ma % 1000U) / 100U));
    } else if (type == PDO_TYPE_APDO && subtype == APDO_SUB_PPS) {
        /* PPS APDO: min_v [15:8] in 100 mV, max_v [24:17] in 100 mV, max_i [6:0] in 50 mA */
        uint32_t min_mv = ((pdo >>  8U) & 0xFFU) * 100U;
        uint32_t max_mv = ((pdo >> 17U) & 0xFFU) * 100U;
        uint32_t max_ma = ((pdo >>  0U) & 0x7FU) * 50U;
        snprintf(buf, 32, "%u: PPS %lu.%lu-%lu.%luv %lu.%luA",
                 (unsigned)idx,
                 (unsigned long)(min_mv / 1000U),
                 (unsigned long)((min_mv % 1000U) / 100U),
                 (unsigned long)(max_mv / 1000U),
                 (unsigned long)((max_mv % 1000U) / 100U),
                 (unsigned long)(max_ma / 1000U),
                 (unsigned long)((max_ma % 1000U) / 100U));
    } else if (type == PDO_TYPE_APDO && subtype == APDO_SUB_AVS) {
        /* EPR AVS APDO: min_v [15:8] in 100 mV, max_v [25:17] in 100 mV, pdp [7:0] in W */
        uint32_t min_mv = ((pdo >>  8U) & 0xFFU)  * 100U;
        uint32_t max_mv = ((pdo >> 17U) & 0x1FFU) * 100U;
        uint32_t pdp_w  =  (pdo >>  0U) & 0xFFU;
        snprintf(buf, 32, "%u: AVS %lu.%lu-%lu.%luv %luW",
                 (unsigned)idx,
                 (unsigned long)(min_mv / 1000U),
                 (unsigned long)((min_mv % 1000U) / 100U),
                 (unsigned long)(max_mv / 1000U),
                 (unsigned long)((max_mv % 1000U) / 100U),
                 (unsigned long)pdp_w);
    } else {
        snprintf(buf, 32, "%u: PDO t%lu.%lu",
                 (unsigned)idx, (unsigned long)type, (unsigned long)subtype);
    }
}

/* -------------------------------------------------------------------------
 * Static helpers — drawing
 * ---------------------------------------------------------------------- */

/** Clear the entire screen to black. */
static void draw_clear(void)
{
    LCD_Fill(0, 0, SCREEN_W - 1, SCREEN_H - 1, COL_BG);
}

/** Draw the title bar. */
static void draw_title(void)
{
    LCD_Fill(0, TITLE_Y, SCREEN_W - 1, TITLE_Y + TITLE_H - 1, COL_TITLE_BG);
    LCD_PutStr(15, TITLE_TY, "AxxPD -- Select Power", FONT_SM, COL_ACCENT, COL_TITLE_BG);
}

/**
 * Draw a single list row at logical position row_pos (0..LIST_ROWS-1).
 * pdo_idx is the 0-based index into the pdos[] array (used for formatting).
 * selected: if non-zero, draw with selection highlight and > cursor.
 */
static void draw_row(uint8_t row_pos, uint8_t pdo_idx,
                     const uint32_t *pdos, uint8_t selected)
{
    char buf[32];
    uint16_t y   = LIST_Y + (uint16_t)row_pos * LIST_ROW_H;
    uint16_t bg  = selected ? COL_SEL_BG  : COL_BG;
    uint16_t fg  = selected ? COL_WHITE   : COL_WHITE;

    /* Row background */
    LCD_Fill(0, y, LIST_TEXT_W - 1, y + LIST_ROW_H - 1, bg);

    /* PDO text */
    format_pdo(buf, pdo_idx + 1U, pdos[pdo_idx]);

    /* Cursor character for selected row */
    char row_buf[34];
    if (selected) {
        snprintf(row_buf, sizeof(row_buf), ">%s", buf);
    } else {
        snprintf(row_buf, sizeof(row_buf), " %s", buf);
    }

    LCD_PutStr(15, y + 2, row_buf, FONT_SM, fg, bg);
}

/**
 * Draw all visible list rows given scroll offset and cursor position.
 * Erases the full list area first so that when EPR PDOs arrive (count grows)
 * or when the source disconnects briefly (count shrinks), stale rows are cleared.
 */
static void draw_list(const uint32_t *pdos, uint8_t count,
                      uint8_t scroll, uint8_t cursor)
{
    /* Erase full list area first to handle count changes cleanly */
    LCD_Fill(0, LIST_Y, SCREEN_W - 1, LIST_Y + LIST_H - 1, COL_BG);

    uint8_t visible = (count < LIST_ROWS) ? count : LIST_ROWS;
    for (uint8_t r = 0; r < visible; r++) {
        uint8_t pdo_idx = scroll + r;
        if (pdo_idx >= count) break;
        draw_row(r, pdo_idx, pdos, (pdo_idx == cursor) ? 1U : 0U);
    }
}

/**
 * Draw a proportional scrollbar.
 * count:  total PDO count
 * scroll: first visible PDO index
 */
static void draw_scrollbar(uint8_t count, uint8_t scroll)
{
    /* Scrollbar track background */
    LCD_Fill(SCROLL_X, SCROLL_Y, SCREEN_W - 1, SCROLL_Y + SCROLL_H - 1,
             COL_TITLE_BG);

    if (count <= LIST_ROWS) {
        /* All items visible — fill entire track with accent colour */
        LCD_Fill(SCROLL_X, SCROLL_Y, SCREEN_W - 1, SCROLL_Y + SCROLL_H - 1,
                 COL_ACCENT);
        return;
    }

    /* Thumb height proportional to visible / total */
    uint16_t thumb_h = (uint16_t)((uint32_t)SCROLL_H * LIST_ROWS / count);
    if (thumb_h < 4U) thumb_h = 4U;

    /* Thumb position proportional to scroll / (count - LIST_ROWS) */
    uint16_t max_scroll = (uint16_t)(count - LIST_ROWS);
    uint16_t thumb_y    = SCROLL_Y
                        + (uint16_t)((uint32_t)(SCROLL_H - thumb_h) * scroll
                                     / max_scroll);

    LCD_Fill(SCROLL_X, thumb_y,
             SCREEN_W - 1, thumb_y + thumb_h - 1, COL_ACCENT);
}

/**
 * Draw the footer line.
 * If countdown_active is non-zero, show "Auto-select in Xs".
 * Otherwise show "SEL to confirm".
 * remaining_ms is used only when countdown_active != 0.
 */
static uint8_t footer_mode = 0xFFU;  /* 1=countdown, 2=confirm; 0xFF=uninitialised; avoids redundant full redraws */

static void draw_footer(uint8_t countdown_active, uint32_t remaining_ms)
{
    if (countdown_active) {
        char buf[32];
        uint32_t secs = (remaining_ms + 999U) / 1000U;
        snprintf(buf, sizeof(buf), "Auto-select in %-2lus  ", (unsigned long)secs);
        if (footer_mode != 1U) {
            /* First countdown draw — fill background */
            LCD_Fill(0, FOOTER_Y, SCREEN_W - 1, SCREEN_H - 1, COL_TITLE_BG);
            footer_mode = 1U;
        }
        LCD_PutStr(15, FOOTER_TY, buf, FONT_SM, COL_ACCENT, COL_TITLE_BG);
    } else {
        if (footer_mode != 2U) {
            /* Transition to confirm — draw once, never redraw */
            LCD_Fill(0, FOOTER_Y, SCREEN_W - 1, SCREEN_H - 1, COL_TITLE_BG);
            LCD_PutStr(15, FOOTER_TY, "SELECT to confirm", FONT_SM,
                       COL_ACCENT, COL_TITLE_BG);
            footer_mode = 2U;
        }
    }
}

/** Draw animated "Connecting" text with 1..3 dots cycling every 250 ms.
 *  Overwrites previous dots with trailing spaces to avoid full-screen clear
 *  (which would cause visible flicker on top of the splash logo). */
static void draw_connecting(uint8_t dots)
{
    /* Use fixed max-length string with trailing spaces to overwrite old dots
     * without needing a full-screen clear (avoids flicker). */
    char buf[16] = "Connecting   ";
    uint8_t d = (dots > 3U) ? 3U : dots;
    for (uint8_t i = 0; i < d; i++) {
        buf[10 + i] = '.';
    }

    /* Centre horizontally based on actual glyph widths + 1 px inter-char gap.
     * "Connecting..." = 106 px glyphs + 12 × 1 px gaps = 118 px total. */
    uint16_t tw   = 118U;
    uint16_t tx   = (SCREEN_W - tw) / 2U;

    /* Position below splash logo (logo drawn at y=26, height=120, bottom=146) */
    uint16_t logo_bottom = 26U + splash_logo.height;  /* 146 */
    uint16_t ty           = logo_bottom + 8U;          /* 8px gap below logo */

    LCD_PutStr(tx, ty, buf, FONT_SM, COL_WHITE, COL_BG);
}

/* -------------------------------------------------------------------------
 * Module-level results — set during boot selection, read by main.c after
 * BootSelector_Run() returns so main.c knows whether to stay in EPR mode
 * and which raw PDO word was selected.
 * ---------------------------------------------------------------------- */
static uint8_t  s_src_epr_capable = 0U;
static uint32_t s_selected_pdo = 0U;

uint8_t BootSelector_SrcEprCapable(void)
{
    return s_src_epr_capable;
}

uint32_t BootSelector_GetSelectedPdo(void)
{
    return s_selected_pdo;
}

/* -------------------------------------------------------------------------
 * Public entry point
 * ---------------------------------------------------------------------- */

int BootSelector_Run(void)
{
    /* ---- Early-out: feature disabled ---------------------------------- */
    if (!Settings_GetBootSelector()) {
        return 0;
    }

    /* ---- Working state ------------------------------------------------ */
    BootSel_State_t state     = STATE_WAIT_FOR_CAPS;
    uint32_t        pdos[MAX_PDOS];   /* sorted PDO buffer (Fixed, PPS, AVS) */
    uint8_t         count     = 0U;   /* number of PDOs currently in pdos[] */
    uint8_t         cursor    = 0U;   /* 0-based index of highlighted PDO */
    uint8_t         scroll    = 0U;   /* 0-based index of first visible PDO */
    uint8_t         user_touched = 0U;   /* any button press cancels auto-select countdown */
    uint32_t        select_start_tick = 0U;  /* tick when SELECTING state was entered */
    uint8_t         epr_requested = 0U;  /* set once EPR entry is triggered at 1s */
    uint8_t         epr_retried   = 0U;  /* set once 3s EPR retry fires */
    uint8_t         epr_shown     = 0U;  /* set once "EPR" badge drawn on title bar */
    uint32_t        dot_tick  = 0U;   /* last "Connecting..." dot animation update */
    uint8_t         dot_phase = 0U;   /* current dot count 1..3 */
    uint32_t        last_btn_tick = 0U;  /* debounce: last Buttons_Update() call */

    /* Do NOT clear screen — keep the splash logo visible for at least 500 ms
     * so the user sees the AxxPD branding even on fast-responding sources. */
    uint32_t splash_start = HAL_GetTick();
    #define SPLASH_MIN_MS 500U

    /* Test beep so we know buzzer works */
    Buzzer_Confirm();
    footer_mode = 0xFFU;  /* reset so footer draws fresh on first use */
    last_btn_tick = HAL_GetTick();

    /* Show "Connecting." immediately over the splash logo so the user sees
     * feedback right away, rather than waiting 250 ms for the first dot tick. */
    draw_connecting(1);
    dot_tick = HAL_GetTick();
    dot_phase = 1U;

    /* ---- Main loop ---------------------------------------------------- */
    while (1) {
        uint32_t now = HAL_GetTick();

        /* IWDG refresh */
        IWDG->KR = 0xAAAAU;

        /* Tick PD stack + buzzer */
        axxpd_run();
        Buzzer_Update();

        /* Poll buttons every ~2 ms for snappy response */
        ButtonEvent_t ev = BTN_NONE;
        uint32_t elapsed = now - last_btn_tick;
        if (elapsed >= 2U) {
            last_btn_tick = now;
            Buttons_Update();
            ev = Buttons_GetEvent();
        }

        /* ----------------------------------------------------------------
         * STATE: WAIT_FOR_CAPS — splash logo visible, polling for PDOs
         * ---------------------------------------------------------------- */
        if (state == STATE_WAIT_FOR_CAPS) {
            /* Animate "Connecting..." dots: cycle 1->2->3->1 every 250 ms */
            if ((now - dot_tick) >= 250U) {
                dot_tick   = now;
                dot_phase  = (dot_phase % 3U) + 1U;
                draw_connecting(dot_phase);
            }

            /* Poll for source PDOs — but keep splash visible for SPLASH_MIN_MS
             * so branding is always shown even if caps arrive instantly. */
            uint32_t tmp[MAX_PDOS];
            uint8_t c = fetch_all_pdos(tmp);
            if (c > 0U && (now - splash_start) >= SPLASH_MIN_MS) {
                /* Got capabilities — copy into working buffer and transition */
                count = c;
                memcpy(pdos, tmp, sizeof(uint32_t) * count);

                /* Single-PDO source (e.g. 5 V only): skip the UI entirely */
                if (count == 1U) {
                    s_selected_pdo = pdos[0];
                    return 1;
                }

                /* Pre-select the last-used PDO (stored 1-based in flash).
                 * Falls back to first PDO if setting is unset or out of range. */
                uint8_t last = Settings_GetLastUsedPdo();
                cursor = (last > 0U && last <= count) ? (uint8_t)(last - 1U) : 0U;

                /* Set scroll so the pre-selected cursor is visible in the
                 * 5-row viewport. If cursor is beyond the first page, scroll
                 * to place it at the bottom of the visible area. */
                scroll = 0U;
                if (cursor >= LIST_ROWS) {
                    scroll = (uint8_t)(cursor - LIST_ROWS + 1U);
                }

                user_touched      = 0U;
                select_start_tick = now;   /* start the 10 s auto-select countdown */

                /* Transition: replace splash with the selection UI */
                draw_clear();
                draw_title();
                if (axxpd_is_src_epr_capable()) {
                    s_src_epr_capable = 1U;
                }
                draw_list(pdos, count, scroll, cursor);
                draw_scrollbar(count, scroll);
                draw_footer(1U /* countdown active */,
                            AUTO_SELECT_MS);

                state = STATE_SELECTING;
                epr_retried = 0U;
                epr_shown   = 0U;
            }

            continue;  /* stay in WAIT_FOR_CAPS — no further processing */
        }

        /* ----------------------------------------------------------------
         * STATE: SELECTING — user browsing the PDO list
         * ---------------------------------------------------------------- */

        /* Detect source disconnect → return to splash/waiting state */
        {
            uint32_t tmp[MAX_PDOS];
            uint8_t c = fetch_all_pdos(tmp);
            if (c == 0U) {
                /* Source removed — reset everything and go back to waiting */
                count     = 0U;
                cursor    = 0U;
                scroll    = 0U;
                dot_phase = 0U;
                dot_tick  = now;
                draw_clear();
                state = STATE_WAIT_FOR_CAPS;
                continue;
            }
            /* Dynamic PDO list update: after EPR entry succeeds, the source
             * re-advertises caps with additional EPR PDOs (AVS/PPS). The count
             * change triggers a full list redraw so new entries appear live. */
            if (c != count) {
                count = c;
                memcpy(pdos, tmp, sizeof(uint32_t) * count);
                /* Clamp cursor/scroll to stay within the new (possibly smaller) list */
                if (cursor >= count) cursor = (uint8_t)(count - 1U);
                if (scroll + LIST_ROWS > count && count > LIST_ROWS)
                    scroll = (uint8_t)(count - LIST_ROWS);
                else if (count <= LIST_ROWS)
                    scroll = 0U;
                /* Full redraw since row content and count changed */
                draw_list(pdos, count, scroll, cursor);
                draw_scrollbar(count, scroll);
            }
        }

        /* EPR auto-entry at 1 s after SELECTING begins:
         * 1) Enable cable_emu FIRST — if the source doesn't provide VCONN,
         *    the real cable e-marker can't respond to SOP' Discover_Identity.
         *    The cable emulator answers instead, preventing a Hard Reset.
         * 2) Then request EPR mode. The source will re-advertise caps with
         *    EPR PDOs, which the dynamic list update above picks up. */
        if (!epr_requested && s_src_epr_capable
            && (now - select_start_tick) >= 1000U) {
            epr_requested = 1U;
            axxpd_enable_cable_emu();
            axxpd_enable_epr();
        }
        /* Retry EPR at 3s if the first attempt failed (some chargers like
         * the Anker A2697 Soft Reset after the first EPR entry, setting
         * EPR_AUTO_ENTER_DISABLED).  The second attempt usually succeeds. */
        if (epr_requested && !axxpd_is_epr_active() && s_src_epr_capable
            && (now - select_start_tick) >= 3000U) {
            if (!epr_retried) {
                epr_retried = 1U;
                axxpd_enable_epr();
            }
        }

        /* Show "EPR" badge on title bar once EPR mode is confirmed active.
         * Drawn once and never cleared (the screen is fully redrawn on
         * state transitions anyway). */
        if (epr_requested && axxpd_is_epr_active()) {
            if (!epr_shown) {
                epr_shown = 1;
                LCD_PutStr(SCREEN_W - 55, TITLE_TY,
                           "EPR", FONT_SM, COL_ACCENT, COL_TITLE_BG);
            }
        }

        /* --- Button handling (ev was polled every ~2 ms above) --- */
        uint8_t prev_cursor = cursor;   /* snapshot for dirty-check */
        uint8_t prev_scroll = scroll;
        uint8_t list_dirty  = 0U;       /* set when list rows need redraw */
        uint8_t footer_dirty = 0U;      /* set when footer text must update */

        if (ev != BTN_NONE) {
            user_touched = 1U;   /* any press cancels the auto-select countdown */
            footer_dirty = 1U;   /* switch footer from countdown to "SEL to confirm" */
            if (ev != BTN_INC_REPEAT && ev != BTN_DEC_REPEAT) {
                Buzzer_Click();  /* audible click on first press only, not repeats */
            }
            char dbg[24];
            snprintf(dbg, sizeof(dbg), "[BTN] ev=%u\r\n", (unsigned)ev);
            CDC_Transmit_Blocking((const uint8_t*)dbg, (uint16_t)strlen(dbg), 50);
        }

        switch (ev) {
            case BTN_DEC_PRESS:
            case BTN_DEC_REPEAT:
                /* Move cursor UP — scroll viewport if cursor moves above visible area */
                if (cursor > 0U) {
                    cursor--;
                    if (cursor < scroll) {
                        scroll = cursor;   /* scroll up to keep cursor in view */
                    }
                    list_dirty = 1U;
                }
                break;

            case BTN_INC_PRESS:
            case BTN_INC_REPEAT:
                /* Move cursor DOWN — scroll viewport if cursor moves below visible area */
                if (cursor < (uint8_t)(count - 1U)) {
                    cursor++;
                    if (cursor >= (uint8_t)(scroll + LIST_ROWS)) {
                        scroll = (uint8_t)(cursor - LIST_ROWS + 1U);
                    }
                    list_dirty = 1U;
                }
                break;

            case BTN_SEL_PRESS:
            case BTN_PWR_SHORT: {
                /* Confirm selection — show voltage, store PDO, return */
                s_selected_pdo = pdos[cursor];
                {
                    uint32_t p = pdos[cursor], t = (p>>30)&3, mv = 0;
                    if (t == 0U) mv = ((p>>10)&0x3FF)*50;
                    else if (t == 3U && ((p>>28)&3)==0) mv = ((p>>17)&0xFF)*100;
                    else if (t == 3U && ((p>>28)&3)==1) mv = ((p>>17)&0x1FF)*100;
                    char msg[32];
                    snprintf(msg, sizeof(msg), "Requesting %lu.%luV...",
                             (unsigned long)(mv/1000), (unsigned long)((mv%1000)/100));
                    LCD_Fill(0, LIST_Y, SCREEN_W - 1, SCREEN_H - 1, COL_BG);
                    LCD_PutStr(70, 80, msg, FONT_SM, COL_ACCENT, COL_BG);
                }
                return (int)(cursor + 1U);
            }

            default:
                break;
        }

        /* Partial redraw optimisation: when the cursor moves within the
         * visible 5-row window, only repaint the old and new row (2 draws).
         * A full list redraw is only needed when the scroll offset changes. */
        if (list_dirty) {
            if (scroll != prev_scroll) {
                /* Viewport shifted — must redraw all visible rows */
                draw_list(pdos, count, scroll, cursor);
            } else {
                /* Cursor moved within visible area — swap highlight on 2 rows */
                uint8_t old_row = prev_cursor - scroll;
                uint8_t new_row = cursor - scroll;
                draw_row(old_row, prev_cursor, pdos, 0U);  /* deselect old */
                draw_row(new_row, cursor, pdos, 1U);        /* highlight new */
            }
            draw_scrollbar(count, scroll);
        }

        /* Footer: countdown if untouched, static "SEL to confirm" after first press */
        if (!user_touched) {
            uint32_t elapsed_sel = now - select_start_tick;
            if (elapsed_sel >= AUTO_SELECT_MS) {
                /* 10 s timeout — auto-confirm.  For safety, if the pre-selected
                 * PDO is >20V, fall back to PDO 0 (typically 5V) to avoid
                 * unattended high-voltage output. */
                {
                    uint32_t p = pdos[cursor], t = (p >> 30) & 3, mv = 0;
                    if (t == 0U) mv = ((p >> 10) & 0x3FF) * 50;
                    else if (t == 3U) mv = ((p >> 17) & 0x1FF) * 100;
                    if (mv > 20000U) cursor = 0U;
                }
                s_selected_pdo = pdos[cursor];
                {
                    uint32_t p = pdos[cursor], t = (p>>30)&3, mv = 0;
                    if (t == 0U) mv = ((p>>10)&0x3FF)*50;
                    else if (t == 3U && ((p>>28)&3)==0) mv = ((p>>17)&0xFF)*100;
                    else if (t == 3U && ((p>>28)&3)==1) mv = ((p>>17)&0x1FF)*100;
                    char msg[32];
                    snprintf(msg, sizeof(msg), "Requesting %lu.%luV...",
                             (unsigned long)(mv/1000), (unsigned long)((mv%1000)/100));
                    LCD_Fill(0, LIST_Y, SCREEN_W - 1, SCREEN_H - 1, COL_BG);
                    LCD_PutStr(70, 80, msg, FONT_SM, COL_ACCENT, COL_BG);
                }
                return (int)(cursor + 1U);
            }
            /* Update countdown text every 200 ms (avoids per-frame LCD writes) */
            static uint32_t last_footer_tick = 0U;
            if (footer_dirty || (now - last_footer_tick) >= 200U) {
                last_footer_tick = now;
                uint32_t remaining = AUTO_SELECT_MS - elapsed_sel;
                draw_footer(1U, remaining);
            }
        } else if (footer_dirty) {
            /* User pressed a button — switch to static "SELECT to confirm" */
            draw_footer(0U, 0U);
        }

        /* 5 ms yield — limits loop to ~200 Hz, reducing LCD bus traffic */
        HAL_Delay(5);
    }

    /* Unreachable — loop always returns inside */
}
