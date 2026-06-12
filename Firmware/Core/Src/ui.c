/**
 * @file    ui.c
 * @brief   TFT UI — dashboard, V/I graph, PDO list, presets, energy, settings (LCD 320x172)
 *
 * Screen layout (320x172 landscape):
 *
 *   +-----------------------------------------+  y=0
 *   |  STATUS BAR  (30px)  ON/OFF + SRC info  |  y=0..29   — color-coded: grey=off, green=on, red=fault
 *   +-----------------------------------------+  y=30
 *   |                                         |  y=37 (CONTENT_Y = STATUSBAR_H + 7)
 *   |  CONTENT AREA  (105px)                  |  — screen-specific: dashboard, graph, PDO list, etc.
 *   |                                         |
 *   +-----------------------------------------+  y=142 (NAVBAR_Y)
 *   |  NAVBAR  (30px)  [DSH] GRP PDO PRE ...  |  y=142..171 — highlights current screen
 *   +-----------------------------------------+  y=171
 *
 * Two input modes drive all screens:
 *   NAVIGATION — INC/DEC cycle screens, SEL enters edit mode
 *   EDIT       — INC/DEC adjust values/cursor, SEL cycles fields, SEL_LONG commits
 *
 * Flicker-free drawing: each function overdraws with background color (no
 * full-screen clears), and incremental redraw tracking skips unchanged rows.
 */

#include "ui.h"
#include "lcd.h"
#include "graph.h"
#include "settings.h"
#include "axxpd_main.h"
#include "buzzer.h"
#include "ina228.h"
#include <stdio.h>
#include <string.h>

/* Fonts (generated with ttf2ugui from arial.ttf) ----------------------------
 * LG/XL only contain digits + V/A/W glyphs — cannot render arbitrary text.
 * SM/MD have the full ASCII charset for labels and menus. */
extern UG_FONT FONT_arial_17X18[];
extern UG_FONT FONT_arial_23X24[];
extern UG_FONT FONT_arial_35X26[];
extern UG_FONT FONT_arial_35X37[];
extern UG_FONT FONT_arial_49X37[];

#define FONT_SM   FONT_arial_17X18   /* 18px — status bar, nav bar, PDO list */
#define FONT_MD   FONT_arial_23X24   /* 24px — labels, menus, power readout */
#define FONT_LG   FONT_arial_35X26   /* 26px — dashboard W, graph numerics */
#define FONT_XL   FONT_arial_49X37   /* 37px — dashboard V+A (digits-only font) */

/* Colors — LEGACY() reproduces the old visual look, RGB() for new colors */
#define COL_BG      RGB(0,0,0)
#define COL_WHITE   RGB(255,255,255)
#define COL_YELLOW  RGB(255,234,0)              /* AxxPD brand yellow */
#define COL_GREEN   RGB(0,200,0)
#define COL_GREY    RGB(150,150,150)
#define COL_BLACK   RGB(0,0,0)
#define COL_SEL_BG  RGB(232, 64, 8)              /* red-orange selection bg (matches original) */
#define TEXT_VPAD   2                              /* vertical text offset: (row_h - font_h) / 2 */

/* Layout — bars fill edge-to-edge, matching boot selector style ------------ */
#define SCREEN_W     320
#define SCREEN_H     172
#define STATUSBAR_H  30                        /* top bar height (matches navbar) */
#define NAVBAR_H     30                        /* bottom bar height */
#define NAVBAR_Y     (SCREEN_H - NAVBAR_H)     /* 142 — top of navbar */
#define CONTENT_Y    (STATUSBAR_H + 7)         /* 37 — first usable content row */
#define DRAW_X       15                        /* left margin for all content text */

/* Padlock icon — 13x18 monochrome bitmap, no keyhole (from lock.png)
 *  Col: C B A 9 8 7 6 5 4 3 2 1 0
 *   0:  . . . . . 1 1 1 . . . . .  shackle top
 *   1:  . . . 1 1 1 1 1 1 1 . . .
 *   2:  . . 1 1 . . . . 1 1 . . .  symmetric shackle
 *   3:  . . 1 1 . . . . 1 1 . . .
 *   4:  . . 1 1 . . . . 1 1 . . .
 *   5:  . . 1 1 . . . . 1 1 . . .
 *   6:  . 1 1 1 1 1 1 1 1 1 1 . .  body
 *   7:  . 1 1 1 1 1 1 1 1 1 1 . .
 *  8-15: same as row 7
 *  16:  . 1 1 1 1 1 1 1 1 1 1 . .
 *  17:  . . 1 1 1 1 1 1 1 1 . . .  rounded bottom
 */
static const uint16_t padlock_bmp[18] = {
    0x00E0,  /* 0000 0000 1110 0000 */
    0x01F8,  /* 0000 0001 1111 1000 */
    0x030C,  /* 0000 0011 0000 1100 */
    0x030C,  /* 0000 0011 0000 1100 */
    0x030C,  /* 0000 0011 0000 1100 */
    0x030C,  /* 0000 0011 0000 1100 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x07FE,  /* 0000 0111 1111 1110 */
    0x03FC,  /* 0000 0011 1111 1100 */
};
#define PADLOCK_W 13
#define PADLOCK_H 18

/** Draw or erase the padlock icon at (x,y) with given foreground color.
 *  To erase, pass bg as fg. */
static void draw_padlock(uint16_t x, uint16_t y, uint16_t fg, uint16_t bg)
{
    /* Clear the icon area first */
    LCD_Fill(x, y, x + PADLOCK_W - 1, y + PADLOCK_H - 1, bg);
    /* Draw foreground pixels */
    for (uint8_t row = 0; row < PADLOCK_H; row++) {
        uint16_t bits = padlock_bmp[row];
        for (uint8_t col = 0; col < PADLOCK_W; col++) {
            if (bits & (1U << (PADLOCK_W - 1 - col))) {
                LCD_Fill(x + col, y + row, x + col, y + row, fg);
            }
        }
    }
}

/* Format a float to always produce exactly 4 characters (3 significant digits):
 *   0.00 ..  9.99  -> "X.XX"  (%.2f)
 *  10.0  .. 99.9   -> "XX.X"  (%.1f)
 * 100    .. 999    -> " XXX"  (%.0f)
 * Thresholds use the ROUNDED boundary to prevent %.2f rounding 9.995 to "10.00". */
static int fmt_3sig(char *buf, size_t sz, float val, const char *unit, const char *trail)
{
    if (val < 0.0f) val = 0.0f;
    if (val >= 99.95f)
        return snprintf(buf, sz, "%4.0f%s%s", (double)val, unit, trail);
    if (val >= 9.995f)
        return snprintf(buf, sz, "%4.1f%s%s", (double)val, unit, trail);
    return snprintf(buf, sz, "%4.2f%s%s", (double)val, unit, trail);
}

/* Scrollbar — shown on right edge of scrollable lists */
#define SBAR_W       4
#define SBAR_X       (SCREEN_W - SBAR_W)
#define SBAR_TRACK   RGB(60,60,60)
#define SBAR_THUMB   RGB(255,234,0)    /* AxxPD brand yellow */

/** Draw a proportional scrollbar for any scrollable list.
 *  total:   total item count
 *  visible: max items visible at once
 *  scroll:  index of first visible item
 *  y0, h:   pixel region for the scrollbar track */
static void ui_draw_scrollbar(uint8_t total, uint8_t visible,
                               uint8_t scroll, uint16_t y0, uint16_t h)
{
    LCD_Fill(SBAR_X, y0, SCREEN_W - 1, y0 + h - 1, SBAR_TRACK);
    if (total <= visible) {
        LCD_Fill(SBAR_X, y0, SCREEN_W - 1, y0 + h - 1, SBAR_THUMB);
        return;
    }
    uint16_t thumb_h = (uint16_t)((uint32_t)h * visible / total);
    if (thumb_h < 4U) thumb_h = 4U;
    uint16_t max_scroll = (uint16_t)(total - visible);
    uint16_t thumb_y = y0 + (uint16_t)((uint32_t)(h - thumb_h) * scroll / max_scroll);
    LCD_Fill(SBAR_X, thumb_y, SCREEN_W - 1, thumb_y + thumb_h - 1, SBAR_THUMB);
}

/* Protection thresholds (match values set in main.c) */
#define OVP_THRESHOLD_MV  52000U
#define OCP_THRESHOLD_A   5.5f

/* Defined in main.c */
extern void App_SetTargetVoltage(uint32_t mv, uint32_t ma);
extern void OVP_SetThreshold(uint32_t vbus_max_mv);
extern volatile uint8_t g_hw_fault;       /* set by HW fault ISR, cleared by UI ack */
extern volatile uint8_t g_fault_source;   /* sequential fault code: FAULT_COMP_OVP, FAULT_LTC4368, etc. */
extern volatile uint8_t g_output_enabled;
extern INA228_t g_ina;

/* Boot autostart countdown (main.c) */
extern uint8_t Boot_AutostartPending(void);
extern int     Boot_AutostartSecsRemaining(void);
extern void    Boot_AutostartAbort(void);
uint16_t Timer_GetRemaining(void);        /* declared in main.h, defined in main.c */


/* ------------------------------------------------------------------ */
/*  UI state                                                           */
/* ------------------------------------------------------------------ */
static UIScreen_t  current_screen = UI_SCREEN_DASHBOARD;
static uint8_t     pdo_cursor  = 0;      /* highlighted row in PDO list */
static uint8_t     pdo_scroll  = 0;      /* first visible row in PDO list */
static uint8_t     preset_cursor = 0;
static uint8_t     settings_group = 0;   /* cursor in group list (level 0) */
static uint8_t     settings_item  = 0;   /* cursor within a group (level 1) */
static uint8_t     settings_level = 0;   /* 0=browsing groups, 1=inside a group */
uint8_t            settings_adjusting_flag = 0;  /* 1 when adjusting a numeric value */

/* Flash message — shown for 2s after an action (e.g. "Defaults loaded") */
static const char *flash_msg = NULL;
static uint32_t    flash_msg_tick = 0;

/* Fault overlay state — fault_acked lets the user dismiss the overlay
 * while g_hw_fault remains set until main.c clears the HW condition. */
static uint8_t     fault_acked = 0;

/* Incremental redraw tracking — 0xFF sentinel means "force full redraw".
 * Each list screen compares prev vs. current cursor/scroll to skip
 * unchanged rows, avoiding expensive full-screen repaints at 30Hz. */
static uint8_t s_pdo_prev_cursor = 0xFF, s_pdo_prev_scroll = 0xFF, s_pdo_prev_count = 0xFF;
static uint8_t s_pre_prev_cursor = 0xFF, s_pre_prev_scroll = 0xFF;

/* Edit mode — entered with SEL press, auto-cancels after 5s idle.
 * While active, INC/DEC adjust values instead of switching screens. */
static uint8_t     edit_mode = 0;
static uint8_t     ui_locked = 0;         /* 1 = UI locked, only PWR works */

static uint32_t    edit_last_input_tick = 0;
#define EDIT_TIMEOUT_MS 5000U

/* Dashboard voltage/current adjust (sub-state of edit mode on DSH).
 * SEL cycles: V coarse (1V steps) -> V fine (0.1V, PPS only) -> I (0.1A).
 * The active field's digit is drawn with inverted colors (white bg). */
static uint8_t     adjust_field = 0;     /* 0=V coarse, 1=V fine, 2=I */
static uint32_t    adjust_target_mv = 0; /* working copy while editing */
static uint32_t    adjust_target_ma = 0;

/* ------------------------------------------------------------------ */
/*  Helpers for PDO voltage extraction                                 */
/*  PDO bit-field decoding per USB PD 3.1 spec:                        */
/*    bits[31:30] = type (0=Fixed, 3=Augmented)                        */
/*    For Fixed:  bits[19:10]*50mV = voltage, bits[9:0]*10mA = current */
/*    For APDO:   bits[29:28] = subtype (0=PPS, 1=AVS/EPR)            */
/* ------------------------------------------------------------------ */

/** Get sorted list of fixed PDO voltages in mV (used for coarse V stepping) */
static uint8_t UI_GetFixedVoltages(uint32_t *out, uint8_t max_out)
{
    uint32_t pdos[7];
    uint8_t count = axxpd_get_src_pdos(pdos, 7);
    uint8_t n = 0;
    for (uint8_t i = 0; i < count && n < max_out; i++) {
        uint32_t pdo = pdos[i];
        uint32_t type = (pdo >> 30U) & 0x3U;
        if (type == 0U) {                      /* Fixed PDO */
            out[n++] = ((pdo >> 10U) & 0x3FFU) * 50U;
        }
    }
    return n;
}

/** Check if any PPS APDO is available — determines whether fine V adjust
 *  and continuous voltage range are available (vs fixed-PDO stepping) */
/** Check if any programmable voltage APDO is available (PPS or AVS). */
static uint8_t UI_HasPPS(void)
{
    uint32_t pdos[14];
    uint8_t count = axxpd_get_src_pdos(pdos, 14);
    for (uint8_t i = 0; i < count; i++) {
        uint32_t pdo = pdos[i];
        if (pdo == 0U) continue;
        uint32_t type = (pdo >> 30U) & 0x3U;
        uint32_t sub  = (pdo >> 28U) & 0x3U;
        /* PPS (sub=0) or EPR AVS (sub=1) */
        if (type == 3U && (sub == 0U || sub == 1U))
            return 1;
    }
    return 0;
}

/** Get combined programmable voltage range (min/max mV) across PPS + AVS.
 *  PPS and AVS may cover different ranges — we take the union. */
static void UI_GetPPSRange(uint32_t *min_mv, uint32_t *max_mv)
{
    *min_mv = 99000U; *max_mv = 0U;
    uint32_t pdos[14];
    uint8_t count = axxpd_get_src_pdos(pdos, 14);
    for (uint8_t i = 0; i < count; i++) {
        uint32_t pdo = pdos[i];
        if (pdo == 0U) continue;
        uint32_t type = (pdo >> 30U) & 0x3U;
        uint32_t sub  = (pdo >> 28U) & 0x3U;
        if (type == 3U && sub == 0U) {
            /* SPR PPS: min [15:8]×100mV, max [24:17]×100mV */
            uint32_t lo = ((pdo >> 8U) & 0xFFU) * 100U;
            uint32_t hi = ((pdo >> 17U) & 0xFFU) * 100U;
            if (lo < *min_mv) *min_mv = lo;
            if (hi > *max_mv) *max_mv = hi;
        } else if (type == 3U && sub == 1U) {
            /* EPR AVS: min [15:8]×100mV, max [25:17]×100mV (9-bit) */
            uint32_t lo = ((pdo >> 8U) & 0xFFU) * 100U;
            uint32_t hi = ((pdo >> 17U) & 0x1FFU) * 100U;
            if (lo < *min_mv) *min_mv = lo;
            if (hi > *max_mv) *max_mv = hi;
        }
    }
}


/* ------------------------------------------------------------------ */
/*  Status bar — 30px, shared across all screens                       */
/*  Shows output state (ON/OFF) and negotiated PD contract info.       */
/*  Background color provides at-a-glance status:                      */
/*    grey = output off, green = output on, red = HW fault active      */
/* ------------------------------------------------------------------ */
static void UI_DrawStatusBar(uint8_t output_on, float ntc_temp)
{
    char buf[32];
    uint16_t bar_bg, bar_fg;

    /* Background color encodes system state visually */
    if (g_hw_fault && !fault_acked) {
        bar_bg = RGB(180,0,0); bar_fg = COL_WHITE;           /* red = fault */
    } else if (output_on) {
        bar_bg = RGB(90,220,90); bar_fg = COL_BLACK;         /* bright green = on */
    } else {
        bar_bg = RGB(122,122,122); bar_fg = RGB(255,255,255); /* grey = off, white text */
    }

    /* Only repaint bar background on state transitions to avoid flicker */
    static uint16_t prev_bar_bg = 0xFFFF;
    static uint8_t prev_lock_drawn = 0xFF;  /* track padlock state to avoid flicker */
    if (bar_bg != prev_bar_bg) {
        LCD_Fill(0, 0, SCREEN_W - 1, STATUSBAR_H - 1, bar_bg);
        prev_bar_bg = bar_bg;
        prev_lock_drawn = 0xFF;  /* force padlock redraw after bg wipe */
    }

    /* Text is drawn with matching bgcolor so it overwrites in-place (no flicker).
     * Bottom-aligned within the 30px bar (font is 18px, 4px pad from bottom). */
    #define SBAR_TY  (STATUSBAR_H - 18)
    if (g_hw_fault && !fault_acked) {
        LCD_PutStr(DRAW_X, SBAR_TY, "FAULT! HW PROTECTION              ", FONT_SM, COL_WHITE, bar_bg);
    } else {
        /* Show PD mode (SPR/PPS/EPR) and negotiated contract */
        const char *mode = axxpd_is_epr_active() ? "EPR" :
                           (axxpd_is_pps_active() ? "PPS" : "SPR");
        float v = axxpd_get_negotiated_v();
        float a = axxpd_get_negotiated_a();
        snprintf(buf, sizeof(buf), "%-3s", output_on ? "ON" : "OFF");
        LCD_PutStr(DRAW_X, SBAR_TY, buf, FONT_SM, bar_fg, bar_bg);
        /* Padlock icon — only redraw on state change to prevent flicker */
        if (ui_locked != prev_lock_drawn) {
            uint16_t lock_x = DRAW_X + 40;
            uint16_t lock_y = SBAR_TY - 1;
            if (ui_locked)
                draw_padlock(lock_x, lock_y, bar_fg, bar_bg);
            else
                LCD_Fill(lock_x, lock_y, lock_x + PADLOCK_W - 1, lock_y + PADLOCK_H - 1, bar_bg);
            prev_lock_drawn = ui_locked;
        }

        snprintf(buf, sizeof(buf), "SRC: %s %.1fV %.1fA  ",
                 mode, (double)v, (double)a);
        LCD_PutStr(DRAW_X + 100, SBAR_TY, buf, FONT_SM, bar_fg, bar_bg);

        /* Timer countdown — right side of status bar (only when active) */
        {
            static uint8_t prev_timer_visible = 0;
            uint16_t rem = Timer_GetRemaining();
            if (rem > 0) {
                snprintf(buf, sizeof(buf), "T:%3us", (unsigned)rem);
                LCD_PutStr(255, SBAR_TY, buf, FONT_SM, COL_WHITE, bar_bg);
                prev_timer_visible = 1;
            } else if (prev_timer_visible) {
                LCD_Fill(255, SBAR_TY, SCREEN_W - 1, SBAR_TY + 18, bar_bg);
                prev_timer_visible = 0;
            }
        }

        /* Padlock icon drawn above alongside ON/OFF text */
    }
}

/* ------------------------------------------------------------------ */
/*  Navbar — bottom 30px, shows all 6 screen tabs with [brackets]      */
/*  on the active one.  Incremental: only repaints changed items.      */
/* ------------------------------------------------------------------ */
#define NAVBAR_BG  RGB(122,122,122)
#define NAVBAR_FG  RGB(255,255,255)  /* white navbar text */

static uint8_t navbar_last_screen = 0xFF;
static uint8_t navbar_drawn = 0;  /* 0 = never drawn, needs full paint */

static const char *nav_labels[UI_SCREEN_COUNT] = {"DSH", "PDO", "GRP", "PRE", "NRG", "SET"};
#define NAV_TY  (NAVBAR_Y + (NAVBAR_H - 18) / 2 - 3)  /* vertically centered, nudged 3px up */

/** Draw one navbar tab — "[GRP]" if selected, " GRP " otherwise */
static void nav_draw_item(uint8_t idx, uint8_t selected)
{
    char buf[6];
    if (selected)
        snprintf(buf, sizeof(buf), "[%s]", nav_labels[idx]);
    else
        snprintf(buf, sizeof(buf), " %s ", nav_labels[idx]);
    /* 6 tabs * 50px = 300px, fits within 320px screen width */
    uint16_t x = (uint16_t)(DRAW_X + idx * 50U);
    LCD_PutStr(x, NAV_TY, buf, FONT_SM, NAVBAR_FG, NAVBAR_BG);
}

static void UI_DrawScreenIndicator(void)
{
    uint8_t cur = (uint8_t)current_screen;
    if (cur == navbar_last_screen && navbar_drawn) return;

    if (!navbar_drawn) {
        /* First frame — fill background and draw all 6 tabs */
        LCD_Fill(0, NAVBAR_Y, SCREEN_W - 1, SCREEN_H - 1, NAVBAR_BG);
        for (uint8_t i = 0; i < UI_SCREEN_COUNT; i++)
            nav_draw_item(i, (i == cur));
        navbar_drawn = 1;
    } else {
        /* Only repaint the deselected old tab and the newly selected tab */
        if (navbar_last_screen < UI_SCREEN_COUNT)
            nav_draw_item(navbar_last_screen, 0);
        nav_draw_item(cur, 1);
    }
    navbar_last_screen = cur;
}

/* ------------------------------------------------------------------ */
/*  Dashboard screen                                                   */
/*                                                                     */
/*  Two-column layout with measured + set-point values:                 */
/*                                                                     */
/*  y=37  | 12.4V  (XL, yellow)  |  1.23A  (XL, orange)  |  measured  */
/*  y=79  | SET: 12.0V           |  SET: 3.0A             |  target    */
/*  y=109 | 15.25 W  (LG, green) | CC/CV (SM, blue)       |  power    */
/*                                                                     */
/*  In edit mode, the active digit is drawn inverted (black on white)  */
/*  to show which field INC/DEC will adjust.                           */
/* ------------------------------------------------------------------ */
static void UI_DrawDashboard(INA228_Reading_t *r, float ntc_temp, uint8_t output_on)
{
    char buf[32];

    UI_DrawStatusBar(output_on, ntc_temp);

    #define COL_CURRENT    RGB(255, 100, 40)   /* lighter orange for current readings */
    #define COL_POWER      RGB(90, 220, 90)    /* bright green for power readings */

    #define COL_RIGHT  168  /* X pixel where the right column (current) begins */

    /* Left column: measured voltage — auto-format to keep 4 significant digits */
    fmt_3sig(buf, sizeof(buf), r->voltage_v, "V", " ");
    LCD_PutStr(DRAW_X, CONTENT_Y, buf, FONT_XL, COL_YELLOW, COL_BG);

    /* SET voltage row — shows working target in edit mode, negotiated otherwise.
     * Each digit is drawn individually so the active one can be inverted. */
    {
        uint32_t sv = edit_mode ? adjust_target_mv : (uint32_t)(axxpd_get_negotiated_v() * 1000.0f);
        uint8_t whole = (uint8_t)(sv / 1000U);
        uint8_t dec   = (uint8_t)((sv % 1000U) / 100U);
        #define SET_Y  (CONTENT_Y + 42)    /* y offset for the SET row */
        #define HI_BG  RGB(255,255,255)    /* inverted highlight background */
        #define HI_FG  RGB(0,0,0)          /* inverted highlight foreground */
        /* Blink active digit at 1Hz (500ms on, 500ms off) */
        uint8_t blink_on = (uint8_t)((HAL_GetTick() / 500U) & 1U);
        /* Clear the SET row when digit count changes (e.g. 5V -> 12V)
         * to avoid leftover pixels from different-width layouts */
        static uint8_t prev_v_digits = 0;
        uint8_t v_digits = (whole >= 10) ? 2 : 1;
        if (v_digits != prev_v_digits) {
            LCD_Fill(DRAW_X, SET_Y, COL_RIGHT - 5, SET_Y + 24, COL_BG);
            prev_v_digits = v_digits;
        }
        LCD_PutStr(DRAW_X, SET_Y, "SET: ", FONT_MD, COL_YELLOW, COL_BG);
        /* Draw whole-volt digit(s) — 2-digit path for >=10V, 1-digit otherwise.
         * adjust_field==0 highlights the whole-volt digit(s) (coarse adjust). */
        char d[2] = {(char)('0' + whole), 0};
        if (whole >= 10) { d[0] = (char)('0' + whole/10); char d2[3] = {d[0], (char)('0'+whole%10), 0};
            /* Field 0 active: blink_on = inverted, blink_off = normal yellow */
            uint8_t f0_hi = (edit_mode && adjust_field == 0 && blink_on);
            LCD_PutStr(DRAW_X + 68, SET_Y, d2, FONT_MD,
                f0_hi ? HI_FG : COL_YELLOW,
                f0_hi ? HI_BG : COL_BG);
            LCD_PutStr(DRAW_X + 96, SET_Y, ".", FONT_MD, COL_YELLOW, COL_BG);
            /* Decimal digit — adjust_field==1 highlights (fine 0.1V adjust, PPS only) */
            d[0] = (char)('0' + dec);
            uint8_t f1_hi = (edit_mode && adjust_field == 1 && blink_on);
            LCD_PutStr(DRAW_X + 106, SET_Y, d, FONT_MD,
                f1_hi ? HI_FG : COL_YELLOW,
                f1_hi ? HI_BG : COL_BG);
            LCD_PutStr(DRAW_X + 120, SET_Y, "V  ", FONT_MD, COL_YELLOW, COL_BG);
        } else {
            uint8_t f0_hi = (edit_mode && adjust_field == 0 && blink_on);
            LCD_PutStr(DRAW_X + 68, SET_Y, d, FONT_MD,
                f0_hi ? HI_FG : COL_YELLOW,
                f0_hi ? HI_BG : COL_BG);
            LCD_PutStr(DRAW_X + 82, SET_Y, ".", FONT_MD, COL_YELLOW, COL_BG);
            d[0] = (char)('0' + dec);
            uint8_t f1_hi = (edit_mode && adjust_field == 1 && blink_on);
            LCD_PutStr(DRAW_X + 92, SET_Y, d, FONT_MD,
                f1_hi ? HI_FG : COL_YELLOW,
                f1_hi ? HI_BG : COL_BG);
            LCD_PutStr(DRAW_X + 106, SET_Y, "V  ", FONT_MD, COL_YELLOW, COL_BG);
        }
    }

    /* Right column: measured current — same large font as voltage */
    fmt_3sig(buf, sizeof(buf), r->current_a, "A", " ");
    LCD_PutStr(COL_RIGHT, CONTENT_Y, buf, FONT_XL, COL_CURRENT, COL_BG);

    /* SET current — adjust_field==2 highlights the decimal digit */
    {
        uint32_t sa = edit_mode ? adjust_target_ma : (uint32_t)(axxpd_get_negotiated_a() * 1000.0f);
        uint8_t a_whole = (uint8_t)(sa / 1000U);
        uint8_t a_dec   = (uint8_t)((sa % 1000U) / 100U);
        uint8_t blink_on_i = (uint8_t)((HAL_GetTick() / 500U) & 1U);
        LCD_PutStr(COL_RIGHT, SET_Y, "SET: ", FONT_MD, COL_CURRENT, COL_BG);
        char d[2] = {(char)('0' + a_whole), 0};
        LCD_PutStr(COL_RIGHT + 68, SET_Y, d, FONT_MD, COL_CURRENT, COL_BG);
        LCD_PutStr(COL_RIGHT + 82, SET_Y, ".", FONT_MD, COL_CURRENT, COL_BG);
        d[0] = (char)('0' + a_dec);
        uint8_t f2_hi = (edit_mode && adjust_field == 2 && blink_on_i);
        LCD_PutStr(COL_RIGHT + 92, SET_Y, d, FONT_MD,
            f2_hi ? HI_FG : COL_CURRENT,
            f2_hi ? HI_BG : COL_BG);
        LCD_PutStr(COL_RIGHT + 106, SET_Y, "A  ", FONT_MD, COL_CURRENT, COL_BG);
    }

    /* Bottom region: normally the power + CC/CV + cable-drop readouts, but while
     * a boot autostart is counting down it is REPLACED by the countdown banner.
     * Drawing one OR the other (never both) in this region is essential — the
     * banner and the power/CC-CV readouts share these pixels, and letting both
     * paint every frame is what caused the flicker/overwrite. Output is off
     * during the countdown, so the readouts carry no useful info meanwhile. */
    {
        static uint8_t s_autostart_banner = 0;
        #define BAN_Y0  (CONTENT_Y + 66)        /* top of the bottom region */
        if (Boot_AutostartPending()) {
            if (!s_autostart_banner) {           /* fill once, then only text repaints */
                LCD_Fill(0, BAN_Y0, SCREEN_W - 1, NAVBAR_Y - 1, COL_SEL_BG);
                s_autostart_banner = 1;
            }
            char ab[40];
            int secs = Boot_AutostartSecsRemaining();
            if (secs > 0) snprintf(ab, sizeof(ab), "Autostarting in: %d", secs);
            else          snprintf(ab, sizeof(ab), "Autostarting...   ");
            LCD_PutStr(DRAW_X, CONTENT_Y + 68, ab,                FONT_SM, COL_WHITE, COL_SEL_BG);
            LCD_PutStr(DRAW_X, CONTENT_Y + 86, "SELECT to abort", FONT_SM, COL_WHITE, COL_SEL_BG);
        } else {
            if (s_autostart_banner) {            /* leaving the countdown — clear the strip */
                LCD_Fill(0, BAN_Y0, SCREEN_W - 1, NAVBAR_Y - 1, COL_BG);
                s_autostart_banner = 0;
            }

            /* Bottom row: power reading — >=100W:"123W"  >=10W:"77.8W"  <10W:"5.23W" */
            fmt_3sig(buf, sizeof(buf), r->power_w, "W", " ");
            LCD_PutStr(DRAW_X, CONTENT_Y + 68, buf, FONT_LG, COL_POWER, COL_BG);

            /* CC/CV mode indicator — current- or voltage-limited */
            {
                static uint8_t prev_cccv = 0; /* 0=none, 1=CV, 2=CC */
                if (output_on) {
                    float neg_a = axxpd_get_negotiated_a();
                    uint8_t is_cc = (r->current_a > 0.1f && neg_a > 0.1f && r->current_a >= neg_a * 0.95f) ? 1U : 0U;
                    uint8_t cccv = is_cc ? 2U : 1U;
                    if (cccv != prev_cccv) {
                        LCD_Fill(DRAW_X + 160, CONTENT_Y + 68, DRAW_X + 195, CONTENT_Y + 86, COL_BG);
                    }
                    #define COL_CCCV  RGB(93, 162, 240)
                    if (is_cc) {
                        LCD_PutStr(DRAW_X + 160, CONTENT_Y + 68, "CC", FONT_SM, COL_CCCV, COL_BG);
                    } else {
                        LCD_PutStr(DRAW_X + 160, CONTENT_Y + 68, "CV", FONT_SM, COL_CCCV, COL_BG);
                    }
                    prev_cccv = cccv;
                } else {
                    if (prev_cccv != 0U) {
                        LCD_Fill(DRAW_X + 160, CONTENT_Y + 68, DRAW_X + 195, CONTENT_Y + 86, COL_BG);
                        prev_cccv = 0;
                    }
                }
            }

            /* Cable voltage drop warning — shown when INA228 available and drop > 0.3V */
            {
                static uint8_t cable_warn_shown = 0;
                if (output_on && g_ina.hi2c != NULL && r->current_a > 0.1f) {
                    float drop = axxpd_get_negotiated_v() - r->voltage_v;
                    if (drop > 0.3f) {
                        char cw[18];
                        snprintf(cw, sizeof(cw), "Cable:-%4.2fV", (double)drop);
                        LCD_PutStr(DRAW_X + 160, CONTENT_Y + 86, cw, FONT_SM, COL_WHITE, COL_BG);
                        cable_warn_shown = 1;
                    } else {
                        if (cable_warn_shown) {
                            LCD_Fill(DRAW_X + 160, CONTENT_Y + 86, SCREEN_W - 1, CONTENT_Y + 104, COL_BG);
                            cable_warn_shown = 0;
                        }
                    }
                } else {
                    if (cable_warn_shown) {
                        LCD_Fill(DRAW_X + 160, CONTENT_Y + 86, SCREEN_W - 1, CONTENT_Y + 104, COL_BG);
                        cable_warn_shown = 0;
                    }
                }
            }
        }
    }

    UI_DrawScreenIndicator();
}

/* ------------------------------------------------------------------ */
/*  Graph screen — V/I rolling trace with numeric readouts above       */
/* ------------------------------------------------------------------ */
static void UI_DrawGraph(INA228_Reading_t *r, float ntc_temp, uint8_t output_on)
{
    char buf[16];

    UI_DrawStatusBar(output_on, ntc_temp);

    /* Numeric readouts above the plot — update at full 30Hz (cheap text draws).
     * Trailing spaces overwrite residual chars when value string shrinks. */
    fmt_3sig(buf, sizeof(buf), r->voltage_v, "V", "  ");
    LCD_PutStr(DRAW_X, CONTENT_Y, buf, FONT_MD, COL_YELLOW, COL_BG);

    fmt_3sig(buf, sizeof(buf), r->current_a, "A", "  ");
    LCD_PutStr(DRAW_X + 155, CONTENT_Y, buf, FONT_MD, COL_CURRENT, COL_BG);

    /* Graph trace is expensive (per-pixel Bresenham + grid restore).
     * Throttled to ~15Hz (66ms) so it doesn't starve the main loop. */
    {
        static uint32_t last_graph_draw = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_graph_draw >= 66U) {
            last_graph_draw = now;
            Graph_Draw();
        }
    }

    UI_DrawScreenIndicator();
}

/* ------------------------------------------------------------------ */
/*  PDO screen — scrollable list of source capabilities                */
/*  Sorted: Fixed PDOs first, then PPS, then AVS (EPR).               */
/*  Edit mode: INC/DEC scroll, SEL_LONG requests the highlighted PDO.  */
/* ------------------------------------------------------------------ */
static void UI_DrawPDOs(float ntc_temp, uint8_t output_on)
{
    uint16_t y;
    uint8_t i;
    uint32_t raw[14], pdos[14];
    uint8_t raw_cnt = axxpd_get_src_pdos(raw, 14);
    uint8_t count = 0;
    /* Sort PDOs into display order: Fixed, PPS, AVS — three passes over raw */
    for (uint8_t f = 0; f < raw_cnt; f++) {
        if (raw[f] != 0U && ((raw[f] >> 30U) & 0x3U) == 0U) pdos[count++] = raw[f];
    }
    for (uint8_t f = 0; f < raw_cnt; f++) {
        if (raw[f] != 0U && ((raw[f] >> 30U) & 0x3U) == 3U && ((raw[f] >> 28U) & 0x3U) == 0U)
            pdos[count++] = raw[f];
    }
    for (uint8_t f = 0; f < raw_cnt; f++) {
        if (raw[f] != 0U && ((raw[f] >> 30U) & 0x3U) == 3U && ((raw[f] >> 28U) & 0x3U) == 1U)
            pdos[count++] = raw[f];
    }
    uint8_t row_h = 20;
    uint8_t visible = (uint8_t)((NAVBAR_Y - CONTENT_Y - 2) / row_h);
    if (visible > 5) visible = 5;  /* cap to avoid overflow into navbar */

    /* Clamp cursor/scroll so they stay valid if PDO count changes at runtime
     * (e.g. charger re-enumerates or cable swap) */
    if (pdo_cursor >= count && count > 0U) pdo_cursor = (uint8_t)(count - 1U);
    if (pdo_cursor < pdo_scroll) pdo_scroll = pdo_cursor;
    if (pdo_cursor >= pdo_scroll + visible) pdo_scroll = (uint8_t)(pdo_cursor - visible + 1U);
    uint8_t start = pdo_scroll;

    UI_DrawStatusBar(output_on, ntc_temp);

    if (count == 0U) {
        LCD_PutStr(DRAW_X, CONTENT_Y, "No source connected       ", FONT_SM, COL_GREY, COL_BG);
        UI_DrawScreenIndicator();
        return;
    }

    if (start + visible > count) {
        start = (count > visible) ? (count - visible) : 0;
    }

    /* Full redraw needed if scroll position or PDO count changed;
     * otherwise only redraw the rows whose selection state changed. */
    uint8_t full_redraw = (pdo_scroll != s_pdo_prev_scroll || count != s_pdo_prev_count);

    for (i = 0; i < visible; i++) {
        y = CONTENT_Y + i * row_h;
        if ((start + i) < count) {
            uint8_t selected = ((start + i) == pdo_cursor) ? 1U : 0U;
            uint8_t was_selected = ((start + i) == s_pdo_prev_cursor) ? 1U : 0U;
            if (!full_redraw && selected == was_selected) continue;
            uint32_t pdo = pdos[start + i];
            uint32_t type = (pdo >> 30U) & 0x3U;
            char marker = selected ? '>' : ' ';
            char line[40];
            if (type == 0U) {
                uint32_t mv  = ((pdo >> 10U) & 0x3FFU) * 50U;
                uint32_t ma  = ((pdo >>  0U) & 0x3FFU) * 10U;
                snprintf(line, sizeof(line), "%c%u: FIXED %2lu.%03luV %lu.%luA   ",
                         marker, (unsigned)(start + i + 1),
                         (unsigned long)(mv / 1000U),
                         (unsigned long)(mv % 1000U),
                         (unsigned long)(ma / 1000U),
                         (unsigned long)((ma % 1000U) / 100U));
            } else if (type == 3U) {
                uint32_t subtype = (pdo >> 28U) & 0x3U;
                if (subtype == 0U) {
                    uint32_t min_mv = ((pdo >> 8U) & 0xFFU) * 100U;
                    uint32_t max_mv = ((pdo >> 17U) & 0xFFU) * 100U;
                    uint32_t max_ma = ((pdo >> 0U) & 0x7FU) * 50U;
                    snprintf(line, sizeof(line), "%c%u: PPS %lu.%lu-%lu.%luV %lu.%luA ",
                             marker, (unsigned)(start + i + 1),
                             (unsigned long)(min_mv / 1000U),
                             (unsigned long)((min_mv % 1000U) / 100U),
                             (unsigned long)(max_mv / 1000U),
                             (unsigned long)((max_mv % 1000U) / 100U),
                             (unsigned long)(max_ma / 1000U),
                             (unsigned long)((max_ma % 1000U) / 100U));
                } else if (subtype == 1U) {
                    uint32_t min_mv = ((pdo >> 8U) & 0xFFU) * 100U;
                    uint32_t max_mv = ((pdo >> 17U) & 0x1FFU) * 100U;
                    uint32_t pdp_w  = (pdo & 0xFFU);
                    snprintf(line, sizeof(line), "%c%u: AVS %lu.%lu-%lu.%luV %luW   ",
                             marker, (unsigned)(start + i + 1),
                             (unsigned long)(min_mv / 1000U),
                             (unsigned long)((min_mv % 1000U) / 100U),
                             (unsigned long)(max_mv / 1000U),
                             (unsigned long)((max_mv % 1000U) / 100U),
                             (unsigned long)pdp_w);
                } else {
                    snprintf(line, sizeof(line), "%c%u: APDO t%lu              ",
                             marker, (unsigned)(start + i + 1),
                             (unsigned long)subtype);
                }
            } else {
                snprintf(line, sizeof(line), "%c%u: PDO type %lu            ",
                         marker, (unsigned)(start + i + 1),
                         (unsigned long)type);
            }
            {
                uint16_t bg = (selected && edit_mode) ? COL_SEL_BG : COL_BG;
                uint16_t fg = COL_WHITE;
                LCD_Fill(0, y, SCREEN_W - 1, y + row_h - 1, bg);
                LCD_PutStr(DRAW_X, y + TEXT_VPAD, line, FONT_SM, fg, bg);
            }
        } else {
            /* Clear unused row */
            LCD_Fill(0, y, SCREEN_W - 1, y + row_h - 1, COL_BG);
        }
    }

    s_pdo_prev_cursor = pdo_cursor;
    s_pdo_prev_scroll = pdo_scroll;
    s_pdo_prev_count  = count;

    if (edit_mode)
        ui_draw_scrollbar(count, visible, pdo_scroll,
                          CONTENT_Y, visible * row_h);

    UI_DrawScreenIndicator();
}

/* ------------------------------------------------------------------ */
/*  Presets screen — 5 fixed slots for saved V/A combinations          */
/*  Edit mode: SEL applies preset, SEL_LONG saves (empty) or deletes   */
/*  (filled).  Slots show "---" when empty.                            */
/* ------------------------------------------------------------------ */
static void UI_DrawPresets(float ntc_temp, uint8_t output_on)
{
    char buf[32];
    uint16_t y;
    uint8_t row_h = 20;

    UI_DrawStatusBar(output_on, ntc_temp);

    /* Force full repaint when entering/leaving edit mode (highlight changes) */
    static uint8_t pre_prev_edit = 0xFF;
    uint8_t pre_full = (s_pre_prev_cursor == 0xFF || pre_prev_edit != edit_mode);

    for (uint8_t i = 0; i < PRESET_SLOTS; i++) {
        y = CONTENT_Y + i * row_h;
        uint8_t sel = (i == preset_cursor);
        uint8_t was = (i == s_pre_prev_cursor);
        if (!pre_full && sel == was) continue;

        uint16_t bg = (sel && edit_mode) ? COL_SEL_BG : COL_BG;
        uint16_t fg = COL_WHITE;
        LCD_Fill(0, y, SCREEN_W - 1, y + row_h - 1, bg);

        if (!Settings_PresetIsEmpty(i)) {
            Preset_t *p = Settings_GetPreset(i);
            snprintf(buf, sizeof(buf), "%c %s", sel ? '>' : ' ', p->name);
        } else {
            snprintf(buf, sizeof(buf), "%c ---", sel ? '>' : ' ');
        }
        LCD_PutStr(DRAW_X, y + TEXT_VPAD, buf, FONT_SM, fg, bg);
    }
    s_pre_prev_cursor = preset_cursor;
    pre_prev_edit = edit_mode;

    UI_DrawScreenIndicator();
}

/* ------------------------------------------------------------------ */
/*  Energy screen — cumulative Ah/Wh from INA228 hardware accumulators */
/*  Edit mode + SEL_LONG resets the energy counters.                   */
/*                                                                     */
/*  y=37  |  12.40V              |  (full width, large font)           */
/*  y=77  |  1.234A   |  15.25 W |  (two columns: current + power)    */
/*  y=103 |  0.123 Ah | 1.234 Wh |  (two columns: charge + energy)    */
/* ------------------------------------------------------------------ */
static void UI_DrawEnergy(INA228_Reading_t *r, float ntc_temp, uint8_t output_on)
{
    char buf[22];

    #define NRG_COL_R  165  /* X for right column (W and Wh readings) */

    UI_DrawStatusBar(output_on, ntc_temp);

    fmt_3sig(buf, sizeof(buf), r->voltage_v, "V", "   ");
    LCD_PutStr(DRAW_X, CONTENT_Y, buf, FONT_XL, COL_YELLOW, COL_BG);

    fmt_3sig(buf, sizeof(buf), r->current_a, "A", "  ");
    LCD_PutStr(DRAW_X, CONTENT_Y + 48, buf, FONT_MD, COL_CURRENT, COL_BG);

    fmt_3sig(buf, sizeof(buf), r->power_w, "W", "  ");
    LCD_PutStr(NRG_COL_R, CONTENT_Y + 48, buf, FONT_MD, COL_POWER, COL_BG);

    if (r->charge_ah >= 1000.0f)
        snprintf(buf, sizeof(buf), "%.0f Ah ", (double)r->charge_ah);
    else if (r->charge_ah >= 100.0f)
        snprintf(buf, sizeof(buf), "%.1f Ah ", (double)r->charge_ah);
    else
        snprintf(buf, sizeof(buf), "%.3f Ah ", (double)r->charge_ah);
    LCD_PutStr(DRAW_X, CONTENT_Y + 74, buf, FONT_MD, COL_CURRENT, COL_BG);

    if (r->energy_wh >= 1000.0f)
        snprintf(buf, sizeof(buf), "%.0f Wh ", (double)r->energy_wh);
    else if (r->energy_wh >= 100.0f)
        snprintf(buf, sizeof(buf), "%.1f Wh ", (double)r->energy_wh);
    else
        snprintf(buf, sizeof(buf), "%.3f Wh ", (double)r->energy_wh);
    LCD_PutStr(NRG_COL_R, CONTENT_Y + 74, buf, FONT_MD, COL_POWER, COL_BG);

    UI_DrawScreenIndicator();
}

/* ------------------------------------------------------------------ */
/*  Settings screen — 2-level hierarchical menu                        */
/*  Level 0: group list (Mode, Sound, System)                          */
/*  Level 1: items within a group + "Back" item at the bottom          */
/*  SEL drills into a group or toggles a bool; SEL_LONG goes back.     */
/*  Special MI_ actions: load defaults, save+reboot, exit without save */
/* ------------------------------------------------------------------ */

/* Settings incremental redraw tracking */
static uint8_t s_set_prev_group = 0xFF, s_set_prev_item = 0xFF;
static uint8_t s_set_prev_level = 0xFF, s_set_prev_edit = 0xFF;
static uint8_t s_set_scroll = 0;
static uint8_t s_set_prev_adj = 0;              /* previous adjusting_flag state */

/* ------------------------------------------------------------------ */
/*  Tool overlay state                                                 */
/* ------------------------------------------------------------------ */
#define TOOL_NONE           0
#define TOOL_CHARGER_INFO   1
#define TOOL_VOLTAGE_SWEEP  2
#define TOOL_CABLE_INFO     3
#define TOOL_SELFTEST       4

static uint8_t tool_active = TOOL_NONE;  /* currently active tool (TOOL_*) */
static int8_t  tool_scroll = 0;           /* scroll offset within tool display */
static uint8_t tool_drawn  = 0;           /* 0 = needs redraw */

/* Voltage sweep results */
typedef struct {
    uint16_t req_mv;
    uint16_t meas_mv;
    uint16_t meas_ma;
    uint8_t  pass;
} SweepResult_t;

static SweepResult_t sweep_results[14];
static uint8_t sweep_count   = 0;
static uint8_t sweep_done    = 0;  /* 1 = sweep complete, showing results */
static uint8_t sweep_running = 0;  /* 1 = sweep in progress (main loop) */
static uint8_t sweep_step    = 0;  /* current PDO index being tested */
static uint32_t sweep_step_tick = 0;  /* tick when current step started */

/* Selftest — walks all PDOs (Fixed + PPS mid + AVS mid) + random, measures, pass/fail */
#define ST_MAX_STEPS 40
#define ST_RANDOM_STEPS 5
typedef struct {
    char     label[26];
    uint16_t req_mv;
    uint16_t meas_mv;
    uint8_t  pass;    /* 0=pending, 1=pass, 2=fail */
} SelftestResult_t;

static SelftestResult_t st_results[ST_MAX_STEPS];
static uint8_t  st_count     = 0;
static uint8_t  st_step      = 0;
static uint8_t  st_done      = 0;
static uint8_t  st_running   = 0;
static uint32_t st_step_tick = 0;

/* PDO list for selftest — stores raw PDOs + their 1-based position in source_caps */
static uint32_t st_pdos[14];
static uint8_t  st_pdo_pos[14];  /* 1-based position for trigger_by_position */
static uint8_t  st_pdo_count = 0;

/* Forward declaration for tool draw functions */
static void Tool_DrawChargerInfo(void);
static void Tool_DrawVoltageSweep(void);
static void Tool_DrawCableInfo(void);
static void Tool_DrawSelftest(void);

/* ------------------------------------------------------------------ */
/*  PDO type decoder — returns a human-readable line into buf          */
/* ------------------------------------------------------------------ */
static void DecodePDO(uint32_t pdo, uint8_t idx, char *buf, uint8_t bufsz)
{
    uint32_t type = (pdo >> 30U) & 0x3U;
    if (type == 0U) {
        /* Fixed PDO */
        uint32_t mv = ((pdo >> 10U) & 0x3FFU) * 50U;
        uint32_t ma = ((pdo >>  0U) & 0x3FFU) * 10U;
        snprintf(buf, bufsz, "PDO%u: Fixed %lu.%luV %lu.%luA",
                 (unsigned)idx,
                 (unsigned long)(mv / 1000U), (unsigned long)((mv % 1000U) / 100U),
                 (unsigned long)(ma / 1000U), (unsigned long)((ma % 1000U) / 100U));
    } else if (type == 3U) {
        uint32_t sub = (pdo >> 28U) & 0x3U;
        if (sub == 0U) {
            /* PPS */
            uint32_t min_mv = ((pdo >> 8U) & 0xFFU) * 100U;
            uint32_t max_mv = ((pdo >> 17U) & 0xFFU) * 100U;
            uint32_t max_ma = ((pdo >> 0U) & 0x7FU) * 50U;
            snprintf(buf, bufsz, "PDO%u: PPS %lu.%lu-%lu.%luV %lu.%luA",
                     (unsigned)idx,
                     (unsigned long)(min_mv / 1000U), (unsigned long)((min_mv % 1000U) / 100U),
                     (unsigned long)(max_mv / 1000U), (unsigned long)((max_mv % 1000U) / 100U),
                     (unsigned long)(max_ma / 1000U), (unsigned long)((max_ma % 1000U) / 100U));
        } else if (sub == 1U) {
            /* AVS/EPR */
            uint32_t min_mv = ((pdo >> 8U) & 0xFFU) * 100U;
            uint32_t max_mv = ((pdo >> 17U) & 0x1FFU) * 100U;
            uint32_t pdp_w  = (pdo & 0xFFU);
            snprintf(buf, bufsz, "PDO%u: AVS %lu.%lu-%lu.%luV %luW",
                     (unsigned)idx,
                     (unsigned long)(min_mv / 1000U), (unsigned long)((min_mv % 1000U) / 100U),
                     (unsigned long)(max_mv / 1000U), (unsigned long)((max_mv % 1000U) / 100U),
                     (unsigned long)pdp_w);
        } else {
            snprintf(buf, bufsz, "PDO%u: APDO sub%lu", (unsigned)idx, (unsigned long)sub);
        }
    } else {
        snprintf(buf, bufsz, "PDO%u: type%lu", (unsigned)idx, (unsigned long)type);
    }
}

/* ------------------------------------------------------------------ */
/*  Tool: Charger Info                                                 */
/* ------------------------------------------------------------------ */
static void Tool_DrawChargerInfo(void)
{
    uint16_t tool_bg = RGB(20, 20, 40);
    LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, tool_bg);

    /* Build scrollable line list */
    char lines[20][40];
    uint8_t nlines = 0;

    snprintf(lines[nlines++], 40, "=== PD Source Info ===");
    snprintf(lines[nlines++], 40, "EPR capable: %s", axxpd_is_src_epr_capable() ? "Yes" : "No");
    snprintf(lines[nlines++], 40, "EPR active:  %s", axxpd_is_epr_active()      ? "Yes" : "No");

    /* SPR PDOs */
    uint32_t pdos[7];
    uint8_t cnt = axxpd_get_src_pdos(pdos, 7);
    snprintf(lines[nlines++], 40, "SPR PDOs: %u", (unsigned)cnt);
    for (uint8_t i = 0; i < cnt && nlines < 18U; i++) {
        if (pdos[i] != 0U) {
            DecodePDO(pdos[i], i + 1U, lines[nlines], 40);
            nlines++;
        }
    }

    /* EPR PDOs */
    uint32_t epdos[7];
    uint8_t ecnt = axxpd_get_epr_src_pdos(epdos, 7);
    if (ecnt > 0U) {
        snprintf(lines[nlines++], 40, "EPR PDOs: %u", (unsigned)ecnt);
        for (uint8_t i = 0; i < ecnt && nlines < 18U; i++) {
            if (epdos[i] != 0U) {
                DecodePDO(epdos[i], i + 1U, lines[nlines], 40);
                nlines++;
            }
        }
    }

    if (nlines < 19U) snprintf(lines[nlines++], 40, "--- SEL_LONG to exit ---");

    /* Clamp scroll */
    uint8_t row_h = 18U;
    uint8_t visible = (uint8_t)((NAVBAR_Y - STATUSBAR_H) / row_h);
    if (tool_scroll < 0) tool_scroll = 0;
    if (nlines > visible && tool_scroll > (int8_t)(nlines - visible))
        tool_scroll = (int8_t)(nlines - visible);

    /* Draw visible lines */
    for (uint8_t r = 0; r < visible; r++) {
        int8_t li = (int8_t)(tool_scroll + (int8_t)r);
        if (li >= 0 && li < (int8_t)nlines) {
            uint16_t y = STATUSBAR_H + r * row_h;
            LCD_PutStr(4, y, lines[li], FONT_SM, COL_WHITE, tool_bg);
        }
    }
    ui_draw_scrollbar(nlines, visible, (uint8_t)tool_scroll,
                      STATUSBAR_H, visible * row_h);
}

/* ------------------------------------------------------------------ */
/*  Tool: Cable Info                                                   */
/* ------------------------------------------------------------------ */
static void Tool_DrawCableInfo(void)
{
    uint16_t tool_bg = RGB(20, 40, 20);
    LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, tool_bg);

    uint8_t type = 0, max_current = 0, max_voltage = 0, usb_ss = 0;
    uint8_t found = axxpd_get_cable_info(&type, &max_current, &max_voltage, &usb_ss);

    LCD_PutStr(4, STATUSBAR_H + 2, "=== Cable Info ===", FONT_SM, COL_YELLOW, tool_bg);

    if (!found) {
        LCD_PutStr(4, STATUSBAR_H + 22, "No e-marker detected", FONT_SM, COL_WHITE, tool_bg);
    } else {
        char buf[40];
        snprintf(buf, sizeof(buf), "Type:        %u", (unsigned)type);
        LCD_PutStr(4, STATUSBAR_H + 22, buf, FONT_SM, COL_WHITE, tool_bg);
        snprintf(buf, sizeof(buf), "Max current: %uA", (unsigned)max_current);
        LCD_PutStr(4, STATUSBAR_H + 40, buf, FONT_SM, COL_WHITE, tool_bg);
        snprintf(buf, sizeof(buf), "Max voltage: %uV", (unsigned)max_voltage);
        LCD_PutStr(4, STATUSBAR_H + 58, buf, FONT_SM, COL_WHITE, tool_bg);
        snprintf(buf, sizeof(buf), "USB SS:      %s", usb_ss ? "Yes" : "No");
        LCD_PutStr(4, STATUSBAR_H + 76, buf, FONT_SM, COL_WHITE, tool_bg);
    }
    LCD_PutStr(4, STATUSBAR_H + 96, "--- SEL_LONG to exit ---", FONT_SM, COL_WHITE, tool_bg);
}

/* ------------------------------------------------------------------ */
/*  Tool: Voltage Sweep                                                */
/* ------------------------------------------------------------------ */
static void Tool_DrawVoltageSweep(void)
{
    uint16_t tool_bg = RGB(40, 20, 20);
    LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, tool_bg);

    if (!sweep_done) {
        LCD_PutStr(4, STATUSBAR_H + 2, "=== Voltage Sweep ===", FONT_SM, COL_YELLOW, tool_bg);
        char buf[40];
        snprintf(buf, sizeof(buf), "Sweeping PDO %u...", (unsigned)(sweep_step + 1U));
        LCD_PutStr(4, STATUSBAR_H + 22, buf, FONT_SM, COL_WHITE, tool_bg);
        LCD_PutStr(4, STATUSBAR_H + 42, "Please wait...", FONT_SM, COL_GREY, tool_bg);
        return;
    }

    /* Show results */
    LCD_PutStr(4, STATUSBAR_H + 2, "=== Sweep Results ===", FONT_SM, COL_YELLOW, tool_bg);

    char lines[14][40];
    uint8_t nlines = 0;
    for (uint8_t i = 0; i < sweep_count && nlines < 14U; i++) {
        char buf[40];
        uint16_t req_v = sweep_results[i].req_mv / 1000U;
        uint16_t req_d = (sweep_results[i].req_mv % 1000U) / 10U;
        uint16_t meas_v = sweep_results[i].meas_mv / 1000U;
        uint16_t meas_d = (sweep_results[i].meas_mv % 1000U) / 10U;
        uint16_t meas_a = sweep_results[i].meas_ma / 1000U;
        uint16_t meas_ad = (sweep_results[i].meas_ma % 1000U) / 10U;
        snprintf(buf, sizeof(buf), "%u.%02uV:%u.%02uV %u.%02uA %s",
                 (unsigned)req_v, (unsigned)req_d,
                 (unsigned)meas_v, (unsigned)meas_d,
                 (unsigned)meas_a, (unsigned)meas_ad,
                 sweep_results[i].pass ? "OK" : "FAIL");
        memcpy(lines[nlines++], buf, 40);
    }
    if (nlines < 13U) {
        memcpy(lines[nlines++], "--- SEL_LONG to exit ---", 25);
    }

    uint8_t row_h = 18U;
    uint8_t visible = (uint8_t)((NAVBAR_Y - STATUSBAR_H - 20U) / row_h);
    if (tool_scroll < 0) tool_scroll = 0;
    if (nlines > visible && tool_scroll > (int8_t)(nlines - visible))
        tool_scroll = (int8_t)(nlines - visible);

    for (uint8_t r = 0; r < visible; r++) {
        int8_t li = (int8_t)(tool_scroll + (int8_t)r);
        uint16_t col = (li >= 0 && li < (int8_t)nlines && sweep_results[li].pass) ?
                       COL_GREEN : RGB(255, 100, 100);
        if (li >= 0 && li < (int8_t)nlines) {
            uint16_t y = STATUSBAR_H + 20U + (uint16_t)r * row_h;
            LCD_PutStr(4, y, lines[li], FONT_SM,
                       (li < (int8_t)sweep_count) ? col : COL_GREY, tool_bg);
        }
    }
    ui_draw_scrollbar(nlines, visible, (uint8_t)tool_scroll,
                      STATUSBAR_H + 20U, visible * row_h);
}

static void Tool_DrawSelftest(void)
{
    #define ST_BG       RGB(20, 20, 40)
    #define ST_COL_PASS RGB(90, 220, 90)    /* bright green — same as output ON */
    #define ST_COL_FAIL RGB(255, 60, 60)    /* red */
    #define ST_COL_PEND RGB(160, 160, 160)  /* grey for pending/in-progress */

    static uint8_t st_cleared;

    /* Clear once on first draw */
    if (!st_cleared) {
        LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, ST_BG);
        st_cleared = 1;
    }

    /* Header line — shows live progress or final summary */
    char hdr[40];
    if (st_done) {
        uint8_t pass_cnt = 0, fail_cnt = 0;
        for (uint8_t i = 0; i < st_count; i++) {
            if (st_results[i].pass == 1) pass_cnt++;
            else if (st_results[i].pass == 2) fail_cnt++;
        }
        snprintf(hdr, sizeof(hdr), "Result: %u/%u PASS  %u FAIL",
                 (unsigned)pass_cnt, (unsigned)st_count, (unsigned)fail_cnt);
        LCD_PutStr(4, STATUSBAR_H + 2, hdr, FONT_SM,
                   fail_cnt == 0 ? ST_COL_PASS : ST_COL_FAIL, ST_BG);
    } else if (st_count == 0) {
        LCD_PutStr(4, STATUSBAR_H + 2, "Building test plan...     ", FONT_SM,
                   COL_YELLOW, ST_BG);
        return;
    } else {
        snprintf(hdr, sizeof(hdr), "Self-Test  %u / %u          ",
                 (unsigned)(st_step + 1U), (unsigned)st_count);
        LCD_PutStr(4, STATUSBAR_H + 2, hdr, FONT_SM, COL_YELLOW, ST_BG);
    }

    /* Scrollable result list — shows completed results live and current step */
    uint8_t row_h = 18U;
    uint8_t visible = (uint8_t)((NAVBAR_Y - STATUSBAR_H - 20U) / row_h);

    /* Total lines: completed results + current in-progress + exit hint when done */
    uint8_t nlines = st_step;  /* completed results */
    if (!st_done && st_step < st_count) nlines++;  /* current in-progress step */
    if (st_done) nlines++;  /* exit hint */

    int8_t max_scroll = (nlines > visible) ? (int8_t)(nlines - visible) : 0;

    /* Auto-scroll to keep latest result visible during test */
    if (!st_done) {
        tool_scroll = max_scroll;
    }
    if (tool_scroll < 0) tool_scroll = 0;
    if (tool_scroll > max_scroll) tool_scroll = max_scroll;

    for (uint8_t r = 0; r < visible; r++) {
        uint8_t li = (uint8_t)((uint8_t)tool_scroll + r);
        uint16_t y = STATUSBAR_H + 20U + (uint16_t)r * row_h;

        /* Clear row first to prevent leftover pixels from previous content */
        LCD_Fill(0, y, SCREEN_W - 1, y + row_h - 1U, ST_BG);

        if (li < st_step) {
            /* Completed result */
            char line[40];
            uint16_t mv = st_results[li].meas_mv;
            snprintf(line, sizeof(line), "%-12s %u.%uV %s",
                     st_results[li].label,
                     (unsigned)(mv / 1000U), (unsigned)((mv % 1000U) / 100U),
                     st_results[li].pass == 1 ? "OK" :
                     st_results[li].pass == 2 ? "FAIL" : "..");
            uint16_t col = st_results[li].pass == 1 ? ST_COL_PASS : ST_COL_FAIL;
            LCD_PutStr(4, y, line, FONT_SM, col, ST_BG);
        } else if (!st_done && li == st_step && st_step < st_count) {
            /* Current in-progress step */
            char line[40];
            snprintf(line, sizeof(line), "%-12s ...", st_results[st_step].label);
            LCD_PutStr(4, y, line, FONT_SM, ST_COL_PEND, ST_BG);
        } else if (st_done && li == st_step) {
            /* Exit hint at bottom */
            LCD_PutStr(4, y, "  SEL_LONG to exit", FONT_SM, COL_WHITE, ST_BG);
        }
    }

    if (nlines > visible) {
        ui_draw_scrollbar(nlines, visible, (uint8_t)tool_scroll,
                          STATUSBAR_H + 20U, visible * row_h);
    }

    if (st_done) {
        st_cleared = 0;  /* reset for next selftest run */
    }
}

static void UI_DrawSettings(float ntc_temp, uint8_t output_on)
{
    char buf[40];
    uint8_t row_h = 20;

    UI_DrawStatusBar(output_on, ntc_temp);

    /* Pre-compute scroll positions BEFORE the full-redraw check so that
     * scroll changes are detected on the same frame they happen. */
    static uint8_t grp_scroll = 0;
    uint8_t grp_visible = 5U;
    if (grp_visible > g_menu_group_count) grp_visible = g_menu_group_count;

    if (settings_level == 0) {
        if (settings_group < grp_scroll) grp_scroll = settings_group;
        if (settings_group >= grp_scroll + grp_visible)
            grp_scroll = (uint8_t)(settings_group - grp_visible + 1U);
    } else {
        const MenuGroup *pre_grp = &g_menu_groups[settings_group];
        uint8_t pre_total = pre_grp->count + 1U;
        uint8_t pre_vis = (uint8_t)((NAVBAR_Y - CONTENT_Y) / row_h - 1U);
        if (pre_vis > 5U) pre_vis = 5U;
        if (pre_vis > pre_total) pre_vis = pre_total;
        if (settings_item < s_set_scroll) s_set_scroll = settings_item;
        if (settings_item >= s_set_scroll + pre_vis)
            s_set_scroll = (uint8_t)(settings_item - pre_vis + 1U);
    }

    /* Full content clear needed when navigating between levels, toggling
     * edit mode, or scrolling — otherwise just repaint changed rows. */
    static uint8_t s_set_prev_scroll = 0xFF;
    static uint8_t s_grp_prev_scroll = 0xFF;
    uint8_t full = (settings_level != s_set_prev_level ||
                    edit_mode != s_set_prev_edit ||
                    s_set_prev_group == 0xFF ||
                    s_set_scroll != s_set_prev_scroll ||
                    grp_scroll != s_grp_prev_scroll);

    if (full) {
        LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, COL_BG);
    }

    if (settings_level == 0) {
        /* Level 0: scrollable list of setting groups, max 5 visible */

        for (uint8_t r = 0; r < grp_visible; r++) {
            uint8_t i = grp_scroll + r;
            uint8_t sel = (i == settings_group);
            uint8_t was = (i == s_set_prev_group);
            if (!full && sel == was) continue;
            uint16_t y = CONTENT_Y + r * row_h;
            /* Highlight only once SELECT has entered the menu (edit mode) —
             * same convention as the PDO screen, where the arrow tracks the
             * cursor and the red row appears only after SELECT. */
            uint16_t bg = (sel && edit_mode) ? COL_SEL_BG : COL_BG;
            uint16_t fg = COL_WHITE;
            snprintf(buf, sizeof(buf), "%c %-20s", sel ? '>' : ' ', g_menu_groups[i].title);
            LCD_Fill(0, y, SCREEN_W - 1, y + row_h - 1, bg);
            LCD_PutStr(DRAW_X, y + TEXT_VPAD, buf, FONT_SM, fg, bg);
        }
        ui_draw_scrollbar(g_menu_group_count, grp_visible, grp_scroll,
                          CONTENT_Y, grp_visible * row_h);
    } else {
        /* Level 1: items within the selected group, plus a "Back" entry.
         * Row 0 is the group title; scrollable items start at row 1. */
        const MenuGroup *grp = &g_menu_groups[settings_group];
        uint8_t total = grp->count + 1U; /* +1 for implicit "Back" item */
        uint8_t visible = (uint8_t)((NAVBAR_Y - CONTENT_Y) / row_h - 1U);
        if (visible > 5U) visible = 5U;   /* max 5 rows to stay above navbar */
        if (visible > total) visible = total;

        /* Scroll already adjusted in pre-compute block above */

        if (full) {
            snprintf(buf, sizeof(buf), "[ %s ]                ", grp->title);
            LCD_PutStr(DRAW_X, CONTENT_Y, buf, FONT_SM, COL_YELLOW, COL_BG);
        }

        for (uint8_t r = 0; r < visible; r++) {
            uint8_t idx = s_set_scroll + r;
            uint8_t sel = (idx == settings_item);
            uint8_t was = (idx == s_set_prev_item);
            uint8_t adjusting_this = (sel && settings_adjusting_flag && idx < grp->count);
            uint8_t adj_exit = (!settings_adjusting_flag && s_set_prev_adj && sel);
            if (!full && sel == was && !adjusting_this && !adj_exit) continue;

            uint16_t y = CONTENT_Y + (r + 1U) * row_h;

            /* Color scheme:
             *   Normal:    white name on black, yellow value on black
             *   Selected:  black name on white (inverted row)
             *   Adjusting: normal name, black value on white (inverted value only) */
            uint16_t row_bg   = sel ? COL_SEL_BG : COL_BG;
            uint16_t name_fg  = COL_WHITE;
            uint16_t val_bg   = (adjusting_this) ? COL_WHITE : row_bg;
            uint16_t val_fg   = (adjusting_this) ? COL_BG : (sel ? COL_WHITE : COL_YELLOW);

            /* When adjusting, row bg is normal, only value inverted */
            if (adjusting_this) {
                row_bg  = COL_BG;
                name_fg = COL_WHITE;
            }

            /* While adjusting, skip LCD_Fill — just overdraw the value.
             * First frame (enter) still needs LCD_Fill for the bg change. */
            if (adjusting_this && s_set_prev_adj && !full) {
                if (idx < grp->count) {
                    uint16_t mi = grp->items[idx];
                    const char *val  = Menu_GetValueStr(mi);
                    snprintf(buf, sizeof(buf), "%-10s", val);
                    LCD_PutStr(220, y + TEXT_VPAD, buf, FONT_SM, val_fg, val_bg);
                }
                continue;
            }

            LCD_Fill(0, y, SCREEN_W - 1, y + row_h - 1, row_bg);

            if (idx < grp->count) {
                uint16_t mi = grp->items[idx];
                const MI_Entry *entry = Menu_FindMI(mi);
                const char *name = entry ? entry->name : "???";
                const char *val  = Menu_GetValueStr(mi);
                snprintf(buf, sizeof(buf), "%c %s", sel ? '>' : ' ', name);
                LCD_PutStr(DRAW_X, y + TEXT_VPAD, buf, FONT_SM, name_fg, row_bg);
                snprintf(buf, sizeof(buf), "%-10s", val);
                LCD_PutStr(220, y + TEXT_VPAD, buf, FONT_SM, val_fg, val_bg);
            } else {
                snprintf(buf, sizeof(buf), "%c Back", sel ? '>' : ' ');
                LCD_PutStr(DRAW_X, y + TEXT_VPAD, buf, FONT_SM, name_fg, row_bg);
            }
        }
        ui_draw_scrollbar(total, visible, s_set_scroll,
                          CONTENT_Y + row_h, visible * row_h);
    }

    s_set_prev_group = settings_group;
    s_set_prev_item  = settings_item;
    s_set_prev_level = settings_level;
    s_set_prev_edit  = edit_mode;
    s_set_prev_scroll = s_set_scroll;
    s_grp_prev_scroll = grp_scroll;
    s_set_prev_adj   = settings_adjusting_flag;

    if (flash_msg != NULL) {
        uint16_t msg_y = NAVBAR_Y - 22;
        LCD_PutStr(DRAW_X, msg_y, (char *)flash_msg, FONT_SM, COL_GREEN, COL_BG);
    }

    UI_DrawScreenIndicator();
}

/* Map the active contract's 1-based source_caps position to its row in
 * the PDO screen's display order (Fixed, then PPS, then AVS — must match
 * the three-pass sort in UI_DrawPDOs).  Returns 0 if no contract. */
static uint8_t UI_ActivePdoDisplayIndex(void)
{
    uint32_t raw[14];
    uint8_t rn = axxpd_get_src_pdos(raw, 14);
    uint8_t active = axxpd_get_active_pdo_index();  /* 1-based, 0 = none */
    if (active == 0U || active > rn) return 0U;
    uint8_t disp = 0;
    for (uint8_t pass = 0; pass < 3U; pass++) {
        for (uint8_t f = 0; f < rn; f++) {
            if (raw[f] == 0U) continue;
            uint32_t type = (raw[f] >> 30U) & 0x3U;
            uint32_t sub  = (raw[f] >> 28U) & 0x3U;
            uint8_t in_pass = (pass == 0U) ? (type == 0U)
                            : (pass == 1U) ? (type == 3U && sub == 0U)
                                           : (type == 3U && sub == 1U);
            if (!in_pass) continue;
            if (f == (uint8_t)(active - 1U)) return disp;
            disp++;
        }
    }
    return 0U;
}

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

void UI_Init(void)
{
    /* LCD already initialized in Phase 2 boot — don't reinit here,
     * it wipes the "Requesting X.XV..." transition message. */
    Graph_Init();
    current_screen = UI_SCREEN_DASHBOARD;
    pdo_cursor     = 0;
    pdo_scroll     = 0;
    preset_cursor  = 0;
    edit_mode      = 0;
}

/** Main UI draw loop — called from main loop at ~30Hz.
 *  Handles edit timeout, screen transitions, fault overlay, and delegates
 *  to the per-screen draw function. */
void UI_Update(INA228_Reading_t *reading, float ntc_temp, uint8_t output_on)
{
    /* Auto-cancel edit mode after 5s of no button input — safety net so
     * an unattended device returns to navigation mode.
     * PDO screen is excluded: the user browses PDOs at their own pace. */
    static uint8_t prev_edit_mode = 0;
    if (edit_mode && current_screen != UI_SCREEN_PDOS &&
        (HAL_GetTick() - edit_last_input_tick > EDIT_TIMEOUT_MS)) {
        edit_mode = 0;
    }
    /* Edit mode toggling changes row highlight state across all list screens,
     * so invalidate all incremental tracking to force a full repaint. */
    if (edit_mode != prev_edit_mode) {
        s_pdo_prev_cursor = 0xFF; s_pdo_prev_scroll = 0xFF; s_pdo_prev_count = 0xFF;
        s_pre_prev_cursor = 0xFF; s_pre_prev_scroll = 0xFF;
        s_set_prev_group = 0xFF; s_set_prev_adj = 0;
        prev_edit_mode = edit_mode;
    }

    /* Flash messages auto-expire after 2 seconds */
    if (flash_msg != NULL && (HAL_GetTick() - flash_msg_tick) >= 2000U) {
        flash_msg = NULL;
    }

    /* Screen transition: instant clear, no animation */
    static UIScreen_t prev_screen = UI_SCREEN_COUNT;
    if (current_screen != prev_screen) {
        LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, COL_BG);
        if (current_screen == UI_SCREEN_GRAPH) {
            Graph_InvalidateGrid();  /* graph needs its grid redrawn from scratch */
        }
        /* Invalidate all list tracking so first frame does a full draw */
        s_pdo_prev_cursor = 0xFF; s_pdo_prev_scroll = 0xFF; s_pdo_prev_count = 0xFF;
        s_pre_prev_cursor = 0xFF; s_pre_prev_scroll = 0xFF;
        s_set_prev_group = 0xFF;
        /* Start the PDO cursor on the active contract (e.g. the PDO picked
         * at the boot selector) instead of always on row 0. */
        if (current_screen == UI_SCREEN_PDOS) {
            pdo_cursor = UI_ActivePdoDisplayIndex();
        }
        /* Always enter settings at the top-level group list */
        if (current_screen == UI_SCREEN_SETTINGS) {
            settings_level = 0;
            settings_group = 0;
            settings_item  = 0;
            s_set_scroll   = 0;
            settings_adjusting_flag = 0;
            s_set_prev_adj = 0;
        }
        prev_screen = current_screen;
    }

    /* ---- Fault overlay state machine ----
     * When g_hw_fault goes 0->1, the overlay draws once and stays visible.
     * User presses SEL to set fault_acked, which hides the overlay.
     * The handler also clears g_hw_fault so the next fault is a fresh edge. */
    static uint8_t fault_overlay_active = 0;
    static uint8_t prev_hw_fault = 0;
    /* Detect rising edge of g_hw_fault to reset ack state */
    if (g_hw_fault && !prev_hw_fault) {
        fault_acked = 0;
        fault_overlay_active = 0;  /* force redraw on next frame */
    }
    prev_hw_fault = g_hw_fault;

    if (g_hw_fault && !fault_acked) {
        /* Draw the fault overlay exactly once (it's static until dismissed) */
        if (!fault_overlay_active) {
            uint16_t fault_bg = RGB(220, 50, 50);
            LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, fault_bg);
            /* Must use FONT_SM/MD here — LG/XL only contain digit glyphs */
            {
                /* Decode g_fault_source to human-readable cause */
                const char *detail;
                if (g_fault_source == FAULT_COMP_OVP)
                    detail = "OVP (COMP1)";
                else if (g_fault_source == FAULT_LTC4368)
                    detail = "OCP (LTC4368)";
                else if (g_fault_source == FAULT_INA228_OCP)
                    detail = "OCP (INA228)";
                else if (g_fault_source == FAULT_LM5166_PGOOD)
                    detail = "PGOOD (LM5166)";
                else if (g_fault_source == FAULT_TPD4S480)
                    detail = "Fault (TPD4S480)";
                else if (g_fault_source == FAULT_OPP)
                    detail = "OPP (Software)";
                else if (g_fault_source == FAULT_TIMER)
                    detail = "Timer expired";
                else if (g_fault_source == FAULT_AH_LIMIT)
                    detail = "Ah limit reached";
                else if (g_fault_source == FAULT_WH_LIMIT)
                    detail = "Wh limit reached";
                else if (g_fault_source == FAULT_CHARGE_COMPLETE)
                    detail = "Charge complete";
                else if (g_fault_source == FAULT_THERMAL)
                    detail = "Thermal shutdown";
                else
                    detail = "Unknown";
                #define FAULT_X  22
                LCD_PutStr(FAULT_X,  50, "HARDWARE FAULT", FONT_SM, COL_WHITE, fault_bg);
                LCD_PutStr(FAULT_X,  75, (char*)detail, FONT_SM, COL_WHITE, fault_bg);
                LCD_PutStr(FAULT_X, 108, "SELECT to Clear", FONT_MD, COL_WHITE, fault_bg);
            }
            fault_overlay_active = 1;
        }
        /* While fault overlay is active, skip all normal screen drawing */
    } else {
        if (fault_overlay_active) {
            /* Fault just dismissed — wipe content area and force full redraw */
            LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, COL_BG);
            prev_screen = UI_SCREEN_COUNT;  /* triggers screen-transition logic */
            fault_overlay_active = 0;
        }

        /* Tool overlay — replaces content area when a tool is active */
        if (tool_active != TOOL_NONE) {
            if (!tool_drawn) {
                switch (tool_active) {
                case TOOL_CHARGER_INFO:  Tool_DrawChargerInfo();  break;
                case TOOL_CABLE_INFO:    Tool_DrawCableInfo();    break;
                case TOOL_VOLTAGE_SWEEP: Tool_DrawVoltageSweep(); break;
                case TOOL_SELFTEST:      Tool_DrawSelftest();     break;
                default: break;
                }
                tool_drawn = 1;
            } else if (tool_active == TOOL_VOLTAGE_SWEEP && !sweep_done) {
                /* Sweep in progress: redraw at 30Hz to show progress */
                Tool_DrawVoltageSweep();
            }
        } else {
        /* Normal screen dispatch */
        switch (current_screen) {
        case UI_SCREEN_DASHBOARD:
            UI_DrawDashboard(reading, ntc_temp, output_on);
            break;
        case UI_SCREEN_GRAPH:
            UI_DrawGraph(reading, ntc_temp, output_on);
            break;
        case UI_SCREEN_PDOS:
            UI_DrawPDOs(ntc_temp, output_on);
            break;
        case UI_SCREEN_PRESETS:
            UI_DrawPresets(ntc_temp, output_on);
            break;
        case UI_SCREEN_ENERGY:
            UI_DrawEnergy(reading, ntc_temp, output_on);
            break;
        case UI_SCREEN_SETTINGS:
            UI_DrawSettings(ntc_temp, output_on);
            break;
        default:
            break;
        }
        } /* end tool_active else */

    }
}

/** Button event handler — called from button driver ISR/callback.
 *
 *  Priority chain:
 *    1. Fault overlay active -> SEL dismisses it, everything else ignored
 *    2. Edit mode active     -> INC/DEC adjust values, SEL cycles fields, SEL_LONG commits
 *    3. Navigation mode      -> INC/DEC switch screens, SEL enters edit mode
 *
 *  The PWR button is handled separately by main.c (output toggle). */
void UI_HandleButton(ButtonEvent_t event)
{
    /* Fault overlay takes absolute priority — SEL clears it, all else blocked.
     * Clearing g_hw_fault ensures the next HW fault is a fresh 0->1 edge. */
    if (g_hw_fault && event == BTN_SEL_PRESS) {
        fault_acked = 1;
        g_hw_fault = 0;
        g_fault_source = FAULT_NONE;
        INA228_ClearAlertLatch(&g_ina);
        return;
    }
    if (g_hw_fault && !fault_acked) return;  /* swallow all buttons during fault */

    /* Autostart countdown — SELECT cancels this boot's auto power-on. Checked
     * high (just below fault) so it works on any screen during the window. */
    if (Boot_AutostartPending() && event == BTN_SEL_PRESS) {
        Boot_AutostartAbort();
        flash_msg = "Autostart cancelled";
        flash_msg_tick = HAL_GetTick();
        return;
    }

    /* UI lock — reject all events except SEL_LONG (unlock) */
    if (ui_locked) {
        if (event == BTN_SEL_LONG) {
            ui_locked = 0;
            Buzzer_Confirm();
        }
        return;
    }

    /* Tool overlay — handle INC/DEC (scroll) and SEL_LONG (exit) */
    if (tool_active != TOOL_NONE) {
        switch (event) {
        case BTN_INC_PRESS: case BTN_INC_REPEAT:
            tool_scroll++;
            tool_drawn = 0;  /* force redraw */
            return;
        case BTN_DEC_PRESS: case BTN_DEC_REPEAT:
            if (tool_scroll > 0) tool_scroll--;
            tool_drawn = 0;
            return;
        case BTN_SEL_LONG:
            /* Exit tool: restore settings screen */
            tool_active = TOOL_NONE;
            tool_scroll = 0;
            tool_drawn  = 0;
            sweep_running = 0;
            st_running    = 0;
            /* Force full settings redraw */
            s_set_prev_group = 0xFF;
            LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, COL_BG);
            return;
        default:
            return;
        }
    }

    /* ================================================================
     * EDIT MODE — INC/DEC adjust values, SEL cycles fields, SEL_LONG commits
     * Each screen defines its own edit behavior below.
     * ================================================================ */
    if (edit_mode) {
        edit_last_input_tick = HAL_GetTick();  /* reset auto-cancel timer */

        /* SEL_LONG handling is per-screen below — each screen commits
         * its own data (dashboard applies voltage, settings saves, etc.)
         * and exits edit mode. No universal handler here. */

        /* --- Dashboard: voltage/current adjust ---
         * Field 0 (V coarse): PPS = 1V steps, Fixed = snap to next fixed PDO
         * Field 1 (V fine):   PPS only, 0.1V steps — skipped for fixed-only sources
         * Field 2 (I):        0.1A steps, clamped to source's max current
         * SEL_LONG: disable output, apply new V/I via PD re-negotiation */
        if (current_screen == UI_SCREEN_DASHBOARD) {
            uint32_t pps_min = 3300U, pps_max = 21000U, max_ma = 3000U;
            if (UI_HasPPS()) UI_GetPPSRange(&pps_min, &pps_max);
            { /* Find max current across all source PDOs (fixed + PPS) */
                uint32_t pdos[14]; uint8_t cnt = axxpd_get_src_pdos(pdos, 14);
                max_ma = 0;
                for (uint8_t k = 0; k < cnt; k++) {
                    if (pdos[k] == 0U) continue;
                    uint32_t t = (pdos[k] >> 30U) & 0x3U;
                    if (t == 3U && ((pdos[k] >> 28U) & 0x3U) == 0U) {
                        uint32_t ma = ((pdos[k] >> 0U) & 0x7FU) * 50U;  /* PPS: 50mA units */
                        if (ma > max_ma) max_ma = ma;
                    } else if (t == 0U) {
                        uint32_t ma = ((pdos[k] >> 0U) & 0x3FFU) * 10U; /* Fixed: 10mA units */
                        if (ma > max_ma) max_ma = ma;
                    }
                }
                if (max_ma == 0U) max_ma = 3000U;  /* safe fallback */
            }
            switch (event) {
            case BTN_INC_PRESS: case BTN_INC_REPEAT:
                if (adjust_field == 0) {
                    if (UI_HasPPS()) { if (adjust_target_mv + 1000U <= pps_max) adjust_target_mv += 1000U; }
                    else { uint32_t f[14]; uint8_t n=UI_GetFixedVoltages(f,14);
                           for(uint8_t i=0;i<n;i++){if(f[i]>adjust_target_mv){adjust_target_mv=f[i];break;}} }
                } else if (adjust_field == 1) {
                    if (adjust_target_mv + 100U <= pps_max) adjust_target_mv += 100U;
                } else {
                    if (adjust_target_ma + 100U <= max_ma) adjust_target_ma += 100U;
                }
                return;
            case BTN_DEC_PRESS: case BTN_DEC_REPEAT:
                if (adjust_field == 0) {
                    if (UI_HasPPS()) { if (adjust_target_mv >= pps_min + 1000U) adjust_target_mv -= 1000U; }
                    else { uint32_t f[14]; uint8_t n=UI_GetFixedVoltages(f,14);
                           for(int8_t i=(int8_t)(n-1);i>=0;i--){if(f[i]<adjust_target_mv){adjust_target_mv=f[i];break;}} }
                } else if (adjust_field == 1) {
                    if (adjust_target_mv >= pps_min + 100U) adjust_target_mv -= 100U;
                } else {
                    if (adjust_target_ma >= 200U) adjust_target_ma -= 100U;
                }
                return;
            case BTN_SEL_PRESS:
                /* Cycle to next field; skip fine-V if source has no PPS */
                adjust_field++;
                if (adjust_field == 1 && !UI_HasPPS()) adjust_field++;
                if (adjust_field > 2) adjust_field = 0;
                return;
            case BTN_SEL_LONG:
                /* Confirm: apply new V/I and exit edit mode. Output stays on. */
                App_SetTargetVoltage(adjust_target_mv, adjust_target_ma);
                edit_mode = 0; return;
            default: return;
            }
        }

        /* --- PDO screen: INC/DEC scroll the list, SEL_LONG requests the PDO --- */
        if (current_screen == UI_SCREEN_PDOS) {
            uint32_t tmp[14]; uint8_t cnt = axxpd_get_src_pdos(tmp, 14);
            uint8_t total = 0; for (uint8_t f=0; f<cnt; f++) { if (tmp[f]!=0U) total++; }
            switch (event) {
            case BTN_DEC_PRESS: case BTN_DEC_REPEAT:
                if (pdo_cursor > 0) pdo_cursor--;
                return;
            case BTN_INC_PRESS: case BTN_INC_REPEAT:
                if (total > 0U && pdo_cursor < (uint8_t)(total-1U)) pdo_cursor++;
                return;
            case BTN_SEL_PRESS: case BTN_SEL_LONG: {
                /* Short or long press confirms PDO selection.
                 * Find the original 1-based position of the sorted PDO
                 * in source_caps and use position-based request. */
                uint32_t raw[14], sorted[14];
                uint8_t rn = axxpd_get_src_pdos(raw, 14), sn = 0;
                for (uint8_t f=0;f<rn;f++) if (raw[f]!=0U && ((raw[f]>>30)&3)==0) sorted[sn++]=raw[f];
                for (uint8_t f=0;f<rn;f++) if (raw[f]!=0U && ((raw[f]>>30)&3)==3 && ((raw[f]>>28)&3)==0) sorted[sn++]=raw[f];
                for (uint8_t f=0;f<rn;f++) if (raw[f]!=0U && ((raw[f]>>30)&3)==3 && ((raw[f]>>28)&3)==1) sorted[sn++]=raw[f];
                if (pdo_cursor < sn) {
                    uint32_t target_pdo = sorted[pdo_cursor];
                    /* Find 1-based position in raw source_caps */
                    uint8_t pos = 0;
                    for (uint8_t f=0; f<rn; f++) {
                        if (raw[f] == target_pdo) { pos = f + 1; break; }
                    }
                    if (pos > 0) {
                        /* Show what we're negotiating */
                        uint32_t tp = target_pdo, tt = (tp>>30)&3;
                        uint32_t mv = 0;
                        if (tt == 0U) mv = ((tp>>10)&0x3FF)*50;
                        else if (tt == 3U && ((tp>>28)&3)==0) mv = ((tp>>17)&0xFF)*100;
                        else if (tt == 3U && ((tp>>28)&3)==1) mv = ((tp>>17)&0x1FF)*100;
                        char msg[32];
                        snprintf(msg, sizeof(msg), "Requesting %lu.%luV...",
                                 (unsigned long)(mv/1000), (unsigned long)((mv%1000)/100));
                        LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, COL_BG);
                        LCD_PutStr(70, 80, msg, FONT_MD, COL_YELLOW, COL_BG);
                        /* Persist this selection for "Restore last V/I". For a
                         * fixed PDO the advertised current is bits[9:0]*10mA;
                         * APDOs request max (0). */
                        if (mv >= 3300U) {
                            uint32_t sel_ma = (tt == 0U) ? (((tp >> 0) & 0x3FFU) * 10U) : 0U;
                            Settings_SaveLastSettings(mv, sel_ma);
                        }
                        axxpd_request_pdo_position(pos);
                    }
                }
                edit_mode = 0;
                current_screen = UI_SCREEN_DASHBOARD;
                return;
            }
            default: return;
            }
        }

        /* --- Presets screen ---
         * SEL short: apply the preset (re-negotiate PD)
         * SEL_LONG on empty slot:  save current negotiated V/A as new preset
         * SEL_LONG on filled slot: delete the preset */
        if (current_screen == UI_SCREEN_PRESETS) {
            switch (event) {
            case BTN_DEC_PRESS: case BTN_DEC_REPEAT:
                if (preset_cursor > 0) preset_cursor--;
                return;
            case BTN_INC_PRESS: case BTN_INC_REPEAT:
                if (preset_cursor < PRESET_SLOTS - 1) preset_cursor++;
                return;
            case BTN_SEL_PRESS:
                if (!Settings_PresetIsEmpty(preset_cursor)) {
                    Preset_t *p = Settings_GetPreset(preset_cursor);
                    g_output_enabled = 0; Output_Disable();
                    App_SetTargetVoltage(p->voltage_mv, p->current_ma);
                }
                return;
            case BTN_SEL_LONG:
                if (Settings_PresetIsEmpty(preset_cursor)) {
                    /* Empty slot: snapshot current negotiated V/A into preset */
                    float v = axxpd_get_negotiated_v();
                    float a = axxpd_get_negotiated_a();
                    uint32_t mv = (uint32_t)(v * 1000.0f);
                    uint32_t ma = (uint32_t)(a * 1000.0f);
                    if (mv < 3300U || g_hw_fault) {
                        Buzzer_Fault();
                        return;
                    }
                    char name[8];
                    snprintf(name, sizeof(name), "%luV/%luA",
                             (unsigned long)(mv / 1000U),
                             (unsigned long)(ma / 1000U));
                    Settings_PresetSet(preset_cursor, mv, ma, name);
                } else {
                    Settings_PresetDelete(preset_cursor);
                }
                s_pre_prev_cursor = 0xFF;  /* force full redraw to reflect change */
                edit_mode = 0;
                return;
            default: return;
            }
        }

        /* --- Energy screen: no INC/DEC editing, fall through to navigation.
         * SEL_LONG resets accumulators without needing edit mode. */
        if (current_screen == UI_SCREEN_ENERGY) {
            if (event == BTN_SEL_LONG) {
                if (g_ina.hi2c != NULL) {
                    INA228_ResetEnergy(&g_ina);
                }
            }
            edit_mode = 0;  /* exit edit mode, let INC/DEC navigate */
        }

        /* --- Settings screen: 2-level group/item navigation ---
         * Level 0: SEL enters the highlighted group
         * Level 1: SEL toggles bools or triggers actions, SEL_LONG backs out */
        if (current_screen == UI_SCREEN_SETTINGS) {
            if (settings_level == 0) {
                switch (event) {
                case BTN_DEC_PRESS: case BTN_DEC_REPEAT:
                    if (settings_group > 0) settings_group--;
                    return;
                case BTN_INC_PRESS: case BTN_INC_REPEAT:
                    if (settings_group < g_menu_group_count - 1U) settings_group++;
                    return;
                case BTN_SEL_PRESS:
                    settings_level = 1;
                    settings_item = 0;
                    s_set_scroll = 0;
                    return;
                case BTN_SEL_LONG:
                    edit_mode = 0; return;
                default: return;
                }
            } else {
                /* Level 1: individual settings + "Back"
                 * settings_adjusting_flag: 0 = browsing items, 1 = adjusting a numeric value.
                 * While adjusting, INC/DEC change the value; SEL exits adjust mode. */
                /* uses file-scope settings_adjusting_flag */
                const MenuGroup *grp = &g_menu_groups[settings_group];
                uint8_t total = grp->count + 1U;
                uint16_t cur_mi = (settings_item < grp->count) ? grp->items[settings_item] : 0xFFFFU;

                if (settings_adjusting_flag) {
                    /* Adjusting a numeric value */
                    switch (event) {
                    case BTN_DEC_PRESS: case BTN_DEC_REPEAT:
                        Menu_AdjustNumeric(cur_mi, -1);
                        if (cur_mi == MI_OCP_LIMIT)
                            INA228_SetAlertOverCurrent(&g_ina, (float)Settings_GetOcpMa() / 1000.0f);
                        if (cur_mi == MI_OVP_LIMIT)
                            OVP_SetThreshold(Settings_GetOvpMv());
                        return;
                    case BTN_INC_PRESS: case BTN_INC_REPEAT:
                        Menu_AdjustNumeric(cur_mi, +1);
                        if (cur_mi == MI_OCP_LIMIT)
                            INA228_SetAlertOverCurrent(&g_ina, (float)Settings_GetOcpMa() / 1000.0f);
                        if (cur_mi == MI_OVP_LIMIT)
                            OVP_SetThreshold(Settings_GetOvpMv());
                        return;
                    case BTN_SEL_PRESS: case BTN_SEL_LONG:
                        settings_adjusting_flag = 0; return;
                    default: return;
                    }
                }

                /* Browsing items */
                switch (event) {
                case BTN_DEC_PRESS: case BTN_DEC_REPEAT:
                    if (settings_item > 0) settings_item--;
                    return;
                case BTN_INC_PRESS: case BTN_INC_REPEAT:
                    if (settings_item < total - 1U) settings_item++;
                    return;
                case BTN_SEL_PRESS:
                    if (settings_item >= grp->count) {
                        /* "Back" item — return to group list */
                        settings_level = 0;
                        settings_adjusting_flag = 0;
                    } else {
                        uint16_t mi = grp->items[settings_item];
                        /* Numeric items: SEL enters adjust mode */
                        if (Menu_IsNumeric(mi)) {
                            settings_adjusting_flag = 1;
                            return;
                        }
                        if (mi == MI_LOAD_DEFAULT) {
                            Settings_LoadDefaults();
                            flash_msg = "Defaults loaded";
                            flash_msg_tick = HAL_GetTick();
                            s_set_prev_group = 0xFF;  /* values changed en masse — force redraw */
                        } else if (mi == MI_SAVE_REBOOT) {
                            Settings_SaveImmediate();  /* blocking — deferred save would be lost on reset */
                            NVIC_SystemReset();
                        } else if (mi == MI_EXIT_NO_SAVE) {
                            /* Just exit. Settings already auto-save on change, so
                             * there is nothing to discard — and reloading flash
                             * here (old Settings_Init) reverted any change still
                             * deferred under EPR (e.g. the graph window), making
                             * it look like the setting "didn't work". */
                            edit_mode = 0;
                            settings_level = 0;
                            current_screen = UI_SCREEN_DASHBOARD;
                        } else if (mi == MI_TOOL_CHARGER_INFO) {
                            tool_active = TOOL_CHARGER_INFO;
                            tool_scroll = 0;
                            tool_drawn  = 0;
                        } else if (mi == MI_TOOL_VOLTAGE_SWEEP) {
                            tool_active   = TOOL_VOLTAGE_SWEEP;
                            tool_scroll   = 0;
                            tool_drawn    = 0;
                            sweep_done    = 0;
                            sweep_running = 1;
                            sweep_step    = 0;
                            sweep_count   = 0;
                            sweep_step_tick = HAL_GetTick();
                        } else if (mi == MI_TOOL_CABLE_INFO) {
                            tool_active = TOOL_CABLE_INFO;
                            tool_scroll = 0;
                            tool_drawn  = 0;
                        } else if (mi == MI_TOOL_SELFTEST) {
                            tool_active   = TOOL_SELFTEST;
                            tool_scroll   = 0;
                            tool_drawn    = 0;
                            st_done       = 0;
                            st_running    = 1;
                            st_step       = 0;
                            st_count      = 0;
                            st_pdo_count  = 0;
                        } else if (Menu_IsNumeric(mi)) {
                            /* Numeric items: SEL cycles forward (same as INC) */
                            Menu_AdjustNumeric(mi, +1);
                        } else {
                            /* Bool toggle — force a settings redraw so the new
                             * YES/NO (or C/F) value shows immediately. Selection
                             * didn't change, so the incremental row-redraw path
                             * would otherwise skip this row until the next scroll. */
                            Menu_ToggleBool(mi);
                            s_set_prev_group = 0xFF;
                        }
                    }
                    return;
                case BTN_SEL_LONG:
                    settings_level = 0;
                    settings_adjusting_flag = 0;
                    return;
                default: return;
                }
            }
        }

        return;
    }

    /* ================================================================
     * NAVIGATION MODE — INC/DEC cycle through the 6 screens (wrapping),
     * SEL enters edit mode for the current screen.
     * ================================================================ */
    switch (event) {
    case BTN_DEC_PRESS:
        if ((uint8_t)current_screen > 0)
            current_screen = (UIScreen_t)((uint8_t)current_screen - 1U);
        else
            current_screen = (UIScreen_t)((uint8_t)UI_SCREEN_COUNT - 1U);
        break;

    case BTN_INC_PRESS:
        current_screen = (UIScreen_t)(((uint8_t)current_screen + 1U) % (uint8_t)UI_SCREEN_COUNT);
        break;

    case BTN_SEL_PRESS:
        /* PDO screen: SEL enters edit mode, then SEL again confirms selection */
        if (current_screen == UI_SCREEN_PDOS && edit_mode) {
            uint32_t raw[14], sorted[14];
            uint8_t rn = axxpd_get_src_pdos(raw, 14), sn = 0;
            for (uint8_t f=0;f<rn;f++) if (raw[f]!=0U && ((raw[f]>>30)&3)==0) sorted[sn++]=raw[f];
            for (uint8_t f=0;f<rn;f++) if (raw[f]!=0U && ((raw[f]>>30)&3)==3 && ((raw[f]>>28)&3)==0) sorted[sn++]=raw[f];
            for (uint8_t f=0;f<rn;f++) if (raw[f]!=0U && ((raw[f]>>30)&3)==3 && ((raw[f]>>28)&3)==1) sorted[sn++]=raw[f];
            if (pdo_cursor < sn) {
                uint32_t target_pdo = sorted[pdo_cursor];
                uint8_t pos = 0;
                for (uint8_t f=0; f<rn; f++) {
                    if (raw[f] == target_pdo) { pos = f + 1; break; }
                }
                if (pos > 0) {
                    uint32_t tp = target_pdo, tt = (tp>>30)&3;
                    uint32_t mv = 0;
                    if (tt == 0U) mv = ((tp>>10)&0x3FF)*50;
                    else if (tt == 3U && ((tp>>28)&3)==0) mv = ((tp>>17)&0xFF)*100;
                    else if (tt == 3U && ((tp>>28)&3)==1) mv = ((tp>>17)&0x1FF)*100;
                    char msg[32];
                    snprintf(msg, sizeof(msg), "Requesting %lu.%luV...",
                             (unsigned long)(mv/1000), (unsigned long)((mv%1000)/100));
                    LCD_Fill(0, STATUSBAR_H, SCREEN_W - 1, NAVBAR_Y - 1, COL_BG);
                    LCD_PutStr(70, 80, msg, FONT_MD, COL_YELLOW, COL_BG);
                    axxpd_request_pdo_position(pos);
                }
            }
            current_screen = UI_SCREEN_DASHBOARD;
            break;
        }
        /* Only enter edit mode on screens that have edit handlers.
         * Graph/Energy/PDO screens don't need edit mode. */
        if (current_screen == UI_SCREEN_GRAPH ||
            current_screen == UI_SCREEN_ENERGY) break;
        edit_mode = 1;
        edit_last_input_tick = HAL_GetTick();
        if (current_screen == UI_SCREEN_DASHBOARD) {
            /* Seed adjust targets from current negotiated contract.
             * Fallback to 5V/3A if no contract yet (e.g. no charger). */
            adjust_field = 0;
            adjust_target_mv = (uint32_t)(axxpd_get_negotiated_v() * 1000.0f);
            adjust_target_ma = (uint32_t)(axxpd_get_negotiated_a() * 1000.0f);
            if (adjust_target_mv < 3300U) adjust_target_mv = 5000U;
            if (adjust_target_ma == 0U) adjust_target_ma = 3000U;
        }
        if (current_screen == UI_SCREEN_SETTINGS) {
            /* Always start at top of group list */
            settings_level = 0;
            settings_group = 0;
            settings_item  = 0;
            s_set_prev_group = 0xFF;
            /* Turn off output when entering settings menu to prevent
             * accidental voltage changes while adjusting protection
             * thresholds or running tools. */
            if (g_output_enabled) {
                extern volatile uint8_t g_output_enabled;
                g_output_enabled = 0;
                Output_Disable();
            }
        }
        break;

    case BTN_SEL_LONG:
        if (!edit_mode) {
            ui_locked = 1;
            Buzzer_Confirm();
        }
        break;

    default:
        break;
    }
}

UIScreen_t UI_GetScreen(void) { return current_screen; }
void UI_SetLocked(uint8_t locked) { ui_locked = locked; }
uint8_t UI_IsLocked(void) { return ui_locked; }

/** Returns 1 if the UI wants to consume PWR_SHORT instead of main.c.
 *  Currently always 0 — PWR button is dedicated to output on/off toggle. */
uint8_t UI_WantsPwrShort(void)
{
    return 0;
}

/** Voltage sweep state machine tick — call from main loop every iteration.
 *  Implements the sweep by stepping through all source fixed PDOs, requesting
 *  each voltage, waiting 2s, then recording the INA228 measurement.
 *  axxpd_run() and IWDG refresh are the caller's responsibility (main loop). */
static void UI_SelftestTick(INA228_Reading_t *reading);

void UI_ToolTick(INA228_Reading_t *reading)
{
    if (tool_active == TOOL_SELFTEST && st_running && !st_done) {
        UI_SelftestTick(reading);
        return;
    }
    if (tool_active != TOOL_VOLTAGE_SWEEP || !sweep_running || sweep_done) return;

    /* Build sorted PDO list (Fixed only for sweep — variable voltages) */
    static uint32_t sweep_pdos[14];
    static uint8_t  sweep_pdo_count = 0;

    if (sweep_step == 0 && sweep_count == 0) {
        /* First call: collect all fixed PDOs */
        uint32_t raw[14];
        uint8_t cnt = axxpd_get_src_pdos(raw, 14);
        sweep_pdo_count = 0;
        for (uint8_t i = 0; i < cnt && sweep_pdo_count < 14U; i++) {
            if (raw[i] != 0U && ((raw[i] >> 30U) & 0x3U) == 0U) {
                sweep_pdos[sweep_pdo_count++] = raw[i];
            }
        }
        if (sweep_pdo_count == 0U) {
            sweep_done = 1;
            sweep_running = 0;
            return;
        }
        /* Request first PDO */
        uint32_t pdo = sweep_pdos[0];
        uint32_t mv = ((pdo >> 10U) & 0x3FFU) * 50U;
        uint32_t ma = ((pdo >>  0U) & 0x3FFU) * 10U;
        axxpd_request_voltage(mv, ma);
        sweep_results[0].req_mv = (uint16_t)mv;
        sweep_results[0].meas_mv = 0;
        sweep_results[0].meas_ma = 0;
        sweep_results[0].pass = 0;
        sweep_step_tick = HAL_GetTick();
        tool_drawn = 0;  /* trigger redraw of "Sweeping..." */
        return;
    }

    /* Wait 2 seconds at each PDO */
    if ((HAL_GetTick() - sweep_step_tick) < 2000U) return;

    /* Record measurement for current step */
    if (reading != NULL && sweep_step < sweep_pdo_count) {
        uint16_t meas_mv = (uint16_t)(reading->voltage_v * 1000.0f);
        uint16_t meas_ma = (uint16_t)(reading->current_a * 1000.0f);
        uint16_t req_mv  = sweep_results[sweep_step].req_mv;
        uint8_t  pass    = 0;
        /* Pass if measured V is within 10% of requested, or requested is 0 */
        if (req_mv > 0U) {
            uint32_t lo = (uint32_t)req_mv * 90U / 100U;
            uint32_t hi = (uint32_t)req_mv * 110U / 100U;
            pass = (meas_mv >= (uint16_t)lo && meas_mv <= (uint16_t)hi) ? 1U : 0U;
        }
        sweep_results[sweep_step].meas_mv = meas_mv;
        sweep_results[sweep_step].meas_ma = meas_ma;
        sweep_results[sweep_step].pass    = pass;
        sweep_count = sweep_step + 1U;
    }

    sweep_step++;

    if (sweep_step >= sweep_pdo_count) {
        /* Sweep complete — return to first PDO (5V) */
        if (sweep_pdo_count > 0U) {
            uint32_t pdo = sweep_pdos[0];
            uint32_t mv = ((pdo >> 10U) & 0x3FFU) * 50U;
            uint32_t ma = ((pdo >>  0U) & 0x3FFU) * 10U;
            axxpd_request_voltage(mv, ma);
        }
        sweep_done    = 1;
        sweep_running = 0;
        tool_drawn    = 0;  /* trigger redraw of results */
        return;
    }

    /* Request next PDO */
    uint32_t pdo = sweep_pdos[sweep_step];
    uint32_t mv  = ((pdo >> 10U) & 0x3FFU) * 50U;
    uint32_t ma  = ((pdo >>  0U) & 0x3FFU) * 10U;
    axxpd_request_voltage(mv, ma);
    sweep_results[sweep_step].req_mv  = (uint16_t)mv;
    sweep_results[sweep_step].meas_mv = 0;
    sweep_results[sweep_step].meas_ma = 0;
    sweep_results[sweep_step].pass    = 0;
    sweep_step_tick = HAL_GetTick();
    tool_drawn = 0;  /* redraw progress display */
}

/* ------------------------------------------------------------------ */
/*  Self-test tick — walks all PDOs (Fixed + PPS mid + AVS mid)        */
/* ------------------------------------------------------------------ */
static void st_add_step(const char *label, uint16_t req_mv)
{
    if (st_count >= ST_MAX_STEPS) return;
    snprintf(st_results[st_count].label, 26, "%s", label);
    st_results[st_count].req_mv = req_mv;
    st_results[st_count].meas_mv = 0;
    st_results[st_count].pass = 0;
    st_count++;
}

/* Simple LFSR pseudo-random for random voltage steps (no stdlib needed) */
static uint32_t st_rng_state = 0;
static uint32_t st_rng_next(void)
{
    /* xorshift32 */
    uint32_t x = st_rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    st_rng_state = x;
    return x;
}

/* Generate a random voltage (in mV) within a PDO's range, respecting step size */
static uint16_t st_random_voltage(uint32_t pdo)
{
    uint32_t type = (pdo >> 30U) & 0x3U;
    if (type == 0U) {
        /* Fixed — just return the fixed voltage */
        return (uint16_t)(((pdo >> 10U) & 0x3FFU) * 50U);
    }
    uint32_t sub = (pdo >> 28U) & 0x3U;
    if (type == 3U && sub == 0U) {
        /* PPS — 20 mV steps */
        uint32_t vmin = ((pdo >> 8U) & 0xFFU) * 100U;
        uint32_t vmax = ((pdo >> 17U) & 0xFFU) * 100U;
        if (vmax <= vmin) return (uint16_t)vmin;
        uint32_t steps = (vmax - vmin) / 20U;
        uint32_t pick = st_rng_next() % (steps + 1U);
        return (uint16_t)(vmin + pick * 20U);
    }
    if (type == 3U && sub == 1U) {
        /* EPR AVS — 100 mV steps */
        uint32_t vmin = ((pdo >> 8U) & 0xFFU) * 100U;
        uint32_t vmax = ((pdo >> 17U) & 0x1FFU) * 100U;
        if (vmax <= vmin) return (uint16_t)vmin;
        uint32_t steps = (vmax - vmin) / 100U;
        uint32_t pick = st_rng_next() % (steps + 1U);
        return (uint16_t)(vmin + pick * 100U);
    }
    return 5000U;
}

static void UI_SelftestTick(INA228_Reading_t *reading)
{
    if (st_step == 0 && st_count == 0) {
        /* First call: build the test plan from source_caps.
         * Fixed: one step per PDO. PPS/AVS: min, mid, max.
         * Then add ST_RANDOM_STEPS random voltage steps. */
        uint32_t raw[14];
        uint8_t cnt = axxpd_get_src_pdos(raw, 14);
        st_count = 0;

        /* Collect non-zero PDOs for random step generation */
        uint32_t avail_pdos[14];
        uint8_t  avail_cnt = 0;

        for (uint8_t i = 0; i < cnt; i++) {
            uint32_t pdo = raw[i];
            if (pdo == 0U) continue;
            uint32_t type = (pdo >> 30U) & 0x3U;
            uint8_t  pos  = (uint8_t)(i + 1U);
            char lbl[26];

            avail_pdos[avail_cnt++] = pdo;

            if (type == 0U) {
                uint32_t mv = ((pdo >> 10U) & 0x3FFU) * 50U;
                snprintf(lbl, 26, "P%u %lu.%luV Fix",
                         (unsigned)pos, (unsigned long)(mv/1000U),
                         (unsigned long)((mv%1000U)/100U));
                st_add_step(lbl, (uint16_t)mv);
            } else if (type == 3U) {
                uint32_t sub = (pdo >> 28U) & 0x3U;
                if (sub == 0U) {
                    /* PPS — min, mid, max */
                    uint32_t vmin = ((pdo >> 8U) & 0xFFU) * 100U;
                    uint32_t vmax = ((pdo >> 17U) & 0xFFU) * 100U;
                    uint32_t vmid = ((vmin + vmax) / 2U / 20U) * 20U;
                    snprintf(lbl, 26, "P%u %lu.%luV PPS min", (unsigned)pos,
                             (unsigned long)(vmin/1000U), (unsigned long)((vmin%1000U)/100U));
                    st_add_step(lbl, (uint16_t)vmin);
                    snprintf(lbl, 26, "P%u %lu.%luV PPS mid", (unsigned)pos,
                             (unsigned long)(vmid/1000U), (unsigned long)((vmid%1000U)/100U));
                    st_add_step(lbl, (uint16_t)vmid);
                    snprintf(lbl, 26, "P%u %lu.%luV PPS max", (unsigned)pos,
                             (unsigned long)(vmax/1000U), (unsigned long)((vmax%1000U)/100U));
                    st_add_step(lbl, (uint16_t)vmax);
                } else if (sub == 1U) {
                    /* EPR AVS — min, mid, max (100mV steps) */
                    uint32_t vmin = ((pdo >> 8U) & 0xFFU) * 100U;
                    uint32_t vmax = ((pdo >> 17U) & 0x1FFU) * 100U;
                    uint32_t vmid = ((vmin + vmax) / 2U / 100U) * 100U;
                    snprintf(lbl, 26, "P%u %lu.%luV AVS min", (unsigned)pos,
                             (unsigned long)(vmin/1000U), (unsigned long)((vmin%1000U)/100U));
                    st_add_step(lbl, (uint16_t)vmin);
                    snprintf(lbl, 26, "P%u %lu.%luV AVS mid", (unsigned)pos,
                             (unsigned long)(vmid/1000U), (unsigned long)((vmid%1000U)/100U));
                    st_add_step(lbl, (uint16_t)vmid);
                    snprintf(lbl, 26, "P%u %lu.%luV AVS max", (unsigned)pos,
                             (unsigned long)(vmax/1000U), (unsigned long)((vmax%1000U)/100U));
                    st_add_step(lbl, (uint16_t)vmax);
                }
            }
        }

        /* Add random voltage steps */
        if (avail_cnt > 0U) {
            st_rng_state = HAL_GetTick() ^ 0xDEADBEEFU;
            for (uint8_t r = 0; r < ST_RANDOM_STEPS && st_count < ST_MAX_STEPS; r++) {
                uint32_t pick_pdo = avail_pdos[st_rng_next() % avail_cnt];
                uint16_t mv = st_random_voltage(pick_pdo);
                char lbl[26];
                snprintf(lbl, 26, "Rnd %lu.%luV",
                         (unsigned long)(mv/1000U), (unsigned long)((mv%1000U)/100U));
                st_add_step(lbl, mv);
            }
        }

        if (st_count == 0U) {
            st_done = 1;
            st_running = 0;
            tool_drawn = 0;
            return;
        }

        /* Enable output so INA228 can measure actual VBUS */
        Output_Enable();

        /* Request first voltage */
        axxpd_request_voltage(st_results[0].req_mv, 0);
        st_step_tick = HAL_GetTick();
        tool_drawn = 0;
        return;
    }

    /* Wait 2.5 seconds at each step for voltage to settle */
    if ((HAL_GetTick() - st_step_tick) < 2500U) return;

    /* Record measurement — use actual INA228 bus voltage reading */
    if (st_step < st_count) {
        extern INA228_Reading_t g_ina_reading;
        uint16_t meas_mv = (uint16_t)(g_ina_reading.voltage_v * 1000.0f + 0.5f);
        st_results[st_step].meas_mv = meas_mv;
        uint16_t req_mv = st_results[st_step].req_mv;
        if (req_mv > 0U) {
            uint32_t lo = (uint32_t)req_mv * 85U / 100U;
            uint32_t hi = (uint32_t)req_mv * 115U / 100U;
            st_results[st_step].pass = (meas_mv >= (uint16_t)lo && meas_mv <= (uint16_t)hi) ? 1U : 2U;
        } else {
            st_results[st_step].pass = 1U;
        }
    }

    st_step++;
    tool_drawn = 0;  /* trigger redraw so new result appears immediately */

    if (st_step >= st_count) {
        /* Done — disable output and return to minimum PDO */
        Output_Disable();
        axxpd_request_voltage(5000, 0);
        st_done    = 1;
        st_running = 0;
        Buzzer_Confirm();
        return;
    }

    /* Request next voltage */
    axxpd_request_voltage(st_results[st_step].req_mv, 0);
    st_step_tick = HAL_GetTick();
}
