/**
 * @file    graph.c
 * @brief   Live V/I plot for the GRAPH UI screen.
 *
 * Single grid, two Y-axes: V (left, yellow), I (right, orange). 100-sample
 * rolling window with configurable sample interval: 50/100/200 ms for
 * 5s/10s/20s history (set via Settings graph_window). V axis snaps to nice
 * round maxima derived from the negotiated PD source voltage; I axis max
 * comes from the negotiated current so traces use the full plot height.
 *
 * Drawing strategy — why column-fill, not Bresenham:
 *   The display is driven over SPI via LCD_Fill (rectangle DMA). Bresenham
 *   plots individual pixels, each needing its own SPI transaction — a
 *   100-point trace produces 30+ tiny writes per segment. Instead, we sweep
 *   left-to-right one X column at a time and fill the Y span of that column
 *   in a single LCD_Fill call, cutting SPI overhead by ~10x.
 *
 * Partial-update strategy (avoids full-screen redraws at 10 Hz):
 *   - Cold start / screen entry / axis rescale → full plot redraw.
 *   - Steady state → for each of the 99 line segments, compare new endpoint
 *     coords against the cached previous-frame coords (v_lines_prev / i_lines_prev).
 *     If unchanged (v_changed==0 && i_changed==0), skip entirely. Otherwise:
 *       1. Erase the old segment by overdrawing it in COL_BG.
 *       2. Restore any grid dash pixels the erase clobbered.
 *       3. Draw the new segment on top.
 *     This is the partial-erase + restore-grid pattern from the original
 *     AxxSolder graph.c, ported from UG_FillFrame to LCD_Fill.
 */

#include "graph.h"
#include "lcd.h"
#include "axxpd_main.h"
#include "settings.h"
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/*  Geometry — see docs/superpowers/specs/2026-04-15-vi-graph-screen-design.md
 *  for the screen layout and gutter math derivation.                  */
/* ------------------------------------------------------------------ */

#define GRAPH_X0        45    /* left edge of plot area (V-axis labels at x=15) */
#define GRAPH_Y0        120   /* bottom edge of plot area (time labels below, navbar at 142) */
#define GRAPH_WIDTH     220   /* plot width in px; right edge at x=265 */
#define GRAPH_HEIGHT    54    /* plot height in px; top edge at y=66, clears numerics row */
#define GRAPH_POINTS    100   /* rolling buffer length; window = 100 * interval_ms */

#define I_AXIS_LABEL_X  285   /* right of graph + margin */

#define GRID_DIVS       4     /* 4 segments = 5 horizontal gridlines */
#define DASH_LEN        3
#define DASH_GAP        4
#define LINE_THICKNESS  2

/* ------------------------------------------------------------------ */
/*  Colors (RGB565)                                                    */
/* ------------------------------------------------------------------ */

#define COL_BG          RGB(0,   0,   0  )
#define COL_GRID        LEGACY(48,  48,  48)
#define COL_AXIS_TIME   LEGACY(160, 160, 160)
#define COL_V_TRACE     RGB(255, 234, 0)           /* yellow — matches dashboard V */
#define COL_I_TRACE     RGB(240, 64,  5)           /* orange — matches dashboard A */
#define COL_V_LABEL     COL_V_TRACE
#define COL_I_LABEL     COL_I_TRACE

/* ------------------------------------------------------------------ */
/*  Fonts                                                              */
/* ------------------------------------------------------------------ */

extern UG_FONT FONT_arial_17X18[];
#define FONT_SM         FONT_arial_17X18   /* 17 px wide, 18 px tall */

/* ------------------------------------------------------------------ */
/*  Negotiated voltage — read via pdsink accessor for V-axis snapping  */
/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */

/** Cached endpoints of a single line segment (one inter-sample connection). */
typedef struct {
    int16_t x1, y1, x2, y2;
} Line_t;

/* --- Rolling sample buffer ---
 * Circular buffer of GRAPH_POINTS samples. sample_index is the write cursor
 * (next slot to overwrite); sample_count rises from 0 to GRAPH_POINTS and
 * then stays there. The oldest visible sample is at
 * (sample_index - sample_count) mod GRAPH_POINTS. */
static uint16_t v_samples_mv[GRAPH_POINTS];
static uint16_t i_samples_ma[GRAPH_POINTS];
static volatile uint8_t  sample_index;       /* next slot to write (head of ring) */
static volatile uint8_t  sample_count;       /* valid samples so far, clamps at GRAPH_POINTS */

/* --- Per-segment line cache ---
 * Stores the screen-space endpoints drawn last frame for each of the 99
 * inter-sample segments. Used by the partial-update loop: if the new
 * endpoints match the cached ones, the segment is skipped entirely. */
static Line_t   v_lines_prev[GRAPH_POINTS - 1];
static Line_t   i_lines_prev[GRAPH_POINTS - 1];

static uint8_t  v_lines_valid;      /* 0 = entire cache is empty (forces draw of all segments) */
static uint8_t  grid_valid;         /* 0 = grid needs full redraw (set by Graph_InvalidateGrid) */
static volatile uint16_t v_max_mv;           /* current V-axis ceiling in mV (from negotiated PD voltage) */
static volatile uint16_t i_max_ma;           /* current I-axis ceiling in mA (from negotiated PD current) */
static uint16_t last_v_max_mv;      /* v_max_mv used for the last full redraw — mismatch triggers rescale */
static uint16_t last_i_max_ma;      /* i_max_ma used for the last full redraw — mismatch triggers rescale */

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */

/** Map a voltage sample (mV) to its Y pixel in the plot.
 *  0 mV → GRAPH_Y0 (bottom edge), v_max_mv → GRAPH_Y0 - GRAPH_HEIGHT (top). */
static int16_t v_to_y(uint16_t mv)
{
    if (v_max_mv == 0U) return (int16_t)GRAPH_Y0;
    uint32_t clamped = (mv > v_max_mv) ? v_max_mv : mv;
    int dy = (int)((clamped * (uint32_t)GRAPH_HEIGHT) / v_max_mv);
    return (int16_t)(GRAPH_Y0 - dy);   /* screen Y grows downward */
}

/** Map a current sample (mA) to its Y pixel in the plot.
 *  Same mapping as v_to_y but against i_max_ma — both traces share the
 *  same pixel height so they overlay directly on the same grid. */
static int16_t i_to_y(uint16_t ma)
{
    if (i_max_ma == 0U) return (int16_t)GRAPH_Y0;
    uint32_t clamped = (ma > i_max_ma) ? i_max_ma : ma;
    int dy = (int)((clamped * (uint32_t)GRAPH_HEIGHT) / i_max_ma);
    return (int16_t)(GRAPH_Y0 - dy);
}

/** Map a display-order slot (0 = oldest/leftmost, GRAPH_POINTS-1 = newest/rightmost)
 *  to its screen X coordinate. Evenly distributes 100 points across GRAPH_WIDTH. */
static int16_t slot_to_x(int i)
{
    return (int16_t)(GRAPH_X0 + (i * GRAPH_WIDTH) / (GRAPH_POINTS - 1));
}

/* Clamp helper */
static inline int clamp(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

/* Safe LCD_Fill wrapper — clamps all coordinates to the 320x172 screen.
 * LCD_Fill takes uint16_t args; passing a negative int (e.g. from line math
 * that overshoots the plot boundary) would wrap to ~65 000 and cause a
 * HardFault or corrupt the framebuffer. This wrapper makes every draw call
 * from the line renderer inherently safe without per-caller bounds checks. */
static void safe_fill(int x1, int y1, int x2, int y2, uint16_t color)
{
    x1 = clamp(x1, 0, 319); x2 = clamp(x2, 0, 319);
    y1 = clamp(y1, 0, 171); y2 = clamp(y2, 0, 171);
    if (x1 > x2 || y1 > y2) return;    /* degenerate after clamp — nothing to draw */
    LCD_Fill((uint16_t)x1, (uint16_t)y1, (uint16_t)x2, (uint16_t)y2, color);
}

/* Column-based thick line draw.
 *
 * Why not Bresenham? Bresenham plots one pixel at a time, and each pixel
 * becomes its own SPI DMA transfer via LCD_Fill(x,y,x,y,...). A typical
 * inter-sample segment spans ~2 px horizontally but many vertically,
 * producing 30+ tiny SPI transactions. Column-fill instead walks X columns
 * left-to-right, computing the min/max Y the line occupies in that column,
 * and issues ONE safe_fill per column — far fewer SPI calls and no visible
 * difference at 2 px line thickness.
 *
 * Thickness is applied by extending each column rectangle by (t-1) pixels
 * in both X and Y, giving a roughly square brush. */
static void draw_line_thick(int x1, int y1, int x2, int y2, uint16_t color, uint8_t thickness)
{
    uint8_t t = (thickness == 0U) ? 1U : thickness;

    /* Normalise so x1 <= x2 — simplifies the left-to-right column sweep. */
    if (x1 > x2) { int tmp; tmp=x1; x1=x2; x2=tmp; tmp=y1; y1=y2; y2=tmp; }

    int dx = x2 - x1;
    if (dx == 0) {
        /* Vertical segment — single rectangle covers the whole line. */
        int ya = (y1 < y2) ? y1 : y2;
        int yb = (y1 > y2) ? y1 : y2;
        safe_fill(x1, ya, x1 + t - 1, yb + t - 1, color);
        return;
    }

    /* General case: for each X column, interpolate the Y at entry (ya) and
     * exit (yb) of that column, then fill the rectangle [x, ymin] → [x+t-1, ymax+t-1]. */
    for (int x = x1; x <= x2; x++) {
        int ya = y1 + (y2 - y1) * (x - x1) / dx;
        int yb = (x == x2) ? y2 : y1 + (y2 - y1) * (x + 1 - x1) / dx;
        int ymin = (ya < yb) ? ya : yb;
        int ymax = (ya > yb) ? ya : yb;
        safe_fill(x, ymin, x + t - 1, ymax + t - 1, color);
    }
}

/* Dashed horizontal line (y constant). */
static void draw_dashed_h(int x1, int x2, int y, uint16_t color, uint8_t dash, uint8_t gap)
{
    int x = x1;
    while (x <= x2) {
        int seg_end = x + dash - 1;
        if (seg_end > x2) seg_end = x2;
        LCD_Fill((uint16_t)x, (uint16_t)y, (uint16_t)seg_end, (uint16_t)y, color);
        x = seg_end + 1 + gap;
    }
}

/* Dashed vertical line (x constant). */
static void draw_dashed_v(int x, int y1, int y2, uint16_t color, uint8_t dash, uint8_t gap)
{
    int y = y1;
    while (y <= y2) {
        int seg_end = y + dash - 1;
        if (seg_end > y2) seg_end = y2;
        LCD_Fill((uint16_t)x, (uint16_t)y, (uint16_t)x, (uint16_t)seg_end, color);
        y = seg_end + 1 + gap;
    }
}

/** Point-level grid restore (unused by the bulk erase path but kept for
 *  potential single-pixel fixups). If (x,y) falls on a grid dash, redraws
 *  that dash segment. The dash pattern is deterministic (DASH_LEN on,
 *  DASH_GAP off, repeating from the axis origin) so we can reconstruct
 *  which dash a pixel belongs to from its coordinate alone. */
static void restore_grid_point(int x, int y)
{
    /* Horizontal grid lines: y0 + k*(GRAPH_HEIGHT/GRID_DIVS), k=0..GRID_DIVS */
    for (int k = 0; k <= GRID_DIVS; k++) {
        int gy = GRAPH_Y0 - k * (GRAPH_HEIGHT / GRID_DIVS);
        if (y == gy && x >= GRAPH_X0 && x <= GRAPH_X0 + GRAPH_WIDTH) {
            int cycle = DASH_LEN + DASH_GAP;
            int rel = x - GRAPH_X0;
            int dash_start = GRAPH_X0 + (rel / cycle) * cycle;
            int dash_end = dash_start + DASH_LEN - 1;
            if (dash_end > GRAPH_X0 + GRAPH_WIDTH) dash_end = GRAPH_X0 + GRAPH_WIDTH;
            LCD_Fill((uint16_t)dash_start, (uint16_t)gy,
                     (uint16_t)dash_end,   (uint16_t)gy, COL_GRID);
            return;
        }
    }
    /* Vertical grid lines: x0 + k*(GRAPH_WIDTH/GRID_DIVS), k=0..GRID_DIVS */
    for (int k = 0; k <= GRID_DIVS; k++) {
        int gx = GRAPH_X0 + k * (GRAPH_WIDTH / GRID_DIVS);
        if (x == gx && y >= GRAPH_Y0 - GRAPH_HEIGHT && y <= GRAPH_Y0) {
            int cycle = DASH_LEN + DASH_GAP;
            int rel = GRAPH_Y0 - y;       /* relative from bottom */
            int dash_top_offset = (rel / cycle) * cycle;
            int dash_top_y = GRAPH_Y0 - dash_top_offset;
            int dash_bot_y = dash_top_y - (DASH_LEN - 1);
            if (dash_bot_y < GRAPH_Y0 - GRAPH_HEIGHT) dash_bot_y = GRAPH_Y0 - GRAPH_HEIGHT;
            /* Note: dash_top_y > dash_bot_y in screen coords (Y grows down) */
            LCD_Fill((uint16_t)gx, (uint16_t)dash_bot_y,
                     (uint16_t)gx, (uint16_t)dash_top_y, COL_GRID);
            return;
        }
    }
}

/** Erase a previously-drawn segment and restore any grid dashes it crossed.
 *
 * Two-pass approach:
 *   1. Overdraw the old line in COL_BG (same column-fill path, just black).
 *   2. Compute the bounding box of the erased region and check which of the
 *      5 horizontal and 5 vertical grid lines intersect it. For each
 *      intersection, repaint the grid line segment within the bbox.
 *
 * This is cheaper than checking every pixel against the dash pattern (as
 * restore_grid_point would do): at most 10 grid-line intersection tests
 * regardless of segment length. */
static void erase_line_and_restore_grid(int x1, int y1, int x2, int y2)
{
    /* Pass 1: erase the old segment with background color. */
    draw_line_thick(x1, y1, x2, y2, COL_BG, LINE_THICKNESS);

    /* Compute the bounding box of the erased pixels (including thickness). */
    int xa = (x1 < x2) ? x1 : x2;
    int xb = ((x1 > x2) ? x1 : x2) + LINE_THICKNESS - 1;
    int ya = (y1 < y2) ? y1 : y2;
    int yb = ((y1 > y2) ? y1 : y2) + LINE_THICKNESS - 1;

    /* Pass 2: restore horizontal grid lines whose Y falls in [ya..yb]. */
    for (int k = 0; k <= GRID_DIVS; k++) {
        int gy = GRAPH_Y0 - k * (GRAPH_HEIGHT / GRID_DIVS);
        if (gy >= ya && gy <= yb) {
            safe_fill(xa, gy, xb, gy, COL_GRID);
        }
    }
    /* Restore vertical grid lines whose X falls in [xa..xb]. */
    for (int k = 0; k <= GRID_DIVS; k++) {
        int gx = GRAPH_X0 + k * (GRAPH_WIDTH / GRID_DIVS);
        if (gx >= xa && gx <= xb) {
            safe_fill(gx, ya, gx, yb, COL_GRID);
        }
    }
}

/* ------------------------------------------------------------------ */
/*  Grid + axis + label drawing                                        */
/* ------------------------------------------------------------------ */

static void draw_grid(void)
{
    /* Horizontal dashed lines (V/I gridlines) */
    for (int k = 0; k <= GRID_DIVS; k++) {
        int y = GRAPH_Y0 - k * (GRAPH_HEIGHT / GRID_DIVS);
        draw_dashed_h(GRAPH_X0, GRAPH_X0 + GRAPH_WIDTH, y, COL_GRID, DASH_LEN, DASH_GAP);
    }
    /* Vertical dashed lines (time) */
    for (int k = 0; k <= GRID_DIVS; k++) {
        int x = GRAPH_X0 + k * (GRAPH_WIDTH / GRID_DIVS);
        draw_dashed_v(x, GRAPH_Y0 - GRAPH_HEIGHT, GRAPH_Y0, COL_GRID, DASH_LEN, DASH_GAP);
    }
}

/** Draw V-axis labels (left gutter) at 0, mid, and max.
 *  Max is derived from v_max_mv which tracks the negotiated PD voltage,
 *  so when the user selects a new PDO the axis rescales to match. */
static void draw_v_axis_labels(void)
{
    char buf[8];
    /* Pad to 5 chars so a shorter value (e.g. "5") overwrites a
     * previous longer one (e.g. "20") without leftover pixels. */
    float max_v = (float)v_max_mv / 1000.0f;
    float vals[3] = {0.0f, max_v / 2.0f, max_v};
    int   pos[3]  = {0, GRAPH_HEIGHT / 2, GRAPH_HEIGHT};
    for (int k = 0; k < 3; k++) {
        int y = GRAPH_Y0 - pos[k] - 6;
        if (y < 62) y = 62;
        snprintf(buf, sizeof(buf), "%-5u", (unsigned)(vals[k] + 0.5f));
        LCD_PutStr(15, (uint16_t)y, buf, FONT_SM, COL_V_LABEL, COL_BG);
    }
}

/** Draw I-axis labels (right gutter) at 0, mid, and max.
 *  Max is derived from i_max_ma which tracks the negotiated PD current. */
static void draw_i_axis_labels(void)
{
    char buf[8];
    float max_a = (float)i_max_ma / 1000.0f;
    float vals[3] = {0.0f, max_a / 2.0f, max_a};
    int   pos[3]  = {0, GRAPH_HEIGHT / 2, GRAPH_HEIGHT};
    for (int k = 0; k < 3; k++) {
        int y = GRAPH_Y0 - pos[k] - 6;
        if (y < 62) y = 62;
        snprintf(buf, sizeof(buf), "%-5.1f", (double)vals[k]);
        LCD_PutStr((uint16_t)I_AXIS_LABEL_X, (uint16_t)y, buf, FONT_SM, COL_I_LABEL, COL_BG);
    }
}

static void draw_time_ticks(void)
{
    /* "0s" at left, window label at right, just below the bottom grid edge. */
    int y = GRAPH_Y0 + 2;
    const char *label;
    switch (Settings_GetGraphWindow()) {
        case 0:  label = "5s "; break;
        case 2:  label = "20s"; break;
        default: label = "10s"; break;
    }
    LCD_PutStr((uint16_t)GRAPH_X0,                          (uint16_t)y, "0s",  FONT_SM, COL_AXIS_TIME, COL_BG);
    LCD_PutStr((uint16_t)(GRAPH_X0 + GRAPH_WIDTH - 20),     (uint16_t)y, label, FONT_SM, COL_AXIS_TIME, COL_BG);
}

/* ------------------------------------------------------------------ */
/*  Full plot redraw                                                   */
/* ------------------------------------------------------------------ */

/** Full plot redraw — used on first entry, screen return, or axis rescale.
 *  Wipes the entire plot rectangle (including label gutters), redraws the
 *  grid and all axis labels, then invalidates the segment cache so the
 *  subsequent per-segment loop in Graph_Draw redraws every trace segment. */
static void redraw_full_plot(void)
{
    /* Wipe plot + gutters + time-tick row with a single LCD_Fill.
     * Clamped to [62..156] to avoid clobbering the top numerics row
     * and the bottom navigation strip. */
    int wipe_y1 = GRAPH_Y0 - GRAPH_HEIGHT - 8;
    int wipe_y2 = GRAPH_Y0 + 15;
    if (wipe_y1 < 62) wipe_y1 = 62;     /* keep numerics row intact */
    if (wipe_y2 > 156) wipe_y2 = 156;   /* keep nav strip intact */
    LCD_Fill(0, (uint16_t)wipe_y1, 319, (uint16_t)wipe_y2, COL_BG);

    draw_grid();
    draw_v_axis_labels();
    draw_i_axis_labels();

    /* Zero the segment cache — every slot reads as "no previous line",
     * so the per-segment loop will draw (not skip) every segment. */
    v_lines_valid = 0;
    for (int s = 0; s < GRAPH_POINTS - 1; s++) {
        v_lines_prev[s].x1 = v_lines_prev[s].y1 = 0;
        v_lines_prev[s].x2 = v_lines_prev[s].y2 = 0;
        i_lines_prev[s].x1 = i_lines_prev[s].y1 = 0;
        i_lines_prev[s].x2 = i_lines_prev[s].y2 = 0;
    }

    grid_valid = 1;
    last_v_max_mv = v_max_mv;      /* snapshot current scale — change detection */
    last_i_max_ma = i_max_ma;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

/** Reset all graph state. Called on screen entry.
 *  last_v_max_mv is set to 0 (different from v_max_mv) so the first
 *  Graph_Draw call detects a scale mismatch and triggers a full redraw. */
void Graph_Init(void)
{
    sample_index = 0;
    sample_count = 0;
    grid_valid = 0;
    v_lines_valid = 0;
    v_max_mv = 5000;     /* placeholder until PD contract sets the real ceiling */
    i_max_ma = 3000;
    last_v_max_mv = 0;   /* intentionally differs from v_max_mv → forces full redraw */
    last_i_max_ma = 0;
    for (int i = 0; i < GRAPH_POINTS; i++) {
        v_samples_mv[i] = 0;
        i_samples_ma[i] = 0;
    }
}

/** Push one V/I sample into the rolling buffer and update axis scaling.
 *  Called from the main loop at the configured graph sample interval
 *  (50/100/200 ms for 5s/10s/20s window respectively). */
void Graph_AddSample(float voltage_v, float current_a)
{
    /* Convert float V/A to integer mV/mA for compact uint16_t storage.
     * Negative clamping is already done in INA228_ReadAll(). */
    int v_mv = (int)(voltage_v * 1000.0f + 0.5f);
    int i_ma = (int)(current_a * 1000.0f + 0.5f);
    if (v_mv > 65535) v_mv = 65535;
    if (i_ma > 65535) i_ma = 65535;

    /* Write into the circular buffer at the current head, then advance. */
    v_samples_mv[sample_index] = (uint16_t)v_mv;
    i_samples_ma[sample_index] = (uint16_t)i_ma;
    sample_index++;
    if (sample_index >= GRAPH_POINTS) sample_index = 0;  /* wrap ring */
    if (sample_count < GRAPH_POINTS) sample_count++;      /* fill phase */

    /* Update axis ceilings from the current PD contract.
     * When the user selects a new PDO (e.g. 5 V → 20 V), v_max_mv changes,
     * and Graph_Draw will detect the mismatch with last_v_max_mv and trigger
     * a full redraw with the new scale. The threshold guards (0.5 V / 0.1 A)
     * prevent glitching to 0 before the first contract is negotiated. */
    float neg_v = axxpd_get_negotiated_v();
    float neg_a = axxpd_get_negotiated_a();
    if (neg_v > 0.5f) v_max_mv = (uint16_t)(neg_v * 1000.0f + 0.5f);
    if (neg_a > 0.1f) i_max_ma = (uint16_t)(neg_a * 1000.0f + 0.5f);
}

/** Mark the grid as dirty — next Graph_Draw will do a full redraw.
 *  Called when switching to the graph screen or after a display wipe. */
void Graph_InvalidateGrid(void)
{
    grid_valid = 0;
}

/** Main draw entry point — called at ~10 Hz from the UI loop.
 *
 *  Two modes:
 *    1. Full redraw — if grid_valid==0, or axis ceiling changed since last
 *       full redraw. Wipes everything and redraws grid + labels + all traces.
 *    2. Per-segment partial update (steady state) — iterates all 99 segments.
 *       For each, compares the new screen-space endpoints against the cached
 *       previous-frame values. Unchanged segments are skipped (zero SPI cost).
 *       Changed segments are erased, grid restored, then redrawn. */
void Graph_Draw(void)
{
    /* --- Mode 1: full redraw on grid invalidation or axis rescale --- */
    if (!grid_valid || v_max_mv != last_v_max_mv || i_max_ma != last_i_max_ma) {
        redraw_full_plot();
        /* Fall through — the per-segment loop below redraws every trace
         * segment since the cache was zeroed by redraw_full_plot. */
    }

    /* --- Mode 2: per-segment partial update ---
     * Loop variable 'i' counts backward from the newest sample: i=1 is the
     * most recent segment, i=GRAPH_POINTS-1 is the oldest. Screen X maps so
     * that newest samples appear at the right edge. */
    for (uint8_t i = 1; i < GRAPH_POINTS; i++) {
        /* Beyond the valid sample horizon — erase any stale cached line
         * (can happen after Graph_Init shrinks sample_count to 0). */
        if (i >= sample_count) {
            Line_t *vp = &v_lines_prev[i - 1];
            Line_t *ip = &i_lines_prev[i - 1];
            if (vp->x1 || vp->y1 || vp->x2 || vp->y2) {
                erase_line_and_restore_grid(vp->x1, vp->y1, vp->x2, vp->y2);
                vp->x1 = vp->y1 = vp->x2 = vp->y2 = 0;
            }
            if (ip->x1 || ip->y1 || ip->x2 || ip->y2) {
                erase_line_and_restore_grid(ip->x1, ip->y1, ip->x2, ip->y2);
                ip->x1 = ip->y1 = ip->x2 = ip->y2 = 0;
            }
            continue;
        }

        /* Map ring-buffer indices to the two samples this segment connects.
         * idx_new is the newer (rightward) sample, idx_old the older (leftward).
         * Both are mod-GRAPH_POINTS offsets from sample_index (the write head). */
        uint8_t idx_new = (uint8_t)((sample_index + GRAPH_POINTS - i) % GRAPH_POINTS);
        uint8_t idx_old = (uint8_t)((sample_index + GRAPH_POINTS - i - 1) % GRAPH_POINTS);

        /* Convert slot positions to screen X — slot 0 is leftmost (oldest). */
        int16_t x_new = slot_to_x(GRAPH_POINTS - i);
        int16_t x_old = slot_to_x(GRAPH_POINTS - i - 1);

        /* --- V trace: compute new segment, compare with cached --- */
        Line_t v_new = {
            .x1 = x_old, .y1 = v_to_y(v_samples_mv[idx_old]),
            .x2 = x_new, .y2 = v_to_y(v_samples_mv[idx_new]),
        };
        Line_t *v_old = &v_lines_prev[i - 1];
        int v_changed = (v_old->x1 != v_new.x1) || (v_old->y1 != v_new.y1) ||
                        (v_old->x2 != v_new.x2) || (v_old->y2 != v_new.y2);
        int v_old_present = (v_old->x1 || v_old->y1 || v_old->x2 || v_old->y2);

        /* --- I trace: compute new segment, compare with cached --- */
        Line_t i_new = {
            .x1 = x_old, .y1 = i_to_y(i_samples_ma[idx_old]),
            .x2 = x_new, .y2 = i_to_y(i_samples_ma[idx_new]),
        };
        Line_t *i_old = &i_lines_prev[i - 1];
        int i_changed = (i_old->x1 != i_new.x1) || (i_old->y1 != i_new.y1) ||
                        (i_old->x2 != i_new.x2) || (i_old->y2 != i_new.y2);
        int i_old_present = (i_old->x1 || i_old->y1 || i_old->x2 || i_old->y2);

        /* Erase ALL stale segments first, THEN draw all new ones. This order
         * prevents the erase pass of one trace from clobbering the freshly-
         * drawn pixels of the other trace at the same X slot. */
        if (v_changed && v_old_present) {
            erase_line_and_restore_grid(v_old->x1, v_old->y1, v_old->x2, v_old->y2);
        }
        if (i_changed && i_old_present) {
            erase_line_and_restore_grid(i_old->x1, i_old->y1, i_old->x2, i_old->y2);
        }

        /* Draw new segments — I first, then V on top. V is the primary
         * readout (voltage) and benefits from being the top-most layer
         * where the two traces overlap. */
        if (v_changed || !v_old_present) {
            draw_line_thick(i_new.x1, i_new.y1, i_new.x2, i_new.y2, COL_I_TRACE, LINE_THICKNESS);
            draw_line_thick(v_new.x1, v_new.y1, v_new.x2, v_new.y2, COL_V_TRACE, LINE_THICKNESS);
        } else if (i_changed) {
            /* Only the I trace moved — but we still redraw both traces at
             * this slot because the erase+grid-restore pass may have
             * partially damaged the (unchanged) V pixels in the overlap. */
            draw_line_thick(i_new.x1, i_new.y1, i_new.x2, i_new.y2, COL_I_TRACE, LINE_THICKNESS);
            draw_line_thick(v_new.x1, v_new.y1, v_new.x2, v_new.y2, COL_V_TRACE, LINE_THICKNESS);
        }

        /* Commit this frame's endpoints into the cache for next-frame diff. */
        *v_old = v_new;
        *i_old = i_new;
    }
}
