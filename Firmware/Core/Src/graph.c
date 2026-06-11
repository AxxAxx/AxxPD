/**
 * @file    graph.c
 * @brief   Live V/I plot for the GRAPH UI screen.
 *
 * Single grid, two Y-axes: V (left, yellow), I (right, orange). 100-sample
 * rolling window with configurable sample interval: 50/100/200 ms for
 * 5s/10s/20s history (set via Settings graph_window). V axis snaps to the
 * negotiated PD source voltage; I axis max comes from the negotiated current.
 *
 * Rendering strategy — RAM framebuffer + single DMA blit:
 *   The plot region (221 x 55 px) is composed in an off-screen uint16_t RAM
 *   buffer — grid + both traces — using plain array writes (no SPI). Once the
 *   frame is fully drawn, the whole buffer is pushed to the panel in ONE
 *   memInc DMA transfer via LCD_DrawImage. This replaces the previous
 *   per-segment erase/restore approach, which issued ~1500 tiny SPI fills per
 *   scroll frame (each dominated by SetWindow command overhead). One blit per
 *   new sample is dramatically faster and flicker-free, and the dotted grid is
 *   simply part of the composed frame. (This mirrors the original AxxSolder
 *   framebuffer approach, which was lost when the port moved to direct LCD_Fill.)
 *
 *   The STM32G491 has 112 KB SRAM; the ~24 KB plot buffer fits comfortably.
 *   Axis labels live in the gutters outside the plot rect and are drawn
 *   directly to the LCD only on entry / axis rescale, so they never flicker.
 */

#include "graph.h"
#include "lcd.h"
#include "ugui.h"
#include "axxpd_main.h"
#include "settings.h"
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>   /* isfinite() */

/* ------------------------------------------------------------------ */
/*  Geometry                                                           */
/* ------------------------------------------------------------------ */

#define GRAPH_X0        40    /* left edge; widened toward the V-axis labels (~x15..35) */
#define GRAPH_Y0        120   /* bottom edge of plot area (time labels below, navbar at 142) */
#define GRAPH_WIDTH     240   /* plot width in px; right edge at x=280, before I-axis labels (x=285) */
#define GRAPH_HEIGHT    54    /* plot height in px; top edge at y=66 */
#define GRAPH_POINTS    100   /* rolling buffer length; window = 100 * interval_ms */

#define I_AXIS_LABEL_X  285   /* right of graph + margin */

#define GRID_DIVS       4     /* 4 segments = 5 horizontal gridlines */
#define DASH_LEN        3
#define DASH_GAP        4
#define LINE_THICKNESS  2

/* Framebuffer covers the plot rect inclusively: x [GRAPH_X0..GRAPH_X0+GRAPH_WIDTH],
 * y [FB_TOP..GRAPH_Y0]. */
#define FB_TOP          (GRAPH_Y0 - GRAPH_HEIGHT)   /* 66 */
#define FB_W            (GRAPH_WIDTH + 1)           /* 221 */
#define FB_H            (GRAPH_HEIGHT + 1)          /* 55  */

/* ------------------------------------------------------------------ */
/*  Colors (RGB565)                                                    */
/* ------------------------------------------------------------------ */

#define COL_BG          RGB(0,   0,   0  )
#define COL_GRID        LEGACY(48,  48,  48)
#define COL_AXIS_TIME   RGB(200, 200, 200)   /* light grey time labels (0s / 10s) */
#define COL_V_TRACE     RGB(255, 234, 0)           /* yellow — matches dashboard V */
#define COL_I_TRACE     RGB(240, 64,  5)           /* orange — matches dashboard A */
#define COL_V_LABEL     COL_V_TRACE
#define COL_I_LABEL     COL_I_TRACE

/* ------------------------------------------------------------------ */
/*  Fonts                                                              */
/* ------------------------------------------------------------------ */

extern UG_FONT FONT_arial_17X18[];
#define FONT_SM         FONT_arial_17X18

/* ------------------------------------------------------------------ */
/*  Module state                                                       */
/* ------------------------------------------------------------------ */

/* Off-screen plot framebuffer (RGB565). ~24 KB. */
static uint16_t plot_fb[FB_W * FB_H];

/* Rolling sample buffer (circular). */
static uint16_t v_samples_mv[GRAPH_POINTS];
static uint16_t i_samples_ma[GRAPH_POINTS];
static volatile uint8_t  sample_index;       /* next slot to write (head of ring) */
static volatile uint8_t  sample_count;       /* valid samples so far, clamps at GRAPH_POINTS */

static uint8_t  grid_valid;         /* 0 = labels/grid need a full redraw */
static uint8_t  last_drawn_index;   /* sample_index at last blit; 0xFF forces a blit */
static volatile uint16_t v_max_mv;  /* current V-axis ceiling in mV */
static volatile uint16_t i_max_ma;  /* current I-axis ceiling in mA */
static uint16_t last_v_max_mv;      /* scale used for last full redraw — mismatch → rescale */
static uint16_t last_i_max_ma;

/* ------------------------------------------------------------------ */
/*  Sample → pixel mapping                                             */
/* ------------------------------------------------------------------ */

static int16_t v_to_y(uint16_t mv)
{
    if (v_max_mv == 0U) return (int16_t)GRAPH_Y0;
    uint32_t clamped = (mv > v_max_mv) ? v_max_mv : mv;
    int dy = (int)((clamped * (uint32_t)GRAPH_HEIGHT) / v_max_mv);
    return (int16_t)(GRAPH_Y0 - dy);
}

static int16_t i_to_y(uint16_t ma)
{
    if (i_max_ma == 0U) return (int16_t)GRAPH_Y0;
    uint32_t clamped = (ma > i_max_ma) ? i_max_ma : ma;
    int dy = (int)((clamped * (uint32_t)GRAPH_HEIGHT) / i_max_ma);
    return (int16_t)(GRAPH_Y0 - dy);
}

static int16_t slot_to_x(int i)
{
    return (int16_t)(GRAPH_X0 + (i * GRAPH_WIDTH) / (GRAPH_POINTS - 1));
}

/* ------------------------------------------------------------------ */
/*  Framebuffer primitives (RAM only — no SPI)                         */
/* ------------------------------------------------------------------ */

/* Plot a pixel using SCREEN coordinates; clipped to the framebuffer. */
static inline void fb_px(int sx, int sy, uint16_t color)
{
    int fx = sx - GRAPH_X0;
    int fy = sy - FB_TOP;
    if (fx < 0 || fx >= FB_W || fy < 0 || fy >= FB_H) return;
    plot_fb[fy * FB_W + fx] = color;
}

/* Column-swept thick line, written into the framebuffer. */
static void fb_line_thick(int x1, int y1, int x2, int y2, uint16_t color, uint8_t thickness)
{
    uint8_t t = (thickness == 0U) ? 1U : thickness;
    if (x1 > x2) { int tmp; tmp=x1; x1=x2; x2=tmp; tmp=y1; y1=y2; y2=tmp; }

    int dx = x2 - x1;
    if (dx == 0) {
        int ya = (y1 < y2) ? y1 : y2;
        int yb = (y1 > y2) ? y1 : y2;
        for (int yy = ya; yy <= yb + t - 1; yy++)
            for (int xx = x1; xx < x1 + t; xx++) fb_px(xx, yy, color);
        return;
    }
    for (int x = x1; x <= x2; x++) {
        int ya = y1 + (y2 - y1) * (x - x1) / dx;
        int yb = (x == x2) ? y2 : y1 + (y2 - y1) * (x + 1 - x1) / dx;
        int ymin = (ya < yb) ? ya : yb;
        int ymax = (ya > yb) ? ya : yb;
        for (int yy = ymin; yy <= ymax + t - 1; yy++)
            for (int xx = x; xx < x + t; xx++) fb_px(xx, yy, color);
    }
}

/* Dashed grid composed into the framebuffer (dotted everywhere). */
static void fb_draw_grid(void)
{
    const int cyc = DASH_LEN + DASH_GAP;
    /* Horizontal gridlines — dashes phased from the left edge. */
    for (int k = 0; k <= GRID_DIVS; k++) {
        int gy = GRAPH_Y0 - k * (GRAPH_HEIGHT / GRID_DIVS);
        for (int x = GRAPH_X0; x <= GRAPH_X0 + GRAPH_WIDTH; x++)
            if (((x - GRAPH_X0) % cyc) < DASH_LEN) fb_px(x, gy, COL_GRID);
    }
    /* Vertical gridlines — always 5 lines (4 divisions), dashes phased from top. */
    for (int k = 0; k <= GRID_DIVS; k++) {
        int gx = GRAPH_X0 + k * (GRAPH_WIDTH / GRID_DIVS);
        for (int y = FB_TOP; y <= GRAPH_Y0; y++)
            if (((y - FB_TOP) % cyc) < DASH_LEN) fb_px(gx, y, COL_GRID);
    }
}

/* ------------------------------------------------------------------ */
/*  Axis labels (drawn directly to LCD, in the gutters)                */
/* ------------------------------------------------------------------ */

static void draw_v_axis_labels(void)
{
    char buf[8];
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
    int y = GRAPH_Y0 + 2;
    const char *label;
    switch (Settings_GetGraphWindow()) {
        case 1:  label = "10s"; break;
        case 2:  label = "30s"; break;
        case 3:  label = "60s"; break;
        default: label = "5s";  break;
    }
    LCD_PutStr((uint16_t)GRAPH_X0, (uint16_t)y, "0s", FONT_SM, COL_AXIS_TIME, COL_BG);
    /* Right-align the window label: end of the 's' lands on the right-edge tick
     * (GRAPH_X0+GRAPH_WIDTH). FONT_SM (arial_17X18) advances: digits 10px, 's' 9px. */
    int lw = 0;
    for (const char *p = label; *p; p++) lw += (*p == 's') ? 9 : 10;
    LCD_PutStr((uint16_t)(GRAPH_X0 + GRAPH_WIDTH - lw), (uint16_t)y, (char *)label, FONT_SM, COL_AXIS_TIME, COL_BG);
}

/** Wipe the plot region + gutters and (re)draw the axis labels. Called on
 *  screen entry and on axis rescale only. The plot interior is repainted by
 *  the framebuffer blit in Graph_Draw. */
static void redraw_full_plot(void)
{
    int wipe_y1 = FB_TOP - 8;
    int wipe_y2 = GRAPH_Y0 + 15;
    if (wipe_y1 < 62) wipe_y1 = 62;
    if (wipe_y2 > 156) wipe_y2 = 156;
    LCD_Fill(0, (uint16_t)wipe_y1, 319, (uint16_t)wipe_y2, COL_BG);

    draw_v_axis_labels();
    draw_i_axis_labels();
    draw_time_ticks();

    grid_valid = 1;
    last_v_max_mv = v_max_mv;
    last_i_max_ma = i_max_ma;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

void Graph_Init(void)
{
    sample_index = 0;
    sample_count = 0;
    grid_valid = 0;
    last_drawn_index = 0xFF;
    v_max_mv = 5000;     /* placeholder until PD contract sets the real ceiling */
    i_max_ma = 3000;
    last_v_max_mv = 0;   /* differs from v_max_mv → forces full redraw */
    last_i_max_ma = 0;
    for (int i = 0; i < GRAPH_POINTS; i++) {
        v_samples_mv[i] = 0;
        i_samples_ma[i] = 0;
    }
}

void Graph_AddSample(float voltage_v, float current_a)
{
    if (!isfinite(voltage_v) || !isfinite(current_a)) return;
    int v_mv = (int)(voltage_v * 1000.0f + 0.5f);
    int i_ma = (int)(current_a * 1000.0f + 0.5f);
    if (v_mv < 0) v_mv = 0;
    if (i_ma < 0) i_ma = 0;
    if (v_mv > 65535) v_mv = 65535;
    if (i_ma > 65535) i_ma = 65535;

    v_samples_mv[sample_index] = (uint16_t)v_mv;
    i_samples_ma[sample_index] = (uint16_t)i_ma;
    sample_index++;
    if (sample_index >= GRAPH_POINTS) sample_index = 0;
    if (sample_count < GRAPH_POINTS) sample_count++;

    float neg_v = axxpd_get_negotiated_v();
    float neg_a = axxpd_get_negotiated_a();
    if (neg_v > 0.5f) v_max_mv = (uint16_t)(neg_v * 1000.0f + 0.5f);
    if (neg_a > 0.1f) i_max_ma = (uint16_t)(neg_a * 1000.0f + 0.5f);
}

void Graph_InvalidateGrid(void)
{
    grid_valid = 0;
}

/** Compose the plot in RAM and blit it in a single DMA transfer.
 *  Cheap no-op when no new sample has arrived since the last blit. */
void Graph_Draw(void)
{
    uint8_t need_full = (!grid_valid || v_max_mv != last_v_max_mv || i_max_ma != last_i_max_ma);

    if (need_full) {
        redraw_full_plot();        /* wipe gutters + redraw labels */
        last_drawn_index = 0xFF;   /* force a blit this frame */
    }

    /* Steady state: skip entirely until a new sample shifts the trace. */
    if (!need_full && sample_index == last_drawn_index) return;

    /* --- Compose the frame in RAM --- */
    memset(plot_fb, 0, sizeof(plot_fb));   /* COL_BG (0x0000) */
    fb_draw_grid();

    for (uint8_t i = 1; i < sample_count; i++) {
        uint8_t idx_new = (uint8_t)((sample_index + GRAPH_POINTS - i) % GRAPH_POINTS);
        uint8_t idx_old = (uint8_t)((sample_index + GRAPH_POINTS - i - 1) % GRAPH_POINTS);
        int16_t x_new = slot_to_x(GRAPH_POINTS - i);
        int16_t x_old = slot_to_x(GRAPH_POINTS - i - 1);

        /* I first, then V on top (V is the primary readout where they overlap). */
        fb_line_thick(x_old, i_to_y(i_samples_ma[idx_old]),
                      x_new, i_to_y(i_samples_ma[idx_new]), COL_I_TRACE, LINE_THICKNESS);
        fb_line_thick(x_old, v_to_y(v_samples_mv[idx_old]),
                      x_new, v_to_y(v_samples_mv[idx_new]), COL_V_TRACE, LINE_THICKNESS);
    }

    /* --- Blit the whole plot rect in one DMA transfer --- */
    UG_BMP bmp = { .p = plot_fb, .width = FB_W, .height = FB_H, .bpp = BMP_BPP_16, .colors = 0 };
    LCD_DrawImage(GRAPH_X0, FB_TOP, &bmp);

    last_drawn_index = sample_index;
}
