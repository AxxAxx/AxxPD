// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

/**
 * @file    graph.h
 * @brief   Live V/I plot for the GRAPH UI screen.
 *
 * Maintains a 241-sample rolling buffer (one per pixel column) of
 * (voltage, current) pairs, so the trace scrolls in 1 px steps.
 * Sample interval is configurable via the graph_window setting:
 *   0=5s window (21ms/sample), 1=10s (42ms), 2=30s (125ms), 3=60s (250ms).
 * Renders a single grid with two Y-axes (V on the left, I on the right).
 * The V axis snaps to a nice maximum derived from the negotiated source
 * voltage; the I axis max is derived from the negotiated current.
 *
 * Sampling runs continuously regardless of which UI screen is active, so
 * history stays continuous when the user navigates away and back. Drawing
 * is gated by Graph_Draw() which is only called while the GRAPH screen is
 * the active screen.
 */
#ifndef __GRAPH_H
#define __GRAPH_H

/** Zero buffers and mark the grid as dirty. Call once from UI_Init(). */
void Graph_Init(void);

/** Push one (V, I) sample. Called from the main loop at the configured
 *  graph sample interval (21/42/125/250 ms depending on graph_window setting). */
void Graph_AddSample(float voltage_v, float current_a);

/** Draw or refresh the plot region. Call from the GRAPH screen draw path
 *  on each UI tick. */
void Graph_Draw(void);

/** Force the next Graph_Draw() to do a full plot redraw (grid + axes +
 *  history). Call when transitioning onto the GRAPH screen, since the UI
 *  performs a full LCD clear at screen transitions. */
void Graph_InvalidateGrid(void);

#endif /* __GRAPH_H */
