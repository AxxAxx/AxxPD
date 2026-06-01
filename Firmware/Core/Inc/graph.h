/**
 * @file    graph.h
 * @brief   Live V/I plot for the GRAPH UI screen.
 *
 * Maintains a 100-sample rolling buffer of (voltage, current) pairs.
 * Sample interval is configurable via the graph_window setting:
 *   0=5s window (50ms/sample), 1=10s (100ms), 2=20s (200ms).
 * Renders a single grid with two Y-axes (V on the left, I on the right).
 * The V axis snaps to a nice maximum derived from the negotiated source
 * voltage; the I axis is fixed at 0..6 A.
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
 *  graph sample interval (50/100/200 ms depending on graph_window setting). */
void Graph_AddSample(float voltage_v, float current_a);

/** Draw or refresh the plot region. Call from the GRAPH screen draw path
 *  on each UI tick. */
void Graph_Draw(void);

/** Force the next Graph_Draw() to do a full plot redraw (grid + axes +
 *  history). Call when transitioning onto the GRAPH screen, since the UI
 *  performs a full LCD clear at screen transitions. */
void Graph_InvalidateGrid(void);

#endif /* __GRAPH_H */
