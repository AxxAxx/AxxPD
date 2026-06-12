// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

/**
 * @file  uart.c
 * @brief UART driver — interrupt-driven RX, blocking TX, double-buffered lines.
 *
 * USART1 is initialised by CubeMX (115200-8N1, PB6 TX / PB7 RX).
 * Call UART_Init() once after MX_USART1_UART_Init(), then poll
 * UART_HasLine() / UART_GetLine() from the main loop.
 *
 * The HAL_UART_RxCpltCallback weak override drives a double-buffered
 * RX scheme: one buffer holds the completed line for the main loop,
 * while a second buffer accumulates the next incoming line. This allows
 * pipelined commands without byte loss.
 */

#include "uart.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Private state                                                               */
/* -------------------------------------------------------------------------- */

#define UART_RX_BUF_LEN  128U

static UART_HandleTypeDef *s_huart     = NULL;

/* Double buffer: buf[0] and buf[1]. The ISR writes to s_rx_active,
 * the main loop reads from the other one when line_ready is set. */
static char     s_rx_buf[2][UART_RX_BUF_LEN];
static uint16_t s_rx_head[2]           = {0, 0};
static uint8_t  s_rx_active            = 0;   /* index ISR writes to */
static uint8_t  s_rx_byte              = 0;   /* single-byte IT target */
static volatile uint8_t s_line_ready   = 0;   /* bitmask: bit 0 = buf[0], bit 1 = buf[1] */
static uint8_t  s_read_idx             = 0;   /* next buffer to read from */

/* -------------------------------------------------------------------------- */
/* Internal helpers                                                            */
/* -------------------------------------------------------------------------- */

static void UART_RearmRx(void)
{
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void UART_Init(UART_HandleTypeDef *huart)
{
    s_huart      = huart;
    s_rx_head[0] = 0;
    s_rx_head[1] = 0;
    s_rx_active  = 0;
    s_line_ready = 0;
    s_read_idx   = 0;
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    UART_RearmRx();
}

void UART_SendString(const char *str)
{
    if (s_huart == NULL || str == NULL) { return; }
    uint16_t len = (uint16_t)strlen(str);
    if (len == 0U) { return; }
    HAL_UART_Transmit(s_huart, (const uint8_t *)str, len, 100U);
}

uint8_t UART_HasLine(void)
{
    return (s_line_ready != 0U) ? 1U : 0U;
}

uint16_t UART_GetLine(char *buf, uint16_t max_len)
{
    if (s_line_ready == 0U || buf == NULL || max_len == 0U) { return 0U; }

    /* Find which buffer has a ready line */
    uint8_t idx = s_read_idx;
    if (!(s_line_ready & (1U << idx))) {
        idx = 1U - idx;  /* try the other one */
        if (!(s_line_ready & (1U << idx))) { return 0U; }
    }

    uint16_t copy_len = (s_rx_head[idx] < max_len) ? s_rx_head[idx] : (max_len - 1U);
    memcpy(buf, s_rx_buf[idx], copy_len);
    buf[copy_len] = '\0';

    /* Clear this buffer and mark as consumed */
    s_rx_head[idx] = 0;
    memset(s_rx_buf[idx], 0, UART_RX_BUF_LEN);

    __disable_irq();
    s_line_ready &= ~(1U << idx);
    __enable_irq();

    /* Advance read index to the other buffer for next call */
    s_read_idx = 1U - idx;

    /* If ISR had stopped because both buffers were full, resume reception
     * into the buffer we just freed — the other buffer still holds an
     * unconsumed ready line and must not receive new bytes. Safe to touch
     * s_rx_active here: RX is stopped, so no RX interrupt can fire. */
    if (s_huart->RxState == HAL_UART_STATE_READY) {
        s_rx_active = idx;
        UART_RearmRx();
    }

    return copy_len;
}

/* -------------------------------------------------------------------------- */
/* HAL callback                                                                */
/* -------------------------------------------------------------------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != s_huart->Instance) { return; }

    uint8_t byte = s_rx_byte;
    uint8_t idx = s_rx_active;

    if (byte == '\r' || byte == '\n')
    {
        if (s_rx_head[idx] > 0U)
        {
            /* Mark this buffer as having a complete line */
            s_line_ready |= (1U << idx);

            /* Switch to the other buffer if available */
            uint8_t next = 1U - idx;
            if (!(s_line_ready & (1U << next))) {
                /* Other buffer is free — switch to it and keep listening */
                s_rx_active = next;
                s_rx_head[next] = 0;
                UART_RearmRx();
            }
            /* else: both buffers full — stop receiving until main loop consumes */
        }
        else
        {
            /* Empty line (bare CR/LF) — discard and keep listening */
            UART_RearmRx();
        }
    }
    else
    {
        if (s_rx_head[idx] < (UART_RX_BUF_LEN - 1U))
        {
            s_rx_buf[idx][s_rx_head[idx]++] = (char)byte;
        }
        /* else: buffer full — drop byte silently */
        UART_RearmRx();
    }
}
