// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#ifndef __UART_H
#define __UART_H

#include "stm32g4xx_hal.h"

void    UART_Init(UART_HandleTypeDef *huart);
void    UART_SendString(const char *str);
uint8_t UART_HasLine(void);
uint16_t UART_GetLine(char *buf, uint16_t max_len);

#endif /* __UART_H */
