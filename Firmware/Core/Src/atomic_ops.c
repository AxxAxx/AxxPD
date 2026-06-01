/**
 * @file    atomic_ops.c
 * @brief   Software atomic-operation stubs for Cortex-M0+.
 *
 * The STM32CubeIDE arm-none-eabi toolchain (13.3.rel1) does not ship
 * libatomic for Cortex-M0+.  pdsink uses std::atomic<> which the compiler
 * lowers to __atomic_* builtins on M0+ (no native LDREX/STREX on M0+).
 *
 * This is a single-core, no-RTOS firmware.  All "atomics" that pdsink needs
 * are safe to implement with IRQ-disable/enable because pdsink is called only
 * from the main-loop task and its IRQ handler (axxpd_ucpd_irq) does not re-
 * enter the state-machine functions.
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx.h"   /* for __disable_irq / __enable_irq */

/* ------------------------------------------------------------------ */

static inline uint32_t irq_save(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void irq_restore(uint32_t primask)
{
    if (!primask) __enable_irq();
}

/* ------------------------------------------------------------------ */
/*  4-byte (uint32_t) ops                                              */
/* ------------------------------------------------------------------ */

uint32_t __atomic_fetch_or_4(volatile void *ptr, uint32_t val, int model)
{
    (void)model;
    volatile uint32_t *p32 = (volatile uint32_t *)ptr;
    uint32_t p = irq_save();
    uint32_t old = *p32;
    *p32 = old | val;
    irq_restore(p);
    return old;
}

uint32_t __atomic_fetch_and_4(volatile void *ptr, uint32_t val, int model)
{
    (void)model;
    volatile uint32_t *p32 = (volatile uint32_t *)ptr;
    uint32_t p = irq_save();
    uint32_t old = *p32;
    *p32 = old & val;
    irq_restore(p);
    return old;
}

uint32_t __atomic_exchange_4(volatile void *ptr, uint32_t val, int model)
{
    (void)model;
    volatile uint32_t *p32 = (volatile uint32_t *)ptr;
    uint32_t p = irq_save();
    uint32_t old = *p32;
    *p32 = val;
    irq_restore(p);
    return old;
}

uint32_t __atomic_load_4(const volatile void *ptr, int model)
{
    (void)model;
    const volatile uint32_t *p32 = (const volatile uint32_t *)ptr;
    uint32_t p = irq_save();
    uint32_t val = *p32;
    irq_restore(p);
    return val;
}

void __atomic_store_4(volatile void *ptr, uint32_t val, int model)
{
    (void)model;
    volatile uint32_t *p32 = (volatile uint32_t *)ptr;
    uint32_t p = irq_save();
    *p32 = val;
    irq_restore(p);
}

_Bool __atomic_compare_exchange_4(volatile void *ptr,
                                  void *expected, uint32_t desired,
                                  _Bool weak, int success_model, int fail_model)
{
    (void)weak; (void)success_model; (void)fail_model;
    volatile uint32_t *p32 = (volatile uint32_t *)ptr;
    uint32_t *exp = (uint32_t *)expected;
    uint32_t p = irq_save();
    uint32_t cur = *p32;
    _Bool ok = (cur == *exp);
    if (ok) *p32 = desired;
    else    *exp = cur;
    irq_restore(p);
    return ok;
}

/* ------------------------------------------------------------------ */
/*  1-byte (uint8_t) ops                                               */
/* ------------------------------------------------------------------ */

uint8_t __atomic_exchange_1(volatile void *ptr, uint8_t val, int model)
{
    (void)model;
    volatile uint8_t *p8 = (volatile uint8_t *)ptr;
    uint32_t p = irq_save();
    uint8_t old = *p8;
    *p8 = val;
    irq_restore(p);
    return old;
}

uint8_t __atomic_load_1(const volatile void *ptr, int model)
{
    (void)model;
    const volatile uint8_t *p8 = (const volatile uint8_t *)ptr;
    uint32_t p = irq_save();
    uint8_t val = *p8;
    irq_restore(p);
    return val;
}

void __atomic_store_1(volatile void *ptr, uint8_t val, int model)
{
    (void)model;
    volatile uint8_t *p8 = (volatile uint8_t *)ptr;
    uint32_t p = irq_save();
    *p8 = val;
    irq_restore(p);
}

uint8_t __atomic_fetch_or_1(volatile void *ptr, uint8_t val, int model)
{
    (void)model;
    volatile uint8_t *p8 = (volatile uint8_t *)ptr;
    uint32_t p = irq_save();
    uint8_t old = *p8;
    *p8 = old | val;
    irq_restore(p);
    return old;
}

uint8_t __atomic_fetch_and_1(volatile void *ptr, uint8_t val, int model)
{
    (void)model;
    volatile uint8_t *p8 = (volatile uint8_t *)ptr;
    uint32_t p = irq_save();
    uint8_t old = *p8;
    *p8 = old & val;
    irq_restore(p);
    return old;
}
