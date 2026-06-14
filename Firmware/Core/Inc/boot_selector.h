// Copyright 2026 Axel Johansson
// SPDX-License-Identifier: GPL-3.0-only
//
// This file is part of AxxPD. See LICENSE for details.

#ifndef __BOOT_SELECTOR_H
#define __BOOT_SELECTOR_H

#include <stdint.h>

/* Run the boot PDO selector. Blocks until user confirms or 10s timeout.
 * Calls axxpd_run() internally to keep PD stack alive.
 * Returns 1-based PDO index chosen, or 0 if selector is disabled. */
int BootSelector_Run(void);

/* Returns the raw PDO word selected by the user (0 if none). */
uint32_t BootSelector_GetSelectedPdo(void);

#endif /* __BOOT_SELECTOR_H */
