/*
 * Copyright (c) 2024 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * Batch timeslot API for PS/2 UART driver.
 *
 * Allows callers (e.g. the mouse driver) to hold a single MPSL
 * timeslot across a batch of PS/2 register writes, preventing
 * BLE radio ZLI from preempting the GPIO bit-bang mid-write.
 */

#pragma once

#include <zephyr/kernel.h>

#if IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION)

/**
 * Begin a batch-protected region.  Acquires a 100 ms MPSL timeslot
 * (the maximum allowed) so that all PS/2 writes within the batch
 * are shielded from BLE ZLI preemption.
 *
 * While the batch is active, individual write_byte_blocking() calls
 * skip their per-byte timeslot acquire/release.
 *
 * If the timeslot expires before the batch ends (e.g. retries push
 * total time past 100 ms), remaining writes gracefully fall back to
 * per-byte timeslot protection.
 *
 * Nesting IS supported — inner batch_begin() increments a refcount,
 * inner batch_end() decrements it.  The timeslot is only released
 * when the outermost batch_end() is called.
 *
 * @return 0 on success, -EBUSY if the timeslot was blocked,
 *         -ENODEV if no session is open.
 */
int ps2_uart_timeslot_batch_begin(void);

/**
 * End a batch-protected region.  Signals that no more writes are
 * expected under this batch.  The underlying MPSL timeslot will
 * end naturally when TIMER0 fires (or has already ended).
 */
void ps2_uart_timeslot_batch_end(void);

#else /* !CONFIG_PS2_UART_TIMESLOT_PROTECTION */

static inline int ps2_uart_timeslot_batch_begin(void) { return 0; }
static inline void ps2_uart_timeslot_batch_end(void) { }

#endif /* CONFIG_PS2_UART_TIMESLOT_PROTECTION */
