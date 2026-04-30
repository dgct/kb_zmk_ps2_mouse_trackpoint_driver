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

/**
 * Inhibit the PS/2 bus by driving CLK LOW.  While inhibited the TP
 * cannot clock data, preventing pin-glitch corruption during UART
 * pin transitions (idle PM suspend/resume).
 *
 * @param dev  The PS/2 UART device (e.g. config->ps2_device).
 */
void ps2_uart_inhibit_bus(const struct device *dev);

/**
 * Release the PS/2 bus (CLK back to input).  The TP can transmit
 * again once the external pull-up brings CLK HIGH.
 *
 * @param dev  The PS/2 UART device (e.g. config->ps2_device).
 */
void ps2_uart_release_bus(const struct device *dev);

/**
 * Purge the PS/2 UART data queue, discarding any stale bytes
 * that accumulated while the callback was disabled (e.g. during
 * idle PM dormant drain window).
 *
 * @param dev  The PS/2 UART device (e.g. config->ps2_device).
 */
void ps2_uart_data_queue_empty(const struct device *dev);

/**
 * Consolidated idle PM suspend for both UARTEs.
 *
 * Stops UARTE1 diversity receiver (disconnects P0.17 for wake GPIO),
 * then suspends UARTE0 via Zephyr PM (STOPRX + disable + sleep pinctrl).
 *
 * Caller MUST call ps2_uart_inhibit_bus() BEFORE this function, and
 * ps2_uart_release_bus() when appropriate afterward.
 *
 * @param dev       The PS/2 UART device.
 * @param uart_dev  The underlying Zephyr UART device (for PM action).
 * @return 0 on success, negative errno on failure.
 */
int ps2_uart_pm_suspend(const struct device *dev, const struct device *uart_dev);

/**
 * Consolidated idle PM resume for both UARTEs.
 *
 * Resumes UARTE0 via Zephyr PM (pinctrl DEFAULT + enable + STARTRX),
 * restores the error interrupt, restarts UARTE1 diversity receiver,
 * and purges the data queue.
 *
 * CLK stays inhibited on return — both UARTEs are armed but the TP
 * cannot transmit, preventing stale bytes from contaminating verify reads.
 * Caller MUST call ps2_uart_inhibit_bus() BEFORE this function.
 *
 * Retries PM RESUME up to 3 times with 500µs between attempts.
 *
 * @param dev       The PS/2 UART device.
 * @param uart_dev  The underlying Zephyr UART device (for PM action).
 * @return 0 on success, negative errno if all resume attempts failed.
 */
int ps2_uart_pm_resume(const struct device *dev, const struct device *uart_dev);
