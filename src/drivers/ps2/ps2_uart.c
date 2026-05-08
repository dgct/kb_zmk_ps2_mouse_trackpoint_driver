/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT uart_ps2

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include <hal/nrf_uarte.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_timer.h>
#include <nrfx.h>
#include <nrfx_ppi.h>
#include <nrfx_gpiote.h>

static const nrfx_gpiote_t nrfx_gpiote = NRFX_GPIOTE_INSTANCE(0);

/*
 * Dual-UARTE Diversity Receiver with erasure-code repair
 *
 * CLK period is measured at boot via GPIOTE+PPI+TIMER3 to auto-calibrate
 * the baud rate. UARTE0 runs at f*(1+δ) and a bare-metal UARTE1 runs at
 * f*(1-δ) on the same DATA pin.  When both decode cleanly, they cross-
 * validate; when one has a framing error, the other repairs it.
 */

/* δ spread: 1/42 of center baud — the information-theoretic
 * optimum for a (2,1) erasure code over an 11-bit UART frame.
 *
 * Single-receiver corruption threshold: bit 10 (stop) drifts
 * (10.5 × δ) bit periods; at δ = 1/21 this reaches 0.5 (limit).
 *
 * At δ = 1/42 (half the tiling maximum), each receiver has full
 * 1/21 headroom.  The two valid-decode bands overlap by 50%:
 *   - Center band |ε| < 1/42: both decode all 11 bits cleanly
 *     → cross-validation (detect non-framing corruption)
 *   - Extended bands 1/42 < |ε| < 1/14: exactly one receiver
 *     valid → clean fallback, no bit-level guessing
 *   - Total coverage: |ε| < 1/14 (3/21 of center baud)
 *
 * fast = round(center × 43/42), slow = round(center × 41/42) */
#define PS2_UART_DIVERSITY_NUMER_FAST 43U
#define PS2_UART_DIVERSITY_NUMER_SLOW 41U
#define PS2_UART_DIVERSITY_DENOM      42U

/* CLK calibration: number of edges to sample and expected tick range */
#define PS2_UART_CAL_EDGES           16
#define PS2_UART_CAL_MIN_TICKS      950   /* ~16.8 kHz */
#define PS2_UART_CAL_MAX_TICKS     1200   /* ~13.3 kHz */
#define PS2_UART_CAL_TIMEOUT_US   50000   /* 50 ms max wait per edge */

/* GPIOTE channel and PPI channel for calibration (lowest = safest) */
/* PPI and GPIOTE channels are allocated at runtime via nrfx to
 * avoid conflicts with MPSL, SoftDevice Controller, and other
 * nrfx drivers that share these fixed-count hardware resources. */

/* Diversity receiver state.
 *
 * UARTE1 runs ISR-free: SHORTS ENDRX_STARTRX re-arms DMA in hardware.
 * UARTE0's ISR polls UARTE1's ENDRX event and reads uarte1_dma_buf +
 * ERRORSRC directly — no volatile handoff variables needed.
 */
static volatile uint8_t  uarte1_dma_buf;
static volatile bool     uarte1_initialized;
static volatile bool     uart_suspended;

/* Error counters */
static atomic_t diversity_err_uarte0_framing;
static atomic_t diversity_err_uarte0_other;
static atomic_t diversity_err_uarte1_recovered;
static atomic_t diversity_err_both_failed;
static atomic_t diversity_err_cross_validated;  /* both clean & agree */
static atomic_t diversity_err_cross_mismatch;   /* both clean but disagree */
static atomic_t diversity_byte_count;

/* Calibrated baud registers */
static uint32_t diversity_baud_center;
static uint32_t diversity_baud_fast;
static uint32_t diversity_baud_slow;

/* Saved UARTE0 DMA config (captured at init, restored after reset) */
static uint32_t uarte0_rxd_ptr;
static uint32_t uarte0_rxd_maxcnt;

/* Deferred CLK calibration */
static const struct device *cal_dev;   /* set once during init */
static int deferred_cal_retries;       /* retries remaining */
static void ps2_uart_deferred_cal_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(deferred_cal_work, ps2_uart_deferred_cal_handler);
#define PS2_UART_DEFERRED_CAL_DELAY_MS 5000
#define PS2_UART_DEFERRED_CAL_RETRY_MS 3000
#define PS2_UART_DEFERRED_CAL_MAX_RETRIES 3

/* Dedicated work queue for calibration — avoids blocking sysworkq
 * during the ~800ms busy-wait measurement loop. */
static K_THREAD_STACK_DEFINE(cal_wq_stack, 1024);
static struct k_work_q cal_wq;
static bool cal_wq_started;

/* Periodic stats logging */
static void diversity_stats_work_handler(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(diversity_stats_work, diversity_stats_work_handler);
#define PS2_UART_DIVERSITY_STATS_INTERVAL_MS 10000

#if IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION)
#include <mpsl_timeslot.h>
#include <mpsl.h>
#include <hal/nrf_timer.h>
#include <mpsl_hwres.h>
#include <ps2_uart_timeslot.h>
#endif /* IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION) */

#define LOG_LEVEL CONFIG_PS2_LOG_LEVEL
LOG_MODULE_REGISTER(ps2_uart);

/*
 * Pin Control
 */

// PINCTRL_DT_DEFINE(DT_INST_BUS(0)); // moved to PS2_UART_DEFINE(n)

/*
 * Settings
 */

#define PS2_UART_WRITE_MAX_RETRY 3
#define PS2_UART_READ_MAX_RETRY 3

#define PS2_UART_DATA_QUEUE_SIZE 100

// Size of the callback ring buffer.
// PS/2 mouse sends 3-byte packets at ~80Hz (240 bytes/sec).
// At ~14.4kHz baud, bytes arrive ~694us apart. A depth of 16 covers
// ~5 full packets of slack between ISR and work queue servicing,
// providing margin during BLE radio events that can preempt the
// work queue for >5ms.
#define PS2_UART_CALLBACK_QUEUE_SIZE 16

// Custom queue for background PS/2 processing work at low priority
// We purposefully want this to be a fairly low priority, because
// this queue is used while we wait to start a write.
// If the system is very busy with interrupts and other threads, then we
// want to wait until that is over so that our write interrupts don't get
// missed.
#define PS2_UART_WORK_QUEUE_PRIORITY 10
#define PS2_UART_WORK_QUEUE_STACK_SIZE 1024

// Custom queue for calling the zephyr ps/2 callback.
// We don't want to hand it off to that API in an ISR since that callback
// could be using blocking functions.
// Priority must be BELOW the BLE host stack (BT_RX_PRIO=8) to avoid
// starving BLE connection event processing, which causes typing lag on
// split peripherals that multiplex TP + keys over the same BLE link.
// Priority 9 sits just below BT_RX and just above the write queue (10).
#define PS2_UART_WORK_QUEUE_CB_PRIORITY 9
#define PS2_UART_WORK_QUEUE_CB_STACK_SIZE 2048

/*
 * PS/2 Defines
 */

#define PS2_UART_POS_START 0
#define PS2_UART_POS_DATA_FIRST 1
#define PS2_UART_POS_DATA_LAST 8
#define PS2_UART_POS_PARITY 9
#define PS2_UART_POS_STOP 10
#define PS2_UART_POS_ACK 11 // Write mode only

#define PS2_UART_RESP_ACK 0xfa
#define PS2_UART_RESP_RESEND 0xfe
#define PS2_UART_RESP_FAILURE 0xfc

/*
 * PS/2 Timings
 */

#define PS2_UART_TIMING_SCL_CYCLE_LEN 69

// The minimum time needed to inhibit clock to start a write
// is 100us, but we triple it just in case.
#define PS2_UART_TIMING_SCL_INHIBITION_MIN 100

// Theoretically, only 100us is required, but practically, trackpoints
// need more than the bare minimum for reliable detection.  The SK8707's
// 14.9 kHz internal clock (67µs/cycle) means 200µs guarantees 2–3
// full sampling opportunities.  Previous value was 5× (500µs) based
// on USB-to-PS/2 adapter behaviour; 2× is the safe minimum for the
// SK8707 and saves ~600µs CPU spin per write byte.
#define PS2_UART_TIMING_SCL_INHIBITION (2 * PS2_UART_TIMING_SCL_INHIBITION_MIN)

// PS2 uses a frequency between 10 kHz and 16.7 kHz. So clocks should arrive
// within 60-100us.
#define PS2_UART_TIMING_SCL_CYCLE_MIN 60
#define PS2_UART_TIMING_SCL_CYCLE_MAX 100

// After inhibiting and releasing the clock, the device starts sending
// the clock.  PS/2 spec allows up to 15ms (Chapweske §Host-to-Device,
// time (a)), but the TP normally responds in <1ms.  Using the full
// 15ms burns the per-byte retry budget on genuine NACKs.  5ms gives
// 50× the typical response time while keeping retries responsive.
#define PS2_UART_TIMING_SCL_INHIBITION_RESP_MAX 5000
#define PS2_UART_TIMEOUT_WRITE_SCL_START K_USEC(PS2_UART_TIMING_SCL_INHIBITION_RESP_MAX)

// Max time we allow the device to send the next clock signal during writes.
// Even though PS/2 devices send the clock at most every 100us, it doesn't mean
// that the interrupts always get triggered within that time. So we allow a
// little extra time.
#define PS2_UART_TIMEOUT_WRITE_SCL K_USEC(PS2_UART_TIMING_SCL_CYCLE_MAX + 50)

// Writes start with us inhibiting the line and then respond
// with 11 bits (start bit included in inhibition time).
// To be conservative we give it another 2 cycles to complete
#define PS2_UART_TIMING_WRITE_MAX_TIME                                                             \
    (PS2_UART_TIMING_SCL_INHIBITION + PS2_UART_TIMING_SCL_INHIBITION_RESP_MAX +                    \
     11 * PS2_UART_TIMING_SCL_CYCLE_MAX + 2 * PS2_UART_TIMING_SCL_CYCLE_MAX)

// Reads are 11bit and we give it another 2 cycles to start and stop
#define PS2_UART_TIMING_READ_MAX_TIME                                                              \
    (11 * PS2_UART_TIMING_SCL_CYCLE_MAX + 2 * PS2_UART_TIMING_SCL_CYCLE_MAX)

// Timeout for write_byte_await_response()
// PS/2 spec says device must respond within 20ms.  50ms gives
// 2.5× margin.  (Was 300ms pre-timeslot, when BLE ISRs could
// delay UART response processing by tens of ms.)
#define PS2_UART_TIMEOUT_WRITE_AWAIT_RESPONSE K_MSEC(50)

/*
 * Driver Defines
 */

// Timeout for blocking read using the zephyr PS2 ps2_read() function
// This is not a matter of PS/2 timings, but a preference of how long we let
// the user wait until we give up on reading.
#define PS2_UART_TIMEOUT_READ K_SECONDS(2)

// Timeout for write_byte_blocking()
#define PS2_UART_TIMEOUT_WRITE_BLOCKING K_USEC(PS2_UART_TIMING_WRITE_MAX_TIME)

/*
 * Global Variables
 */

// Used to keep track of blocking write status
typedef enum {
    PS2_UART_WRITE_STATUS_INACTIVE,
    PS2_UART_WRITE_STATUS_ACTIVE,
    PS2_UART_WRITE_STATUS_SUCCESS,
    PS2_UART_WRITE_STATUS_FAILURE,
} ps2_uart_write_status;

struct ps2_uart_data_queue_item {
    uint8_t byte;
};

struct ps2_uart_config {
    int ps2_uart_idx;

    const struct device *uart_dev;
    struct gpio_dt_spec scl_gpio;
    struct gpio_dt_spec sda_gpio;
    const struct pinctrl_dev_config *pcfg;

    int scl_gpio_port_num;
    int sda_gpio_port_num;
};

struct ps2_uart_data {
    const struct device *dev;

    // SCL GPIO callback for writing
    struct gpio_callback scl_cb_data;

    // PS2 driver interface callback
    struct k_work callback_work;
    struct k_msgq callback_msgq;
    uint8_t callback_msgq_buffer[PS2_UART_CALLBACK_QUEUE_SIZE];
    ps2_callback_t callback_isr;
#if IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK)
    ps2_resend_callback_t resend_callback_isr;
#endif /* IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK) */

    bool callback_enabled;

    // Queue for ps2_read()
    struct k_msgq data_queue;
    char data_queue_buffer[PS2_UART_DATA_QUEUE_SIZE * sizeof(struct ps2_uart_data_queue_item)];

    // Write byte
    ps2_uart_write_status cur_write_status;
    uint8_t cur_write_byte;
    int cur_write_pos;
    volatile bool write_awaits_resp;
    uint8_t write_awaits_resp_byte;
    struct k_sem write_awaits_resp_sem;
    struct k_sem write_lock;
    struct k_work_delayable write_scl_timout;
    atomic_t write_cleanup_done;

    struct k_work resend_cmd_work;

    void *scl_rupt_async;
    void *scl_rupt_blocking;
};

K_THREAD_STACK_DEFINE(ps2_uart_work_queue_stack_area, PS2_UART_WORK_QUEUE_STACK_SIZE);
static struct k_work_q ps2_uart_work_queue;

K_THREAD_STACK_DEFINE(ps2_uart_work_queue_cb_stack_area, PS2_UART_WORK_QUEUE_CB_STACK_SIZE);
static struct k_work_q ps2_uart_work_queue_cb;

/*
 * MPSL Timeslot Protection
 */

#if IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION)

// Per-byte timeslot: covers a single PS/2 write (~5.4ms) with margin.
// TIMER0 fires 500µs before end to cleanly return ACTION_END.
#define PS2_UART_TIMESLOT_LENGTH_US 8000
#define PS2_UART_TIMESLOT_TIMER_EXPIRY_US (PS2_UART_TIMESLOT_LENGTH_US - 500)

// Batch timeslot: covers an entire settings batch (~40 writes, ~80ms).
// Uses the MPSL maximum of 100ms so all register writes are protected
// by a single timeslot with no inter-write radio gaps.
#define PS2_UART_TIMESLOT_BATCH_LENGTH_US 100000
#define PS2_UART_TIMESLOT_BATCH_TIMER_EXPIRY_US \
    (PS2_UART_TIMESLOT_BATCH_LENGTH_US - 500)

// How long to wait for MPSL to schedule the timeslot before giving up
// and writing unprotected (2 BLE connection intervals at 7.5ms).
#define PS2_UART_TIMESLOT_TIMEOUT_US 15000

// Batch timeslot timeout: allow more time — the scheduler may need
// to wait for a long-enough gap in the radio schedule.
#define PS2_UART_TIMESLOT_BATCH_TIMEOUT_US 50000

// Polling interval while waiting for timeslot grant
#define PS2_UART_TIMESLOT_POLL_INTERVAL_US 10

// Max poll iterations as safety net (timeout_us / poll_interval_us)
#define PS2_UART_TIMESLOT_MAX_POLL_ITERS \
    (PS2_UART_TIMESLOT_TIMEOUT_US / PS2_UART_TIMESLOT_POLL_INTERVAL_US)

#define PS2_UART_TIMESLOT_BATCH_MAX_POLL_ITERS \
    (PS2_UART_TIMESLOT_BATCH_TIMEOUT_US / PS2_UART_TIMESLOT_POLL_INTERVAL_US)

// Atomic flags for signal handler → thread communication.
// Signal handler runs at priority 0, cannot use kernel APIs.
static atomic_t ts_started = ATOMIC_INIT(0);
static atomic_t ts_blocked = ATOMIC_INIT(0);
static atomic_t ts_force_end = ATOMIC_INIT(0);
static atomic_t ts_session_idle = ATOMIC_INIT(1);

// Batch refcount: when > 0, write_byte_blocking() skips per-byte
// timeslot acquire/release and relies on the batch timeslot.
// Supports nesting — inner batch_begin() increments, inner
// batch_end() decrements. Timeslot is only released at 0.
// If the batch timeslot expires mid-batch (ts_started goes to 0),
// individual writes fall back to per-byte protection.
static atomic_t ts_batch_active = ATOMIC_INIT(0);

// The timeslot length granted by MPSL for the current session.
// Set before each request so the signal handler uses the correct
// TIMER0 expiry value.  Updated atomically from the thread side
// before mpsl_timeslot_request() — safe because the signal handler
// only reads it after SIGNAL_START, which comes after the request.
static uint32_t ts_current_timer_expiry_us;

static mpsl_timeslot_signal_return_param_t ts_return_param;
static mpsl_timeslot_session_id_t ts_session_id;
static bool ts_session_open;

static mpsl_timeslot_request_t ts_request_earliest = {
    .request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
    .params.earliest = {
        .hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE,
        .priority = MPSL_TIMESLOT_PRIORITY_NORMAL,
        .length_us = PS2_UART_TIMESLOT_LENGTH_US,
        .timeout_us = PS2_UART_TIMESLOT_TIMEOUT_US,
    },
};

static mpsl_timeslot_request_t ts_request_batch = {
    .request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST,
    .params.earliest = {
        .hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE,
        .priority = MPSL_TIMESLOT_PRIORITY_HIGH,
        .length_us = PS2_UART_TIMESLOT_BATCH_LENGTH_US,
        .timeout_us = PS2_UART_TIMESLOT_BATCH_TIMEOUT_US,
    },
};

#endif /* IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION) */

/*
 * Function Definitions
 */

int ps2_uart_write_byte(const struct device *dev, uint8_t byte);

/*
 * Helpers functions
 */

#define PS2_UART_GET_BIT(data, bit_pos) ((data >> bit_pos) & 0x1)

int ps2_uart_get_scl(const struct device *dev) {
    const struct ps2_uart_config *config = dev->config;
    int rc = gpio_pin_get_dt(&config->scl_gpio);

    return rc;
}

int ps2_uart_get_sda(const struct device *dev) {
    const struct ps2_uart_config *config = dev->config;
    int rc = gpio_pin_get_dt(&config->sda_gpio);

    return rc;
}

void ps2_uart_set_scl(const struct device *dev, int state) {
    const struct ps2_uart_config *config = dev->config;

    gpio_pin_set_dt(&config->scl_gpio, state);
}

void ps2_uart_set_sda(const struct device *dev, int state) {
    const struct ps2_uart_config *config = dev->config;

    // LOG_INF("Seting sda to %d", state);
    gpio_pin_set_dt(&config->sda_gpio, state);
}

int ps2_uart_configure_pin_scl(const struct device *dev, gpio_flags_t flags, const char *descr) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    err = gpio_pin_configure_dt(&config->scl_gpio, flags);
    if (err) {
        LOG_ERR("failed to configure SCL GPIO pin to %s (err %d)", descr, err);
    }

    return err;
}

int ps2_uart_configure_pin_scl_input(const struct device *dev) {
    return ps2_uart_configure_pin_scl(dev, (GPIO_INPUT), "input");
}

int ps2_uart_configure_pin_scl_output(const struct device *dev) {
    return ps2_uart_configure_pin_scl(dev, (GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN), "output");
}

int ps2_uart_configure_pin_sda(const struct device *dev, gpio_flags_t flags, char *descr) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    err = gpio_pin_configure_dt(&config->sda_gpio, flags);
    if (err) {
        LOG_ERR("failed to configure SDA GPIO pin to %s (err %d)", descr, err);
    }

    return err;
}

int ps2_uart_configure_pin_sda_input(const struct device *dev) {
    return ps2_uart_configure_pin_sda(dev, (GPIO_INPUT), "input");
}

int ps2_uart_configure_pin_sda_output(const struct device *dev) {
    return ps2_uart_configure_pin_sda(dev, (GPIO_OUTPUT_HIGH | GPIO_OPEN_DRAIN), "output");
}

int ps2_uart_set_scl_callback_enabled(const struct device *dev, bool enabled) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    // LOG_INF("Setting ps2_uart_set_scl_callback_enabled: %d", enabled);

    if (enabled) {
        err = gpio_pin_interrupt_configure_dt(&config->scl_gpio, (GPIO_INT_EDGE_FALLING));
        if (err) {
            LOG_ERR("failed to enable interrupt on "
                    "SCL GPIO pin (err %d)",
                    err);
            return err;
        }
    } else {
        err = gpio_pin_interrupt_configure_dt(&config->scl_gpio, (GPIO_INT_DISABLE));
        if (err) {
            LOG_ERR("failed to disable interrupt on "
                    "SCL GPIO pin (err %d)",
                    err);
            return err;
        }
    }

    return err;
}

/*
 * Bus inhibit / release — used by idle PM to prevent the TP from
 * clocking data during UART pin transitions.  Host drives CLK LOW
 * to inhibit; releases to GPIO input (idle HIGH via external pull-up)
 * to allow TP transmissions again.
 *
 * CRITICAL: Configure as GPIO_OUTPUT_LOW, not GPIO_OUTPUT_HIGH.
 * If the TP is mid-clock and driving CLK LOW (open-drain) when we
 * switch to OUTPUT_HIGH (push-pull), the nRF overcomes the TP and
 * forces a LOW→HIGH→LOW glitch — a complete spurious CLK cycle.
 * The TP latches whatever DATA is at that moment, potentially
 * accumulating bits of a ghost PS/2 write command across multiple
 * inhibit/release cycles.  GPIO_OUTPUT_LOW enables the output
 * driver already driving LOW, matching the TP's driven state — no
 * glitch, no spurious edge.
 */
void ps2_uart_inhibit_bus(const struct device *dev) {
    ps2_uart_configure_pin_scl(dev, GPIO_OUTPUT_LOW | GPIO_OPEN_DRAIN, "output-low (inhibit)");
    k_busy_wait(100);  /* PS/2 spec: host must hold CLK LOW ≥100µs */
}

void ps2_uart_release_bus(const struct device *dev) {
    ps2_uart_configure_pin_scl_input(dev);
}

/*
 * Consolidated idle PM suspend.
 *
 * Stops UARTE1 diversity receiver, then suspends UARTE0 via Zephyr PM.
 * Caller MUST have called ps2_uart_inhibit_bus() before this and
 * MUST call ps2_uart_release_bus() when appropriate afterward.
 *
 * Ordering rationale:
 *   1. diversity_stop_rx() — disables UARTE1 and disconnects PSEL.RXD
 *      so P0.17 is free for the wake GPIO interrupt.
 *   2. PM SUSPEND — Zephyr STOPRX + disable + sleep pinctrl on UARTE0.
 *
 * Returns 0 on success, negative errno on failure.
 */
int ps2_uart_pm_suspend(const struct device *dev, const struct device *uart_dev) {
    uart_suspended = true;
    ps2_uart_diversity_stop_rx();
    k_work_cancel_delayable(&diversity_stats_work);

    int err = pm_device_action_run(uart_dev, PM_DEVICE_ACTION_SUSPEND);
    if (err && err != -EALREADY) {
        LOG_WRN("ps2_uart_pm_suspend: UART suspend failed (%d)", err);
        return err;
    }

    return 0;
}

/*
 * Consolidated idle PM resume.
 *
 * Resumes UARTE0 via Zephyr PM, restores the error interrupt that PM
 * doesn't save/restore, restarts UARTE1 diversity receiver, and purges
 * the data queue of any stale bytes from the dormant drain window.
 *
 * Caller MUST have called ps2_uart_inhibit_bus() before this.
 * CLK stays inhibited on return — both UARTEs are armed but the TP
 * cannot transmit.  This prevents stale movement bytes from landing
 * in the data queue before the caller can issue verify reads.
 *
 * Ordering rationale:
 *   1. PM RESUME — Zephyr pinctrl DEFAULT + enable + STARTRX on UARTE0.
 *   2. uart_irq_err_enable() — Zephyr PM doesn't save/restore error
 *      interrupts (only ENDRX is preserved).
 *   3. diversity_start_rx() — reconnects P0.17 to UARTE1, enables, STARTRX.
 *   4. data_queue_empty() — purges bytes from the 5ms drain window
 *      (phase 1 → phase 2).  CLK is inhibited so no new bytes can arrive.
 *
 * Retries PM RESUME up to 3 times (500µs between attempts).
 *
 * Returns 0 on success, negative errno if RESUME failed after all retries.
 */
int ps2_uart_pm_resume(const struct device *dev, const struct device *uart_dev) {
    int err = -EAGAIN;
    for (int attempt = 0; attempt < 3; attempt++) {
        err = pm_device_action_run(uart_dev, PM_DEVICE_ACTION_RESUME);
        if (err == 0 || err == -EALREADY) {
            break;
        }
        LOG_WRN("ps2_uart_pm_resume: attempt %d/3 failed (%d)", attempt + 1, err);
        k_busy_wait(500);
    }
    if (err && err != -EALREADY) {
        LOG_ERR("ps2_uart_pm_resume: failed after 3 attempts (%d)", err);
        return err;
    }

    uart_suspended = false;

    /* Restore UART error interrupt — Zephyr PM doesn't save/restore it */
    uart_irq_err_enable(uart_dev);

    /* Restart diversity receiver now that UARTE0 is active */
    ps2_uart_diversity_start_rx();

    /* Restart periodic diversity stats logging if diversity is active */
    if (uarte1_initialized) {
        k_work_schedule(&diversity_stats_work,
                        K_MSEC(PS2_UART_DIVERSITY_STATS_INTERVAL_MS));
    }

    /* Purge stale bytes from the data queue.  During the 5ms drain
     * window (dormant phase 1 → phase 2), the UART was still running
     * but the callback was disabled — any TP movement bytes went into
     * the data queue.  CLK is inhibited so no new bytes can arrive. */
    ps2_uart_data_queue_empty(dev);

    return 0;
}

/* Diversity receiver forward declarations */
static int ps2_uart_diversity_check_err(uint32_t errorsrc);
void ps2_uart_diversity_stop_rx(void);
void ps2_uart_diversity_start_rx(void);

static int ps2_uart_set_mode_read(const struct device *dev) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    /* Inhibit CLK before any DATA pin transitions.  The steps below
     * (pinctrl DEFAULT, diversity_start) change P0.17's
     * electrical state.  Without CLK inhibit the TP could clock
     * during these transitions and interpret glitches as a
     * host-initiated PS/2 write, corrupting registers.
     *
     * Use GPIO_OUTPUT_LOW to avoid a HIGH→LOW glitch that would
     * create a spurious CLK cycle (see inhibit_bus comment). */
    ps2_uart_configure_pin_scl(dev, GPIO_OUTPUT_LOW | GPIO_OPEN_DRAIN, "output-low (inhibit)");
    k_busy_wait(100);  /* PS/2 spec: host must hold CLK LOW ≥100µs */

    // Set the SDA pin for the uart device
    err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
    if (err < 0) {
        LOG_ERR("Could not switch pinctrl state to DEFAULT: %d", err);
        /* Release CLK even on error to avoid bus lockup */
        ps2_uart_configure_pin_scl_input(dev);
        return err;
    }

    // Make sure SCL interrupt is disabled (will be re-enabled by
    // release below when SCL goes back to input mode)
    ps2_uart_set_scl_callback_enabled(dev, false);

    // Re-arm UARTE DMA for receiving the next byte.
    // If STOPRX timed out in set_mode_write(), the UARTE was fully
    // reset (disable/enable) and PTR/MAXCNT were restored there.
    // If STOPRX succeeded normally, PTR/MAXCNT survive STOPRX.
    // Either way, DMA config is valid — just clear stale events.
    nrf_uarte_event_clear(NRF_UARTE0, NRF_UARTE_EVENT_ENDRX);

    // Clear any stale ERRORSRC bits left over from a previous framing
    // error whose byte was never stored to DMA (so uart_err_check was
    // never called).  Without this, the next valid byte would inherit
    // the stale error and be misclassified/dropped.
    nrf_uarte_errorsrc_get_and_clear(NRF_UARTE0);

    nrf_uarte_task_trigger(NRF_UARTE0, NRF_UARTE_TASK_STARTRX);

    // Restart diversity receiver
    ps2_uart_diversity_start_rx();

    // Enable UART interrupt
    uart_irq_rx_enable(config->uart_dev);

    /* Release CLK — UART pins are stable, TP can transmit again. */
    ps2_uart_configure_pin_scl_input(dev);

    return err;
}

static int ps2_uart_set_mode_write(const struct device *dev) {
    const struct ps2_uart_config *config = dev->config;
    int err;

    /* Inhibit CLK before any DATA pin transitions.  The steps below
     * (STOPRX, diversity_stop, pinctrl SLEEP, GPIO reconfig) all
     * change the electrical state of P0.17 (DATA).  Without CLK
     * inhibit the TP could clock during these transitions and
     * interpret the glitches as a host-initiated PS/2 write,
     * corrupting registers (e.g. config byte 0x2C).
     *
     * CLK stays inhibited here — write_byte_start() takes over
     * CLK control for the actual PS/2 host write protocol.
     *
     * Use GPIO_OUTPUT_LOW to avoid a HIGH→LOW glitch that would
     * create a spurious CLK cycle (see inhibit_bus comment). */
    ps2_uart_set_scl_callback_enabled(dev, false);
    ps2_uart_configure_pin_scl(dev, GPIO_OUTPUT_LOW | GPIO_OPEN_DRAIN, "output-low (inhibit)");
    k_busy_wait(100);  /* PS/2 spec: host must hold CLK LOW ≥100µs */

    // Cleanly stop UARTE RX DMA before disconnecting the pin.
    // Without this, pinctrl_apply_state(SLEEP) can yank the RX pin while
    // DMA is mid-byte, permanently stalling the UARTE receiver.
    // Sequence modeled on uarte_pm_suspend() in uart_nrfx_uarte.c.
    if (nrf_uarte_event_check(NRF_UARTE0, NRF_UARTE_EVENT_RXSTARTED)) {
        uart_irq_rx_disable(config->uart_dev);
        nrf_uarte_task_trigger(NRF_UARTE0, NRF_UARTE_TASK_STOPRX);

        // Busy-wait for RXTO (receiver fully stopped).  After STOPRX
        // the UARTE runs an internal timer clocked at the baud rate,
        // counting 4 byte-times before firing RXTO.  At ~14.8 kHz:
        // 4 × (11 bits / 14800) ≈ 3 ms.  Use 4 ms for 33% margin.
        int timeout_us = 4000;
        while (!nrf_uarte_event_check(NRF_UARTE0, NRF_UARTE_EVENT_RXTO) &&
               timeout_us > 0) {
            k_busy_wait(2);
            timeout_us -= 2;
        }
        if (timeout_us <= 0) {
            /* STOPRX failed — CLK is inhibited so the UARTE is stuck
             * mid-byte with no more clock edges to finish.  The only
             * way to guarantee a clean state machine is a full
             * disable/enable cycle.  This resets all internal FIFO
             * and DMA state.  We restore RXD.PTR/MAXCNT (cleared by
             * the cycle) so set_mode_read()'s STARTRX works. */
            LOG_WRN("STOPRX: timed out — resetting UARTE0");
            uint32_t saved_baud = NRF_UARTE0->BAUDRATE;
            nrf_uarte_disable(NRF_UARTE0);
            nrf_uarte_enable(NRF_UARTE0);
            NRF_UARTE0->BAUDRATE = saved_baud;
            NRF_UARTE0->RXD.PTR = uarte0_rxd_ptr;
            NRF_UARTE0->RXD.MAXCNT = uarte0_rxd_maxcnt;
        }

        nrf_uarte_event_clear(NRF_UARTE0, NRF_UARTE_EVENT_RXSTARTED);
        nrf_uarte_event_clear(NRF_UARTE0, NRF_UARTE_EVENT_RXTO);
        nrf_uarte_event_clear(NRF_UARTE0, NRF_UARTE_EVENT_ENDRX);
    } else {
        uart_irq_rx_disable(config->uart_dev);
    }

    // Stop diversity receiver before disconnecting pin
    ps2_uart_diversity_stop_rx();

    // Now safe to disconnect the RX pin — DMA is fully stopped
    err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
    if (err < 0) {
        LOG_ERR("Could not switch pinctrl state to OFF: %d", err);
        return err;
    }

    // Configure data line for output (CLK already configured above)
    ps2_uart_configure_pin_sda_output(dev);

    return err;
}

void log_binary(uint8_t value) {
    char binary_str[9];

    for (int i = 7; i >= 0; --i) {
        binary_str[7 - i] = PS2_UART_GET_BIT(value, i) ? '1' : '0';
    }

    binary_str[8] = '\0';

    LOG_INF("Binary Value of 0x%x: %s", value, binary_str);
}

bool ps2_uart_get_byte_parity(uint8_t byte) {
    int byte_parity = __builtin_parity(byte);

    // gcc parity returns 1 if there is an odd number of bits in byte
    // But the PS2 protocol sets the parity bit to 0 if there is an odd number
    return !byte_parity;
}

int ps2_uart_data_queue_get_next(const struct device *dev, uint8_t *dst_byte, k_timeout_t timeout) {
    struct ps2_uart_data *data = dev->data;
    struct ps2_uart_data_queue_item queue_data;
    int ret;

    ret = k_msgq_get(&data->data_queue, &queue_data, timeout);
    if (ret != 0) {
        LOG_WRN("Data queue timed out...");
        return -ETIMEDOUT;
    }

    *dst_byte = queue_data.byte;

    return 0;
}

void ps2_uart_data_queue_empty(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;

    k_msgq_purge(&data->data_queue);
}

void ps2_uart_data_queue_add(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;

    int ret;

    struct ps2_uart_data_queue_item queue_data;
    queue_data.byte = byte;

    LOG_DBG("Adding byte to data queue: 0x%x", byte);

    for (int i = 0; i < 2; i++) {
        ret = k_msgq_put(&data->data_queue, &queue_data, K_NO_WAIT);
        if (ret == 0) {
            break;
        } else {
            LOG_WRN("Data queue full. Removing oldest item.");

            uint8_t tmp_byte;
            ps2_uart_data_queue_get_next(dev, &tmp_byte, K_NO_WAIT);
        }
    }

    if (ret != 0) {
        LOG_ERR("Failed to add byte 0x%x to the data queue.", byte);
    }
}

void ps2_uart_send_cmd_resend_worker(struct k_work *item) {

    // struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct ps2_uart_data *data = CONTAINER_OF(item,
                                              struct ps2_uart_data,
                                              resend_cmd_work);
    const struct device *dev = data->dev;

#if IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK)

    // Notify the PS/2 device driver that we are requesting a resend.
    // PS/2 devices don't just resend the last byte that was sent, but the
    // entire command packet, which can be multiple bytes.
    if (data->resend_callback_isr != NULL && data->callback_enabled) {

        data->resend_callback_isr(data->dev);
    }

#endif /* IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK) */

    uint8_t cmd = 0xfe;
    // LOG_DBG("Requesting resend of data with command: 0x%x", cmd);
    ps2_uart_write_byte(dev, cmd);
}

void ps2_uart_send_cmd_resend(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;

    /* Submit on the cb queue (not the inhibition-delay queue) to avoid
     * blocking the semaphore.  Safe from both ISR and thread context. */
    k_work_submit_to_queue(&ps2_uart_work_queue_cb, &data->resend_cmd_work);
}

// Extract port and pin from pinctrl to log it
// Based on the defines in zephyr/dt-bindings/pinctrl/nrf-pinctrl.h:
// https://docs.zephyrproject.org/apidoc/latest/nrf-pinctrl_8h_source.html
#define PINCTRL_PSEL_NRF_EXTRACT_PORT(x) (((x) >> 5) & 0x03)
#define PINCTRL_PSEL_NRF_EXTRACT_PIN(x) ((x) & 0x1F)
int ps2_uart_get_pinctrl_port_pin(const struct pinctrl_dev_config *pcfg, uint8_t state_id,
                                  uint8_t pin_id, int *res_port, int *res_pin) {

#if IS_ENABLED(CONFIG_SOC_SERIES_NRF52X)

    if (state_id >= pcfg->state_cnt) {
        LOG_ERR("Could not retrieve pinctrl pin config, because there is no state_id %d ",
                state_id);
        return -1;
    }

    const struct pinctrl_state *state = &pcfg->states[state_id];

    if (pin_id >= state->pin_cnt) {
        LOG_ERR("Could not retrieve pinctrl pin config, because there is no pin_id %d ", pin_id);
        return -1;
    }

    pinctrl_soc_pin_t pin_psel = state->pins[pin_id];

    *res_port = PINCTRL_PSEL_NRF_EXTRACT_PORT(pin_psel);
    *res_pin = PINCTRL_PSEL_NRF_EXTRACT_PIN(pin_psel);

    return 0;
#else

    LOG_ERR("Could not retrieve pinctrl pin config, because the code is only designed to run on "
            "nrf52, but you are running on %s",
            CONFIG_SOC_SERIES);

    *res_port = -1;
    *res_pin = -1;

    return -1;

#endif
}

/*
 * Reading PS2 data
 */
static void ps2_uart_interrupt_handler(const struct device *uart_dev, void *user_data);
void ps2_uart_read_interrupt_handler(const struct device *uart_dev, void *user_data);
static int ps2_uart_read_err_check(const struct device *dev);
void ps2_uart_read_process_received_byte(const struct device *dev, uint8_t byte);
const char *ps2_uart_read_get_error_str(int err);

static void ps2_uart_interrupt_handler(const struct device *uart_dev, void *user_data) {
    // const struct device* dev = (const struct device*)user_data;
    int err;

    err = uart_irq_update(uart_dev);
    if (err != 1) {
        LOG_ERR("uart_irq_update returned: %d", err);
        return;
    }

    while (uart_irq_rx_ready(uart_dev)) {
        ps2_uart_read_interrupt_handler(uart_dev, user_data);
    }
}

void ps2_uart_read_interrupt_handler(const struct device *uart_dev, void *user_data) {
    const struct device* dev = (const struct device*)user_data;
    uint8_t byte;

    int byte_len = uart_fifo_read(uart_dev, &byte, 1);
    if (byte_len < 1) {
        LOG_ERR("UART read failed with error: %d", byte_len);
        return;
    }

    ps2_uart_read_process_received_byte(dev, byte);
}

static int ps2_uart_read_err_check(const struct device *dev) {
    int err = uart_err_check(dev);

    /* PS/2 uses odd parity; nRF52 only supports even.  A parity
     * error from the UARTE means it expected even but got odd — which
     * IS the correct PS/2 parity.  So PARITY bit SET = no error.
     *
     * Check order matters for compound errors:
     *   OVERRUN first (always fatal — data lost in shift register),
     *   then FRAMING (recoverable via diversity receiver),
     *   then parity-absence (real PS/2 parity error).
     */
    if (err & NRF_UARTE_ERROR_OVERRUN_MASK) {
        return UART_ERROR_OVERRUN;
    } else if (err & NRF_UARTE_ERROR_FRAMING_MASK) {
        return UART_ERROR_FRAMING;
    } else if ((err & NRF_UARTE_ERROR_PARITY_MASK) == 0) {
        return UART_ERROR_PARITY;
    }

    return 0;
}

void ps2_uart_read_process_received_byte(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;
    const struct ps2_uart_config *config = dev->config;

    int err;

    LOG_DBG("UART Received: 0x%x", byte);

    atomic_inc(&diversity_byte_count);

    /* Poll UARTE1 diversity receiver (ISR-free: SHORTS re-arms DMA).
     *
     * UARTE1 fires ENDRX ~30µs before UARTE0, so by the time we
     * reach here, uarte1_dma_buf and ERRORSRC are stable.  The next
     * PS/2 byte won't overwrite uarte1_dma_buf for at least another
     * ~37µs (67µs inter-byte gap minus 30µs timing offset).
     *
     * On framing error, UARTE does NOT store the byte to DMA and does
     * NOT fire ENDRX.  ERRORSRC retains the error until we clear it.
     * The receiver stays armed — no STARTRX needed, no ENDRX to clear.
     * We detect this via EVENTS_ERROR without EVENTS_ENDRX.
     */
    bool u1_got_byte = false;
    bool u1_got_error_only = false;
    uint8_t u1_byte = 0;
    uint32_t u1_errorsrc = 0;

    if (uarte1_initialized) {
        if (nrf_uarte_event_check(NRF_UARTE1, NRF_UARTE_EVENT_ENDRX)) {
            nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_ENDRX);
            u1_byte = uarte1_dma_buf;
            u1_errorsrc = NRF_UARTE1->ERRORSRC;
            NRF_UARTE1->ERRORSRC = u1_errorsrc;  /* W1C clear */
            u1_got_byte = true;

            /* Clear any ERROR event that accompanied this ENDRX.
             * (e.g. parity mismatch — byte IS stored, ENDRX fires.) */
            nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_ERROR);
        } else if (nrf_uarte_event_check(NRF_UARTE1, NRF_UARTE_EVENT_ERROR)) {
            /* ERROR without ENDRX: framing error — byte not stored.
             * Clear ERROR + ERRORSRC so they don't leak into the next
             * byte's check.  DMA stays armed (UARTE continues). */
            nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_ERROR);
            u1_errorsrc = NRF_UARTE1->ERRORSRC;
            NRF_UARTE1->ERRORSRC = u1_errorsrc;  /* W1C clear */
            u1_got_error_only = true;
        }
    }

    err = ps2_uart_read_err_check(config->uart_dev);
    if (err != 0) {
        const char *err_str = ps2_uart_read_get_error_str(err);

        /* With δ = 1/42 the two receivers' valid-decode bands
         * overlap by 50%.  Arbitration strategy:
         *
         * UARTE0 framing + UARTE1 clean → UARTE1 recovered (fallback)
         * UARTE0 framing + UARTE1 framing → outside coverage, discard
         * UARTE0 framing + UARTE1 not ready → discard (no data bits
         *   guaranteed valid at δ=1/42 — stop bit drift is only 0.25,
         *   but without the second receiver we can't verify)
         * Non-framing error → real corruption, discard */

        if (err == UART_ERROR_FRAMING) {
            atomic_inc(&diversity_err_uarte0_framing);

            if (u1_got_byte) {
                int u1_err = ps2_uart_diversity_check_err(u1_errorsrc);

                if (u1_err == 0) {
                    /* Normal fallback: UARTE1 decoded cleanly */
                    atomic_inc(&diversity_err_uarte1_recovered);
                    LOG_DBG("Diversity: UARTE1 recovered byte 0x%02x "
                            "(UARTE0 had framing for 0x%02x)",
                            u1_byte, byte);
                    byte = u1_byte;
                    err = 0;
                } else if (u1_err == UART_ERROR_FRAMING) {
                    /* Both framing errors — TP clock outside the
                     * combined coverage band.  Discard. */
                    atomic_inc(&diversity_err_both_failed);
                    LOG_WRN("Diversity: both framing "
                            "(u0=0x%02x, u1=0x%02x), discarding",
                            byte, u1_byte);
                    return;
                } else {
                    /* UARTE1 has a non-framing error (parity, overrun).
                     * Real corruption.  Discard. */
                    atomic_inc(&diversity_err_both_failed);
                    LOG_WRN("Diversity: both failed "
                            "(u0=0x%02x framing, u1=0x%02x %s)",
                            byte, u1_byte,
                            ps2_uart_read_get_error_str(u1_err));
                    return;
                }
            } else if (u1_got_error_only) {
                /* UARTE1 also had a framing error (no byte stored).
                 * Both failed — outside coverage band. */
                atomic_inc(&diversity_err_both_failed);
                LOG_WRN("Diversity: both framing (u0=0x%02x, u1=error-only), "
                        "discarding", byte);
                return;
            } else {
                /* UARTE1 not ready — can't repair.  Discard. */
                atomic_inc(&diversity_err_both_failed);
                LOG_DBG("Diversity: UARTE0 framing, UARTE1 not ready, "
                        "discarding 0x%02x", byte);
                return;
            }
        } else {
            /* Non-framing error on UARTE0 (parity, overrun) — real
             * corruption, not baud mismatch.  Discard. */
            atomic_inc(&diversity_err_uarte0_other);
            LOG_WRN("UART RX error for byte 0x%x: %s (%d)",
                    byte, err_str, err);
            return;
        }
    } else if (u1_got_byte) {
        /* UARTE0 clean — cross-validate against UARTE1.
         * In the overlap band (|ε| < 1/42), both should agree. */
        int u1_err = ps2_uart_diversity_check_err(u1_errorsrc);

        if (u1_err == 0) {
            if (u1_byte == byte) {
                atomic_inc(&diversity_err_cross_validated);
            } else {
                /* Both decoded cleanly but disagree — indicates
                 * non-framing corruption (bit flip, noise).
                 * Prefer UARTE0 (primary, Zephyr-managed) but
                 * log for telemetry. */
                atomic_inc(&diversity_err_cross_mismatch);
                LOG_WRN("Diversity: cross-validation mismatch "
                        "(u0=0x%02x, u1=0x%02x)", byte, u1_byte);
            }
        }
        /* If UARTE1 had a framing error, that just means the TP
         * clock is in the UARTE0-only band — normal, no action. */
    }

    // If write_byte_await_response() is waiting, check whether this
    // byte is a PS/2 protocol response (ACK/NAK/ERR).  Only those
    // values wake the blocked writer.  Any other byte (e.g. movement
    // data from the TP's stream-mode buffer) is routed to the normal
    // callback / data_queue path and the writer keeps waiting for the
    // real ACK.  This matches Linux libps2's ps2_handle_ack() which
    // silently drains non-ACK bytes during command exchanges.
    //
    // Why this is correct:
    //  - PS/2 spec says the device ACKs (0xFA) each host byte before
    //    sending any data.  So 0xFA IS the ACK, never movement data.
    //  - When the host inhibits CLK for a write, the TP buffers at
    //    most one movement packet.  That packet transmits after CLK
    //    is released, BEFORE the ACK.  Without draining, the first
    //    movement byte was mistaken for the ACK and the real ACK
    //    leaked into the callback queue, corrupting packet alignment.
    //  - The 50ms timeout handles the case where the ACK never comes.
    if (data->write_awaits_resp) {
        if (byte == PS2_UART_RESP_ACK || byte == PS2_UART_RESP_RESEND ||
            byte == PS2_UART_RESP_FAILURE) {
            data->write_awaits_resp_byte = byte;
            data->write_awaits_resp = false;
            k_sem_give(&data->write_awaits_resp_sem);
            return;
        }

        // Non-protocol byte while awaiting ACK — drain it to the
        // normal receive path.  The writer stays blocked.
        LOG_DBG("write_awaits_resp: draining non-ACK 0x%02x", byte);
    }

    // If no callback is set, we add the data to a fifo queue
    // that can be read later with the read using `ps2_read`
    if (data->callback_isr != NULL && data->callback_enabled) {

        // Enqueue the byte into a ring buffer instead of a single variable
        // to prevent data loss when the work queue can't service between
        // consecutive UART interrupts (e.g. during BLE radio events).
        if (k_msgq_put(&data->callback_msgq, &byte, K_NO_WAIT) != 0) {
            LOG_WRN("Callback queue full, dropping byte 0x%x", byte);
        }
        k_work_submit_to_queue(&ps2_uart_work_queue_cb, &data->callback_work);
    } else {
        LOG_DBG("data_queue <-- 0x%02x (awaits_resp=%d, cb_en=%d)",
                byte, data->write_awaits_resp, data->callback_enabled);
        ps2_uart_data_queue_add(dev, byte);
    }
}

const char *ps2_uart_read_get_error_str(int err) {
    switch (err) {
    case UART_ERROR_OVERRUN:
        return "Overrun error";
    case UART_ERROR_PARITY:
        return "Parity error";
    case UART_ERROR_FRAMING:
        return "Framing error";
    case UART_BREAK:
        return "Break interrupt";
    case UART_ERROR_COLLISION:
        return "Collision error";
    default:
        return "Unknown error";
    }
}

void ps2_uart_read_callback_work_handler(struct k_work *work) {

    struct ps2_uart_data *data = CONTAINER_OF(work,
                                              struct ps2_uart_data,
                                              callback_work);

    // Drain all queued bytes. Multiple bytes may have arrived between
    // the ISR enqueue and work queue servicing. Processing them in a
    // batch keeps packet bytes temporally close, reducing alignment drift.
    uint8_t byte;
    while (k_msgq_get(&data->callback_msgq, &byte, K_NO_WAIT) == 0) {
        data->callback_isr(data->dev, byte);
    }
}

/*
 * Writing PS2 data
 */

int ps2_uart_write_byte_await_response(const struct device *dev, uint8_t byte);
int ps2_uart_write_byte_blocking(const struct device *dev, uint8_t byte);
int ps2_uart_write_byte_start(const struct device *dev, uint8_t byte);
void ps2_uart_write_scl_interrupt_handler(const struct device *dev, struct gpio_callback *cb,
                                          uint32_t pins);
void ps2_uart_write_scl_timeout(struct k_work *item);
void ps2_uart_write_finish(const struct device *dev, bool successful, char *descr);

// Returned when there was an error writing to the PS2 device, such
// as not getting a clock from the device or receiving an invalid
// ack bit.
#define PS2_UART_E_WRITE_TRANSMIT 1

// Returned when the semaphore times out. Theoretically this shouldn't be
// happening. But it can happen if the same thread is used for both the
// semaphore wait and the inhibition timeout.
#define PS2_UART_E_WRITE_SEM_TIMEOUT 2

// Returned when the write finished seemingly successful, but the
// device didn't send a response in time.
#define PS2_UART_E_WRITE_RESPONSE 3

// Returned when the write finished seemingly successful, but the
// device responded with 0xfe (request to resend) and we ran out of
// retry attempts.
#define PS2_UART_E_WRITE_RESEND 4

// Returned when the write finished seemingly successful, but the
// device responded with 0xfc (failure / cancel).
#define PS2_UART_E_WRITE_FAILURE 5

/*
 * MPSL Timeslot Functions
 */

#if IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION)

// Signal callback — runs at priority 0 (ZLI context).
// MUST NOT use any kernel APIs (k_sem, k_work, LOG, etc).
// Only atomic ops and direct HW register writes are safe here.
static mpsl_timeslot_signal_return_param_t *ps2_uart_timeslot_cb(
    mpsl_timeslot_session_id_t session_id, uint32_t signal_type)
{
    (void)session_id;

    switch (signal_type) {
    case MPSL_TIMESLOT_SIGNAL_START:
        // Timeslot granted. Set up TIMER0 to fire before timeslot ends
        // so we can return ACTION_END cleanly.
        // Use the expiry value set by the caller before the request —
        // this is either the per-byte or batch expiry.
        nrf_timer_cc_set(MPSL_TIMER0, NRF_TIMER_CC_CHANNEL0,
                         ts_current_timer_expiry_us);
        nrf_timer_int_enable(MPSL_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);

        // Signal the polling thread that timeslot is active
        atomic_set(&ts_started, 1);

        ts_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        return &ts_return_param;

    case MPSL_TIMESLOT_SIGNAL_TIMER0:
        // Timer expired — end the timeslot or extend if batch is active.
        nrf_timer_int_disable(MPSL_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);
        nrf_timer_event_clear(MPSL_TIMER0, NRF_TIMER_EVENT_COMPARE0);

        if (atomic_get(&ts_batch_active) > 0 &&
            !atomic_get(&ts_force_end)) {
            // Batch still running — request extension instead of ending.
            // MPSL grants extension only if no BLE activity is scheduled.
            ts_return_param.callback_action =
                MPSL_TIMESLOT_SIGNAL_ACTION_EXTEND;
            ts_return_param.params.extend.length_us =
                PS2_UART_TIMESLOT_BATCH_LENGTH_US;
            return &ts_return_param;
        }

        atomic_set(&ts_force_end, 0);
        atomic_set(&ts_started, 0);

        ts_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
        return &ts_return_param;

    case MPSL_TIMESLOT_SIGNAL_EXTEND_SUCCEEDED:
        // Extension granted — reset TIMER0 for the new window.
        nrf_timer_task_trigger(MPSL_TIMER0, NRF_TIMER_TASK_CLEAR);
        nrf_timer_cc_set(MPSL_TIMER0, NRF_TIMER_CC_CHANNEL0,
                         PS2_UART_TIMESLOT_BATCH_TIMER_EXPIRY_US);
        nrf_timer_int_enable(MPSL_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);

        ts_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        return &ts_return_param;

    case MPSL_TIMESLOT_SIGNAL_EXTEND_FAILED:
        // BLE needs the radio — end timeslot now.
        // Remaining writes will fall back to per-byte protection.
        atomic_set(&ts_started, 0);

        ts_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
        return &ts_return_param;

    case MPSL_TIMESLOT_SIGNAL_BLOCKED:
    case MPSL_TIMESLOT_SIGNAL_CANCELLED:
        // MPSL couldn't schedule the timeslot. Signal the polling thread
        // to fall through to an unprotected write.
        // The session is idle after BLOCKED/CANCELLED (the request was
        // rejected, no timeslot was started).  MPSL does NOT send
        // SESSION_IDLE after these signals — only after ACTION_END.
        // We must restore the idle flag here so the next acquire() can
        // proceed.
        atomic_set(&ts_blocked, 1);
        atomic_set(&ts_session_idle, 1);

        ts_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        return &ts_return_param;

    case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
        // Session is truly idle — safe to call mpsl_timeslot_request().
        atomic_set(&ts_session_idle, 1);
        // Return value is ignored for SESSION_IDLE.
        ts_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        return &ts_return_param;

    default:
        ts_return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
        return &ts_return_param;
    }
}

// Open a timeslot session during driver init. Called once.
static int ps2_uart_timeslot_init(void)
{
    int err;

    if (ts_session_open) {
        return 0;
    }

    err = mpsl_timeslot_session_open(ps2_uart_timeslot_cb, &ts_session_id);
    if (err) {
        LOG_ERR("MPSL timeslot session open failed: %d", err);
        return err;
    }

    ts_session_open = true;
    LOG_INF("MPSL timeslot session opened (id=%u) for PS/2 write protection",
            ts_session_id);

    return 0;
}

// Request a timeslot and poll until granted or blocked.
// Returns 0 if timeslot is active, -EBUSY if blocked/timed out.
static int ps2_uart_timeslot_acquire(void)
{
    int err;

    if (!ts_session_open) {
        return -ENODEV;
    }

    atomic_set(&ts_started, 0);
    atomic_set(&ts_blocked, 0);

    // Set the TIMER0 expiry for the per-byte timeslot length
    // before requesting — the signal handler reads this on START.
    ts_current_timer_expiry_us = PS2_UART_TIMESLOT_TIMER_EXPIRY_US;

    // Retry mpsl_timeslot_request() with backoff.  It returns
    // -NRF_EAGAIN (-35) when the session is not IDLE — which happens
    // briefly after ACTION_END (before SESSION_IDLE signal) or after
    // BLOCKED/CANCELLED (where SESSION_IDLE is never sent).
    // Rather than tracking MPSL's internal state with a flag (which
    // races), just retry the actual request.
    for (int i = 0; i < 100; i++) {
        err = mpsl_timeslot_request(ts_session_id, &ts_request_earliest);
        if (err == 0) {
            break;
        }
        if (err != -NRF_EAGAIN) {
            LOG_WRN("MPSL timeslot request error: %d", err);
            return -EBUSY;
        }
        k_yield();
    }
    if (err) {
        LOG_DBG("MPSL timeslot request gave up after 10ms");
        return -EBUSY;
    }

    // Poll until signal handler sets one of the flags.
    // k_yield() lets the CPU idle (WFI) between checks — the MPSL
    // signal fires at ISR priority and sets the atomic, after which
    // the scheduler returns us here on the next pass.
    for (int i = 0; i < PS2_UART_TIMESLOT_MAX_POLL_ITERS; i++) {
        if (atomic_get(&ts_started)) {
            return 0;
        }
        if (atomic_get(&ts_blocked)) {
            LOG_DBG("MPSL timeslot blocked");
            return -EBUSY;
        }
        k_yield();
    }

    LOG_DBG("MPSL timeslot poll timed out");
    return -ETIMEDOUT;
}

// Mark that we're done with the timeslot. TIMER0 will end it naturally.
static void ps2_uart_timeslot_end_and_wait(void)
{
    // If the timeslot already ended (EXTEND_FAILED → ACTION_END set
    // ts_started=0), skip the TIMER0 manipulation — the peripheral
    // is no longer ours.
    if (!atomic_get(&ts_started)) {
        // Timeslot already ended.  Wait briefly for ts_started to
        // settle (it's set from ISR context).
        atomic_set(&ts_force_end, 0);
        return;
    }

    // Timeslot is still active — trigger TIMER0 to end it.
    // Capture current TIMER0 counter and set CC0 just ahead of it.
    // Setting CC0=1 doesn't work because the counter has already
    // passed 1 (it's at ~37000µs for a batch).  COMPARE0 only fires
    // when the counter REACHES the CC value, not when it's past it.
    nrf_timer_task_trigger(MPSL_TIMER0, NRF_TIMER_TASK_CAPTURE1);
    uint32_t now = nrf_timer_cc_get(MPSL_TIMER0, NRF_TIMER_CC_CHANNEL1);
    nrf_timer_cc_set(MPSL_TIMER0, NRF_TIMER_CC_CHANNEL0, now + 1);
    nrf_timer_int_enable(MPSL_TIMER0, NRF_TIMER_INT_COMPARE0_MASK);

    // Wait for ACTION_END to clear ts_started.  We don't need to wait
    // for SESSION_IDLE here — the retry loop in timeslot_acquire()
    // and batch_begin() handles -NRF_EAGAIN if we request too soon.
    for (int i = 0; i < 2000; i++) {
        if (!atomic_get(&ts_started)) {
            break;
        }
        k_busy_wait(10);
    }
    atomic_set(&ts_force_end, 0);
}

static void ps2_uart_timeslot_release(void)
{
    // Tell the signal handler to return ACTION_END unconditionally,
    // even if batch_active > 0.
    atomic_set(&ts_force_end, 1);
    ps2_uart_timeslot_end_and_wait();
}

// --- Batch timeslot API (exposed via ps2_uart_timeslot.h) ---

int ps2_uart_timeslot_batch_begin(void)
{
    int err;

    if (!ts_session_open) {
        return -ENODEV;
    }

    // Nested call — the outer batch's timeslot is still active.
    // Just bump the refcount so batch_end() knows not to release yet.
    // BUT: if ts_started is 0 the outer timeslot died (extension
    // failed).  Fall through to request a fresh one.
    //
    // NOTE (benign race): Between the two atomic_get() reads, TIMER0
    // could fire and clear ts_started.  In that case we fall through
    // and request a fresh timeslot, resetting ts_batch_active — the
    // outer batch's refcount is lost, degrading to per-byte
    // protection.  A spinlock would fix this but is incompatible
    // with the MPSL signal handler context.  The worst-case effect
    // is one redundant timeslot request per batch boundary.
    if (atomic_get(&ts_batch_active) > 0 && atomic_get(&ts_started)) {
        atomic_inc(&ts_batch_active);
        return 0;
    }

    atomic_set(&ts_started, 0);
    atomic_set(&ts_blocked, 0);
    atomic_set(&ts_batch_active, 0);

    // Set the TIMER0 expiry for the batch timeslot length.
    ts_current_timer_expiry_us = PS2_UART_TIMESLOT_BATCH_TIMER_EXPIRY_US;

    // Retry mpsl_timeslot_request() with backoff — same rationale as
    // timeslot_acquire().  Returns -NRF_EAGAIN when session not IDLE.
    for (int i = 0; i < 100; i++) {
        err = mpsl_timeslot_request(ts_session_id, &ts_request_batch);
        if (err == 0) {
            break;
        }
        if (err != -NRF_EAGAIN) {
            LOG_WRN("MPSL batch timeslot request error: %d", err);
            return -EBUSY;
        }
        k_yield();
    }
    if (err) {
        LOG_DBG("MPSL batch timeslot request gave up after 10ms");
        return -EBUSY;
    }

    for (int i = 0; i < PS2_UART_TIMESLOT_BATCH_MAX_POLL_ITERS; i++) {
        if (atomic_get(&ts_started)) {
            atomic_set(&ts_batch_active, 1);  // outermost caller sets to 1
            LOG_INF("MPSL batch timeslot acquired (100ms)");
            return 0;
        }
        if (atomic_get(&ts_blocked)) {
            LOG_WRN("MPSL batch timeslot blocked — writes will use per-byte protection");
            return -EBUSY;
        }
        k_yield();
    }

    LOG_WRN("MPSL batch timeslot poll timed out — writes will use per-byte protection");
    return -ETIMEDOUT;
}

void ps2_uart_timeslot_batch_end(void)
{
    atomic_val_t prev = atomic_get(&ts_batch_active);
    if (prev <= 0) {
        return;
    }

    if (prev > 1) {
        // Inner (nested) batch_end — decrement but keep timeslot.
        atomic_dec(&ts_batch_active);
        return;
    }

    // Outermost batch_end — release the timeslot.
    // Clear batch_active FIRST so the next TIMER0 expiry returns
    // ACTION_END instead of requesting an extension.
    atomic_set(&ts_batch_active, 0);

    // End the timeslot properly and wait for SESSION_IDLE.
    ps2_uart_timeslot_end_and_wait();
    LOG_INF("MPSL batch timeslot released");
}

#endif /* IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION) */

K_MUTEX_DEFINE(ps2_uart_write_mutex);

int ps2_uart_write_byte(const struct device *dev, uint8_t byte) {
    int err;

    LOG_DBG("\n");
    LOG_DBG("Writing: 0x%x", byte);

    k_mutex_lock(&ps2_uart_write_mutex, K_FOREVER);

    for (int i = 0; i < PS2_UART_WRITE_MAX_RETRY; i++) {
        if (i > 0) {
            LOG_WRN("Attempting write re-try #%d of %d...", i + 1, PS2_UART_WRITE_MAX_RETRY);
        }

        err = ps2_uart_write_byte_await_response(dev, byte);

        if (err == 0) {
            if (i > 0) {
                LOG_WRN("Successfully wrote 0x%x on try #%d of %d...", byte, i + 1,
                        PS2_UART_WRITE_MAX_RETRY);
            }
            break;
        } else if (err == PS2_UART_E_WRITE_FAILURE) {
            // Write failed and the device requested to stop trying
            // to resend.
            break;
        }
    }

    LOG_DBG("END WRITE: 0x%x\n", byte);
    k_mutex_unlock(&ps2_uart_write_mutex);

    return err;
}

// Writes the byte and blocks execution until we read a PS/2 protocol
// response byte (0xFA ACK, 0xFE resend, or 0xFC failure).
// Non-protocol bytes (e.g. buffered movement data) are drained by
// the ISR handler and do NOT wake this function.
// Returns 0 on ACK, error code on NAK/failure/timeout.
int ps2_uart_write_byte_await_response(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;
    int err;

    // Set the response flag BEFORE the blocking write so it is already
    // armed when write_finish() re-enables UART RX.  The TP sends its
    // ACK byte ~67µs after the last write bit — without this ordering
    // the response can arrive before the flag is set and get routed to
    // the callback queue as data, causing persistent packet desync.
    // Drain any stale sem give from a previous timed-out response first.
    k_sem_reset(&data->write_awaits_resp_sem);
    data->write_awaits_resp = true;

    err = ps2_uart_write_byte_blocking(dev, byte);
    if (err) {
        data->write_awaits_resp = false;
        return err;
    }

    err = k_sem_take(&data->write_awaits_resp_sem, PS2_UART_TIMEOUT_WRITE_AWAIT_RESPONSE);

    uint8_t resp_byte = data->write_awaits_resp_byte;
    data->write_awaits_resp_byte = 0x0;
    data->write_awaits_resp = false;

    if (err) {
        LOG_WRN("Write response didn't arrive in time for byte "
                "0x%x. Considering send a failure.",
                byte);

        return PS2_UART_E_WRITE_RESPONSE;
    }

    if (resp_byte == PS2_UART_RESP_RESEND || resp_byte == PS2_UART_RESP_FAILURE) {
        LOG_WRN("Write of 0x%x received error response: 0x%x", byte, resp_byte);
    } else {
        LOG_DBG("Write for byte 0x%x received response: 0x%x", byte, resp_byte);
    }

    // We fail the write since we got an error response
    if (resp_byte == PS2_UART_RESP_RESEND) {

        return PS2_UART_E_WRITE_RESEND;
    } else if (resp_byte == PS2_UART_RESP_FAILURE) {

        return PS2_UART_E_WRITE_FAILURE;
    }

    // ACK (0xFA) — the only expected success response.
    // (Non-protocol bytes are drained by the ISR and never reach here.)
    return 0;
}

int ps2_uart_write_byte_blocking(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;
    int err;

#if IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION)
    // Check whether a batch timeslot is covering this write.
    // If the batch timeslot is active AND hasn't expired, skip
    // per-byte acquire/release — the batch provides protection.
    // If the batch was requested but the timeslot expired (TIMER0
    // fired, ts_started went to 0), fall back to per-byte acquire.
    bool in_batch = (atomic_get(&ts_batch_active) > 0 && atomic_get(&ts_started));
    int ts_err = -EBUSY;

    if (!in_batch) {
        // Per-byte timeslot acquire with exponential backoff.
        // BLE connection events typically run 1.25–7.5ms, so
        // fixed 1ms retries often land inside the same event.
        // Exponential backoff (2, 4, 8, 16ms) spans progressively
        // longer windows, giving MPSL more opportunities to fit
        // our timeslot between radio events.
        //
        // HARD GATE: if all attempts fail, we return an error
        // instead of writing unprotected. The outer write_byte()
        // retry loop will try again.
        for (int ts_attempt = 0; ts_attempt < 5; ts_attempt++) {
            ts_err = ps2_uart_timeslot_acquire();
            if (ts_err == 0) {
                break;
            }
            if (ts_attempt < 4) {
                k_msleep(2 << ts_attempt);  // 2, 4, 8, 16ms
            }
        }
        if (ts_err != 0) {
            LOG_WRN("Timeslot acquire failed after 5 attempts for "
                    "byte 0x%x — refusing unprotected write", byte);
            return PS2_UART_E_WRITE_TRANSMIT;
        }
    }
#endif

    // LOG_DBG("ps2_uart_write_byte_blocking called with byte=0x%x", byte);

    err = ps2_uart_write_byte_start(dev, byte);
    if (err) {
        LOG_ERR("Could not initiate writing of byte.");
#if IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION)
        if (!in_batch && ts_err == 0) {
            ps2_uart_timeslot_release();
        }
#endif
        return PS2_UART_E_WRITE_TRANSMIT;
    }

    // The async `write_byte_start` function takes the only available semaphor.
    // This causes the `k_sem_take` call below to block until
    // `ps2_uart_write_finish` gives it back.
    err = k_sem_take(&data->write_lock, PS2_UART_TIMEOUT_WRITE_BLOCKING);
    if (err) {

        // This usually means the controller is busy with other interrupts,
        // timed out processing the interrupts and even the scl timeout
        // delayable wasn't called due to the delay.
        //
        // So we abort the write and try again.
        LOG_ERR("Blocking write failed due to semaphore timeout for byte "
                "0x%x: %d",
                byte, err);

        // Clean up stale write state.  Use cancel_sync (not plain
        // cancel) to wait for any in-flight write_scl_timeout handler
        // on ps2_uart_work_queue to complete.  Without _sync, the
        // handler can still be running set_mode_read concurrently
        // with the cleanup below, corrupting UARTE1 PSEL state.
        // Safe: this thread (tp_mgmt_wq / sysworkq) is never the
        // same as ps2_uart_work_queue, so no deadlock.
        struct k_work_sync write_sync;
        k_work_cancel_delayable_sync(&data->write_scl_timout, &write_sync);
        ps2_uart_set_scl_callback_enabled(dev, false);

        // Single-entry guard: if write_finish already ran (the SCL
        // timeout handler completed before cancel_sync waited), it
        // already did set_mode_read + sem_give.  The CAS prevents
        // a second set_mode_read and the double sem_give.
        if (atomic_cas(&data->write_cleanup_done, 0, 1)) {
            ps2_uart_set_mode_read(dev);
        }
        data->cur_write_status = PS2_UART_WRITE_STATUS_INACTIVE;

#if IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION)
        if (!in_batch && ts_err == 0) {
            ps2_uart_timeslot_release();
        }
#endif
        return PS2_UART_E_WRITE_SEM_TIMEOUT;
    }

#if IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION)
    if (!in_batch && ts_err == 0) {
        ps2_uart_timeslot_release();
    }
#endif

    if (data->cur_write_status == PS2_UART_WRITE_STATUS_SUCCESS) {
        // LOG_DBG("Blocking write finished successfully for byte 0x%x", byte);
        err = 0;
    } else {
        LOG_ERR("Blocking write finished with failure for byte 0x%x status: %d", byte,
                data->cur_write_status);
        err = -data->cur_write_status;
    }

    data->cur_write_status = PS2_UART_WRITE_STATUS_INACTIVE;

    return err;
}

int ps2_uart_write_byte_start(const struct device *dev, uint8_t byte) {
    struct ps2_uart_data *data = dev->data;
    int err;

    // Take semaphore so that when `ps2_uart_write_byte_blocking` attempts
    // taking it, the process gets blocked.
    err = k_sem_take(&data->write_lock, K_NO_WAIT);
    if (err != 0 && err != -EBUSY) {
        LOG_ERR("ps2_uart_write_byte_start could not take semaphore: %d", err);

        return err;
    }

    /* Reset single-entry guard for this write cycle.  write_finish
     * and the sem timeout path both CAS on this to ensure exactly
     * one of them performs hardware cleanup (set_mode_read + sem give). */
    atomic_set(&data->write_cleanup_done, 0);

    err = ps2_uart_set_mode_write(dev);
    if (err != 0) {
        LOG_ERR("Could not configure driver for write mode: %d", err);
        /* Restore UART state — set_mode_write may have partially
         * executed (CLK inhibited, RX stopped, diversity stopped).
         * Without this, the UART stays in a broken state and all
         * future writes fail, permanently freezing the TP.
         * set_mode_read is the same recovery used by write_finish
         * and the semaphore timeout handler — proven safe after
         * partial set_mode_write. */
        ps2_uart_set_mode_read(dev);
        return err;
    }

    // Set the write byte so it can be used in
    // the downstream write function that is called
    // from the SCL interrupt
    data->cur_write_byte = byte;
    data->cur_write_pos = PS2_UART_POS_START;

    // Inhibit the line by setting clock low and data high for 100us
    ps2_uart_set_scl(dev, 0);
    ps2_uart_set_sda(dev, 1);
    k_busy_wait(PS2_UART_TIMING_SCL_INHIBITION);

    // Set data to value of start bit
    ps2_uart_set_sda(dev, 0);
    k_busy_wait(PS2_UART_TIMING_SCL_INHIBITION);

    // The start bit was sent by setting sda to low
    // So the next scl interrupt will be for the first
    // data bit.
    data->cur_write_pos += 1;

    // Release the clock line and configure it as input
    // This let's the device take control of the clock again
    ps2_uart_set_scl(dev, 1);
    ps2_uart_configure_pin_scl_input(dev);

    // We need to wait for the first SCL clock
    // Execution continues once it arrives in
    // `ps2_uart_write_scl_interrupt_handler`
    ps2_uart_set_scl_callback_enabled(dev, true);

    // And if the PS/2 device doesn't start the clock, we want to
    // handle that error...
    k_work_schedule_for_queue(&ps2_uart_work_queue, &data->write_scl_timout,
                              PS2_UART_TIMEOUT_WRITE_SCL_START);

    // NOTE: Do NOT unlock the mutex here. The calling thread (write_byte)
    // holds the mutex for the entire write operation and unlocks it when
    // the write completes or all retries are exhausted. write_finish()
    // signals completion via k_sem_give (ISR-safe), not the mutex.

    return 0;
}

void ps2_uart_write_scl_timeout(struct k_work *item) {

    struct k_work_delayable *work_delayable = (struct k_work_delayable *)item;
    struct ps2_uart_data *data = CONTAINER_OF(work_delayable,
                                              struct ps2_uart_data,
                                              write_scl_timout);
    const struct device *dev = data->dev;

    // Once we start a transmission we expect the device to
    // to send a new clock/interrupt within
    // PS2_UART_TIMEOUT_WRITE_SCL_START us.
    // If we don't receive the next interrupt within that timeframe,
    // we abort the write.

    ps2_uart_write_finish(dev, false, "scl timeout");
}

// The nrf52 is too slow to process all SCL interrupts, so we
// try to avoid them as much as possible.
//
// But, after we initiate the write transmission with SCL and SDA LOW,
// the PS/2 device doesn't always respond right away. It can take as
// much as 5,000us for it to start sending the clock for the
// transmission.
//
// Once it does start sending the clock the cycles are pretty
// consistently between 67 and 70us (at least on the trackpoints I
// tested).
//
// So, we use a GPIO interrupt to wait for the first clock cycle and
// then use delays to send the actual data at the same rate as the
// UART baud rate.
void ps2_uart_write_scl_interrupt_handler_blocking(struct ps2_uart_data *data,
                                                   const struct device *gpio_dev,
                                                   struct gpio_callback *cb, 
                                                   uint32_t pins) {
    LOG_INF("Inside ps2_uart_write_scl_interrupt_handler_blocking");

    // Cancel the SCL timeout
    k_work_cancel_delayable(&data->write_scl_timout);

    // Disable the SCL interrupt again.
    // From here we will just use time delays.
    ps2_uart_set_scl_callback_enabled(data->dev, false);

    for (int i = PS2_UART_POS_DATA_FIRST; i <= PS2_UART_POS_STOP; i++) {

        if (i >= PS2_UART_POS_DATA_FIRST && i <= PS2_UART_POS_DATA_LAST) {

            int data_pos = i - PS2_UART_POS_DATA_FIRST;
            bool data_bit = PS2_UART_GET_BIT(data->cur_write_byte, data_pos);

            ps2_uart_set_sda(data->dev, data_bit);
        } else if (i == PS2_UART_POS_PARITY) {

            bool byte_parity = ps2_uart_get_byte_parity(data->cur_write_byte);

            ps2_uart_set_sda(data->dev, byte_parity);
        } else if (i == PS2_UART_POS_STOP) {

            ps2_uart_set_sda(data->dev, 1);

            // Give control over data pin back to device after sending
            // the stop bit so that we can receive the ack bit from the
            // device
            ps2_uart_configure_pin_sda_input(data->dev);
        } else {
            LOG_ERR("UART unknown TX bit number: %d", i);
        }

        // Sleep for the cycle length
        k_busy_wait(PS2_UART_TIMING_SCL_CYCLE_LEN);
    }

    // Check Ack
    int ack_val = ps2_uart_get_sda(data->dev);

    if (ack_val == 0) {
        ps2_uart_write_finish(data->dev, true, "successful ack");
    } else {
        // TODO: Properly handle write ack errors
        LOG_WRN("Ack bit was invalid for write of 0x%x", data->cur_write_byte);
        ps2_uart_write_finish(data->dev, false, "failed ack");
    }
}

void ps2_uart_write_scl_interrupt_handler_async(struct ps2_uart_data *data,
                                                const struct device *gpio_dev,
                                                struct gpio_callback *cb,
                                                uint32_t pins) {
    k_work_cancel_delayable(&data->write_scl_timout);

    if (data->cur_write_pos == PS2_UART_POS_START) {
        // This should not be happening, because the PS2_UART_POS_START bit
        // is sent in ps2_uart_write_byte_start during inhibition
        return;
    } else if (data->cur_write_pos >= PS2_UART_POS_DATA_FIRST &&
               data->cur_write_pos <= PS2_UART_POS_DATA_LAST) {

        int data_pos = data->cur_write_pos - PS2_UART_POS_DATA_FIRST;
        bool data_bit = PS2_UART_GET_BIT(data->cur_write_byte, data_pos);

        ps2_uart_set_sda(data->dev, data_bit);
    } else if (data->cur_write_pos == PS2_UART_POS_PARITY) {

        bool byte_parity = ps2_uart_get_byte_parity(data->cur_write_byte);

        ps2_uart_set_sda(data->dev, byte_parity);
    } else if (data->cur_write_pos == PS2_UART_POS_STOP) {

        ps2_uart_set_sda(data->dev, 1);

        // Give control over data pin back to device after sending
        // the stop bit so that we can receive the ack bit from the
        // device
        ps2_uart_configure_pin_sda_input(data->dev);
    } else if (data->cur_write_pos == PS2_UART_POS_ACK) {

        int ack_val = ps2_uart_get_sda(data->dev);

        if (ack_val == 0) {
            ps2_uart_write_finish(data->dev, true, "successful ack");
        } else {
            // TODO: Properly handle write ack errors
            LOG_WRN("Ack bit was invalid for write of 0x%x", data->cur_write_byte);
            ps2_uart_write_finish(data->dev, false, "failed ack");
        }
    } else {
        LOG_ERR("UART unknown TX bit number: %d", data->cur_write_pos);
    }

    if (data->cur_write_pos < PS2_UART_POS_ACK) {
        k_work_schedule_for_queue(&ps2_uart_work_queue, &data->write_scl_timout,
                                  PS2_UART_TIMEOUT_WRITE_SCL);
    }

    data->cur_write_pos += 1;
}

void ps2_uart_write_finish(const struct device *dev, bool successful, char *descr) {
    struct ps2_uart_data *data = dev->data;
    int err;

    /* Disable SCL interrupt FIRST, before any state changes.  This
     * prevents a stale ACK GPIO ISR from racing with a timeout-triggered
     * write_finish: the timeout (work queue) enters here and disables
     * the interrupt, so even if a delayed ACK edge was pending it cannot
     * fire and cause a second write_finish / double k_sem_give.
     * Idempotent with the later set_mode_read() disable. */
    ps2_uart_set_scl_callback_enabled(dev, false);

    k_work_cancel_delayable(&data->write_scl_timout);

    if (successful) {
        LOG_DBG("Successfully wrote value 0x%x", data->cur_write_byte);
        data->cur_write_status = PS2_UART_WRITE_STATUS_SUCCESS;
    } else { // Failure
        LOG_ERR("Failed to write value 0x%x: %s", data->cur_write_byte, descr);

        data->cur_write_status = PS2_UART_WRITE_STATUS_FAILURE;
    }

    /* Single-entry guard: only the first caller (write_finish or
     * sem timeout) performs hardware cleanup + sem give.  The
     * sem timeout path (caller thread) and write_scl_timeout
     * (work queue thread) can race here — CAS ensures exactly
     * one runs set_mode_read and gives the semaphore. */
    if (!atomic_cas(&data->write_cleanup_done, 0, 1)) {
        /* Another path already cleaned up — nothing to do. */
        return;
    }

    err = ps2_uart_set_mode_read(dev);
    if (err != 0) {
        LOG_ERR("Could not configure driver for read mode: %d", err);
        // Fall through — MUST still give write_lock, otherwise all
        // future writes deadlock and RX stays dead (TP dies permanently).
        // The next write cycle's set_mode_write() will handle the
        // stale RX state.
    }

    LOG_DBG("END WRITE: 0x%x\n", data->cur_write_byte);

    data->cur_write_byte = 0x0;

    // Give the semaphore to allow write_byte_blocking to continue.
    // NOTE: Do NOT unlock the mutex here — write_finish is called
    // from ISR or work queue context, neither of which owns the
    // mutex. k_mutex_unlock from ISR is undefined behavior in
    // Zephyr. The calling thread (write_byte) owns the mutex and
    // unlocks it after all retries are exhausted.
    k_sem_give(&data->write_lock);
}

/*
 * Zephyr PS/2 driver interface
 */
static int ps2_uart_enable_callback(const struct device *dev);

#if IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK)

static int ps2_uart_configure(const struct device *dev, ps2_callback_t callback_isr,
                              ps2_resend_callback_t resend_callback_isr) {
    struct ps2_uart_data *data = dev->data;

    if (!callback_isr && !resend_callback_isr) {
        return -EINVAL;
    }

    if (callback_isr) {
        data->callback_isr = callback_isr;
        ps2_uart_enable_callback(dev);
    }

    if (resend_callback_isr) {
        data->resend_callback_isr = resend_callback_isr;
    }

    return 0;
}

#else

static int ps2_uart_configure(const struct device *dev, ps2_callback_t callback_isr) {
    struct ps2_uart_data *data = dev->data;

    if (!callback_isr) {
        return -EINVAL;
    }

    data->callback_isr = callback_isr;
    ps2_uart_enable_callback(dev);

    return 0;
}

#endif /* IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK) */

int ps2_uart_read(const struct device *dev, uint8_t *value) {
    uint8_t queue_byte;
    int err = ps2_uart_data_queue_get_next(dev, &queue_byte, PS2_UART_TIMEOUT_READ);
    if (err) { // Timeout due to no data to read in data queue
        // LOG_DBG("ps2_uart_read: Fifo timed out...");

        return -ETIMEDOUT;
    }

    // LOG_DBG("ps2_uart_read: Returning 0x%x", queue_byte);
    *value = queue_byte;

    return 0;
}

static int ps2_uart_write(const struct device *dev, uint8_t value) {
    int ret = ps2_uart_write_byte(dev, value);

    return ret;
}

static int ps2_uart_disable_callback(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;

    // Make sure there are no stale items in the data queue
    // from before the callback was disabled.
    ps2_uart_data_queue_empty(dev);
    k_msgq_purge(&data->callback_msgq);

    data->callback_enabled = false;

    // LOG_DBG("Disabled PS2 callback.");

    return 0;
}

static int ps2_uart_enable_callback(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;
    data->callback_enabled = true;

    // LOG_DBG("Enabled PS2 callback.");

    ps2_uart_data_queue_empty(dev);
    k_msgq_purge(&data->callback_msgq);

    return 0;
}

static const struct ps2_driver_api ps2_uart_driver_api = {
    .config = ps2_uart_configure,
    .read = ps2_uart_read,
    .write = ps2_uart_write,
    .disable_callback = ps2_uart_disable_callback,
    .enable_callback = ps2_uart_enable_callback,
};

/*
 * Dual-UARTE Diversity: CLK Calibration & UARTE1 Setup
 */

/**
 * Measure the PS/2 CLK period using GPIOTE + PPI + TIMER3.
 *
 * Captures falling-edge timestamps on P0.20 (CLK) via hardware PPI path
 * (zero CPU jitter). Returns the average CLK period in TIMER3 ticks
 * (16 MHz clock, 62.5 ns/tick), or 0 on failure.
 *
 * Resources are fully torn down after measurement.
 */
static uint32_t ps2_uart_calibrate_clk_period(const struct device *dev) {
    const struct ps2_uart_config *config = dev->config;
    uint32_t deltas[PS2_UART_CAL_EDGES];
    uint32_t prev_ts = 0;
    int count = 0;
    nrfx_err_t nerr;
    nrf_ppi_channel_t cal_ppi_ch;
    uint8_t cal_gpiote_ch;

    /* Allocate PPI channel (respects MPSL/SDC reservations) */
    nerr = nrfx_ppi_channel_alloc(&cal_ppi_ch);
    if (nerr != NRFX_SUCCESS) {
        LOG_WRN("CLK calibration: PPI channel alloc failed (0x%x)", nerr);
        return 0;
    }

    /* Allocate GPIOTE channel */
    nerr = nrfx_gpiote_channel_alloc(&nrfx_gpiote, &cal_gpiote_ch);
    if (nerr != NRFX_SUCCESS) {
        LOG_WRN("CLK calibration: GPIOTE channel alloc failed (0x%x)", nerr);
        nrfx_ppi_channel_free(cal_ppi_ch);
        return 0;
    }

    LOG_DBG("CLK calibration: allocated PPI CH%u, GPIOTE CH%u",
            (unsigned)cal_ppi_ch, cal_gpiote_ch);

    /* --- Configure TIMER3: 16 MHz, 32-bit, timer mode --- */
    nrf_timer_mode_set(NRF_TIMER3, NRF_TIMER_MODE_TIMER);
    nrf_timer_bit_width_set(NRF_TIMER3, NRF_TIMER_BIT_WIDTH_32);
    nrf_timer_prescaler_set(NRF_TIMER3, NRF_TIMER_FREQ_16MHz);
    nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_CLEAR);
    nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_START);

    /* --- Configure GPIOTE CH[n]: Event mode, falling edge on CLK pin --- */
    nrf_gpiote_event_configure(NRF_GPIOTE, cal_gpiote_ch,
                               config->scl_gpio.pin,
                               NRF_GPIOTE_POLARITY_HITOLO);
    nrf_gpiote_event_enable(NRF_GPIOTE, cal_gpiote_ch);

    /* --- Configure PPI: GPIOTE IN[n] → TIMER3 CAPTURE[0] --- */
    nrf_ppi_channel_endpoint_setup(
        NRF_PPI, cal_ppi_ch,
        nrf_gpiote_event_address_get(NRF_GPIOTE,
                                     nrf_gpiote_in_event_get(cal_gpiote_ch)),
        nrf_timer_task_address_get(NRF_TIMER3, NRF_TIMER_TASK_CAPTURE0));
    nrf_ppi_channel_enable(NRF_PPI, cal_ppi_ch);

    /* --- Sample CLK edges --- */
    for (int i = 0; i < PS2_UART_CAL_EDGES; i++) {
        nrf_gpiote_event_clear(NRF_GPIOTE,
                               nrf_gpiote_in_event_get(cal_gpiote_ch));

        int timeout_us = PS2_UART_CAL_TIMEOUT_US;
        while (!nrf_gpiote_event_check(NRF_GPIOTE,
                                       nrf_gpiote_in_event_get(cal_gpiote_ch))) {
            k_busy_wait(1);
            if (--timeout_us <= 0) {
                LOG_WRN("CLK calibration: timeout waiting for edge %d", i);
                goto teardown;
            }
        }

        uint32_t ts = nrf_timer_cc_get(NRF_TIMER3, NRF_TIMER_CC_CHANNEL0);

        if (i > 0) {
            /* Unsigned subtraction handles 32-bit wrap correctly */
            uint32_t delta = ts - prev_ts;
            if (delta >= PS2_UART_CAL_MIN_TICKS &&
                delta <= PS2_UART_CAL_MAX_TICKS) {
                deltas[count++] = delta;
            }
        }
        prev_ts = ts;
    }

teardown:
    /* --- Release all resources --- */
    nrf_ppi_channel_disable(NRF_PPI, cal_ppi_ch);
    nrf_gpiote_event_disable(NRF_GPIOTE, cal_gpiote_ch);
    nrf_gpiote_te_default(NRF_GPIOTE, cal_gpiote_ch);
    nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_STOP);
    nrf_timer_task_trigger(NRF_TIMER3, NRF_TIMER_TASK_CLEAR);
    nrfx_ppi_channel_free(cal_ppi_ch);
    nrfx_gpiote_channel_free(&nrfx_gpiote, cal_gpiote_ch);

    if (count < 4) {
        LOG_WRN("CLK calibration: only %d valid samples, need >= 4", count);
        return 0;
    }

    /* Average valid deltas */
    uint64_t sum = 0;
    for (int i = 0; i < count; i++) {
        sum += deltas[i];
    }
    uint32_t avg = (uint32_t)(sum / count);

    LOG_INF("CLK calibration: %d samples, avg period = %u ticks (%.1f kHz)",
            count, avg, 16000.0f / avg);

    return avg;
}

/**
 * Compute BAUDRATE register value from a CLK period in timer ticks.
 * TIMER3 runs at 16 MHz, so: baud = 16e6 / ticks, REG = baud * 2^32 / 16e6
 * Simplifies to: REG = 2^32 / ticks
 */
static uint32_t ps2_uart_ticks_to_baud_reg(uint32_t ticks) {
    return (uint32_t)(0x100000000ULL / ticks);
}

/**
 * Initialize the diversity receiver: schedule deferred CLK calibration.
 *
 * At boot the TP hasn't started streaming yet, so there are no CLK edges
 * to measure.  We start with the hardcoded CONFIG_PS2_UART_CUSTOM_BAUDRATE_REG
 * (the empirically-proven "magic number") and schedule a calibration attempt
 * for T+2s, by which time the TP is streaming movement data and CLK edges
 * are plentiful.
 *
 * If the deferred calibration succeeds, it computes the center baud from
 * measured CLK period, derives the fast/slow δ-spread registers, configures
 * UARTE1, and updates UARTE0's BAUDRATE.
 */
static int ps2_uart_diversity_init(const struct device *dev) {
    cal_dev = dev;
    deferred_cal_retries = PS2_UART_DEFERRED_CAL_MAX_RETRIES;

    if (!cal_wq_started) {
        k_work_queue_init(&cal_wq);
        k_work_queue_start(&cal_wq, cal_wq_stack,
                           K_THREAD_STACK_SIZEOF(cal_wq_stack),
                           K_PRIO_PREEMPT(14), NULL);
        k_thread_name_set(&cal_wq.thread, "ps2_cal");
        cal_wq_started = true;
    }

    LOG_INF("Diversity: using hardcoded baud, deferred calibration in %d ms",
            PS2_UART_DEFERRED_CAL_DELAY_MS);
    k_work_schedule_for_queue(&cal_wq, &deferred_cal_work,
                              K_MSEC(PS2_UART_DEFERRED_CAL_DELAY_MS));
    return 0;
}

/**
 * Stop UARTE1 receiver and release the RX pin.
 *
 * Called from set_mode_write() and indirectly during idle PM suspend.
 * Must fully disable UARTE1 and disconnect PSEL.RXD so the pin can be
 * reclaimed as a GPIO wake interrupt source by the idle PM layer.
 */
void ps2_uart_diversity_stop_rx(void) {
    if (!uarte1_initialized) {
        return;
    }

    /* Disable SHORTS before STOPRX to prevent the shortcut from
     * re-arming DMA during the shutdown sequence. */
    NRF_UARTE1->SHORTS = 0;

    nrf_uarte_task_trigger(NRF_UARTE1, NRF_UARTE_TASK_STOPRX);

    // Wait for RXTO: UARTE's internal 4-byte timer needs ~3 ms at
    // ~14.8 kHz baud.  Use 4 ms for 33% margin.
    int timeout_us = 4000;
    while (!nrf_uarte_event_check(NRF_UARTE1, NRF_UARTE_EVENT_RXTO) &&
           timeout_us > 0) {
        k_busy_wait(2);
        timeout_us -= 2;
    }
    if (timeout_us <= 0) {
        LOG_WRN("Diversity: UARTE1 STOPRX timed out");
    }

    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_RXTO);
    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_ENDRX);
    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_RXSTARTED);
    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_ERROR);
    NRF_UARTE1->ERRORSRC = 0x0F;  /* Clear any stale error bits */

    /* Fully disable UARTE1 and release the RX pin.
     * PSEL can only be modified while the peripheral is disabled.
     * This allows the GPIO subsystem to reclaim P0.17 for the
     * idle PM wake interrupt (falling-edge detect on DATA). */
    nrf_uarte_disable(NRF_UARTE1);
    NRF_UARTE1->PSEL.RXD = NRF_UARTE_PSEL_DISCONNECTED;
}

/**
 * Restart UARTE1 receiver and reclaim the RX pin.
 *
 * Called from set_mode_read() after UARTE0 is restarted.
 * Re-enables UARTE1 with the RX pin reconnected.
 */
void ps2_uart_diversity_start_rx(void) {
    if (!uarte1_initialized) {
        return;
    }

    /* Reconnect P0.17 and re-enable UARTE1.
     * PSEL must be set while disabled (done by stop_rx). */
    NRF_UARTE1->PSEL.RXD = 17;
    nrf_uarte_enable(NRF_UARTE1);

    /* Re-arm DMA buffer (PTR/MAXCNT cleared by disable/enable cycle) */
    NRF_UARTE1->RXD.PTR = (uint32_t)&uarte1_dma_buf;
    NRF_UARTE1->RXD.MAXCNT = 1;

    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_ENDRX);
    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_ERROR);
    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_RXSTARTED);
    NRF_UARTE1->ERRORSRC = 0x0F;

    /* Enable SHORTS before STARTRX so DMA re-arms in hardware */
    NRF_UARTE1->SHORTS = UARTE_SHORTS_ENDRX_STARTRX_Msk;

    nrf_uarte_task_trigger(NRF_UARTE1, NRF_UARTE_TASK_STARTRX);
}

/**
 * Apply the PS/2 parity trick to raw UARTE ERRORSRC bits.
 * Returns 0 if byte is good, or a UART_ERROR_* code if corrupted.
 *
 * PS/2 uses odd parity but we configure UARTE for even parity, so:
 *  - PARITY bit SET in ERRORSRC = expected (odd parity detected) = OK
 *  - PARITY bit CLEAR = actual even parity = real parity error
 *  - FRAMING/OVERRUN/BREAK = real errors regardless
 */
static int ps2_uart_diversity_check_err(uint32_t errorsrc) {
    if ((errorsrc & NRF_UARTE_ERROR_PARITY_MASK) == 0) {
        return UART_ERROR_PARITY;
    } else if (errorsrc & NRF_UARTE_ERROR_OVERRUN_MASK) {
        return UART_ERROR_OVERRUN;
    } else if (errorsrc & NRF_UARTE_ERROR_FRAMING_MASK) {
        return UART_ERROR_FRAMING;
    } else if (errorsrc & NRF_UARTE_ERROR_BREAK_MASK) {
        return UART_BREAK;
    }
    return 0;  /* No error */
}

/**
 * Deferred CLK calibration work handler.
 *
 * Runs ~2s after boot when the TP is streaming and CLK edges are available.
 * Measures CLK period, computes optimal BAUDRATE, and optionally brings up
 * the UARTE1 diversity receiver.
 */
static void ps2_uart_deferred_cal_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (cal_dev == NULL) {
        return;
    }

    /* Don't burn retries while the UART is suspended (TP dormant) —
     * no CLK edges are possible.  Re-schedule for after wake. */
    if (uart_suspended) {
        LOG_INF("Deferred calibration: UART suspended, deferring %d ms",
                PS2_UART_DEFERRED_CAL_RETRY_MS);
        k_work_schedule_for_queue(&cal_wq, &deferred_cal_work,
                                  K_MSEC(PS2_UART_DEFERRED_CAL_RETRY_MS));
        return;
    }

    uint32_t old_baud = NRF_UARTE0->BAUDRATE;

    uint32_t clk_ticks = ps2_uart_calibrate_clk_period(cal_dev);
    if (clk_ticks == 0) {
        if (deferred_cal_retries > 0) {
            deferred_cal_retries--;
            LOG_WRN("Deferred calibration: no CLK edges, retrying in %d ms "
                    "(%d retries left)",
                    PS2_UART_DEFERRED_CAL_RETRY_MS, deferred_cal_retries);
            k_work_schedule_for_queue(&cal_wq, &deferred_cal_work,
                                      K_MSEC(PS2_UART_DEFERRED_CAL_RETRY_MS));
        } else {
            LOG_WRN("Deferred calibration: no CLK edges after all retries, "
                    "keeping baud 0x%08x", old_baud);
        }
        return;
    }

    /* Compute center baud and δ-spread registers.
     * fast = round(center × 43/42), slow = round(center × 41/42). */
    diversity_baud_center = ps2_uart_ticks_to_baud_reg(clk_ticks);
    diversity_baud_fast = (diversity_baud_center * PS2_UART_DIVERSITY_NUMER_FAST
                           + PS2_UART_DIVERSITY_DENOM / 2)
                          / PS2_UART_DIVERSITY_DENOM;
    diversity_baud_slow = (diversity_baud_center * PS2_UART_DIVERSITY_NUMER_SLOW
                           + PS2_UART_DIVERSITY_DENOM / 2)
                          / PS2_UART_DIVERSITY_DENOM;

    LOG_INF("Deferred cal: center=0x%08x, fast=0x%08x, slow=0x%08x "
            "(was 0x%08x)",
            diversity_baud_center, diversity_baud_fast,
            diversity_baud_slow, old_baud);

    /* Apply calibrated slow baud to UARTE0 (primary, Zephyr-managed).
     * UARTE0 fires ENDRX ~30µs AFTER UARTE1, so by the time the
     * decision logic polls UARTE1's ENDRX event, the DMA byte and
     * ERRORSRC are stable — enabling correct cross-validation.
     *
     * Inhibit CLK while changing BAUDRATE to prevent corruption of
     * any in-flight byte. */
    ps2_uart_inhibit_bus(cal_dev);
    NRF_UARTE0->BAUDRATE = diversity_baud_slow;
    ps2_uart_release_bus(cal_dev);
    LOG_INF("Deferred cal: UARTE0 BAUDRATE 0x%08x → 0x%08x",
            old_baud, diversity_baud_slow);

    /* Bring up UARTE1 diversity receiver with the fast baud.
     * ISR-free: SHORTS ENDRX_STARTRX re-arms DMA in hardware.
     * UARTE0's ISR polls UARTE1's ENDRX event and reads the
     * DMA byte + ERRORSRC directly. */
    nrf_uarte_enable(NRF_UARTE1);
    nrf_uarte_disable(NRF_UARTE1);

    NRF_UARTE1->PSEL.RXD = 17;
    NRF_UARTE1->PSEL.TXD = NRF_UARTE_PSEL_DISCONNECTED;
    NRF_UARTE1->PSEL.CTS = NRF_UARTE_PSEL_DISCONNECTED;
    NRF_UARTE1->PSEL.RTS = NRF_UARTE_PSEL_DISCONNECTED;
    /* CONFIG: HWFC=disabled, PARITY=included (bits 1-3 = 0x7),
     * matching UARTE0's even-parity config for PS/2 odd-parity trick.
     * STOP=one (bit 4=0). */
    NRF_UARTE1->CONFIG =
        (UARTE_CONFIG_PARITY_Included << UARTE_CONFIG_PARITY_Pos);
    NRF_UARTE1->BAUDRATE = diversity_baud_fast;
    NRF_UARTE1->RXD.PTR = (uint32_t)&uarte1_dma_buf;
    NRF_UARTE1->RXD.MAXCNT = 1;

    nrf_uarte_enable(NRF_UARTE1);
    NRF_UARTE1->ERRORSRC = 0x0F;
    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_ENDRX);
    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_ERROR);
    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_RXTO);
    nrf_uarte_event_clear(NRF_UARTE1, NRF_UARTE_EVENT_RXSTARTED);

    /* Hardware re-arm: ENDRX → STARTRX shortcut, no ISR needed.
     * DMA re-arms in the same cycle as ENDRX — immune to BLE
     * preemption, zero CPU involvement per byte. */
    NRF_UARTE1->SHORTS = UARTE_SHORTS_ENDRX_STARTRX_Msk;

    nrf_uarte_task_trigger(NRF_UARTE1, NRF_UARTE_TASK_STARTRX);
    uarte1_initialized = true;

    LOG_INF("Deferred cal: UARTE1 diversity receiver up (ISR-free, SHORTS) "
            "at 0x%08x", diversity_baud_fast);

    k_work_schedule(&diversity_stats_work,
                    K_MSEC(PS2_UART_DIVERSITY_STATS_INTERVAL_MS));
}

/**
 * Periodic stats logging work handler.
 */
static void diversity_stats_work_handler(struct k_work *work) {
    atomic_val_t total = atomic_get(&diversity_byte_count);
    atomic_val_t recovered = atomic_get(&diversity_err_uarte1_recovered);
    if (total > 0 || recovered > 0) {
        LOG_INF("Diversity stats: total=%u, u0_framing=%u, u0_other=%u, "
                "u1_recovered=%u, both_failed=%u, "
                "xval_ok=%u, xval_mismatch=%u",
                (uint32_t)total,
                (uint32_t)atomic_get(&diversity_err_uarte0_framing),
                (uint32_t)atomic_get(&diversity_err_uarte0_other),
                (uint32_t)recovered,
                (uint32_t)atomic_get(&diversity_err_both_failed),
                (uint32_t)atomic_get(&diversity_err_cross_validated),
                (uint32_t)atomic_get(&diversity_err_cross_mismatch));
    }
    k_work_schedule(&diversity_stats_work,
                    K_MSEC(PS2_UART_DIVERSITY_STATS_INTERVAL_MS));
}

/*
 * PS/2 UART Driver Init
 */
static int ps2_uart_init_uart(const struct device *dev);
static int ps2_uart_init_gpio(const struct device *dev);

static int ps2_uart_init(const struct device *dev) {
    int err;
    struct ps2_uart_data *data = dev->data;
    const struct ps2_uart_config *config = dev->config;

    // Set the ps2 device so we can retrieve it later for
    // the ps2 callback
    data->dev = dev;

    // callback_msgq is properly initialized later via k_msgq_init.
    data->callback_isr = NULL;
    data->callback_enabled = false;
    data->cur_write_status = PS2_UART_WRITE_STATUS_INACTIVE;
    data->cur_write_byte = 0x0;
    data->cur_write_pos = 0;
    data->write_awaits_resp = false;
    data->write_awaits_resp_byte = 0x0;

#if IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK)
    data->resend_callback_isr = NULL;
#endif /* IS_ENABLED(CONFIG_PS2_UART_ENABLE_PS2_RESEND_CALLBACK) */

    // Get the P0.08 pin notation of the configured UART pinctrl pin.
    // The UART TX pin is the second (index 1) of the pins
    int pinctrl_port = -1;
    int pinctrl_pin = -1;
    ps2_uart_get_pinctrl_port_pin(config->pcfg, PINCTRL_STATE_DEFAULT, 1, &pinctrl_port,
                                  &pinctrl_pin);

    LOG_INF("Initializing ps2_uart driver with pins... SCL: P%d.%02d; SDA: P%d.%02d; SDA Pinctrl: "
            "P%d.%02d",
            config->scl_gpio_port_num, config->scl_gpio.pin, config->sda_gpio_port_num,
            config->sda_gpio.pin, pinctrl_port, pinctrl_pin);

    // Init data queue for synchronous read operations
    k_msgq_init(&data->data_queue, data->data_queue_buffer, sizeof(struct ps2_uart_data_queue_item),
                PS2_UART_DATA_QUEUE_SIZE);

    if (config->ps2_uart_idx == 0) {
        // Custom queue for background PS/2 processing work at high priority
        k_work_queue_start(&ps2_uart_work_queue, ps2_uart_work_queue_stack_area,
                        K_THREAD_STACK_SIZEOF(ps2_uart_work_queue_stack_area),
                        PS2_UART_WORK_QUEUE_PRIORITY, NULL);

        // Custom queue for calling the zephyr ps/2 callback at lower priority
        k_work_queue_start(&ps2_uart_work_queue_cb, ps2_uart_work_queue_cb_stack_area,
                        K_THREAD_STACK_SIZEOF(ps2_uart_work_queue_cb_stack_area),
                        PS2_UART_WORK_QUEUE_CB_PRIORITY, NULL);
    }

    k_work_init(&data->callback_work, ps2_uart_read_callback_work_handler);
    k_work_init(&data->resend_cmd_work, ps2_uart_send_cmd_resend_worker);
    k_msgq_init(&data->callback_msgq, data->callback_msgq_buffer,
                sizeof(uint8_t), PS2_UART_CALLBACK_QUEUE_SIZE);

    k_work_init_delayable(&data->write_scl_timout, ps2_uart_write_scl_timeout);

    // Init semaphore for blocking writes
    k_sem_init(&data->write_lock, 0, 1);

    // Init semaphore that waits for read after write
    k_sem_init(&data->write_awaits_resp_sem, 0, 1);

    err = ps2_uart_init_uart(dev);
    if (err != 0) {
        LOG_ERR("Could not init UART: %d", err);
        return err;
    }

    err = ps2_uart_init_gpio(dev);
    if (err != 0) {
        LOG_ERR("Could not init GPIO: %d", err);
        return err;
    }

    err = ps2_uart_set_mode_read(dev);
    if (err != 0) {
        LOG_ERR("Could not initialize in UART mode read: %d", err);
        return err;
    }

#if IS_ENABLED(CONFIG_PS2_UART_TIMESLOT_PROTECTION)
    err = ps2_uart_timeslot_init();
    if (err != 0) {
        LOG_ERR("Could not init MPSL timeslot session: %d (writes will be unprotected)", err);
        // Non-fatal: fall through to unprotected writes
    }
#endif

    return 0;
}

static int ps2_uart_init_uart(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;
    const struct ps2_uart_config *config = dev->config;
    int err;

    if (!device_is_ready(config->uart_dev)) {
        LOG_ERR("UART device not ready");
        return -ENODEV;
    } else {
        LOG_INF("UART device is ready");
    }

    struct uart_config uart_cfg;
    err = uart_config_get(config->uart_dev, &uart_cfg);
    if (err != 0) {
        LOG_ERR("Could not retrieve UART config...");
        return -ENODEV;
    }

    uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
    uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

    // PS/2 uses odd parity, but nrf52840 doesn't support
    // odd parity. Despite that, setting none works.
    uart_cfg.parity = UART_CFG_PARITY_EVEN;

    err = uart_configure(config->uart_dev, &uart_cfg);
    if (err != 0) {
        LOG_ERR("Could not configure UART device: %d", err);
        return -EINVAL;
    }

#if CONFIG_PS2_UART_CUSTOM_BAUDRATE_REG
    NRF_UARTE0->BAUDRATE = CONFIG_PS2_UART_CUSTOM_BAUDRATE_REG;
    LOG_INF("Overrode UARTE BAUDRATE register to 0x%08x", CONFIG_PS2_UART_CUSTOM_BAUDRATE_REG);
#endif

    /* Save UARTE0 DMA config — needed to restore after disable/enable
     * reset cycles in set_mode_write()'s STOPRX timeout handler. */
    uarte0_rxd_ptr = NRF_UARTE0->RXD.PTR;
    uarte0_rxd_maxcnt = NRF_UARTE0->RXD.MAXCNT;

    /* Schedule deferred CLK calibration + diversity receiver bring-up.
     * At boot the TP isn't streaming yet, so we start with the hardcoded
     * baud and calibrate once CLK edges are available (~2s). */
    err = ps2_uart_diversity_init(dev);
    if (err != 0) {
        LOG_WRN("Diversity init failed (%d), running single-UARTE mode", err);
    }

    uart_irq_callback_user_data_set(config->uart_dev, ps2_uart_interrupt_handler,
                                    (void *)data->dev);

    uart_irq_rx_enable(config->uart_dev);
    uart_irq_err_enable(config->uart_dev);

    return 0;
}

static int ps2_uart_init_gpio(const struct device *dev) {
    struct ps2_uart_data *data = dev->data;
    const struct ps2_uart_config *config = dev->config;
    int err;

    // Interrupt for clock line
#if IS_ENABLED(CONFIG_PS2_UART_WRITE_MODE_BLOCKING)
    gpio_init_callback(&data->scl_cb_data, data->scl_rupt_blocking,
                       BIT(config->scl_gpio.pin));
#else /* IS_ENABLED(CONFIG_PS2_UART_WRITE_MODE_BLOCKING) */
    gpio_init_callback(&data->scl_cb_data, data->scl_rupt_async,
                       BIT(config->scl_gpio.pin));
#endif /* IS_ENABLED(CONFIG_PS2_UART_WRITE_MODE_BLOCKING) */

    err = gpio_add_callback(config->scl_gpio.port, &data->scl_cb_data);
    if (err) {
        LOG_ERR("failed to enable interrupt callback on "
                "SCL GPIO pin (err %d)",
                err);
    }

    LOG_INF("Disabling callback...");
    ps2_uart_set_scl_callback_enabled(dev, false);

    return err;
}


// Define wrapper function declaration for write_scl_interrupt_handler_*
// will assign to ps2_uart_data in below
#define PS2_UART_SCL_INTERRUPT_DEFINE(n)                                             \
    void ps2_uart_write_scl_interrupt_handler_blocking_##n(                          \
        const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins);     \
    void ps2_uart_write_scl_interrupt_handler_async_##n(                             \
        const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins);

DT_INST_FOREACH_STATUS_OKAY(PS2_UART_SCL_INTERRUPT_DEFINE)


#define PS2_UART_DEFINE(n)                                                           \
    PINCTRL_DT_DEFINE(DT_INST_BUS(n));                                               \
    static struct ps2_uart_data data##n = {                                          \
        .scl_rupt_blocking = &ps2_uart_write_scl_interrupt_handler_blocking_##n,     \
        .scl_rupt_async = &ps2_uart_write_scl_interrupt_handler_async_##n,           \
    };                                                                               \
    static const struct ps2_uart_config config##n = {                                \
        .ps2_uart_idx = n,                                                           \
        .uart_dev = DEVICE_DT_GET(DT_INST_BUS(n)),                                   \
        .scl_gpio = GPIO_DT_SPEC_INST_GET(n, scl_gpios),                             \
        .sda_gpio = GPIO_DT_SPEC_INST_GET(n, sda_gpios),                             \
        .pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_INST_BUS(n)),                           \
        .scl_gpio_port_num = DT_PROP(DT_INST_PHANDLE(n, scl_gpios), port),           \
        .sda_gpio_port_num = DT_PROP(DT_INST_PHANDLE(n, sda_gpios), port),           \
    };                                                                               \
    DEVICE_DT_INST_DEFINE(n, &ps2_uart_init, NULL, &data##n, &config##n,             \
                          POST_KERNEL, 80,                                           \
                          &ps2_uart_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PS2_UART_DEFINE)


// Define wrapper function implementation for write_scl_interrupt_handler_*
// assigned to ps2_uart_data on above
#define PS2_UART_SCL_INTERRUPT_IMPL_DEFINE(n)                                        \
    void ps2_uart_write_scl_interrupt_handler_blocking_##n(                          \
        const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins) {    \
        ps2_uart_write_scl_interrupt_handler_blocking(&data##n, gpio_dev, cb, pins); \
    }                                                                                \
    void ps2_uart_write_scl_interrupt_handler_async_##n(                             \
        const struct device *gpio_dev, struct gpio_callback *cb, uint32_t pins) {    \
        ps2_uart_write_scl_interrupt_handler_async(&data##n, gpio_dev, cb, pins);    \
    }

DT_INST_FOREACH_STATUS_OKAY(PS2_UART_SCL_INTERRUPT_IMPL_DEFINE)
