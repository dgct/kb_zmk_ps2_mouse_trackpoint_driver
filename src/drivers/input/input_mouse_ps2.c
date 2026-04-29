/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zmk_input_mouse_ps2

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/ps2.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <ps2_uart_timeslot.h>

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM)
#include <zephyr/drivers/uart.h>
#include <zephyr/pm/device.h>
#include <zmk/activity.h>
#include <zmk/event_manager.h>

/* Diversity receiver lifecycle — defined in ps2_uart.c */
extern void ps2_uart_diversity_stop_rx(void);
extern void ps2_uart_diversity_start_rx(void);
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/position_state_changed.h>

/* Forward declaration — definition is near the end of this file */
struct zmk_mouse_ps2_data;
static void tp_idle_pm_notify_activity(struct zmk_mouse_ps2_data *data);
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * Settings
 */

// Delay mouse init to give the mouse time to send the init sequence.
#define ZMK_MOUSE_PS2_INIT_THREAD_DELAY_MS 1000

// How often the driver try to initialize a mouse before we give up.
#define MOUSE_PS2_INIT_ATTEMPTS 10

// Mouse activity packets are at least three bytes.
// This defines how much time between bytes can pass before
// we give up on the packet and start fresh.
#define MOUSE_PS2_TIMEOUT_ACTIVITY_PACKET K_MSEC(500)

/*
 * PS/2 Defines
 */

// According to the `IBM TrackPoint System Version 4.0 Engineering
// Specification`...
// "The POR shall be timed to occur 600 ms ± 20 % from the time power is
//  applied to the TrackPoint controller."
#define MOUSE_PS2_POWER_ON_RESET_TIME K_MSEC(CONFIG_ZMK_INPUT_MOUSE_PS2_POWER_ON_RESET_TIME)

// Common PS/2 Mouse commands
#define MOUSE_PS2_CMD_GET_DEVICE_ID "\xf2"
#define MOUSE_PS2_CMD_GET_DEVICE_ID_RESP_LEN 1

#define MOUSE_PS2_CMD_SET_SAMPLING_RATE "\xf3"
#define MOUSE_PS2_CMD_SET_SAMPLING_RATE_RESP_LEN 0
#define MOUSE_PS2_CMD_SET_SAMPLING_RATE_DEFAULT 100

#define MOUSE_PS2_CMD_STATUS_REQUEST "\xe9"
#define MOUSE_PS2_CMD_STATUS_REQUEST_RESP_LEN 3

#define MOUSE_PS2_CMD_ENABLE_REPORTING "\xf4"
#define MOUSE_PS2_CMD_ENABLE_REPORTING_RESP_LEN 0

#define MOUSE_PS2_CMD_DISABLE_REPORTING "\xf5"
#define MOUSE_PS2_CMD_DISABLE_REPORTING_RESP_LEN 0

#define MOUSE_PS2_CMD_RESEND "\xfe"
#define MOUSE_PS2_CMD_RESEND_RESP_LEN 0

#define MOUSE_PS2_CMD_RESET "\xff"
#define MOUSE_PS2_CMD_RESET_RESP_LEN 0

// Trackpoint Commands
// They can be found in the `IBM TrackPoint System Version 4.0 Engineering
// Specification` (YKT3Eext.pdf)...

#define MOUSE_PS2_CMD_TP_GET_SECONDARY_ID "\xe1"
#define MOUSE_PS2_CMD_TP_GET_SECONDARY_ID_RESP_LEN 2

#define MOUSE_PS2_CMD_TP_GET_ROM_ID "\xe2\x46"
#define MOUSE_PS2_CMD_TP_GET_ROM_ID_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_GET_CONFIG_BYTE "\xe2\x80\x2c"
#define MOUSE_PS2_CMD_TP_GET_CONFIG_BYTE_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_CONFIG_BYTE "\xe2\x81\x2c"
#define MOUSE_PS2_CMD_TP_SET_CONFIG_BYTE_RESP_LEN 0

#define MOUSE_PS2_ST_TP_SENSITIVITY "tp_sensitivity"
#define MOUSE_PS2_CMD_TP_GET_SENSITIVITY "\xe2\x80\x4a"
#define MOUSE_PS2_CMD_TP_GET_SENSITIVITY_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_SENSITIVITY "\xe2\x81\x4a"
#define MOUSE_PS2_CMD_TP_SET_SENSITIVITY_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_SENSITIVITY_MIN 0
#define MOUSE_PS2_CMD_TP_SET_SENSITIVITY_MAX 255
#define MOUSE_PS2_CMD_TP_SET_SENSITIVITY_DEFAULT 128

#define MOUSE_PS2_ST_TP_NEG_INERTIA "tp_neg_inertia"
#define MOUSE_PS2_CMD_TP_GET_NEG_INERTIA "\xe2\x80\x4d"
#define MOUSE_PS2_CMD_TP_GET_NEG_INERTIA_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_NEG_INERTIA "\xe2\x81\x4d"
#define MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_MIN 0
#define MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_MAX 255
#define MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_DEFAULT 0x06

#define MOUSE_PS2_ST_TP_VALUE6 "tp_value6"
#define MOUSE_PS2_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED "\xe2\x80\x60"
#define MOUSE_PS2_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED "\xe2\x81\x60"
#define MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MIN 0
#define MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MAX 255
#define MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_DEFAULT 0x61

#define MOUSE_PS2_ST_TP_PTS_THRESHOLD "tp_pts_threshold"
#define MOUSE_PS2_CMD_TP_GET_PTS_THRESHOLD "\xe2\x80\x5c"
#define MOUSE_PS2_CMD_TP_GET_PTS_THRESHOLD_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD "\xe2\x81\x5c"
#define MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_MIN 0
#define MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_MAX 255
#define MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_DEFAULT 0x08

#define MOUSE_PS2_ST_TP_UP_THRESH "tp_up_thresh"
#define MOUSE_PS2_CMD_TP_GET_UP_THRESH "\xe2\x80\x5a"
#define MOUSE_PS2_CMD_TP_GET_UP_THRESH_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_UP_THRESH "\xe2\x81\x5a"
#define MOUSE_PS2_CMD_TP_SET_UP_THRESH_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_UP_THRESH_MIN 0
#define MOUSE_PS2_CMD_TP_SET_UP_THRESH_MAX 255
#define MOUSE_PS2_CMD_TP_SET_UP_THRESH_DEFAULT 0xFF

#define MOUSE_PS2_ST_TP_Z_TIME "tp_z_time"
#define MOUSE_PS2_CMD_TP_GET_Z_TIME "\xe2\x80\x5e"
#define MOUSE_PS2_CMD_TP_GET_Z_TIME_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_Z_TIME "\xe2\x81\x5e"
#define MOUSE_PS2_CMD_TP_SET_Z_TIME_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_Z_TIME_MIN 0
#define MOUSE_PS2_CMD_TP_SET_Z_TIME_MAX 255
#define MOUSE_PS2_CMD_TP_SET_Z_TIME_DEFAULT 0x26

#define MOUSE_PS2_ST_TP_JENKS_CURV "tp_jenks_curv"
#define MOUSE_PS2_CMD_TP_GET_JENKS_CURV "\xe2\x80\x5d"
#define MOUSE_PS2_CMD_TP_GET_JENKS_CURV_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_JENKS_CURV "\xe2\x81\x5d"
#define MOUSE_PS2_CMD_TP_SET_JENKS_CURV_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_JENKS_CURV_MIN 0
#define MOUSE_PS2_CMD_TP_SET_JENKS_CURV_MAX 255
#define MOUSE_PS2_CMD_TP_SET_JENKS_CURV_DEFAULT 0x87

#define MOUSE_PS2_ST_TP_DRAG_HYSTERESIS "tp_draghys"
#define MOUSE_PS2_CMD_TP_GET_DRAG_HYSTERESIS "\xe2\x80\x58"
#define MOUSE_PS2_CMD_TP_GET_DRAG_HYSTERESIS_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS "\xe2\x81\x58"
#define MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_MIN 0
#define MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_MAX 255
#define MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_DEFAULT 0xFF

#define MOUSE_PS2_ST_TP_MIN_DRAG "tp_mindrag"
#define MOUSE_PS2_CMD_TP_GET_MIN_DRAG "\xe2\x80\x59"
#define MOUSE_PS2_CMD_TP_GET_MIN_DRAG_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_MIN_DRAG "\xe2\x81\x59"
#define MOUSE_PS2_CMD_TP_SET_MIN_DRAG_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_MIN_DRAG_MIN 0
#define MOUSE_PS2_CMD_TP_SET_MIN_DRAG_MAX 255
#define MOUSE_PS2_CMD_TP_SET_MIN_DRAG_DEFAULT 0x14

#define MOUSE_PS2_ST_TP_REACH "tp_reach"
#define MOUSE_PS2_CMD_TP_GET_REACH "\xe2\x80\x57"
#define MOUSE_PS2_CMD_TP_GET_REACH_RESP_LEN 1

#define MOUSE_PS2_CMD_TP_SET_REACH "\xe2\x81\x57"
#define MOUSE_PS2_CMD_TP_SET_REACH_RESP_LEN 0
#define MOUSE_PS2_CMD_TP_SET_REACH_MIN 0
#define MOUSE_PS2_CMD_TP_SET_REACH_MAX 255
#define MOUSE_PS2_CMD_TP_SET_REACH_DEFAULT 0x0A

// Trackpoint Config Bits
#define MOUSE_PS2_TP_CONFIG_BIT_PRESS_TO_SELECT 0x00
#define MOUSE_PS2_TP_CONFIG_BIT_RESERVED 0x01
#define MOUSE_PS2_TP_CONFIG_BIT_BUTTON2 0x02
#define MOUSE_PS2_TP_CONFIG_BIT_INVERT_X 0x03
#define MOUSE_PS2_TP_CONFIG_BIT_INVERT_Y 0x04
#define MOUSE_PS2_TP_CONFIG_BIT_INVERT_Z 0x05
#define MOUSE_PS2_TP_CONFIG_BIT_SWAP_XY 0x06
#define MOUSE_PS2_TP_CONFIG_BIT_FORCE_TRANSPARENT 0x07

// Responses
#define MOUSE_PS2_RESP_SELF_TEST_PASS 0xaa
#define MOUSE_PS2_RESP_SELF_TEST_FAIL 0xfc

/*
 * ZMK Defines
 */

#define MOUSE_PS2_BUTTON_L_IDX 0
#define MOUSE_PS2_BUTTON_R_IDX 1
#define MOUSE_PS2_BUTTON_M_IDX 3

#define MOUSE_PS2_THREAD_STACK_SIZE 2048
#define MOUSE_PS2_THREAD_PRIORITY 10

/*
 * Dedicated work queue for TP management operations (self-reset recovery,
 * liveness watchdog, idle PM wake/dormant).  These involve blocking PS/2
 * writes that would otherwise stall the system workqueue for 50-80 ms on
 * a healthy bus and potentially seconds on a degraded one.
 *
 * Priority 10: below sysworkq (-1), BLE (-8), and the PS/2 callback WQ
 * (prio 9) so byte delivery is never starved by management work.
 */
#define TP_MGMT_WQ_STACK_SIZE 2048
#define TP_MGMT_WQ_PRIORITY 10
K_THREAD_STACK_DEFINE(tp_mgmt_wq_stack, TP_MGMT_WQ_STACK_SIZE);
static struct k_work_q tp_mgmt_wq;
static bool tp_mgmt_wq_started;

/*
 * Global Variables
 */

#define MOUSE_PS2_SETTINGS_SUBTREE "mouse_ps2"

typedef enum {
    MOUSE_PS2_PACKET_MODE_PS2_DEFAULT,
    MOUSE_PS2_PACKET_MODE_SCROLL,
} zmk_mouse_ps2_packet_mode;

struct zmk_mouse_ps2_config {
    const struct device *ps2_device;
    bool has_rst_gpio;
    struct gpio_dt_spec rst_gpio;
    int rst_gpio_port_num;

    bool scroll_mode;
    bool disable_clicking;
    int sampling_rate;

    bool tp_press_to_select;
    int tp_press_to_select_threshold;
    int tp_sensitivity;
    int tp_neg_inertia;
    int tp_val6_upper_speed;
    int tp_up_thresh;
    int tp_z_time;
    int tp_jenks_curv;
    int tp_drag_hysteresis;
    int tp_min_drag;
    int tp_reach;
    bool tp_x_invert;
    bool tp_y_invert;
    bool tp_xy_swap;

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM)
    bool has_wake_gpio;
    struct gpio_dt_spec wake_gpio;
#endif
};

struct zmk_mouse_ps2_packet {
    int16_t mov_x;
    int16_t mov_y;
    int8_t scroll;
    bool overflow_x;
    bool overflow_y;
    bool button_l;
    bool button_m;
    bool button_r;
};

struct zmk_mouse_ps2_data {
    const struct device *dev;
    struct gpio_dt_spec rst_gpio; /* GPIO used for Power-On-Reset line */

    K_KERNEL_STACK_MEMBER(thread_stack, MOUSE_PS2_THREAD_STACK_SIZE);
    struct k_thread thread;

    zmk_mouse_ps2_packet_mode packet_mode;
    uint8_t packet_buffer[4];
    int packet_idx;
    struct zmk_mouse_ps2_packet prev_packet;
    struct k_work_delayable packet_buffer_timeout;
#if IS_ENABLED(CONFIG_SETTINGS)
    struct k_work_delayable zmk_mouse_ps2_save_work;
#endif

    bool button_l_is_held;
    bool button_m_is_held;
    bool button_r_is_held;

    bool activity_reporting_on;
    bool is_trackpoint;
    uint8_t manufacturer_id;
    uint8_t secondary_id;
    uint8_t rom_id;

    uint8_t sampling_rate;
    uint8_t tp_sensitivity;
    uint8_t tp_neg_inertia;
    uint8_t tp_value6;
    uint8_t tp_pts_threshold;
    uint8_t tp_up_thresh;
    uint8_t tp_z_time;
    uint8_t tp_jenks_curv;
    uint8_t tp_drag_hysteresis;
    uint8_t tp_min_drag;
    uint8_t tp_reach;

    bool tp_self_reset_pending;  /* true after receiving 0xAA, awaiting 0x00 device ID */
    int64_t tp_self_reset_time;   /* uptime_ms when tp_self_reset_pending was set */
    struct k_work tp_self_reset_work;

    int64_t last_byte_time;       /* uptime_ms of most recent PS/2 byte from TP */
    int liveness_retries;         /* consecutive watchdog recovery attempts */
    struct k_work_delayable liveness_watchdog;

    void *activity_callback;
    void *activity_resend_callback;

#if CONFIG_ZMK_INPUT_MOUSE_PS2_REPORT_INTERVAL_MIN > 0
    int64_t adx;
    int64_t ady;
    int64_t last_smp_time;
    int64_t last_rpt_time;
#endif

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM)
    enum {
        TP_PM_UNINITIALIZED = 0, /* zero-init sentinel: listeners check != DORMANT, safe */
        TP_PM_ACTIVE,
        TP_PM_ENTERING_DORMANT, /* phase 1 done, draining; phase 2 pending */
        TP_PM_DORMANT,
    } pm_state;
    struct k_work_delayable idle_pm_dormant_work;
    struct k_work_delayable idle_pm_dormant_finish_work;
    struct k_work idle_pm_wake_work;
    const struct device *uart_dev;
    struct gpio_callback wake_gpio_cb;
#endif
};

// Compose the desired config byte from immutable DT flags.
// No PS/2 I/O — pure bit arithmetic.
static inline uint8_t zmk_mouse_ps2_tp_desired_config_byte(
        const struct zmk_mouse_ps2_config *config) {
    uint8_t byte = 0;
    if (config->tp_press_to_select) {
        byte |= (1u << MOUSE_PS2_TP_CONFIG_BIT_PRESS_TO_SELECT);
    }
    if (config->tp_x_invert) {
        byte |= (1u << MOUSE_PS2_TP_CONFIG_BIT_INVERT_X);
    }
    if (config->tp_y_invert) {
        byte |= (1u << MOUSE_PS2_TP_CONFIG_BIT_INVERT_Y);
    }
    if (config->tp_xy_swap) {
        byte |= (1u << MOUSE_PS2_TP_CONFIG_BIT_SWAP_XY);
    }
    return byte;
}

// declare datas and configs for all devices
// NOTES: Settings will be assigned to all devices via exposed api from behaviors
#define ZMK_PS2_MOUSE_DEFINE_DATA_N_CFG(n)                  \
    static struct zmk_mouse_ps2_data data##n;               \
    static const struct zmk_mouse_ps2_config config##n;
DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_DATA_N_CFG)

static int allowed_sampling_rates[] = {
    10, 20, 40, 60, 80, 100, 200,
};

/*
 * Function Definitions
 */

int zmk_mouse_ps2_settings_save();

/*
 * Helpers
 */

#define MOUSE_PS2_GET_BIT(data, bit_pos) ((data >> bit_pos) & 0x1)
#define MOUSE_PS2_SET_BIT(data, bit_val, bit_pos) \
	(data = (data & ~(1u << (bit_pos))) | (((bit_val) ? 1u : 0u) << (bit_pos)))

/*
 * Mouse Activity Packet Reading
 */

void zmk_mouse_ps2_activity_process_cmd(const struct device *dev, 
                                        zmk_mouse_ps2_packet_mode packet_mode, uint8_t packet_state,
                                        uint8_t packet_x, uint8_t packet_y, uint8_t packet_extra);
void zmk_mouse_ps2_activity_abort_cmd(const struct device *dev, char *reason);
void zmk_mouse_ps2_activity_move_mouse(const struct device *dev, int16_t mov_x, int16_t mov_y);
void zmk_mouse_ps2_activity_scroll(const struct device *dev, int8_t scroll_y);
void zmk_mouse_ps2_activity_click_buttons(const struct device *dev, 
                                          bool button_l, bool button_m, bool button_r);
void zmk_mouse_ps2_activity_reset_packet_buffer(const struct device *dev);

int zmk_mouse_ps2_tp_sensitivity_set(const struct device *dev, int sensitivity);
int zmk_mouse_ps2_tp_neg_inertia_set(const struct device *dev, int neg_inertia);
int zmk_mouse_ps2_tp_value6_upper_plateau_speed_set(const struct device *dev, int value6);
int zmk_mouse_ps2_tp_press_to_select_set(const struct device *dev, bool enabled);
int zmk_mouse_ps2_tp_pts_threshold_set(const struct device *dev, int pts_threshold);
int zmk_mouse_ps2_tp_up_thresh_set(const struct device *dev, int up_thresh);
int zmk_mouse_ps2_tp_z_time_set(const struct device *dev, int z_time);
int zmk_mouse_ps2_tp_jenks_curv_set(const struct device *dev, int jenks_curv);
int zmk_mouse_ps2_tp_drag_hysteresis_set(const struct device *dev, int drag_hysteresis);
int zmk_mouse_ps2_tp_min_drag_set(const struct device *dev, int min_drag);
int zmk_mouse_ps2_tp_reach_set(const struct device *dev, int reach);
int zmk_mouse_ps2_tp_invert_x_set(const struct device *dev, bool enabled);
int zmk_mouse_ps2_tp_invert_y_set(const struct device *dev, bool enabled);
int zmk_mouse_ps2_tp_swap_xy_set(const struct device *dev, bool enabled);
int zmk_mouse_ps2_tp_set_config_byte_direct(const struct device *dev, uint8_t desired);
int zmk_mouse_ps2_set_sampling_rate(const struct device *dev, uint8_t sampling_rate);
int zmk_mouse_ps2_set_packet_mode(const struct device *dev, zmk_mouse_ps2_packet_mode mode);

/*
 * Apply all TP register settings from data-> to hardware.
 * Used by: init, self-reset recovery, idle PM wake.
 * Each _set() validates, sends the PS/2 command, and updates data-> on success.
 *
 * Optimization: if reporting is currently on, we disable it once (F5) before
 * the batch and re-enable (F4) after, rather than letting each _set() call
 * individually wrap with F5/F4.  This avoids ~13 redundant F5/F4 round-trips
 * on the bus (saves ~25ms of PS/2 traffic).
 */
static int zmk_mouse_ps2_tp_apply_all_settings(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;
    int failures = 0;
    int total = 0;

    /* Acquire a batch MPSL timeslot (100ms) to protect all register
     * writes from BLE radio ZLI preemption.  If this fails, writes
     * fall back to per-byte timeslot protection (existing behavior). */
    int batch_err = ps2_uart_timeslot_batch_begin();
    if (batch_err) {
        LOG_WRN("TP settings: batch timeslot unavailable (%d), "
                "using per-byte protection", batch_err);
    }

    /* If reporting is on, disable it once for the whole batch.
     * The _set() functions pass pause_reporting=true to send_cmd(),
     * but send_cmd() only actually sends F5/F4 when
     * data->activity_reporting_on == true.  By disabling reporting
     * here (which sets the flag to false), all _set() calls below
     * will skip their individual F5/F4 wrapping. */
    bool was_reporting = data->activity_reporting_on;
    if (was_reporting) {
        int err = zmk_mouse_ps2_activity_reporting_disable(dev);
        if (err) {
            LOG_ERR("TP settings: failed to disable reporting before batch (%d)", err);
            /* Continue anyway — individual _set calls will still
             * wrap with F5/F4 as before (no worse than status quo). */
            was_reporting = false;
        }
    }

#define APPLY_SETTING(call) do { total++; if ((call) != 0) { failures++; } } while (0)

    APPLY_SETTING(zmk_mouse_ps2_tp_sensitivity_set(dev, data->tp_sensitivity));
    APPLY_SETTING(zmk_mouse_ps2_tp_neg_inertia_set(dev, data->tp_neg_inertia));
    APPLY_SETTING(zmk_mouse_ps2_tp_value6_upper_plateau_speed_set(dev, data->tp_value6));
    APPLY_SETTING(zmk_mouse_ps2_tp_up_thresh_set(dev, data->tp_up_thresh));
    APPLY_SETTING(zmk_mouse_ps2_tp_z_time_set(dev, data->tp_z_time));
    APPLY_SETTING(zmk_mouse_ps2_tp_jenks_curv_set(dev, data->tp_jenks_curv));
    APPLY_SETTING(zmk_mouse_ps2_tp_drag_hysteresis_set(dev, data->tp_drag_hysteresis));
    APPLY_SETTING(zmk_mouse_ps2_tp_min_drag_set(dev, data->tp_min_drag));
    APPLY_SETTING(zmk_mouse_ps2_tp_reach_set(dev, data->tp_reach));

    // Blind-write the entire config byte (register 0x2C) in one shot.
    // This covers PTS, InvertX, InvertY, SwapXY without a read-modify-write,
    // eliminating the risk of preserving corrupted orientation bits from a
    // garbled read.  The batch timeslot protects against ZLI corruption.
    {
        uint8_t desired = zmk_mouse_ps2_tp_desired_config_byte(config);
        APPLY_SETTING(zmk_mouse_ps2_tp_set_config_byte_direct(dev, desired));
    }

    if (config->tp_press_to_select) {
        APPLY_SETTING(zmk_mouse_ps2_tp_pts_threshold_set(dev, data->tp_pts_threshold));
    }

#undef APPLY_SETTING

    /* Re-enable reporting if we disabled it at the top. */
    if (was_reporting) {
        int err = zmk_mouse_ps2_activity_reporting_enable(dev);
        if (err) {
            LOG_ERR("TP settings: failed to re-enable reporting after batch (%d)", err);
        }
    }

    if (failures > 0) {
        LOG_ERR("TP settings: %d/%d failed to apply", failures, total);
    } else {
        LOG_INF("TP settings: all %d applied successfully", total);
    }

    /* Release the batch timeslot.  No-op if batch_begin failed. */
    ps2_uart_timeslot_batch_end();

    return failures;
}

static void zmk_mouse_ps2_tp_self_reset_work_handler(struct k_work *work);
static void zmk_mouse_ps2_liveness_watchdog_handler(struct k_work *work);

struct zmk_mouse_ps2_packet
zmk_mouse_ps2_activity_parse_packet_buffer(zmk_mouse_ps2_packet_mode packet_mode,
                                           uint8_t packet_state, uint8_t packet_x, uint8_t packet_y,
                                           uint8_t packet_extra);

// Called by the PS/2 driver whenver the mouse sends a byte and
// reporting is enabled through `zmk_mouse_ps2_activity_reporting_enable`.
void zmk_mouse_ps2_activity_callback(const struct device *dev,
                                     const struct device *ps2_device, uint8_t byte) {
    struct zmk_mouse_ps2_data *data = dev->data;

    data->last_byte_time = k_uptime_get();
    data->liveness_retries = 0;
    k_work_schedule_for_queue(&tp_mgmt_wq, &data->liveness_watchdog, K_SECONDS(5));

    k_work_cancel_delayable(&data->packet_buffer_timeout);

    // LOG_DBG("Received mouse movement data: 0x%x", byte);

    if (data->packet_idx >= sizeof(data->packet_buffer)) {
        LOG_ERR("Packet index %d out of bounds, resetting", data->packet_idx);
        zmk_mouse_ps2_activity_reset_packet_buffer(data->dev);
        return;
    }

    data->packet_buffer[data->packet_idx] = byte;

    if (data->packet_idx == 0) {

        // Detect TP spontaneous self-reset: the TP sends 0xAA (BAT pass)
        // followed by 0x00 (device ID) when it internally resets (ESD,
        // power glitch, internal watchdog).  All extended registers revert
        // to factory defaults.  We intercept the two-byte sequence here
        // and schedule a work item to re-apply settings.
        if (data->tp_self_reset_pending) {
            data->tp_self_reset_pending = false;
            int64_t elapsed = k_uptime_get() - data->tp_self_reset_time;
            if (elapsed > 50) {
                LOG_WRN("Stale self-reset flag (%lld ms old), ignoring.", elapsed);
                zmk_mouse_ps2_activity_reset_packet_buffer(data->dev);
                return;
            }
            if (byte == 0x00) {
                LOG_WRN("TP self-reset detected (0xAA 0x00). "
                        "Scheduling re-apply of all TP settings.");
                k_work_submit_to_queue(&tp_mgmt_wq, &data->tp_self_reset_work);
            } else {
                LOG_WRN("Got 0xAA followed by 0x%02x (not 0x00). "
                        "Ignoring as spurious.", byte);
            }
            zmk_mouse_ps2_activity_reset_packet_buffer(data->dev);
            return;
        }

        if (byte == MOUSE_PS2_RESP_SELF_TEST_PASS) {
            LOG_WRN("TP sent 0xAA (BAT pass) — possible self-reset. "
                    "Waiting for device ID byte.");
            data->tp_self_reset_pending = true;
            data->tp_self_reset_time = k_uptime_get();
            zmk_mouse_ps2_activity_reset_packet_buffer(data->dev);
            return;
        }

        // Bit 3 of the first command byte should always be 1
        // If it is not, then we are definitely out of alignment.
        // So we ask the device to resend the entire 3-byte command
        // again.
        int alignment_bit = MOUSE_PS2_GET_BIT(byte, 3);
        if (alignment_bit != 1) {

            zmk_mouse_ps2_activity_abort_cmd(data->dev, "Bit 3 of packet is 0 instead of 1");
            return;
        }
    } else if (data->packet_idx == 1) {
        // Do nothing
    } else if ((data->packet_mode == MOUSE_PS2_PACKET_MODE_PS2_DEFAULT && data->packet_idx == 2) ||
               (data->packet_mode == MOUSE_PS2_PACKET_MODE_SCROLL && data->packet_idx == 3)) {

        zmk_mouse_ps2_activity_process_cmd(data->dev,
                                           data->packet_mode, data->packet_buffer[0],
                                           data->packet_buffer[1], data->packet_buffer[2],
                                           data->packet_buffer[3]);
        zmk_mouse_ps2_activity_reset_packet_buffer(data->dev);
        return;
    }

    data->packet_idx += 1;

    k_work_schedule(&data->packet_buffer_timeout, MOUSE_PS2_TIMEOUT_ACTIVITY_PACKET);
}

void zmk_mouse_ps2_activity_abort_cmd(const struct device *dev, char *reason) {
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;
    const struct device *ps2_device = config->ps2_device;

    LOG_ERR("PS/2 Mouse cmd buffer is out of aligment. Requesting resend: %s", reason);

    data->packet_idx = 0;
    ps2_write(ps2_device, MOUSE_PS2_CMD_RESEND[0]);

    zmk_mouse_ps2_activity_reset_packet_buffer(dev);
}

// Called if the PS/2 driver encounters a transmission error and asks the
// device to resend the packet.
// The device will resend all bytes of the packet. So we need to reset our
// buffer.
void zmk_mouse_ps2_activity_resend_callback(const struct device *dev, 
                                            const struct device *ps2_device) {

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_ENABLE_PS2_RESEND_CALLBACK)

    struct zmk_mouse_ps2_data *data = dev->data;

    LOG_WRN("Mouse movement cmd had transmission error on idx=%d", data->packet_idx);

    zmk_mouse_ps2_activity_reset_packet_buffer(dev);

#endif /* IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_ENABLE_PS2_RESEND_CALLBACK) */
}

// Called if no new byte arrives within
// MOUSE_PS2_TIMEOUT_ACTIVITY_PACKET
void zmk_mouse_ps2_activity_packet_timout(struct k_work *item) {

    struct k_work_delayable *work_delayable = (struct k_work_delayable *)item;
    struct zmk_mouse_ps2_data *data = CONTAINER_OF(work_delayable,
                                                   struct zmk_mouse_ps2_data,
                                                   packet_buffer_timeout);
    const struct device *dev = data->dev;

    LOG_DBG("Mouse movement cmd timed out on idx=%d", data->packet_idx);

    // Reset the cmd buffer in case we are out of alignment.
    // This way if the mouse ever gets out of alignment, the user
    // can reset it by just not moving it for a second.
    zmk_mouse_ps2_activity_reset_packet_buffer(dev);
}

void zmk_mouse_ps2_activity_reset_packet_buffer(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;

    data->packet_idx = 0;
    memset(data->packet_buffer, 0x0, sizeof(data->packet_buffer));
}

/*
 * Restore all TP state after a confirmed or suspected self-reset.
 * Covers sampling rate (standard PS/2), Intellimouse scroll mode
 * (magic sequence), and all extended 0xE2 registers.
 *
 * Self-contained — handles internally:
 *   1. Clears activity_reporting_on (suppresses F5/F4 chatter in
 *      send_cmd / apply_all_settings, avoids TARE recalibration).
 *   2. Resets the packet buffer (stale bytes from self-reset).
 *   3. Restores sampling rate, all extended registers, scroll mode.
 *
 * Does NOT acquire a batch timeslot or re-enable reporting (F4).
 * Use tp_recover_and_enable() for the full protected transaction.
 *
 * Returns the number of extended-register write failures (0 = success).
 */
static int zmk_mouse_ps2_tp_recover_all(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;

    /* Clear the reporting flag before any PS/2 commands.  This
     * prevents send_cmd and apply_all_settings from wrapping each
     * command with F5/F4, which would cause a TARE recalibration
     * if the user's finger is on the stick.  The caller (or
     * tp_recover_and_enable) is responsible for sending F4 after. */
    data->activity_reporting_on = false;

    /* Reset packet buffer — a self-reset 0xAA 0x00 sequence or stale
     * bytes from a bus glitch may have left it in a partial state. */
    zmk_mouse_ps2_activity_reset_packet_buffer(dev);

    /* Restore sampling rate — after a reset the TP reverts to the PS/2
     * default of 100 samples/sec.  This is a standard PS/2 command
     * (0xF3), not an extended register, so apply_all_settings (which
     * only handles 0xE2 registers) does not cover it. */
    if (data->sampling_rate != MOUSE_PS2_CMD_SET_SAMPLING_RATE_DEFAULT) {
        int rate_err = zmk_mouse_ps2_set_sampling_rate(dev, data->sampling_rate);
        if (rate_err) {
            LOG_ERR("TP recovery: failed to restore sampling rate %d (%d)",
                    data->sampling_rate, rate_err);
        }
    }

    int failures = zmk_mouse_ps2_tp_apply_all_settings(dev);

    /* Restore Intellimouse scroll mode if configured.  After a
     * self-reset the TP reverts to 3-byte PS/2 default packets.
     * Without this, the driver expects 4-byte packets but the TP
     * sends 3-byte, causing persistent packet misalignment. */
    if (config->scroll_mode) {
        int scroll_err = zmk_mouse_ps2_set_packet_mode(dev,
                                                       MOUSE_PS2_PACKET_MODE_SCROLL);
        if (scroll_err) {
            LOG_ERR("TP recovery: failed to restore scroll mode (%d)",
                    scroll_err);
        }
    }

    return failures;
}

/*
 * Full TP recovery transaction: restore all settings + re-enable
 * reporting.  Single entry point for self-reset recovery, liveness
 * watchdog, and any future recovery path.
 *
 * Owns the batch timeslot lifecycle: acquires a 100 ms MPSL timeslot
 * before any writes and holds it through the F4 re-enable, so every
 * byte — including the critical F4 — is shielded from BLE ZLI.
 *
 * If all F4 attempts fail, force-enables the PS/2 callback so future
 * self-resets can still be detected, and the liveness watchdog can
 * retry on its next firing.
 *
 * Returns 0 on full success, negative errno on F4 failure (settings
 * may still have been applied), or a positive count of setting
 * failures if F4 succeeded but some registers failed.
 */
static int zmk_mouse_ps2_tp_recover_and_enable(const struct device *dev) {
    const struct zmk_mouse_ps2_config *config = dev->config;

    /* Acquire a batch timeslot covering ALL recovery writes + F4.
     * The inner tp_apply_all_settings() batch_begin nests harmlessly
     * (refcount 1→2→1).  Held until after F4 so the most critical
     * write is also ZLI-proof. */
    int batch_err = ps2_uart_timeslot_batch_begin();
    if (batch_err) {
        LOG_WRN("TP recovery: batch timeslot unavailable (%d), "
                "using per-byte protection", batch_err);
    }

    int failures = zmk_mouse_ps2_tp_recover_all(dev);

    /* Re-enable reporting with retry.  After a self-reset the TP
     * reverts to reporting-disabled, and recover_all cleared
     * activity_reporting_on.  A single failed F4 would leave the
     * TP permanently mute. */
    int err = -EAGAIN;
    for (int attempt = 0; attempt < 3; attempt++) {
        err = zmk_mouse_ps2_activity_reporting_enable(dev);
        if (err == 0) {
            break;
        }
        LOG_WRN("TP recovery: F4 re-enable attempt %d/3 failed (%d)",
                attempt + 1, err);
        k_sleep(K_MSEC(100 * (attempt + 1)));
    }
    if (err) {
        LOG_ERR("TP recovery: all F4 re-enable attempts failed (%d)", err);
        /* Force-enable the callback so we can still detect future
         * self-resets even if the F4 command failed.  The liveness
         * watchdog will retry on its next firing. */
        ps2_enable_callback(config->ps2_device);
        ps2_uart_timeslot_batch_end();
        return -EIO;
    }

    ps2_uart_timeslot_batch_end();

    if (failures > 0) {
        LOG_WRN("TP recovery: completed with %d setting failure(s)", failures);
    }

    return failures;
}

/*
 * TP Self-Reset Recovery
 *
 * When a TrackPoint spontaneously resets (ESD, power glitch, internal
 * watchdog), it sends 0xAA 0x00 and all extended registers revert to
 * factory defaults.  The activity_callback detects this two-byte
 * sequence and schedules this work handler to re-apply all register
 * settings from the in-memory data struct.
 */
static void zmk_mouse_ps2_tp_self_reset_work_handler(struct k_work *work) {
    struct zmk_mouse_ps2_data *data = CONTAINER_OF(work, struct zmk_mouse_ps2_data,
                                                   tp_self_reset_work);
    const struct device *dev = data->dev;

    LOG_WRN("TP self-reset recovery: re-applying all TP register settings");

    if (!data->is_trackpoint) {
        return;
    }

    int ret = zmk_mouse_ps2_tp_recover_and_enable(dev);
    if (ret < 0) {
        LOG_ERR("TP self-reset recovery: F4 re-enable failed (%d)", ret);
    } else if (ret > 0) {
        LOG_WRN("TP self-reset recovery: completed with %d setting failure(s)", ret);
    } else {
        LOG_WRN("TP self-reset recovery: all settings re-applied");
    }
}

/*
 * TP Liveness Watchdog
 *
 * One-shot watchdog rescheduled on every incoming byte.  If no bytes
 * arrive for 5 seconds while reporting is supposed to be on, something
 * silently killed the TP.  Force-resend F4 and re-enable the callback.
 *
 * Escalation ladder:
 *   Attempt  0:   full recovery (re-apply all settings + F4), 5s interval.
 *   Attempts 1-2: lightweight F4 probe only, 5s interval.
 *   Attempt  3+:  send 0xFF (full reset), re-apply all settings via
 *                 the self-reset work handler, 30s interval.
 *
 * Never gives up.  When the TP recovers, activity_callback resets
 * retries to 0 and the watchdog returns to the normal 5s cadence.
 */
static void zmk_mouse_ps2_liveness_watchdog_handler(struct k_work *work) {
    struct k_work_delayable *dwork = (struct k_work_delayable *)work;
    struct zmk_mouse_ps2_data *data = CONTAINER_OF(dwork, struct zmk_mouse_ps2_data,
                                                   liveness_watchdog);
    const struct device *dev = data->dev;
    const struct zmk_mouse_ps2_config *config = dev->config;

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM)
    /* Don't probe while dormant or entering dormant — the UART may be
     * suspended and no bytes are expected.  The wake handler restarts
     * the watchdog when it returns to ACTIVE. */
    if (data->pm_state != TP_PM_ACTIVE) {
        return;
    }
#endif

    int64_t silence = k_uptime_get() - data->last_byte_time;
    if (silence < 5000) {
        /* A byte arrived between scheduling and firing; reschedule. */
        k_work_schedule_for_queue(&tp_mgmt_wq, &data->liveness_watchdog, K_SECONDS(5));
        return;
    }

    int attempt = data->liveness_retries++;

    if (attempt == 0) {
        /* --- First attempt after wake from dormant (or fresh boot).
         *
         * The TP may have self-reset during dormant (ESD, power glitch)
         * which reverts all registers to factory defaults AND disables
         * reporting.  We missed the 0xAA 0x00 sequence because the UART
         * was suspended.
         *
         * Do a full recovery: re-apply all settings + F4.  This is
         * idempotent if the TP didn't self-reset (settings are written
         * to the same values), costs ~20ms, and fixes the "factory
         * settings after wake" problem. */
        LOG_WRN("TP liveness: first probe — no data for %lld ms, "
                "doing full settings re-apply + F4", silence);

        int ret = zmk_mouse_ps2_tp_recover_and_enable(dev);
        if (ret < 0) {
            LOG_ERR("TP liveness: F4 re-enable failed (%d)", ret);
        } else if (ret > 0) {
            LOG_WRN("TP liveness: %d setting(s) failed during recovery", ret);
        }

        k_work_schedule_for_queue(&tp_mgmt_wq, &data->liveness_watchdog, K_SECONDS(5));
    } else if (attempt < 3) {
        /* --- Stage 1: lightweight F4 probe --- */
        LOG_WRN("TP liveness: attempt %d/3 — no data for %lld ms, sending F4",
                attempt + 1, silence);

        data->activity_reporting_on = false;
        int err = zmk_mouse_ps2_activity_reporting_enable(dev);
        if (err) {
            LOG_ERR("TP liveness: failed to re-enable reporting (%d)", err);
            ps2_enable_callback(config->ps2_device);
        }

        k_work_schedule_for_queue(&tp_mgmt_wq, &data->liveness_watchdog, K_SECONDS(5));
    } else {
        /* --- Stage 2: escalate to full 0xFF reset --- */
        LOG_WRN("TP liveness: attempt %d — F4 failed 3×, sending 0xFF reset "
                "(%lld ms silence)", attempt + 1, silence);

        int err = zmk_mouse_ps2_reset(dev, config->ps2_device);
        if (err) {
            LOG_ERR("TP liveness: 0xFF reset failed (%d)", err);
            /* Still try re-enabling the callback in case the bus is
             * stuck but partially functional. */
            ps2_enable_callback(config->ps2_device);
        } else {
            /* Give the TP time to complete BAT (up to ~500ms). */
            k_sleep(K_MSEC(600));

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM)
            /* Re-check: idle PM may have started a dormant transition
             * while we were sleeping.  Abort recovery — the wake
             * handler will take over when the user next moves the TP. */
            if (data->pm_state != TP_PM_ACTIVE) {
                LOG_WRN("TP liveness: PM state changed during BAT wait, aborting");
                return;
            }
#endif

            /* Reset the packet buffer — any partial state is stale
             * after a full reset. */
            zmk_mouse_ps2_activity_reset_packet_buffer(dev);
        }

        /* Re-apply all settings and re-enable reporting.  Reuse the
         * self-reset work handler which already does the full
         * apply-all-settings + retry-F4 sequence. */
        k_work_submit_to_queue(&tp_mgmt_wq, &data->tp_self_reset_work);

        k_work_schedule_for_queue(&tp_mgmt_wq, &data->liveness_watchdog, K_SECONDS(30));
    }
}

void zmk_mouse_ps2_activity_process_cmd(const struct device *dev,
                                        zmk_mouse_ps2_packet_mode packet_mode, uint8_t packet_state,
                                        uint8_t packet_x, uint8_t packet_y, uint8_t packet_extra) {
    struct zmk_mouse_ps2_data *data = dev->data;
    struct zmk_mouse_ps2_packet packet;
    packet = zmk_mouse_ps2_activity_parse_packet_buffer(packet_mode, packet_state, packet_x,
                                                        packet_y, packet_extra);

    int x_delta = abs(data->prev_packet.mov_x - packet.mov_x);
    int y_delta = abs(data->prev_packet.mov_y - packet.mov_y);

    LOG_DBG("Got mouse activity cmd "
            "(mov_x=%d, mov_y=%d, o_x=%d, o_y=%d, scroll=%d, "
            "b_l=%d, b_m=%d, b_r=%d) and ("
            "x_delta=%d, y_delta=%d)",
            packet.mov_x, packet.mov_y, packet.overflow_x, packet.overflow_y, packet.scroll,
            packet.button_l, packet.button_m, packet.button_r, x_delta, y_delta);

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_ENABLE_ERROR_MITIGATION)
    if (packet.overflow_x == 1 && packet.overflow_y == 1) {
        LOG_WRN("Detected overflow in both x and y. "
                "Probably mistransmission. Aborting...");

        zmk_mouse_ps2_activity_abort_cmd(data->dev, "Overflow in both x and y");
        return;
    }

    // If the mouse exceeds the allowed threshold of movement, it's probably
    // a mistransmission or misalignment.
    // But we only do this check if there was prior movement that wasn't
    // reset in `zmk_mouse_ps2_activity_packet_timout`.
    if ((packet.mov_x != 0 && packet.mov_y != 0) && (x_delta > 150 || y_delta > 150)) {
        LOG_WRN("Detected malformed packet with "
                "(mov_x=%d, mov_y=%d, o_x=%d, o_y=%d, scroll=%d, "
                "b_l=%d, b_m=%d, b_r=%d) and ("
                "x_delta=%d, y_delta=%d)",
                packet.mov_x, packet.mov_y, packet.overflow_x, packet.overflow_y, packet.scroll,
                packet.button_l, packet.button_m, packet.button_r, x_delta, y_delta);
        zmk_mouse_ps2_activity_abort_cmd(data->dev, "Exceeds movement threshold.");
        return;
    }
#endif

    zmk_mouse_ps2_activity_move_mouse(data->dev, packet.mov_x, packet.mov_y);
    zmk_mouse_ps2_activity_click_buttons(data->dev, packet.button_l, packet.button_m, packet.button_r);

    data->prev_packet = packet;

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM)
    tp_idle_pm_notify_activity(data);
#endif
}

struct zmk_mouse_ps2_packet
zmk_mouse_ps2_activity_parse_packet_buffer(zmk_mouse_ps2_packet_mode packet_mode,
                                           uint8_t packet_state, uint8_t packet_x, uint8_t packet_y,
                                           uint8_t packet_extra) {
    struct zmk_mouse_ps2_packet packet;

    packet.button_l = MOUSE_PS2_GET_BIT(packet_state, 0);
    packet.button_r = MOUSE_PS2_GET_BIT(packet_state, 1);
    packet.button_m = MOUSE_PS2_GET_BIT(packet_state, 2);
    packet.overflow_x = MOUSE_PS2_GET_BIT(packet_state, 6);
    packet.overflow_y = MOUSE_PS2_GET_BIT(packet_state, 7);
    packet.scroll = 0;

    // The coordinates are delivered as a signed 9bit integers.
    // But a PS/2 packet is only 8 bits, so the most significant
    // bit with the sign is stored inside the state packet.
    //
    // Since we are converting the uint8_t into a int16_t
    // we must pad the unused most significant bits with
    // the sign bit.
    //
    // Example:
    //                              ↓ x sign bit
    //  - State: 0x18 (          0001 1000)
    //                             ↑ y sign bit
    //  - X:     0xfd (          1111 1101) / decimal 253
    //  - New X:      (1111 1111 1111 1101) / decimal -3
    //
    //  - Y:     0x02 (          0000 0010) / decimal 2
    //  - New Y:      (0000 0000 0000 0010) / decimal 2
    //
    // The code below creates a signed int and is from...
    // https://wiki.osdev.org/PS/2_Mouse
    packet.mov_x = packet_x - ((packet_state << 4) & 0x100);
    packet.mov_y = packet_y - ((packet_state << 3) & 0x100);

    // If packet mode scroll or scroll+5 buttons is used,
    // then the first 4 bit of the extra byte are used for the
    // scroll wheel. It is a signed number with the rango of
    // -8 to +7.
    if (packet_mode == MOUSE_PS2_PACKET_MODE_SCROLL) {
        MOUSE_PS2_SET_BIT(packet.scroll, MOUSE_PS2_GET_BIT(packet_extra, 0), 0);
        MOUSE_PS2_SET_BIT(packet.scroll, MOUSE_PS2_GET_BIT(packet_extra, 1), 1);
        MOUSE_PS2_SET_BIT(packet.scroll, MOUSE_PS2_GET_BIT(packet_extra, 2), 2);
        packet.scroll = packet_extra - ((packet.scroll << 3) & 0x100);
    }

    return packet;
}

/*
 * Mouse Moving and Clicking
 */

static bool zmk_mouse_ps2_is_non_zero_1d_movement(int16_t speed) { return speed != 0; }

void zmk_mouse_ps2_activity_move_mouse(const struct device *dev, int16_t mov_x, int16_t mov_y) {
    struct zmk_mouse_ps2_data *data = dev->data;
    int ret = 0;

    bool have_x = zmk_mouse_ps2_is_non_zero_1d_movement(mov_x);
    bool have_y = zmk_mouse_ps2_is_non_zero_1d_movement(mov_y);


#if CONFIG_ZMK_INPUT_MOUSE_PS2_REPORT_INTERVAL_MIN > 0

    int64_t now = k_uptime_get();
    if (now - data->last_smp_time >= CONFIG_ZMK_INPUT_MOUSE_PS2_REPORT_INTERVAL_MIN) {
        data->adx = data->ady = 0;
    }
    data->last_smp_time = now;
    data->adx += mov_x;
    data->ady += mov_y;
    if (now - data->last_rpt_time < CONFIG_ZMK_INPUT_MOUSE_PS2_REPORT_INTERVAL_MIN) {
        return;
    }
    bool report_x = (data->adx != 0);
    bool report_y = (data->ady != 0);
    if (report_x || report_y) {
        data->last_rpt_time = now;
        if (report_x) {
            ret = input_report_rel(data->dev, INPUT_REL_X, data->adx, !report_y, K_NO_WAIT);
        }
        if (report_y) {
            ret = input_report_rel(data->dev, INPUT_REL_Y, data->ady, true, K_NO_WAIT);
        }
        data->adx = data->ady = 0;
    }

#else /* CONFIG_ZMK_INPUT_MOUSE_PS2_REPORT_INTERVAL_MIN > 0 */

    if (have_x) {
        ret = input_report_rel(data->dev, INPUT_REL_X, mov_x, !have_y, K_NO_WAIT);
    }
    if (have_y) {
        ret = input_report_rel(data->dev, INPUT_REL_Y, mov_y, true, K_NO_WAIT);
    }

#endif /* CONFIG_ZMK_INPUT_MOUSE_PS2_REPORT_INTERVAL_MIN > 0 */

}

void zmk_mouse_ps2_activity_click_buttons(const struct device *dev, 
                                          bool button_l, bool button_m, bool button_r) {
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;

    // TODO: Integrate this with the proper button mask instead
    // of hardcoding the mouse button indeces.
    // Check hid.c and zmk_hid_mouse_buttons_press() for more info.

    int buttons_pressed = 0;
    int buttons_released = 0;

    // First we check which mouse button press states have changed
    bool button_l_pressed = false;
    bool button_l_released = false;
    if (button_l == true && data->button_l_is_held == false) {
        LOG_INF("Pressed button_l");

        button_l_pressed = true;
        buttons_pressed++;
    } else if (button_l == false && data->button_l_is_held == true) {
        LOG_INF("Releasing button_l");

        button_l_released = true;
        buttons_released++;
    }

    bool button_m_released = false;
    bool button_m_pressed = false;
    if (button_m == true && data->button_m_is_held == false) {
        LOG_INF("Pressing button_m");

        button_m_pressed = true;
        buttons_pressed++;
    } else if (button_m == false && data->button_m_is_held == true) {
        LOG_INF("Releasing button_m");

        button_m_released = true;
        buttons_released++;
    }

    bool button_r_released = false;
    bool button_r_pressed = false;
    if (button_r == true && data->button_r_is_held == false) {
        LOG_INF("Pressing button_r");

        button_r_pressed = true;
        buttons_pressed++;
    } else if (button_r == false && data->button_r_is_held == true) {
        LOG_INF("Releasing button_r");

        button_r_released = true;
        buttons_released++;
    }

    // Then we check if this is likely a transmission error
    if (buttons_pressed > 1 || buttons_released > 1) {
        LOG_WRN("Ignoring button presses: Received %d button presses "
                "and %d button releases in one packet. "
                "Probably tranmission error.",
                buttons_pressed, buttons_released);

        zmk_mouse_ps2_activity_abort_cmd(dev, "Multiple button presses");
        return;
    }

    if (config->disable_clicking != true) {
        // If it wasn't, we actually send the events.
        if (buttons_pressed > 0 || buttons_released > 0) {

            int buttons_need_reporting = buttons_pressed + buttons_released;

            // Left button
            if (button_l_pressed) {

                input_report_key(data->dev, INPUT_BTN_0, 1,
                                 buttons_need_reporting == 1 ? true : false, K_FOREVER);
                data->button_l_is_held = true;
            } else if (button_l_released) {

                input_report_key(data->dev, INPUT_BTN_0, 0,
                                 buttons_need_reporting == 1 ? true : false, K_FOREVER);
                data->button_l_is_held = false;
            }

            buttons_need_reporting--;

            // Right button
            if (button_r_pressed) {

                input_report_key(data->dev, INPUT_BTN_1, 1,
                                 buttons_need_reporting == 1 ? true : false, K_FOREVER);
                data->button_r_is_held = true;
            } else if (button_r_released) {

                input_report_key(data->dev, INPUT_BTN_1, 0,
                                 buttons_need_reporting == 1 ? true : false, K_FOREVER);
                data->button_r_is_held = false;
            }

            buttons_need_reporting--;

            // Middle Button
            if (button_m_pressed) {

                input_report_key(data->dev, INPUT_BTN_2, 1,
                                 buttons_need_reporting == 1 ? true : false, K_FOREVER);
                data->button_m_is_held = true;
            } else if (button_m_released) {

                input_report_key(data->dev, INPUT_BTN_2, 0,
                                 buttons_need_reporting == 1 ? true : false, K_FOREVER);
                data->button_m_is_held = false;
            }
        }
    }
}

/*
 * PS/2 Command Sending Wrapper
 */
int zmk_mouse_ps2_activity_reporting_enable(const struct device *dev);
int zmk_mouse_ps2_activity_reporting_disable(const struct device *dev);

struct zmk_mouse_ps2_send_cmd_resp {
    int err;
    char err_msg[80];
    uint8_t resp_buffer[8];
    int resp_len;
};

struct zmk_mouse_ps2_send_cmd_resp zmk_mouse_ps2_send_cmd(const struct device *dev,
                                                          char *cmd, int cmd_len, uint8_t *arg,
                                                          int resp_len, bool pause_reporting) {
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;
    const struct device *ps2_device = config->ps2_device;
    int err = 0;
    bool prev_activity_reporting_on = data->activity_reporting_on;

    struct zmk_mouse_ps2_send_cmd_resp resp = {
        .err = 0,
        .err_msg = "",
        .resp_len = 0,
    };
    memset(resp.resp_buffer, 0x0, sizeof(resp.resp_buffer));

    // Don't send the string termination NULL byte
    int cmd_bytes = cmd_len - 1;
    if (cmd_bytes < 1) {
        resp.err = -10;
        snprintf(resp.err_msg, sizeof(resp.err_msg),
                 "Cannot send cmd with less than 1 byte length");

        return resp;
    }

    if (resp_len > sizeof(resp.resp_buffer)) {
        resp.err = -11;
        snprintf(resp.err_msg, sizeof(resp.err_msg),
                 "Response can't be longer than the resp_buffer (%d)", sizeof(resp.err_msg));

        return resp;
    }

    if (pause_reporting == true && data->activity_reporting_on == true) {
        LOG_DBG("Disabling mouse activity reporting...");

        resp.err = zmk_mouse_ps2_activity_reporting_disable(dev);
        if (resp.err) {
            snprintf(resp.err_msg, sizeof(resp.err_msg), "Could not disable data reporting (%d)",
                     resp.err);
        }
    }

    if (resp.err == 0) {
        LOG_DBG("Sending cmd...");

        for (int i = 0; i < cmd_bytes; i++) {
            resp.err = ps2_write(ps2_device, cmd[i]);
            if (resp.err) {
                snprintf(resp.err_msg, sizeof(resp.err_msg), "Could not send cmd byte %d/%d (%d)",
                         i + 1, cmd_bytes, resp.err);
                if (i > 0 && cmd[0] == '\xe2') {
                    LOG_WRN("Partial 0xE2 extended command: %d/%d bytes sent. "
                            "Sleeping 25ms to let TP command parser timeout "
                            "and discard the partial sequence.",
                            i, cmd_bytes);
                    k_msleep(25);
                }
                break;
            }
        }
    }

    if (resp.err == 0 && arg != NULL) {
        LOG_DBG("Sending arg...");
        resp.err = ps2_write(ps2_device, *arg);
        if (resp.err) {
            snprintf(resp.err_msg, sizeof(resp.err_msg), "Could not send arg (%d)", resp.err);
            if (cmd[0] == '\xe2') {
                LOG_WRN("0xE2 extended command sent but arg byte failed. "
                        "Sleeping 25ms to let TP command parser timeout "
                        "and discard the pending write.");
                k_msleep(25);
            }
        }
    }

    if (resp.err == 0 && resp_len > 0) {
        LOG_DBG("Reading response...");
        for (int i = 0; i < resp_len; i++) {
            resp.err = ps2_read(ps2_device, &resp.resp_buffer[i]);
            if (resp.err) {
                snprintf(resp.err_msg, sizeof(resp.err_msg),
                         "Could not read response cmd byte %d/%d (%d)", i + 1, resp_len, resp.err);
                break;
            }
        }
    }

    if (pause_reporting == true && prev_activity_reporting_on == true) {
        LOG_DBG("Enabling mouse activity reporting...");

        err = zmk_mouse_ps2_activity_reporting_enable(dev);
        if (err) {
            // Don't overwrite existing error
            if (resp.err == 0) {
                resp.err = err;
                snprintf(resp.err_msg, sizeof(resp.err_msg),
                         "Could not re-enable data reporting (%d)", err);
            }
        }
    }

    return resp;
}

int zmk_mouse_ps2_activity_reporting_enable(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;
    const struct device *ps2_device = config->ps2_device;

    if (data->activity_reporting_on == true) {
        return 0;
    }

    uint8_t cmd = MOUSE_PS2_CMD_ENABLE_REPORTING[0];
    int err = ps2_write(ps2_device, cmd);
    if (err) {
        LOG_ERR("Could not enable data reporting: %d", err);
        return err;
    }

    err = ps2_enable_callback(ps2_device);
    if (err) {
        LOG_ERR("Could not enable ps2 callback: %d", err);
        return err;
    }

    data->activity_reporting_on = true;

    return 0;
}

int zmk_mouse_ps2_activity_reporting_disable(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;
    const struct device *ps2_device = config->ps2_device;

    if (data->activity_reporting_on == false) {
        return 0;
    }

    uint8_t cmd = MOUSE_PS2_CMD_DISABLE_REPORTING[0];
    int err = ps2_write(ps2_device, cmd);
    if (err) {
        LOG_ERR("Could not disable data reporting: %d", err);
        return err;
    }

    err = ps2_disable_callback(ps2_device);
    if (err) {
        LOG_ERR("Could not disable ps2 callback: %d", err);
        return err;
    }

    data->activity_reporting_on = false;

    return 0;
}

/*
 * Idle Power Management
 *
 * After CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM_TIMEOUT_MS of no trackpoint
 * packets, the driver disables the PS/2 callback, drains in-flight
 * bytes, and suspends the UART peripheral (which also applies the
 * sleep pinctrl state to disconnect the RX input buffer).
 *
 * Reporting is deliberately left enabled (no F5) so the trackpoint
 * can still drive the data line low on motion, firing a GPIO
 * falling-edge interrupt that wakes us (if wake-gpios is configured).
 *
 * Wake is triggered by the GPIO interrupt (trackpoint motion) or by
 * ZMK activity events (local keypress).  The UART is resumed and the
 * PS/2 callback is re-enabled (no F4 needed since reporting was
 * never disabled).
 */

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM)

static void tp_idle_pm_wake_gpio_isr(const struct device *port,
                                     struct gpio_callback *cb, uint32_t pins) {
    struct zmk_mouse_ps2_data *data =
        CONTAINER_OF(cb, struct zmk_mouse_ps2_data, wake_gpio_cb);

    /* Just queue the wake; the handler disables the GPIO interrupt
     * and resumes the UART in thread context (the nRF GPIO driver's
     * interrupt reconfigure can grab a spinlock, which we'd rather
     * not do from ISR). */
    k_work_submit_to_queue(&tp_mgmt_wq, &data->idle_pm_wake_work);
}

static void tp_idle_pm_dormant_finish_handler(struct k_work *work);

static void tp_idle_pm_dormant_handler(struct k_work *work) {
    struct k_work_delayable *dwork = (struct k_work_delayable *)work;
    struct zmk_mouse_ps2_data *data =
        CONTAINER_OF(dwork, struct zmk_mouse_ps2_data, idle_pm_dormant_work);
    const struct device *dev = data->dev;
    const struct zmk_mouse_ps2_config *config = dev->config;
    int err;

    if (data->pm_state != TP_PM_ACTIVE) {
        return;
    }

    data->pm_state = TP_PM_ENTERING_DORMANT;

    LOG_INF("TP idle PM: entering DORMANT (phase 1: disable callback)");

    /* Cancel the liveness watchdog — UART will be suspended and no bytes
     * will arrive.  Without this, the watchdog fires on a dead bus and
     * exhausts its retries before the wake handler can restart it. */
    k_work_cancel_delayable(&data->liveness_watchdog);

    /* 1. Disable the PS/2 callback so no packets are processed during
     *    the transition.  Crucially, we do NOT send F5 (disable
     *    reporting) — the TP must remain in reporting-enabled state so
     *    it can transmit a start bit when the stick is moved, which is
     *    what fires the GPIO wake interrupt. */
    err = ps2_disable_callback(config->ps2_device);
    if (err) {
        LOG_WRN("TP idle PM: ps2_disable_callback failed (%d)", err);
    }

    /* 2. Schedule phase 2 after 5ms to drain in-flight UART bytes.
     *    This replaces the old k_sleep(5ms) that blocked the system
     *    workqueue. The delayable work yields back to sysworkq, so
     *    other work items (BLE, display) can run in the gap. */
    k_work_schedule_for_queue(&tp_mgmt_wq, &data->idle_pm_dormant_finish_work, K_MSEC(5));
}

static void tp_idle_pm_dormant_finish_handler(struct k_work *work) {
    struct k_work_delayable *dwork = (struct k_work_delayable *)work;
    struct zmk_mouse_ps2_data *data =
        CONTAINER_OF(dwork, struct zmk_mouse_ps2_data, idle_pm_dormant_finish_work);
    const struct device *dev = data->dev;
    const struct zmk_mouse_ps2_config *config = dev->config;
    int err;

    /* A wake or activity notification may have aborted the dormant
     * transition during the 5ms drain window by resetting pm_state
     * to ACTIVE.  Only proceed if we're still in ENTERING_DORMANT. */
    if (data->pm_state != TP_PM_ENTERING_DORMANT) {
        LOG_INF("TP idle PM: dormant phase 2 aborted (pm_state=%d)", data->pm_state);
        return;
    }

    LOG_INF("TP idle PM: entering DORMANT (phase 2: suspend UART)");

    /* Inhibit CLK before any pin transitions.  With CLK held LOW the
     * TP cannot clock data, so the UART pin disconnection during
     * suspend won't be interpreted as a host-initiated PS/2 write
     * that could corrupt TP registers (e.g. config byte 0x2C
     * controlling InvertX/InvertY/SwapXY orientation bits). */
    ps2_uart_inhibit_bus(config->ps2_device);
    k_busy_wait(100); /* PS/2 spec: host must hold CLK LOW ≥100µs */

    /* Stop diversity receiver BEFORE suspending UARTE0.
     * UARTE1 must release P0.17 so the GPIO wake interrupt can
     * detect the TP's start-bit falling edge during dormant. */
    ps2_uart_diversity_stop_rx();

    /* 3. Suspend the UART peripheral (applies sleep pinctrl automatically) */
    err = pm_device_action_run(data->uart_dev, PM_DEVICE_ACTION_SUSPEND);
    if (err && err != -EALREADY) {
        LOG_WRN("TP idle PM: UART suspend failed (%d)", err);
    }

    /* 4. Reclaim the data pin as a GPIO input with falling-edge interrupt.
     * The TP pulls the line low on its next transmission (PS/2 start
     * bit), which fires the ISR and wakes us.  This clears the
     * INPUT_DISCONNECT applied by the UART sleep pinctrl. */
    if (config->has_wake_gpio) {
        err = gpio_pin_configure_dt(&config->wake_gpio, GPIO_INPUT);
        if (err) {
            LOG_WRN("TP idle PM: wake GPIO configure failed (%d)", err);
        } else {
            err = gpio_pin_interrupt_configure_dt(&config->wake_gpio,
                                                  GPIO_INT_EDGE_TO_INACTIVE);
            if (err) {
                LOG_WRN("TP idle PM: wake GPIO interrupt failed (%d)", err);
            }
        }
    }

    /* Release CLK so the TP can transmit a start bit to trigger the
     * wake GPIO interrupt.  If the TP had buffered movement while
     * CLK was inhibited, it will immediately pull DATA LOW → wake
     * fires.  The wake handler always runs full recovery so any
     * bytes lost during this transition are harmless. */
    ps2_uart_release_bus(config->ps2_device);

    data->pm_state = TP_PM_DORMANT;
}

static void tp_idle_pm_wake_handler(struct k_work *work) {
    struct zmk_mouse_ps2_data *data =
        CONTAINER_OF(work, struct zmk_mouse_ps2_data, idle_pm_wake_work);
    const struct device *dev = data->dev;
    const struct zmk_mouse_ps2_config *config = dev->config;
    int err;

    if (data->pm_state == TP_PM_ENTERING_DORMANT) {
        /* Lightweight abort: phase 1 ran (callback disabled, liveness
         * cancelled) but UART is still active and TP is still reporting.
         * Just undo phase 1 and return to ACTIVE — no tare risk. */
        LOG_INF("TP idle PM: aborting dormant transition (activity during drain)");

        k_work_cancel_delayable(&data->idle_pm_dormant_finish_work);

        err = ps2_enable_callback(config->ps2_device);
        if (err) {
            LOG_WRN("TP idle PM: ps2_enable_callback failed on abort (%d)", err);
        }

        data->pm_state = TP_PM_ACTIVE;

        /* Restart liveness watchdog (phase 1 cancelled it) */
        data->last_byte_time = k_uptime_get();
        data->liveness_retries = 0;
        k_work_schedule_for_queue(&tp_mgmt_wq, &data->liveness_watchdog, K_SECONDS(5));

        /* Restart idle timer */
        k_work_reschedule_for_queue(&tp_mgmt_wq, &data->idle_pm_dormant_work,
                                    K_MSEC(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM_TIMEOUT_MS));
        return;
    }

    if (data->pm_state != TP_PM_DORMANT) {
        return;
    }

    LOG_INF("TP idle PM: waking to ACTIVE");

    /* Cancel any pending dormant transition (both phases) */
    k_work_cancel_delayable(&data->idle_pm_dormant_work);
    k_work_cancel_delayable(&data->idle_pm_dormant_finish_work);

    /* 1. Disable the wake GPIO interrupt (idempotent — ISR may have
     * already done this) and release the pin so the UART pinctrl can
     * reclaim it on resume. */
    if (config->has_wake_gpio) {
        gpio_pin_interrupt_configure_dt(&config->wake_gpio, GPIO_INT_DISABLE);
        /* DISCONNECTED — hand the pin back to the pinctrl framework */
        gpio_pin_configure_dt(&config->wake_gpio, GPIO_DISCONNECTED);
    }

    /* 2. Resume the UART peripheral with retry (restores default pinctrl).
     *    If resume fails after all attempts, stay in DORMANT to avoid an
     *    unrecoverable broken-ACTIVE state (driver thinks it's alive but
     *    UART is dead → no TP data → idle timer expires → dormant → wake
     *    → fail again → infinite cycle). */
    err = -EAGAIN;
    for (int attempt = 0; attempt < 3; attempt++) {
        err = pm_device_action_run(data->uart_dev, PM_DEVICE_ACTION_RESUME);
        if (err == 0 || err == -EALREADY) {
            break;
        }
        LOG_WRN("TP idle PM: UART resume attempt %d/3 failed (%d)", attempt + 1, err);
        k_busy_wait(500); /* 500µs between retries — short enough to not block sysworkq */
    }
    if (err && err != -EALREADY) {
        LOG_ERR("TP idle PM: UART resume failed after 3 attempts (%d), staying DORMANT", err);
        /* Re-arm the wake GPIO so another motion can retry. */
        if (config->has_wake_gpio) {
            gpio_pin_configure_dt(&config->wake_gpio, GPIO_INPUT);
            gpio_pin_interrupt_configure_dt(&config->wake_gpio,
                                            GPIO_INT_EDGE_TO_INACTIVE);
        }
        return;
    }

    /* 3. Restore the UART error interrupt which the Zephyr nRF UARTE
     *    PM driver does not save/restore across suspend/resume
     *    (only ENDRX is preserved).  Without this, framing/parity
     *    errors after wake are silently swallowed. */
    uart_irq_err_enable(data->uart_dev);

    /* Restart diversity receiver now that UARTE0 is active again. */
    ps2_uart_diversity_start_rx();

    /* 4. Clear stale self-reset flag to avoid misalignment.  If the TP
     *    sent 0xAA just before suspend and we never received the 0x00,
     *    the flag would stick and cause the first real movement byte
     *    to be swallowed. */
    data->tp_self_reset_pending = false;

    /* Transition to ACTIVE before recovery so the watchdog handler
     * and activity callback see the correct PM state. */
    data->pm_state = TP_PM_ACTIVE;

    /* 5. Full recovery: re-apply all settings + F4.
     *
     *    The UART pin transition during dormant entry can cause the TP
     *    to misinterpret pin glitches as a host-initiated PS/2 write,
     *    corrupting registers (especially config byte 0x2C which
     *    controls InvertX/InvertY/SwapXY orientation bits).  CLK
     *    inhibition during dormant entry (see dormant_finish_handler)
     *    prevents this, but we run recovery unconditionally as a
     *    safety net — it's idempotent (~162ms, batch-protected).
     *
     *    Also handles the case where the TP self-reset during dormant
     *    (ESD, power glitch), which reverts all registers to factory
     *    defaults and disables reporting.
     *
     *    Previously this was deferred to the liveness watchdog, but
     *    that only fires when the TP is silent.  If the TP is still
     *    reporting (with corrupted orientation), the watchdog sees
     *    activity and never triggers recovery. */
    int ret = zmk_mouse_ps2_tp_recover_and_enable(dev);
    if (ret < 0) {
        LOG_ERR("TP idle PM: wake recovery F4 failed (%d)", ret);
    } else if (ret > 0) {
        LOG_WRN("TP idle PM: wake recovery: %d setting(s) failed", ret);
    }

    /* 6. Restart the liveness watchdog and idle timer. */
    data->last_byte_time = k_uptime_get();
    data->liveness_retries = 0;
    k_work_schedule_for_queue(&tp_mgmt_wq, &data->liveness_watchdog, K_SECONDS(5));

    /* Restart the idle timer */
    k_work_reschedule_for_queue(&tp_mgmt_wq, &data->idle_pm_dormant_work,
                                K_MSEC(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM_TIMEOUT_MS));
}

static void tp_idle_pm_notify_activity(struct zmk_mouse_ps2_data *data) {
    /* Reschedule the idle timer — trackpoint is active.
     * Also cancel any in-progress dormant phase 2 (finish work) so
     * activity during the 5ms drain window aborts the transition.
     * If phase 1 already disabled the PS/2 callback, re-enable it.
     *
     * k_work_cancel_delayable returns >0 if work was pending and
     * successfully cancelled.  If it returns 0, the work is either
     * idle (no transition in progress) or already executing (the
     * narrow S8 race — let it complete; GPIO wake self-recovers). */
    int cancel_ret = k_work_cancel_delayable(&data->idle_pm_dormant_finish_work);
    if (cancel_ret > 0) {
        /* Phase 2 was pending — phase 1 already disabled the callback.
         * Re-enable it since we're aborting the dormant transition. */
        const struct device *dev = data->dev;
        const struct zmk_mouse_ps2_config *config = dev->config;
        int err = ps2_enable_callback(config->ps2_device);
        if (err) {
            LOG_WRN("TP idle PM: ps2_enable_callback failed on abort (%d)", err);
        }

        /* Restore pm_state — phase 1 set it to ENTERING_DORMANT */
        data->pm_state = TP_PM_ACTIVE;

        /* Restart liveness watchdog (phase 1 cancelled it) */
        data->last_byte_time = k_uptime_get();
        data->liveness_retries = 0;
        k_work_schedule_for_queue(&tp_mgmt_wq, &data->liveness_watchdog, K_SECONDS(5));
    }
    k_work_reschedule_for_queue(&tp_mgmt_wq, &data->idle_pm_dormant_work,
                                K_MSEC(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM_TIMEOUT_MS));
}

/*
 * ZMK event listeners for idle PM wake.
 *
 * We subscribe to both activity_state_changed and position_state_changed
 * as belt-and-suspenders: the former fires when ZMK's activity subsystem
 * detects input, the latter fires directly on local key events.  Both
 * are confirmed to run on peripheral builds.
 */

static int tp_idle_pm_activity_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    if (ev == NULL || ev->state != ZMK_ACTIVITY_ACTIVE) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    /* Wake all driver instances */
    #define TP_IDLE_PM_WAKE_N(n)                                              \
        if (data##n.pm_state == TP_PM_DORMANT ||                              \
            data##n.pm_state == TP_PM_ENTERING_DORMANT) {                     \
            k_work_submit_to_queue(&tp_mgmt_wq, &data##n.idle_pm_wake_work);  \
        }
    DT_INST_FOREACH_STATUS_OKAY(TP_IDLE_PM_WAKE_N)

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(tp_idle_pm_activity, tp_idle_pm_activity_listener);
ZMK_SUBSCRIPTION(tp_idle_pm_activity, zmk_activity_state_changed);

static int tp_idle_pm_position_listener(const zmk_event_t *eh) {
    /* Any key press/release — wake the TP if dormant or entering dormant */
    #define TP_IDLE_PM_POS_WAKE_N(n)                                          \
        if (data##n.pm_state == TP_PM_DORMANT ||                              \
            data##n.pm_state == TP_PM_ENTERING_DORMANT) {                     \
            k_work_submit_to_queue(&tp_mgmt_wq, &data##n.idle_pm_wake_work);  \
        }
    DT_INST_FOREACH_STATUS_OKAY(TP_IDLE_PM_POS_WAKE_N)

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(tp_idle_pm_position, tp_idle_pm_position_listener);
ZMK_SUBSCRIPTION(tp_idle_pm_position, zmk_position_state_changed);

#endif /* CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM */

/*
 * PS/2 Command Helpers
 */

int zmk_mouse_ps2_array_get_elem_index(int elem, int *array, size_t array_size) {
    int elem_index = -1;
    for (int i = 0; i < array_size; i++) {
        if (array[i] == elem) {
            elem_index = i;
            break;
        }
    }

    return elem_index;
}

int zmk_mouse_ps2_array_get_next_elem(int elem, int *array, size_t array_size) {
    int elem_index = zmk_mouse_ps2_array_get_elem_index(elem, array, array_size);
    if (elem_index == -1) {
        return -1;
    }

    int next_index = elem_index + 1;
    if (next_index >= array_size) {
        return -1;
    }

    return array[next_index];
}

int zmk_mouse_ps2_array_get_prev_elem(int elem, int *array, size_t array_size) {
    int elem_index = zmk_mouse_ps2_array_get_elem_index(elem, array, array_size);
    if (elem_index == -1) {
        return -1;
    }

    int prev_index = elem_index - 1;
    if (prev_index < 0 || prev_index >= array_size) {
        return -1;
    }

    return array[prev_index];
}

/*
 * PS/2 Commands
 */

int zmk_mouse_ps2_reset(const struct device *dev, const struct device *ps2_device) {
    struct zmk_mouse_ps2_send_cmd_resp resp =
        zmk_mouse_ps2_send_cmd(dev,
                               MOUSE_PS2_CMD_RESET, sizeof(MOUSE_PS2_CMD_RESET), NULL,
                               MOUSE_PS2_CMD_RESET_RESP_LEN, false);
    if (resp.err) {
        LOG_ERR("Could not send reset cmd");
    }

    return resp.err;
}

int zmk_mouse_ps2_set_sampling_rate(const struct device *dev, uint8_t sampling_rate) {
    struct zmk_mouse_ps2_data *data = dev->data;

    int rate_idx = zmk_mouse_ps2_array_get_elem_index(sampling_rate, allowed_sampling_rates,
                                                      ARRAY_SIZE(allowed_sampling_rates));
    if (rate_idx == -1) {
        LOG_ERR("Requested to set illegal sampling rate: %d", sampling_rate);
        return -1;
    }

    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_SET_SAMPLING_RATE, sizeof(MOUSE_PS2_CMD_SET_SAMPLING_RATE), &sampling_rate,
        MOUSE_PS2_CMD_SET_SAMPLING_RATE_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set sample rate to %d", sampling_rate);
        return resp.err;
    }

    data->sampling_rate = sampling_rate;

    LOG_INF("Successfully set sampling rate to %d", sampling_rate);

    return resp.err;
}

int zmk_mouse_ps2_get_device_id(const struct device *dev, uint8_t *device_id) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_GET_DEVICE_ID, sizeof(MOUSE_PS2_CMD_GET_DEVICE_ID), NULL, 1, true);
    if (resp.err) {
        LOG_ERR("Could not get device id");
        return resp.err;
    }

    *device_id = resp.resp_buffer[0];

    return 0;
}

int zmk_mouse_ps2_set_packet_mode(const struct device *dev, zmk_mouse_ps2_packet_mode mode) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (mode == MOUSE_PS2_PACKET_MODE_PS2_DEFAULT) {
        // Do nothing. Mouse devices enable this by
        // default.
        return 0;
    }

    bool prev_activity_reporting_on = data->activity_reporting_on;
    zmk_mouse_ps2_activity_reporting_disable(dev);

    // Setting a mouse mode is a bit like using a cheat code
    // in a video game.
    // You have to send a specific sequence of sampling rates.
    if (mode == MOUSE_PS2_PACKET_MODE_SCROLL) {

        zmk_mouse_ps2_set_sampling_rate(dev, 200);
        zmk_mouse_ps2_set_sampling_rate(dev, 100);
        zmk_mouse_ps2_set_sampling_rate(dev, 80);
    }

    // Scroll mouse + 5 buttons mode can be enabled with the
    // following sequence, but since I don't have a mouse to
    // test it, I am commenting it out for now.
    // else if(mode == MOUSE_PS2_PACKET_MODE_SCROLL_5_BUTTONS) {

    //     zmk_mouse_ps2_set_sampling_rate(dev, 200);
    //     zmk_mouse_ps2_set_sampling_rate(dev, 200);
    //     zmk_mouse_ps2_set_sampling_rate(dev, 80);
    // }

    uint8_t device_id;
    int err = zmk_mouse_ps2_get_device_id(dev, &device_id);
    if (err) {
        LOG_ERR("Could not enable packet mode %d. Failed to get device id with "
                "error %d",
                mode, err);
    } else {
        if (device_id == 0x00) {
            LOG_ERR("Could not enable packet mode %d. The device does not "
                    "support it",
                    mode);

            data->packet_mode = MOUSE_PS2_PACKET_MODE_PS2_DEFAULT;
            err = 1;
        } else if (device_id == 0x03 || device_id == 0x04) {
            LOG_INF("Successfully activated packet mode %d. Mouse returned "
                    "device id: %d",
                    mode, device_id);

            data->packet_mode = MOUSE_PS2_PACKET_MODE_SCROLL;
            err = 0;
        }
        // else if(device_id == 0x04) {
        //     LOG_INF(
        //         "Successfully activated packet mode %d. Mouse returned device "
        //         "id: %d", mode, device_id
        //     );

        //     data->packet_mode = MOUSE_PS2_PACKET_MODE_SCROLL_5_BUTTONS;
        //     err = 0;
        // }
        else {
            LOG_ERR("Could not enable packet mode %d. Received an invalid "
                    "device id: %d",
                    mode, device_id);

            data->packet_mode = MOUSE_PS2_PACKET_MODE_PS2_DEFAULT;
            err = 1;
        }
    }

    // Restore sampling rate to prev value
    zmk_mouse_ps2_set_sampling_rate(dev, data->sampling_rate);

    if (prev_activity_reporting_on == true) {
        zmk_mouse_ps2_activity_reporting_enable(dev);
    }

    return err;
}

/*
 * Trackpoint Commands
 */

int zmk_mouse_ps2_tp_get_secondary_id(const struct device *dev,
                                      uint8_t *manufacturer_id, uint8_t *secondary_id) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_SECONDARY_ID, sizeof(MOUSE_PS2_CMD_TP_GET_SECONDARY_ID), NULL,
        MOUSE_PS2_CMD_TP_GET_SECONDARY_ID_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get secondary id");
        return resp.err;
    }

    *manufacturer_id = resp.resp_buffer[0];
    *secondary_id = resp.resp_buffer[1];

    return 0;
}

int zmk_mouse_ps2_tp_get_rom_id(const struct device *dev, uint8_t *rom_id) {
    struct zmk_mouse_ps2_send_cmd_resp resp =
        zmk_mouse_ps2_send_cmd(dev,
                               MOUSE_PS2_CMD_TP_GET_ROM_ID, sizeof(MOUSE_PS2_CMD_TP_GET_ROM_ID),
                               NULL, MOUSE_PS2_CMD_TP_GET_ROM_ID_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get secondary id");
        return resp.err;
    }

    *rom_id = resp.resp_buffer[0];

    return 0;
}

char *zmk_mouse_ps2_get_manufacturer_str(uint8_t manufacturer_id) {

    switch (manufacturer_id) {
    case 0x1:
        return "IBM";
    case 0x2:
        return "Alps";
    case 0x3:
        return "Elan";
    case 0x4:
        return "NXP";
    case 0x5:
        return "JYT Synaptics";
    case 0x6:
        return "Synaptics";
    }

    return "Unknown";
}

// On non-trackpoints this command is not supported and returns nothing.
//
// On trackpoints it returns the manufacturer id and firmware id.
//
// Trackpoints from IBM/Lenovo laptops up until aproximately 2016 used IBM
// trackpoints. After that they started to use other manufacturers.
//
// Page 19 of the IBM TP spec describes the features of different firmware ids.
int zmk_mouse_ps2_tp_get_device_info(const struct device *dev,
                                     bool *is_tp, uint8_t *tp_manufacturer_id,
                                     uint8_t *tp_secondary_id, uint8_t *tp_rom_id, char *device_str,
                                     int device_str_size) {

    int err = zmk_mouse_ps2_tp_get_secondary_id(dev, tp_manufacturer_id, tp_secondary_id);
    if (err) {
        // Only TPs implement this command. So, if it fails, it means the
        // device is not a TP.

        *is_tp = false;
        *tp_manufacturer_id = 0x0;
        *tp_secondary_id = 0x0;
        *tp_rom_id = 0x0;

        snprintf(device_str, device_str_size, "Generic PS/2 Mouse");

        return 0;
    }

    *is_tp = true;

    err = zmk_mouse_ps2_tp_get_rom_id(dev, tp_rom_id);
    if (err) {
        LOG_ERR("Could not determine TP rom id: %d", err);
        *tp_rom_id = 0x0;
        err = -1;
    }

    char *manufacturer_str = zmk_mouse_ps2_get_manufacturer_str(*tp_manufacturer_id);

    snprintf(device_str, device_str_size,
             "Trackpoint by %s (0x%02X); Secondary ID: 0x%02X; Rom Version: %02X", manufacturer_str,
             *tp_manufacturer_id, *tp_secondary_id, *tp_rom_id);

    return err;
}

int zmk_mouse_ps2_tp_get_config_byte(const struct device* dev, uint8_t *config_byte) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_CONFIG_BYTE, sizeof(MOUSE_PS2_CMD_TP_GET_CONFIG_BYTE), NULL,
        MOUSE_PS2_CMD_TP_GET_CONFIG_BYTE_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not read trackpoint config byte");
        return resp.err;
    }

    *config_byte = resp.resp_buffer[0];

    return 0;
}

int zmk_mouse_ps2_tp_set_config_byte_direct(const struct device *dev, uint8_t desired) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_CONFIG_BYTE, sizeof(MOUSE_PS2_CMD_TP_SET_CONFIG_BYTE), &desired,
        MOUSE_PS2_CMD_TP_SET_CONFIG_BYTE_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not write trackpoint config byte 0x%02x", desired);
        return resp.err;
    }

    return 0;
}

int zmk_mouse_ps2_tp_set_config_option(const struct device *dev,
                                       int config_bit, bool enabled, char *descr) {
    uint8_t config_byte;
    int err = zmk_mouse_ps2_tp_get_config_byte(dev, &config_byte);
    if (err) {
        return err;
    }

    bool is_enabled = MOUSE_PS2_GET_BIT(config_byte, config_bit);

    if (is_enabled == enabled) {
        LOG_DBG("Trackpoint %s was already %s... not doing anything.", descr,
                is_enabled ? "enabled" : "disabled");
        return 0;
    }

    LOG_DBG("Setting trackpoint %s: %s", descr, enabled ? "enabled" : "disabled");

    MOUSE_PS2_SET_BIT(config_byte, enabled, config_bit);

    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_CONFIG_BYTE, sizeof(MOUSE_PS2_CMD_TP_SET_CONFIG_BYTE), &config_byte,
        MOUSE_PS2_CMD_TP_SET_CONFIG_BYTE_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set trackpoint %s to %s", descr, enabled ? "enabled" : "disabled");
        return resp.err;
    }

    LOG_INF("Successfully set config option %s to %s", descr, enabled ? "enabled" : "disabled");

    return 0;
}

int zmk_mouse_ps2_tp_press_to_select_set(const struct device *dev, bool enabled) {
    int err = zmk_mouse_ps2_tp_set_config_option(dev,
                                                 MOUSE_PS2_TP_CONFIG_BIT_PRESS_TO_SELECT, enabled,
                                                 "Press To Select");

    return err;
}

int zmk_mouse_ps2_tp_invert_x_set(const struct device *dev, bool enabled) {
    int err = zmk_mouse_ps2_tp_set_config_option(dev,
                                                 MOUSE_PS2_TP_CONFIG_BIT_INVERT_X, enabled,
                                                 "Invert X");

    return err;
}

int zmk_mouse_ps2_tp_invert_y_set(const struct device *dev, bool enabled) {
    int err = zmk_mouse_ps2_tp_set_config_option(dev,
                                                 MOUSE_PS2_TP_CONFIG_BIT_INVERT_Y, enabled,
                                                 "Invert Y");

    return err;
}

int zmk_mouse_ps2_tp_swap_xy_set(const struct device *dev, bool enabled) {
    int err = zmk_mouse_ps2_tp_set_config_option(dev,
                                                 MOUSE_PS2_TP_CONFIG_BIT_SWAP_XY, enabled,
                                                 "Swap XY");

    return err;
}

int zmk_mouse_ps2_tp_sensitivity_get(const struct device *dev, uint8_t *sensitivity) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_SENSITIVITY, sizeof(MOUSE_PS2_CMD_TP_GET_SENSITIVITY), NULL,
        MOUSE_PS2_CMD_TP_GET_SENSITIVITY_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint sensitivity");
        return resp.err;
    }

    // Convert uint8_t to float
    // 0x80 (128) represents 1.0
    uint8_t sensitivity_int = resp.resp_buffer[0];
    *sensitivity = sensitivity_int;

    LOG_DBG("Trackpoint sensitivity is %d", sensitivity_int);

    return 0;
}

int zmk_mouse_ps2_tp_sensitivity_set(const struct device *dev, int sensitivity) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (sensitivity < MOUSE_PS2_CMD_TP_SET_SENSITIVITY_MIN ||
        sensitivity > MOUSE_PS2_CMD_TP_SET_SENSITIVITY_MAX) {
        LOG_ERR("Invalid sensitivity value %d. Min: %d; Max: %d", sensitivity,
                MOUSE_PS2_CMD_TP_SET_SENSITIVITY_MIN, MOUSE_PS2_CMD_TP_SET_SENSITIVITY_MAX);
        return 1;
    }

    uint8_t arg = sensitivity;

    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_SENSITIVITY, sizeof(MOUSE_PS2_CMD_TP_SET_SENSITIVITY), &arg,
        MOUSE_PS2_CMD_TP_SET_SENSITIVITY_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set sensitivity to %d", sensitivity);
        return resp.err;
    }

    data->tp_sensitivity = sensitivity;

    LOG_INF("Successfully set TP sensitivity to %d", sensitivity);

    return 0;
}

int zmk_mouse_ps2_tp_sensitivity_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;

    int new_val = data->tp_sensitivity + amount;

    LOG_INF("Setting trackpoint sensitivity to %d", new_val);
    int err = zmk_mouse_ps2_tp_sensitivity_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }

    return err;
}

int zmk_mouse_ps2_tp_sensitivity_change(int amount) {

    #define ZMK_PS2_MOUSE_DEFINE_SETTINGS_SEN_CHG_DEV(n) \
        zmk_mouse_ps2_tp_sensitivity_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_SETTINGS_SEN_CHG_DEV)

    return 0;
}

int zmk_mouse_ps2_tp_negative_inertia_get(const struct device *dev, uint8_t *neg_inertia) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_NEG_INERTIA, sizeof(MOUSE_PS2_CMD_TP_GET_NEG_INERTIA), NULL,
        MOUSE_PS2_CMD_TP_GET_NEG_INERTIA_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint negative inertia");
        return resp.err;
    }

    uint8_t neg_inertia_int = resp.resp_buffer[0];
    *neg_inertia = neg_inertia_int;

    LOG_DBG("Trackpoint negative inertia is %d", neg_inertia_int);

    return 0;
}

int zmk_mouse_ps2_tp_neg_inertia_set(const struct device *dev, int neg_inertia) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (neg_inertia < MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_MIN ||
        neg_inertia > MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_MAX) {
        LOG_ERR("Invalid negative inertia value %d. Min: %d; Max: %d", neg_inertia,
                MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_MIN, MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_MAX);
        return 1;
    }

    uint8_t arg = neg_inertia;

    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_NEG_INERTIA, sizeof(MOUSE_PS2_CMD_TP_SET_NEG_INERTIA), &arg,
        MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set negative inertia to %d", neg_inertia);
        return resp.err;
    }

    data->tp_neg_inertia = neg_inertia;

    LOG_INF("Successfully set TP negative inertia to %d", neg_inertia);

    return 0;
}

int zmk_mouse_ps2_tp_neg_inertia_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;

    int new_val = data->tp_neg_inertia + amount;

    LOG_INF("Setting negative inertia to %d", new_val);
    int err = zmk_mouse_ps2_tp_neg_inertia_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }

    return err;
}

int zmk_mouse_ps2_tp_neg_inertia_change(int amount) {

    #define ZMK_PS2_MOUSE_DEFINE_NEG_INERTIA_CHG_DEV(n) \
        zmk_mouse_ps2_tp_neg_inertia_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_NEG_INERTIA_CHG_DEV)

    return 0;
}

int zmk_mouse_ps2_tp_value6_upper_plateau_speed_get(const struct device *dev, uint8_t *value6) {
    struct zmk_mouse_ps2_send_cmd_resp resp =
        zmk_mouse_ps2_send_cmd(dev,
                               MOUSE_PS2_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED,
                               sizeof(MOUSE_PS2_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED), NULL,
                               MOUSE_PS2_CMD_TP_GET_VALUE6_UPPER_PLATEAU_SPEED_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint value6 upper plateau speed");
        return resp.err;
    }

    uint8_t value6_int = resp.resp_buffer[0];
    *value6 = value6_int;

    LOG_DBG("Trackpoint value6 upper plateau speed is %d", value6_int);

    return 0;
}

int zmk_mouse_ps2_tp_value6_upper_plateau_speed_set(const struct device *dev, int value6) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (value6 < MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MIN ||
        value6 > MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MAX) {
        LOG_ERR("Invalid value6 upper plateau speed value %d. Min: %d; Max: %d", value6,
                MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MIN,
                MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_MAX);
        return 1;
    }

    uint8_t arg = value6;

    struct zmk_mouse_ps2_send_cmd_resp resp =
        zmk_mouse_ps2_send_cmd(dev,
                               MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED,
                               sizeof(MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED), &arg,
                               MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set value6 upper plateau speed to %d", value6);
        return resp.err;
    }

    data->tp_value6 = value6;

    LOG_INF("Successfully set TP value6 upper plateau speed to %d", value6);

    return 0;
}

int zmk_mouse_ps2_tp_value6_upper_plateau_speed_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;

    int new_val = data->tp_value6 + amount;

    LOG_INF("Setting value6 upper plateau speed to %d", new_val);
    int err = zmk_mouse_ps2_tp_value6_upper_plateau_speed_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }

    return err;
}

int zmk_mouse_ps2_tp_value6_upper_plateau_speed_change(int amount) {

    #define ZMK_PS2_MOUSE_DEFINE_VALUE6_CHG_DEV(n) \
        zmk_mouse_ps2_tp_value6_upper_plateau_speed_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_VALUE6_CHG_DEV)

    return 0;
}

int zmk_mouse_ps2_tp_pts_threshold_get(const struct device *dev, uint8_t *pts_threshold) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_PTS_THRESHOLD, sizeof(MOUSE_PS2_CMD_TP_GET_PTS_THRESHOLD), NULL,
        MOUSE_PS2_CMD_TP_GET_PTS_THRESHOLD_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint press-to-select threshold");
        return resp.err;
    }

    uint8_t pts_threshold_int = resp.resp_buffer[0];
    *pts_threshold = pts_threshold_int;

    LOG_DBG("Trackpoint press-to-select threshold is %d", pts_threshold_int);

    return 0;
}

int zmk_mouse_ps2_tp_pts_threshold_set(const struct device *dev, int pts_threshold) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (pts_threshold < MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_MIN ||
        pts_threshold > MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_MAX) {
        LOG_ERR("Invalid press-to-select threshold value %d. Min: %d; Max: %d", pts_threshold,
                MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_MIN, MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_MAX);
        return 1;
    }

    uint8_t arg = pts_threshold;

    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD, sizeof(MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD), &arg,
        MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set press-to-select threshold to %d", pts_threshold);
        return resp.err;
    }

    data->tp_pts_threshold = pts_threshold;

    LOG_INF("Successfully set TP press-to-select threshold to %d", pts_threshold);

    return 0;
}

int zmk_mouse_ps2_tp_pts_threshold_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;

    int new_val = data->tp_pts_threshold + amount;

    LOG_INF("Setting press-to-select threshold to %d", new_val);
    int err = zmk_mouse_ps2_tp_pts_threshold_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }

    return err;
}

int zmk_mouse_ps2_tp_pts_threshold_change(int amount) {

    #define ZMK_PS2_MOUSE_DEFINE_PTS_THRESHOLD_CHG_DEV(n) \
        zmk_mouse_ps2_tp_pts_threshold_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_PTS_THRESHOLD_CHG_DEV)

    return 0;
}

int zmk_mouse_ps2_tp_up_thresh_get(const struct device *dev, uint8_t *up_thresh) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_UP_THRESH, sizeof(MOUSE_PS2_CMD_TP_GET_UP_THRESH), NULL,
        MOUSE_PS2_CMD_TP_GET_UP_THRESH_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint up threshold");
        return resp.err;
    }

    *up_thresh = resp.resp_buffer[0];
    LOG_DBG("Trackpoint up threshold is %d", *up_thresh);
    return 0;
}

int zmk_mouse_ps2_tp_up_thresh_set(const struct device *dev, int up_thresh) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (up_thresh < MOUSE_PS2_CMD_TP_SET_UP_THRESH_MIN ||
        up_thresh > MOUSE_PS2_CMD_TP_SET_UP_THRESH_MAX) {
        LOG_ERR("Invalid up threshold value %d. Min: %d; Max: %d", up_thresh,
                MOUSE_PS2_CMD_TP_SET_UP_THRESH_MIN, MOUSE_PS2_CMD_TP_SET_UP_THRESH_MAX);
        return 1;
    }

    uint8_t arg = up_thresh;
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_UP_THRESH, sizeof(MOUSE_PS2_CMD_TP_SET_UP_THRESH), &arg,
        MOUSE_PS2_CMD_TP_SET_UP_THRESH_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set up threshold to %d", up_thresh);
        return resp.err;
    }

    data->tp_up_thresh = up_thresh;
    LOG_INF("Successfully set TP up threshold to %d", up_thresh);
    return 0;
}

int zmk_mouse_ps2_tp_up_thresh_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;
    int new_val = data->tp_up_thresh + amount;

    LOG_INF("Setting up threshold to %d", new_val);
    int err = zmk_mouse_ps2_tp_up_thresh_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }
    return err;
}

int zmk_mouse_ps2_tp_up_thresh_change(int amount) {
    #define ZMK_PS2_MOUSE_DEFINE_UP_THRESH_CHG_DEV(n) \
        zmk_mouse_ps2_tp_up_thresh_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_UP_THRESH_CHG_DEV)
    return 0;
}

int zmk_mouse_ps2_tp_z_time_get(const struct device *dev, uint8_t *z_time) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_Z_TIME, sizeof(MOUSE_PS2_CMD_TP_GET_Z_TIME), NULL,
        MOUSE_PS2_CMD_TP_GET_Z_TIME_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint z-time");
        return resp.err;
    }

    *z_time = resp.resp_buffer[0];
    LOG_DBG("Trackpoint z-time is %d", *z_time);
    return 0;
}

int zmk_mouse_ps2_tp_z_time_set(const struct device *dev, int z_time) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (z_time < MOUSE_PS2_CMD_TP_SET_Z_TIME_MIN ||
        z_time > MOUSE_PS2_CMD_TP_SET_Z_TIME_MAX) {
        LOG_ERR("Invalid z-time value %d. Min: %d; Max: %d", z_time,
                MOUSE_PS2_CMD_TP_SET_Z_TIME_MIN, MOUSE_PS2_CMD_TP_SET_Z_TIME_MAX);
        return 1;
    }

    uint8_t arg = z_time;
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_Z_TIME, sizeof(MOUSE_PS2_CMD_TP_SET_Z_TIME), &arg,
        MOUSE_PS2_CMD_TP_SET_Z_TIME_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set z-time to %d", z_time);
        return resp.err;
    }

    data->tp_z_time = z_time;
    LOG_INF("Successfully set TP z-time to %d", z_time);
    return 0;
}

int zmk_mouse_ps2_tp_z_time_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;
    int new_val = data->tp_z_time + amount;

    LOG_INF("Setting z-time to %d", new_val);
    int err = zmk_mouse_ps2_tp_z_time_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }
    return err;
}

int zmk_mouse_ps2_tp_z_time_change(int amount) {
    #define ZMK_PS2_MOUSE_DEFINE_Z_TIME_CHG_DEV(n) \
        zmk_mouse_ps2_tp_z_time_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_Z_TIME_CHG_DEV)
    return 0;
}

int zmk_mouse_ps2_tp_jenks_curv_get(const struct device *dev, uint8_t *jenks_curv) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_JENKS_CURV, sizeof(MOUSE_PS2_CMD_TP_GET_JENKS_CURV), NULL,
        MOUSE_PS2_CMD_TP_GET_JENKS_CURV_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint jenks curvature");
        return resp.err;
    }

    *jenks_curv = resp.resp_buffer[0];
    LOG_DBG("Trackpoint jenks curvature is %d", *jenks_curv);
    return 0;
}

int zmk_mouse_ps2_tp_jenks_curv_set(const struct device *dev, int jenks_curv) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (jenks_curv < MOUSE_PS2_CMD_TP_SET_JENKS_CURV_MIN ||
        jenks_curv > MOUSE_PS2_CMD_TP_SET_JENKS_CURV_MAX) {
        LOG_ERR("Invalid jenks curvature value %d. Min: %d; Max: %d", jenks_curv,
                MOUSE_PS2_CMD_TP_SET_JENKS_CURV_MIN, MOUSE_PS2_CMD_TP_SET_JENKS_CURV_MAX);
        return 1;
    }

    uint8_t arg = jenks_curv;
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_JENKS_CURV, sizeof(MOUSE_PS2_CMD_TP_SET_JENKS_CURV), &arg,
        MOUSE_PS2_CMD_TP_SET_JENKS_CURV_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set jenks curvature to %d", jenks_curv);
        return resp.err;
    }

    data->tp_jenks_curv = jenks_curv;
    LOG_INF("Successfully set TP jenks curvature to %d", jenks_curv);
    return 0;
}

int zmk_mouse_ps2_tp_jenks_curv_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;
    int new_val = data->tp_jenks_curv + amount;

    LOG_INF("Setting jenks curvature to %d", new_val);
    int err = zmk_mouse_ps2_tp_jenks_curv_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }
    return err;
}

int zmk_mouse_ps2_tp_jenks_curv_change(int amount) {
    #define ZMK_PS2_MOUSE_DEFINE_JENKS_CURV_CHG_DEV(n) \
        zmk_mouse_ps2_tp_jenks_curv_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_JENKS_CURV_CHG_DEV)
    return 0;
}

int zmk_mouse_ps2_tp_drag_hysteresis_get(const struct device *dev, uint8_t *drag_hysteresis) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_DRAG_HYSTERESIS, sizeof(MOUSE_PS2_CMD_TP_GET_DRAG_HYSTERESIS), NULL,
        MOUSE_PS2_CMD_TP_GET_DRAG_HYSTERESIS_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint drag hysteresis");
        return resp.err;
    }

    *drag_hysteresis = resp.resp_buffer[0];
    LOG_DBG("Trackpoint drag hysteresis is %d", *drag_hysteresis);
    return 0;
}

int zmk_mouse_ps2_tp_drag_hysteresis_set(const struct device *dev, int drag_hysteresis) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (drag_hysteresis < MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_MIN ||
        drag_hysteresis > MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_MAX) {
        LOG_ERR("Invalid drag hysteresis value %d. Min: %d; Max: %d", drag_hysteresis,
                MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_MIN, MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_MAX);
        return 1;
    }

    uint8_t arg = drag_hysteresis;
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS, sizeof(MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS), &arg,
        MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set drag hysteresis to %d", drag_hysteresis);
        return resp.err;
    }

    data->tp_drag_hysteresis = drag_hysteresis;
    LOG_INF("Successfully set TP drag hysteresis to %d", drag_hysteresis);
    return 0;
}

int zmk_mouse_ps2_tp_drag_hysteresis_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;
    int new_val = data->tp_drag_hysteresis + amount;

    LOG_INF("Setting drag hysteresis to %d", new_val);
    int err = zmk_mouse_ps2_tp_drag_hysteresis_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }
    return err;
}

int zmk_mouse_ps2_tp_drag_hysteresis_change(int amount) {
    #define ZMK_PS2_MOUSE_DEFINE_DRAG_HYS_CHG_DEV(n) \
        zmk_mouse_ps2_tp_drag_hysteresis_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_DRAG_HYS_CHG_DEV)
    return 0;
}

int zmk_mouse_ps2_tp_min_drag_get(const struct device *dev, uint8_t *min_drag) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_MIN_DRAG, sizeof(MOUSE_PS2_CMD_TP_GET_MIN_DRAG), NULL,
        MOUSE_PS2_CMD_TP_GET_MIN_DRAG_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint minimum drag");
        return resp.err;
    }

    *min_drag = resp.resp_buffer[0];
    LOG_DBG("Trackpoint minimum drag is %d", *min_drag);
    return 0;
}

int zmk_mouse_ps2_tp_min_drag_set(const struct device *dev, int min_drag) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (min_drag < MOUSE_PS2_CMD_TP_SET_MIN_DRAG_MIN ||
        min_drag > MOUSE_PS2_CMD_TP_SET_MIN_DRAG_MAX) {
        LOG_ERR("Invalid minimum drag value %d. Min: %d; Max: %d", min_drag,
                MOUSE_PS2_CMD_TP_SET_MIN_DRAG_MIN, MOUSE_PS2_CMD_TP_SET_MIN_DRAG_MAX);
        return 1;
    }

    uint8_t arg = min_drag;
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_MIN_DRAG, sizeof(MOUSE_PS2_CMD_TP_SET_MIN_DRAG), &arg,
        MOUSE_PS2_CMD_TP_SET_MIN_DRAG_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set minimum drag to %d", min_drag);
        return resp.err;
    }

    data->tp_min_drag = min_drag;
    LOG_INF("Successfully set TP minimum drag to %d", min_drag);
    return 0;
}

int zmk_mouse_ps2_tp_min_drag_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;
    int new_val = data->tp_min_drag + amount;

    LOG_INF("Setting minimum drag to %d", new_val);
    int err = zmk_mouse_ps2_tp_min_drag_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }
    return err;
}

int zmk_mouse_ps2_tp_min_drag_change(int amount) {
    #define ZMK_PS2_MOUSE_DEFINE_MIN_DRAG_CHG_DEV(n) \
        zmk_mouse_ps2_tp_min_drag_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_MIN_DRAG_CHG_DEV)
    return 0;
}

int zmk_mouse_ps2_tp_reach_get(const struct device *dev, uint8_t *reach) {
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_GET_REACH, sizeof(MOUSE_PS2_CMD_TP_GET_REACH), NULL,
        MOUSE_PS2_CMD_TP_GET_REACH_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not get trackpoint reach");
        return resp.err;
    }

    *reach = resp.resp_buffer[0];
    LOG_DBG("Trackpoint reach is %d", *reach);
    return 0;
}

int zmk_mouse_ps2_tp_reach_set(const struct device *dev, int reach) {
    struct zmk_mouse_ps2_data *data = dev->data;

    if (reach < MOUSE_PS2_CMD_TP_SET_REACH_MIN ||
        reach > MOUSE_PS2_CMD_TP_SET_REACH_MAX) {
        LOG_ERR("Invalid reach value %d. Min: %d; Max: %d", reach,
                MOUSE_PS2_CMD_TP_SET_REACH_MIN, MOUSE_PS2_CMD_TP_SET_REACH_MAX);
        return 1;
    }

    uint8_t arg = reach;
    struct zmk_mouse_ps2_send_cmd_resp resp = zmk_mouse_ps2_send_cmd(
        dev,
        MOUSE_PS2_CMD_TP_SET_REACH, sizeof(MOUSE_PS2_CMD_TP_SET_REACH), &arg,
        MOUSE_PS2_CMD_TP_SET_REACH_RESP_LEN, true);
    if (resp.err) {
        LOG_ERR("Could not set reach to %d", reach);
        return resp.err;
    }

    data->tp_reach = reach;
    LOG_INF("Successfully set TP reach to %d", reach);
    return 0;
}

int zmk_mouse_ps2_tp_reach_change_dev(const struct device *dev, int amount) {
    struct zmk_mouse_ps2_data *data = dev->data;
    int new_val = data->tp_reach + amount;

    LOG_INF("Setting reach to %d", new_val);
    int err = zmk_mouse_ps2_tp_reach_set(dev, new_val);
    if (err == 0) {
        zmk_mouse_ps2_settings_save(dev);
    }
    return err;
}

int zmk_mouse_ps2_tp_reach_change(int amount) {
    #define ZMK_PS2_MOUSE_DEFINE_REACH_CHG_DEV(n) \
        zmk_mouse_ps2_tp_reach_change_dev(data##n.dev, amount);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_REACH_CHG_DEV)
    return 0;
}

/*
 * State Saving
 */

#if IS_ENABLED(CONFIG_SETTINGS)

int zmk_mouse_ps2_settings_save_setting(char *setting_name, const void *value, size_t val_len) {
    char setting_path[40];
    snprintf(setting_path, sizeof(setting_path), "%s/%s", MOUSE_PS2_SETTINGS_SUBTREE, setting_name);

    LOG_DBG("Saving setting to `%s`", setting_path);
    int err = settings_save_one(setting_path, value, val_len);
    if (err) {
        LOG_ERR("Could not save setting to `%s`: %d", setting_path, err);
    }

    return err;
}

int zmk_mouse_ps2_settings_reset_setting(char *setting_name) {
    char setting_path[40];
    snprintf(setting_path, sizeof(setting_path), "%s/%s", MOUSE_PS2_SETTINGS_SUBTREE, setting_name);

    LOG_DBG("Reseting setting `%s`", setting_path);
    int err = settings_delete(setting_path);
    if (err) {
        LOG_ERR("Could not reset setting `%s`", setting_path);
    }

    return err;
}

static void zmk_mouse_ps2_settings_save_work(struct k_work *work) {

    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct zmk_mouse_ps2_data *data = CONTAINER_OF(work_delayable,
                                                   struct zmk_mouse_ps2_data,
                                                   zmk_mouse_ps2_save_work);
    // const struct device *dev = data->dev;

    LOG_INF("Saving PS/2 Mouse Settings.");

    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_SENSITIVITY, &data->tp_sensitivity,
                                        sizeof(data->tp_sensitivity));
    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_NEG_INERTIA, &data->tp_neg_inertia,
                                        sizeof(data->tp_neg_inertia));
    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_VALUE6, &data->tp_value6,
                                        sizeof(data->tp_value6));
    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_PTS_THRESHOLD, &data->tp_pts_threshold,
                                        sizeof(data->tp_pts_threshold));
    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_UP_THRESH, &data->tp_up_thresh,
                                        sizeof(data->tp_up_thresh));
    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_Z_TIME, &data->tp_z_time,
                                        sizeof(data->tp_z_time));
    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_JENKS_CURV, &data->tp_jenks_curv,
                                        sizeof(data->tp_jenks_curv));
    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_DRAG_HYSTERESIS, &data->tp_drag_hysteresis,
                                        sizeof(data->tp_drag_hysteresis));
    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_MIN_DRAG, &data->tp_min_drag,
                                        sizeof(data->tp_min_drag));
    zmk_mouse_ps2_settings_save_setting(MOUSE_PS2_ST_TP_REACH, &data->tp_reach,
                                        sizeof(data->tp_reach));
}
#endif

int zmk_mouse_ps2_settings_save(const struct device *dev) {
    LOG_DBG("");
    struct zmk_mouse_ps2_data *data = dev->data;

#if IS_ENABLED(CONFIG_SETTINGS)
    int ret =
        k_work_reschedule(&data->zmk_mouse_ps2_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
#else
    return 0;
#endif
}

int zmk_mouse_ps2_settings_reset_dev(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;

    LOG_INF("Deleting runtime settings...");
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_SENSITIVITY);
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_NEG_INERTIA);
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_VALUE6);
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_PTS_THRESHOLD);
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_UP_THRESH);
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_Z_TIME);
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_JENKS_CURV);
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_DRAG_HYSTERESIS);
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_MIN_DRAG);
    zmk_mouse_ps2_settings_reset_setting(MOUSE_PS2_ST_TP_REACH);

    LOG_INF("Restoring default settings to TP..");
    data->tp_sensitivity = MOUSE_PS2_CMD_TP_SET_SENSITIVITY_DEFAULT;
    data->tp_neg_inertia = MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_DEFAULT;
    data->tp_value6 = MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_DEFAULT;
    data->tp_pts_threshold = MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_DEFAULT;
    data->tp_up_thresh = MOUSE_PS2_CMD_TP_SET_UP_THRESH_DEFAULT;
    data->tp_z_time = MOUSE_PS2_CMD_TP_SET_Z_TIME_DEFAULT;
    data->tp_jenks_curv = MOUSE_PS2_CMD_TP_SET_JENKS_CURV_DEFAULT;
    data->tp_drag_hysteresis = MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_DEFAULT;
    data->tp_min_drag = MOUSE_PS2_CMD_TP_SET_MIN_DRAG_DEFAULT;
    data->tp_reach = MOUSE_PS2_CMD_TP_SET_REACH_DEFAULT;

    int tp_failures = zmk_mouse_ps2_tp_apply_all_settings(dev);
    if (tp_failures > 0) {
        LOG_WRN("Settings reset: %d TP setting(s) failed to apply", tp_failures);
    }

    return 0;
}

int zmk_mouse_ps2_settings_reset() {

    #define ZMK_PS2_MOUSE_DEFINE_SETTINGS_RESET_DEV(n) \
        zmk_mouse_ps2_settings_reset_dev(data##n.dev);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_SETTINGS_RESET_DEV)

    return 0;
}

int zmk_mouse_ps2_settings_log_dev(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;

    char settings_str[512];

    snprintf(settings_str, sizeof(settings_str), " \n\
&mouse_ps2_conf = { \n\
    tp-sensitivity = <%d>; \n\
    tp-neg-inertia = <%d>; \n\
    tp-val6-upper-speed = <%d>; \n\
    tp-press-to-select-threshold = <%d>; \n\
    tp-up-thresh = <%d>; \n\
    tp-z-time = <%d>; \n\
    tp-jenks-curv = <%d>; \n\
    tp-drag-hysteresis = <%d>; \n\
    tp-min-drag = <%d>; \n\
    tp-reach = <%d>; \n\
}",
             data->tp_sensitivity, data->tp_neg_inertia, data->tp_value6,
             data->tp_pts_threshold, data->tp_up_thresh, data->tp_z_time,
             data->tp_jenks_curv, data->tp_drag_hysteresis, data->tp_min_drag,
             data->tp_reach);

    LOG_INF("Current settings... %s", settings_str);

    return 0;
}

int zmk_mouse_ps2_settings_log() {

    #define ZMK_PS2_MOUSE_DEFINE_SETTINGS_LOG_DEV(n) \
        zmk_mouse_ps2_settings_log_dev(data##n.dev);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_SETTINGS_LOG_DEV)

    return 0;
}

// This function is called when settings are loaded from flash by
// `settings_load_subtree`.
// It's called once for each PS/2 mouse setting that has been stored.
static int zmk_mouse_ps2_settings_restore_dev(const struct device *dev,
                                              const char *name, size_t len, settings_read_cb read_cb,
                                              void *cb_arg) {
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;

    uint8_t setting_val;

    if (len != sizeof(setting_val)) {
        LOG_ERR("Could not restore settings %s: Len mismatch", name);

        return -EINVAL;
    }

    int rc = read_cb(cb_arg, &setting_val, sizeof(setting_val));
    if (rc <= 0) {
        LOG_ERR("Could not restore setting %s: %d", name, rc);
        return -EINVAL;
    }

    if (data->is_trackpoint == false) {
        LOG_INF("Mouse device is not a trackpoint. Not restoring setting %s.", name);

        return 0;
    }

    LOG_INF("Restoring setting %s with value: %d", name, setting_val);

    if (strcmp(name, MOUSE_PS2_ST_TP_SENSITIVITY) == 0) {

        if (config->tp_sensitivity != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_sensitivity);

            return 0;
        }

        return zmk_mouse_ps2_tp_sensitivity_set(dev, setting_val);
    } else if (strcmp(name, MOUSE_PS2_ST_TP_NEG_INERTIA) == 0) {
        if (config->tp_neg_inertia != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_neg_inertia);

            return 0;
        }

        return zmk_mouse_ps2_tp_neg_inertia_set(dev, setting_val);
    } else if (strcmp(name, MOUSE_PS2_ST_TP_VALUE6) == 0) {
        if (config->tp_val6_upper_speed != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_val6_upper_speed);

            return 0;
        }

        return zmk_mouse_ps2_tp_value6_upper_plateau_speed_set(dev, setting_val);
    } else if (strcmp(name, MOUSE_PS2_ST_TP_PTS_THRESHOLD) == 0) {
        if (config->tp_press_to_select_threshold != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_press_to_select_threshold);

            return 0;
        }

        return zmk_mouse_ps2_tp_pts_threshold_set(dev, setting_val);
    } else if (strcmp(name, MOUSE_PS2_ST_TP_UP_THRESH) == 0) {
        if (config->tp_up_thresh != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_up_thresh);
            return 0;
        }
        return zmk_mouse_ps2_tp_up_thresh_set(dev, setting_val);
    } else if (strcmp(name, MOUSE_PS2_ST_TP_Z_TIME) == 0) {
        if (config->tp_z_time != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_z_time);
            return 0;
        }
        return zmk_mouse_ps2_tp_z_time_set(dev, setting_val);
    } else if (strcmp(name, MOUSE_PS2_ST_TP_JENKS_CURV) == 0) {
        if (config->tp_jenks_curv != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_jenks_curv);
            return 0;
        }
        return zmk_mouse_ps2_tp_jenks_curv_set(dev, setting_val);
    } else if (strcmp(name, MOUSE_PS2_ST_TP_DRAG_HYSTERESIS) == 0) {
        if (config->tp_drag_hysteresis != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_drag_hysteresis);
            return 0;
        }
        return zmk_mouse_ps2_tp_drag_hysteresis_set(dev, setting_val);
    } else if (strcmp(name, MOUSE_PS2_ST_TP_MIN_DRAG) == 0) {
        if (config->tp_min_drag != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_min_drag);
            return 0;
        }
        return zmk_mouse_ps2_tp_min_drag_set(dev, setting_val);
    } else if (strcmp(name, MOUSE_PS2_ST_TP_REACH) == 0) {
        if (config->tp_reach != -1) {
            LOG_WRN("Not restoring runtime settings for %s with value %d, because deviceconfig "
                    "defines the setting with value %d",
                    name, setting_val, config->tp_reach);
            return 0;
        }
        return zmk_mouse_ps2_tp_reach_set(dev, setting_val);
    }

    return -EINVAL;
}

static int zmk_mouse_ps2_settings_restore(const char *name, size_t len, settings_read_cb read_cb,
                                          void *cb_arg) {

    #define ZMK_PS2_MOUSE_DEFINE_SETTINGS_RESTORE_DEV(n) \
        zmk_mouse_ps2_settings_restore_dev(data##n.dev, name, len, read_cb, cb_arg);
    DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE_SETTINGS_RESTORE_DEV)

    return -EINVAL;
}

struct settings_handler zmk_mouse_ps2_settings_conf = {
    .name = MOUSE_PS2_SETTINGS_SUBTREE,
    .h_set = zmk_mouse_ps2_settings_restore,
};

int zmk_mouse_ps2_settings_init(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;

    k_work_init(&data->tp_self_reset_work, zmk_mouse_ps2_tp_self_reset_work_handler);
    k_work_init_delayable(&data->liveness_watchdog, zmk_mouse_ps2_liveness_watchdog_handler);

#if IS_ENABLED(CONFIG_SETTINGS)
    LOG_DBG("");

    settings_subsys_init();

    int err = settings_register(&zmk_mouse_ps2_settings_conf);
    if (err) {
        LOG_ERR("Failed to register the PS/2 mouse settings handler (err %d)", err);
        return err;
    }

    k_work_init_delayable(&data->zmk_mouse_ps2_save_work, zmk_mouse_ps2_settings_save_work);

    // This will load the settings and then call
    // `zmk_mouse_ps2_settings_restore`, which will set the settings
    settings_load_subtree(MOUSE_PS2_SETTINGS_SUBTREE);
#endif

    return 0;
}

/*
 * Init PS2 mouse
 */

static void zmk_mouse_ps2_init_thread(int dev_ptr, int unused);
int zmk_mouse_ps2_init_power_on_reset(const struct device *dev);
int zmk_mouse_ps2_init_wait_for_mouse(const struct device *dev);

static int zmk_mouse_ps2_init(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;
    // const struct zmk_mouse_ps2_config *config = dev->config;

    LOG_DBG("Inside zmk_mouse_ps2_init");
    data->dev = dev;

    LOG_DBG("Creating mouse_ps2 init thread.");
    k_thread_create(&data->thread, data->thread_stack,
                    MOUSE_PS2_THREAD_STACK_SIZE, (k_thread_entry_t)zmk_mouse_ps2_init_thread,
                    (struct device *)dev, 0, NULL, K_PRIO_COOP(MOUSE_PS2_THREAD_PRIORITY), 0,
                    K_MSEC(ZMK_MOUSE_PS2_INIT_THREAD_DELAY_MS));

    return 0;
}

static void zmk_mouse_ps2_init_thread(int dev_ptr, int unused) {
    int err;
    const struct device *dev = INT_TO_POINTER(dev_ptr);
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;

    zmk_mouse_ps2_init_power_on_reset(dev);

    LOG_INF("Waiting for mouse to connect...");
    err = zmk_mouse_ps2_init_wait_for_mouse(dev);
    if (err) {
        LOG_ERR("Could not init a mouse in %d attempts. Giving up. "
                "Power cycle the mouse and reset zmk to try again.",
                MOUSE_PS2_INIT_ATTEMPTS);
        return;
    }

    if (config->sampling_rate != MOUSE_PS2_CMD_SET_SAMPLING_RATE_DEFAULT) {

        LOG_INF("Setting sample rate to %d...", config->sampling_rate);
        err = zmk_mouse_ps2_set_sampling_rate(dev, config->sampling_rate);
        if (err) {
            LOG_ERR("Could not set sampling rate to %d: %d", config->sampling_rate, err);
            return;
        }
    }

    char device_descr[64] = "undetermined device";
    zmk_mouse_ps2_tp_get_device_info(dev,
                                     &data->is_trackpoint, &data->manufacturer_id,
                                     &data->secondary_id, &data->rom_id, device_descr,
                                     sizeof(device_descr));

    LOG_INF("Connected device is a %s", device_descr);

    if (data->is_trackpoint == true) {

        // Copy devicetree overrides into data struct (sentinel -1 = not set)
        if (config->tp_sensitivity != -1) {
            data->tp_sensitivity = config->tp_sensitivity;
        }
        if (config->tp_neg_inertia != -1) {
            data->tp_neg_inertia = config->tp_neg_inertia;
        }
        if (config->tp_val6_upper_speed != -1) {
            data->tp_value6 = config->tp_val6_upper_speed;
        }
        if (config->tp_press_to_select_threshold != -1) {
            data->tp_pts_threshold = config->tp_press_to_select_threshold;
        }
        if (config->tp_up_thresh != -1) {
            data->tp_up_thresh = config->tp_up_thresh;
        }
        if (config->tp_z_time != -1) {
            data->tp_z_time = config->tp_z_time;
        }
        if (config->tp_jenks_curv != -1) {
            data->tp_jenks_curv = config->tp_jenks_curv;
        }
        if (config->tp_drag_hysteresis != -1) {
            data->tp_drag_hysteresis = config->tp_drag_hysteresis;
        }
        if (config->tp_min_drag != -1) {
            data->tp_min_drag = config->tp_min_drag;
        }
        if (config->tp_reach != -1) {
            data->tp_reach = config->tp_reach;
        }

        /* Acquire a batch timeslot covering apply_all + scroll mode. */
        int batch_err = ps2_uart_timeslot_batch_begin();
        if (batch_err) {
            LOG_WRN("Init: batch timeslot unavailable (%d), "
                    "using per-byte protection", batch_err);
        }

        int tp_failures = zmk_mouse_ps2_tp_apply_all_settings(dev);
        if (tp_failures > 0) {
            LOG_WRN("Init: %d TP setting(s) failed to apply", tp_failures);
        }

        if (config->scroll_mode) {
            LOG_INF("Enabling scroll mode.");
            zmk_mouse_ps2_set_packet_mode(dev, MOUSE_PS2_PACKET_MODE_SCROLL);
        }

        ps2_uart_timeslot_batch_end();
    } else if (config->scroll_mode) {
        LOG_INF("Enabling scroll mode.");
        zmk_mouse_ps2_set_packet_mode(dev, MOUSE_PS2_PACKET_MODE_SCROLL);
    }

    zmk_mouse_ps2_settings_init(dev);

    // Configure read callback
    LOG_DBG("Configuring ps2 callback...");
#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_ENABLE_PS2_RESEND_CALLBACK)

    err = ps2_config(config->ps2_device, data->activity_callback,
                     data->activity_resend_callback);

#else

    err = ps2_config(config->ps2_device, data->activity_callback);

#endif /* IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_ENABLE_PS2_RESEND_CALLBACK) */

    if (err) {
        LOG_ERR("Could not configure ps2 interface: %d", err);
        return;
    }

    // Initialise the packet-buffer timeout BEFORE enabling data
    // reporting — as soon as activity_reporting_enable succeeds,
    // bytes can arrive and the callback handler schedules this
    // timeout.  If it's uninitialised, Zephyr dereferences a NULL
    // work handler and faults.
    k_work_init_delayable(&data->packet_buffer_timeout, zmk_mouse_ps2_activity_packet_timout);

    LOG_INF("Enabling data reporting and ps2 callback...");
    err = zmk_mouse_ps2_activity_reporting_enable(dev);
    if (err) {
        LOG_ERR("Could not activate ps2 callback: %d", err);
    } else {
        LOG_DBG("Successfully activated ps2 callback");
        data->last_byte_time = k_uptime_get();

        if (!tp_mgmt_wq_started) {
            k_work_queue_init(&tp_mgmt_wq);
            k_work_queue_start(&tp_mgmt_wq, tp_mgmt_wq_stack,
                               K_THREAD_STACK_SIZEOF(tp_mgmt_wq_stack),
                               TP_MGMT_WQ_PRIORITY, NULL);
            k_thread_name_set(&tp_mgmt_wq.thread, "tp_mgmt");
            tp_mgmt_wq_started = true;
        }

        k_work_schedule_for_queue(&tp_mgmt_wq, &data->liveness_watchdog, K_SECONDS(5));
    }

#if IS_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM)
    /*
     * The UART device sits one level above the PS/2 protocol device in
     * the devicetree: uart0 → uart_ps2 → mouse_ps2.  We need the UART
     * for PM suspend/resume.  DT_BUS(ps2_node) gives us uart0.
     */
    data->uart_dev = DEVICE_DT_GET(
        DT_BUS(DT_PHANDLE(DT_DRV_INST(0), ps2_device)));

    k_work_init_delayable(&data->idle_pm_dormant_work, tp_idle_pm_dormant_handler);
    k_work_init_delayable(&data->idle_pm_dormant_finish_work, tp_idle_pm_dormant_finish_handler);
    k_work_init(&data->idle_pm_wake_work, tp_idle_pm_wake_handler);
    data->pm_state = TP_PM_ACTIVE;

    /* Wire up the wake GPIO callback (interrupt is armed lazily in
     * the dormant handler, so the ISR doesn't fire during normal
     * UART operation) */
    if (config->has_wake_gpio) {
        if (device_is_ready(config->wake_gpio.port)) {
            gpio_init_callback(&data->wake_gpio_cb,
                               tp_idle_pm_wake_gpio_isr,
                               BIT(config->wake_gpio.pin));
            err = gpio_add_callback(config->wake_gpio.port,
                                    &data->wake_gpio_cb);
            if (err) {
                LOG_WRN("TP idle PM: gpio_add_callback failed (%d)", err);
            } else {
                LOG_INF("TP idle PM: wake GPIO armed on pin %d",
                        config->wake_gpio.pin);
            }
        } else {
            LOG_WRN("TP idle PM: wake GPIO port not ready");
        }
    }

    /* Start the idle timer */
    k_work_reschedule_for_queue(&tp_mgmt_wq, &data->idle_pm_dormant_work,
                                K_MSEC(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM_TIMEOUT_MS));
    LOG_INF("TP idle PM: enabled (timeout %d ms, wake-gpio=%s)",
            CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM_TIMEOUT_MS,
            config->has_wake_gpio ? "yes" : "no");
#endif

    return;
}

// Power-On-Reset for trackpoints (and possibly other devices).
// From the `IBM TrackPoint System Version 4.0 Engineering
// Specification`...
// "The TrackPoint logic shall execute a Power On Reset (POR) when power is
//  applied to the device. The POR shall be timed to occur 600 ms ± 20 % from
//  the time power is applied to the TrackPoint controller. Activity on the
//  clock and data lines is ignored prior to the completion of the diagnostic
//  sequence. (See RESET mode of operation.)"
int zmk_mouse_ps2_init_power_on_reset(const struct device *dev) {
    struct zmk_mouse_ps2_data *data = dev->data;
    const struct zmk_mouse_ps2_config *config = dev->config;

    // Check if the optional rst-gpios setting was set
    if (config->rst_gpio.port == NULL) {
        return 0;
    }

    LOG_INF("Performing Power-On-Reset on pin P%d.%02d...", config->rst_gpio_port_num,
            config->rst_gpio.pin);

    if (data->rst_gpio.port == NULL) {
        data->rst_gpio = config->rst_gpio;

        // Overwrite any user-provided flags from the devicetree
        data->rst_gpio.dt_flags = 0;
    }

    //  Set reset pin low...
    int err = gpio_pin_configure_dt(&data->rst_gpio, (GPIO_OUTPUT_HIGH));
    if (err) {
        LOG_ERR("Failed Power-On-Reset: Failed to configure RST GPIO pin to "
                "output low (err %d)",
                err);
        return err;
    }

    // Wait 600ms
    k_sleep(MOUSE_PS2_POWER_ON_RESET_TIME);

    // Set pin high
    err = gpio_pin_set_dt(&data->rst_gpio, 0);
    if (err) {
        LOG_ERR("Failed Power-On-Reset: Failed to set RST GPIO pin to "
                "low (err %d)",
                err);
        return err;
    }

    LOG_DBG("Finished Power-On-Reset successfully...");

    return 0;
}

int zmk_mouse_ps2_init_wait_for_mouse(const struct device *dev) {
    const struct zmk_mouse_ps2_config *config = dev->config;
    int err;

    uint8_t read_val;

    for (int i = 0; i < MOUSE_PS2_INIT_ATTEMPTS; i++) {

        LOG_INF("Trying to initialize mouse device (attempt %d / %d)", i + 1,
                MOUSE_PS2_INIT_ATTEMPTS);

        // PS/2 Devices do a self-test and send the result when they power up.

        err = ps2_read(config->ps2_device, &read_val);
        if (err == 0) {
            if (read_val != MOUSE_PS2_RESP_SELF_TEST_PASS) {
                LOG_WRN("Got invalid PS/2 self-test result: 0x%x", read_val);

                LOG_INF("Trying to reset PS2 device...");
                zmk_mouse_ps2_reset(dev, config->ps2_device);

                continue;
            }

            LOG_INF("PS/2 Device passed self-test: 0x%x", read_val);

            // Read device id
            LOG_INF("Reading PS/2 device id...");
            err = ps2_read(config->ps2_device, &read_val);
            if (err) {
                LOG_WRN("Could not read PS/2 device id: %d", err);
            } else {
                if (read_val == 0) {
                    LOG_INF("Connected PS/2 device is a mouse...");
                    return 0;
                } else {
                    LOG_WRN("PS/2 device is not a mouse: 0x%x", read_val);
                    return 1;
                }
            }
        } else {
            LOG_WRN("Could not read PS/2 device self-test result: %d. ", err);
        }

        // But when a zmk device is reset, it doesn't cut the power to external
        // devices. So the device acts as if it was never disconnected.
        // So we try sending the reset command.
        if (i % 2 == 0) {
            LOG_INF("Trying to reset PS2 device...");
            zmk_mouse_ps2_reset(dev, config->ps2_device);
            continue;
        }

        k_sleep(K_SECONDS(5));
    }

    return 1;
}


// Define wrapper function declaration for zmk_mouse_ps2_activity*_callback
// will assign to zmk_mouse_ps2_data in below
#define PS2_MOUSE_CALLBACK_DEFINE(n)                                                          \
    void zmk_mouse_ps2_activity_callback##n(const struct device *ps2_device, uint8_t byte);   \
    void zmk_mouse_ps2_activity_resend_callback##n(const struct device *ps2_device);

DT_INST_FOREACH_STATUS_OKAY(PS2_MOUSE_CALLBACK_DEFINE)


// Depends on the UART and PS2 init priorities, which are 55 and 45 by default
#define ZMK_MOUSE_PS2_INIT_PRIORITY 90

#define ZMK_PS2_MOUSE_DEFINE(n)                                                               \
    static struct zmk_mouse_ps2_data data##n = {                                              \
        .packet_mode = MOUSE_PS2_PACKET_MODE_PS2_DEFAULT,                                     \
        .packet_idx = 0,                                                                      \
        .prev_packet = {                                                                      \
            .button_l = false, .button_r = false, .button_m = false,                          \
            .overflow_x = 0, .overflow_y = 0, .mov_x = 0, .mov_y = 0, .scroll = 0,            \
        },                                                                                    \
        .button_l_is_held = false,                                                            \
        .button_m_is_held = false,                                                            \
        .button_r_is_held = false,                                                            \
        .activity_reporting_on = false,                                                       \
        .is_trackpoint = false,                                                               \
        .manufacturer_id = 0x0,                                                               \
        .secondary_id = 0x0,                                                                  \
        .rom_id = 0x0,                                                                        \
        .sampling_rate = MOUSE_PS2_CMD_SET_SAMPLING_RATE_DEFAULT,                             \
        .tp_sensitivity = MOUSE_PS2_CMD_TP_SET_SENSITIVITY_DEFAULT,                           \
        .tp_neg_inertia = MOUSE_PS2_CMD_TP_SET_NEG_INERTIA_DEFAULT,                           \
        .tp_value6 = MOUSE_PS2_CMD_TP_SET_VALUE6_UPPER_PLATEAU_SPEED_DEFAULT,                 \
        .tp_pts_threshold = MOUSE_PS2_CMD_TP_SET_PTS_THRESHOLD_DEFAULT,                       \
        .tp_up_thresh = MOUSE_PS2_CMD_TP_SET_UP_THRESH_DEFAULT,                               \
        .tp_z_time = MOUSE_PS2_CMD_TP_SET_Z_TIME_DEFAULT,                                     \
        .tp_jenks_curv = MOUSE_PS2_CMD_TP_SET_JENKS_CURV_DEFAULT,                             \
        .tp_drag_hysteresis = MOUSE_PS2_CMD_TP_SET_DRAG_HYSTERESIS_DEFAULT,                   \
        .tp_min_drag = MOUSE_PS2_CMD_TP_SET_MIN_DRAG_DEFAULT,                                 \
        .tp_reach = MOUSE_PS2_CMD_TP_SET_REACH_DEFAULT,                                       \
        .activity_callback = &zmk_mouse_ps2_activity_callback##n,                             \
        .activity_resend_callback = &zmk_mouse_ps2_activity_resend_callback##n,               \
    };                                                                                        \
    static const struct zmk_mouse_ps2_config config##n = {                                    \
        .ps2_device = DEVICE_DT_GET(DT_INST_PHANDLE(n, ps2_device)),                          \
        .has_rst_gpio = DT_INST_NODE_HAS_PROP(n, rst_gpio),                                   \
        .rst_gpio = COND_CODE_1(                                                              \
            DT_INST_NODE_HAS_PROP(n, rst_gpio),                                               \
            (GPIO_DT_SPEC_INST_GET(n, rst_gpios)),                                            \
            ({ .port = NULL, .pin = 0, .dt_flags = 0, })),                                    \
        .rst_gpio_port_num = COND_CODE_1(                                                     \
            DT_INST_NODE_HAS_PROP(n, rst_gpio),                                               \
            (DT_PROP(DT_INST_PHANDLE(n, rst_gpios), port)),                                   \
            (0)),                                                                             \
        .scroll_mode = DT_INST_PROP_OR(n, scroll_mode, false),                                \
        .disable_clicking = DT_INST_PROP_OR(n, disable_clicking, false),                      \
        .sampling_rate = DT_INST_PROP_OR(n, sampling_rate,                                    \
                                         MOUSE_PS2_CMD_SET_SAMPLING_RATE_DEFAULT),            \
        .tp_press_to_select = DT_INST_PROP_OR(n, tp_press_to_select, false),                  \
        .tp_press_to_select_threshold = DT_INST_PROP_OR(n, tp_press_to_select_threshold, -1), \
        .tp_sensitivity = DT_INST_PROP_OR(n, tp_sensitivity, -1),                             \
        .tp_neg_inertia = DT_INST_PROP_OR(n, tp_neg_inertia, -1),                             \
        .tp_val6_upper_speed = DT_INST_PROP_OR(n, tp_val6_upper_speed, -1),                   \
        .tp_up_thresh = DT_INST_PROP_OR(n, tp_up_thresh, -1),                                 \
        .tp_z_time = DT_INST_PROP_OR(n, tp_z_time, -1),                                      \
        .tp_jenks_curv = DT_INST_PROP_OR(n, tp_jenks_curv, -1),                               \
        .tp_drag_hysteresis = DT_INST_PROP_OR(n, tp_drag_hysteresis, -1),                     \
        .tp_min_drag = DT_INST_PROP_OR(n, tp_min_drag, -1),                                   \
        .tp_reach = DT_INST_PROP_OR(n, tp_reach, -1),                                         \
        .tp_x_invert = DT_INST_PROP_OR(n, tp_x_invert, false),                                \
        .tp_y_invert = DT_INST_PROP_OR(n, tp_y_invert, false),                                \
        .tp_xy_swap = DT_INST_PROP_OR(n, tp_xy_swap, false),                                  \
        IF_ENABLED(CONFIG_ZMK_INPUT_MOUSE_PS2_IDLE_PM, (                                      \
        .has_wake_gpio = DT_INST_NODE_HAS_PROP(n, wake_gpios),                                \
        .wake_gpio = COND_CODE_1(                                                             \
            DT_INST_NODE_HAS_PROP(n, wake_gpios),                                             \
            (GPIO_DT_SPEC_INST_GET(n, wake_gpios)),                                           \
            ({ .port = NULL, .pin = 0, .dt_flags = 0, })),                                    \
        ))                                                                                    \
    };                                                                                        \
    DEVICE_DT_INST_DEFINE(n, &zmk_mouse_ps2_init, NULL, &data##n, &config##n,                 \
                        POST_KERNEL, ZMK_MOUSE_PS2_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(ZMK_PS2_MOUSE_DEFINE)


// Define wrapper function implementation for zmk_mouse_ps2_activity*_callback
// assigned to zmk_mouse_ps2_data on above
#define PS2_MOUSE_CALLBACK_IMPL_DEFINE(n)                                                     \
    void zmk_mouse_ps2_activity_callback##n(const struct device *ps2_device, uint8_t byte) {  \
        zmk_mouse_ps2_activity_callback(data##n.dev, ps2_device, byte);                       \
    }                                                                                         \
    void zmk_mouse_ps2_activity_resend_callback##n(const struct device *ps2_device) {         \
        zmk_mouse_ps2_activity_resend_callback(data##n.dev, ps2_device);                      \
    }

DT_INST_FOREACH_STATUS_OKAY(PS2_MOUSE_CALLBACK_IMPL_DEFINE)
