/*
 * Copyright (c) 2023 FTP Technologies
 * Copyright (c) 2026 Fenix Auto
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_shock_sensor

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/devicetree/dma.h>
#include <zephyr/devicetree/io-channels.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/cache.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include <stm32h5xx_ll_adc.h>
#include <stm32h5xx_ll_dma.h>
#include <stm32h5xx_ll_tim.h>

#include "shock-sensor.h"

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

LOG_MODULE_REGISTER(shock_sensor, CONFIG_SENSOR_SHOCK_LOG_LEVEL);

/* STM32H5 ADC regular data is transferred as one 16-bit DMA item.
 * Detection intentionally follows the original calibrated algorithm:
 * one raw min/max peak per completed DMA half-buffer.
 */
#define ADC_READING_TYPE uint16_t

#define MAX_MAIN_TAP_LEVEL 1201
#define MAX_WARN_TAP_LEVEL 241

#define MIN_TAP_INTERVAL_MS 2000
#define STOP_ALARM_INTERVAL_MS 5000
#define MULTIPLIER 16
#define ADC_BASELINE_CENTER 2048
#define SHAKE_MINIMUM_LEVEL 15U
#define DMA_CHUNK_QUEUE_LEN 8
#define DMA_BUFFER_ALIGNMENT 32
#define DMA_RESTART_DELAY_MS 250
#define DMA_RESTART_MAX_ERRORS 3
#define STARTUP_CALIBRATION_MAX_BLOCKS 16U

#define CTRL_REARM          BIT(0)
#define CTRL_RESTORE_WARN   BIT(1)
#define CTRL_RESTORE_MAIN   BIT(2)
#define CTRL_RESTART_STREAM BIT(3)

#define __TEMP_SENSOR_SHAKE_WARN_ZONE 14
#define __TEMP_SENSOR_SHAKE_MAIN_ZONE 100

struct shock_sensor_dt_spec {
    const struct adc_dt_spec port;
    uint32_t sampling_frequency_hz;
    uint32_t event_lockout_ms;
    uint32_t startup_calibration_blocks;
    uint32_t startup_blanking_ms;
    uint32_t baseline_max_deviation;
};

struct shock_dma_chunk_header {
    uint8_t half;
    uint8_t reserved[3];
};

struct sensor_config {
    struct shock_sensor_dt_spec sensor;
    struct gpio_dt_spec gpio_power;

    const struct device *dma_dev;
    uint32_t dma_channel;
    uint32_t dma_slot;

    ADC_TypeDef *adc;
    TIM_TypeDef *timer;
    struct stm32_pclken timer_pclken;

    k_thread_stack_t *work_q_stack;
    size_t work_q_stack_size;
};

struct sensor_data {
    const struct device *dev;
    struct k_work_q workq;
    struct k_work process_work;
    struct k_work maintenance_work;
    struct k_work_delayable restart_work;
#ifdef CONFIG_SENSOR_SHOCK_DMA_STATS
    struct k_work_delayable stats_work;
#endif

    struct k_msgq chunk_queue;
    uint8_t *chunk_queue_buffer;
    uint8_t *process_item;
    uint8_t *isr_queue_item;
    size_t chunk_queue_item_size;
    size_t chunk_samples_offset;

    ADC_READING_TYPE *dma_buffer;
    size_t dma_buffer_samples;
    size_t dma_half_samples;

    struct dma_block_config dma_block;
    struct dma_config dma_cfg;
    bool stream_running;
    bool backend_ready;
    atomic_t control_flags;
    atomic_t dropped_chunks_pending;
    atomic_t dma_error_count;
    atomic_t last_dma_error;
    atomic_t dma_half_callbacks[2];
    atomic_t dma_half_queued[2];
    atomic_t dma_half_processed[2];
    atomic_t dma_half_dropped[2];
    atomic_t dma_unexpected_status;
    atomic_t dma_sequence_errors;
    atomic_t dma_expected_half;

    int adc_centered_value;
    bool baseline_calibrated;
    bool startup_calibration_active;
    uint16_t startup_block_means[STARTUP_CALIBRATION_MAX_BLOCKS];
    uint32_t startup_calibration_blocks_seen;
    uint32_t startup_blanking_samples_remaining;
    uint32_t startup_blanking_samples_target;

    uint32_t lockout_samples;
    uint32_t lockout_samples_target;

    sensor_trigger_handler_t warn_handler;
    const struct sensor_trigger *warn_trigger;
    sensor_trigger_handler_t main_handler;
    const struct sensor_trigger *main_trigger;

    int warn_zones[10];
    int main_zones[10];
    int selected_warn_zone;
    int selected_main_zone;
    int current_warn_zone;
    int current_main_zone;
    bool max_level_alert_warn;
    bool max_level_alert_main;
    bool warn_zone_active;
    bool main_zone_active;
    int treshold_warn;
    int treshold_main;

    int64_t last_tap_time_warn;
    int64_t last_tap_time_main;

    int64_t max_main_noise_level_time;
    int64_t max_warn_noise_level_time;
    int64_t noise_sampling_interval_msec;
    int64_t noise_sampling_interval_sec;
    int max_main_noise_level;
    int max_warn_noise_level;

    int increase_sensivity_interval;

    struct k_timer reset_timer_alarm;
    struct k_timer increase_sensivity_timer_warn;
    struct k_timer increase_sensivity_timer_main;

    int mode;
};

static const int warn_zones_initial[10] = {4, 6, 10, 16, 25, 39, 61, 97, 152, 240};
static const float koeff[10] = {
    1.76893602f,
    1.698646465f,
    1.614054238f,
    1.539948249f,
    1.472733358f,
    1.408677779f,
    1.34705442f,
    1.285999948f,
    1.229514814f,
    1.174618943f,
};

static const float little_val = 0.03f;
static const float warn_noise_divider = 1.6f;

static void reset_timer_handler_alarm(struct k_timer *timer);
static void increase_sensivity_warn_handler(struct k_timer *timer);
static void increase_sensivity_main_handler(struct k_timer *timer);

static void set_zones(const struct device *dev, int warn_zone, int main_zone);
static void set_warn_zones(const struct device *dev);
static void create_main_zones(const struct device *dev, int zone);
static void coarsering_warn(struct sensor_data *data, bool increase);
static void coarsering_main(struct sensor_data *data, bool increase);
static void register_tap_main(struct sensor_data *data);
static void register_tap_warn(struct sensor_data *data);
static int shock_stream_start(struct sensor_data *data, bool recalibrate_baseline);
static int shock_backend_prepare(struct sensor_data *data);
static void shock_stream_stop(struct sensor_data *data);
static void detector_reset(struct sensor_data *data);

static int fetch(const struct device *dev, enum sensor_channel chan)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(chan);
    return 0;
}

static int get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(chan);
    ARG_UNUSED(val);
    return 0;
}

static void reset_noise_tracking(struct sensor_data *data, int64_t now)
{
    data->max_main_noise_level = 0;
    data->max_warn_noise_level = 0;
    data->max_main_noise_level_time = now;
    data->max_warn_noise_level_time = now;
}

static void restore_selected_zones(struct sensor_data *data)
{
    data->current_warn_zone = data->selected_warn_zone;
    data->current_main_zone = data->selected_main_zone;
    data->max_level_alert_warn = false;
    data->max_level_alert_main = false;

    if (!data->warn_zone_active) {
        data->current_warn_zone = 9;
    }

    set_zones(data->dev, data->current_warn_zone, data->current_main_zone);
}

static void reset_dma_stats(struct sensor_data *data)
{
    atomic_set(&data->dropped_chunks_pending, 0);
    atomic_set(&data->dma_expected_half, 0);
    for (size_t half = 0U; half < 2U; ++half) {
        atomic_set(&data->dma_half_callbacks[half], 0);
        atomic_set(&data->dma_half_queued[half], 0);
        atomic_set(&data->dma_half_processed[half], 0);
        atomic_set(&data->dma_half_dropped[half], 0);
    }
    atomic_set(&data->dma_unexpected_status, 0);
    atomic_set(&data->dma_sequence_errors, 0);
}

static void enter_disarmed(struct sensor_data *data)
{
    const int64_t now = k_uptime_get();

    data->mode = SHOCK_SENSOR_MODE_DISARMED;
    (void)k_work_cancel_delayable(&data->restart_work);
    shock_stream_stop(data);
    detector_reset(data);

    /* A new DISARMED -> ARMED transition must always measure a fresh centre. */
    data->baseline_calibrated = false;
    data->startup_calibration_active = false;
    data->startup_calibration_blocks_seen = 0U;
    data->startup_blanking_samples_remaining = 0U;
    data->adc_centered_value = ADC_BASELINE_CENTER * MULTIPLIER;

    k_timer_stop(&data->reset_timer_alarm);
    k_timer_stop(&data->increase_sensivity_timer_warn);
    k_timer_stop(&data->increase_sensivity_timer_main);

    restore_selected_zones(data);
    reset_noise_tracking(data, now);
    LOG_DBG("Sensor is forced to disarmed mode");
}

static int enter_armed(struct sensor_data *data, bool recalibrate_baseline)
{
    const int64_t now = k_uptime_get();

    data->mode = SHOCK_SENSOR_MODE_ARMED;
    atomic_set(&data->dma_error_count, 0);
    atomic_set(&data->last_dma_error, 0);
    reset_dma_stats(data);
    data->last_tap_time_warn = now;
    data->last_tap_time_main = now;
    reset_noise_tracking(data, now);
    detector_reset(data);

    k_timer_stop(&data->reset_timer_alarm);

    int ret = shock_stream_start(data, recalibrate_baseline);
    if (ret != 0) {
        data->mode = SHOCK_SENSOR_MODE_DISARMED;
        LOG_ERR("Failed to start shock stream: %d", ret);
        return ret;
    }

    LOG_DBG("Sensor is armed (%s baseline)",
            recalibrate_baseline ? "calibrate" : "reuse");
    return 0;
}

static void enter_alarm(struct sensor_data *data, int duration_ms)
{
    data->mode = SHOCK_SENSOR_MODE_ALARM;
    (void)k_work_cancel_delayable(&data->restart_work);
    shock_stream_stop(data);
    detector_reset(data);

    if (duration_ms > 0 && duration_ms <= 3000) {
        k_timer_start(&data->reset_timer_alarm, K_MSEC(STOP_ALARM_INTERVAL_MS), K_NO_WAIT);
        LOG_DBG("Entering alarm mode for %d ms", STOP_ALARM_INTERVAL_MS);
    } else {
        LOG_DBG("Entering alarm mode until alarm-stop/disarm");
    }
}

static int attr_set(const struct device *dev,
                    enum sensor_channel chan,
                    enum sensor_attribute attr,
                    const struct sensor_value *val)
{
    struct sensor_data *data = dev->data;
    const int64_t current_time = k_uptime_get();

    if (chan == SENSOR_CHAN_PROX && attr == SENSOR_ATTR_UPPER_THRESH) {
        data->treshold_warn = val->val1;
        data->treshold_main = val->val2;
        LOG_DBG("Set threshold warn=%d main=%d", data->treshold_warn, data->treshold_main);
        return 0;
    }

    if (chan == (enum sensor_channel)SHOCK_SENSOR_INCREASE_SENSIVITY_INTERVAL_SEC &&
        attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        data->increase_sensivity_interval = MAX(val->val1, 1);
        return 0;
    }

    if (chan == (enum sensor_channel)SHOCK_SENSOR_NOISE_SAMPLING_TIME_SEC &&
        attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        data->noise_sampling_interval_sec = MAX(val->val1, 1);
        data->noise_sampling_interval_msec = data->noise_sampling_interval_sec * 1000;
        reset_noise_tracking(data, current_time);
        return 0;
    }

    if (chan == (enum sensor_channel)SHOCK_SENSOR_CHANNEL_WARN_ZONE &&
        attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        if (val->val1 == 0) {
            data->warn_zone_active = false;
            data->current_warn_zone = 9;
            data->selected_warn_zone = 9;
        } else {
            int value = val->val1 / 10;
            data->current_warn_zone = CLAMP(10 - value, 0, 9);
            data->selected_warn_zone = data->current_warn_zone;
            data->warn_zone_active = true;
        }

        create_main_zones(dev, data->current_warn_zone);
        data->current_main_zone = data->selected_main_zone;
        data->last_tap_time_warn = current_time;
        data->last_tap_time_main = current_time;
        data->max_level_alert_warn = false;
        data->max_level_alert_main = false;
        reset_noise_tracking(data, current_time);
        set_zones(dev, data->current_warn_zone, data->current_main_zone);
        detector_reset(data);
        return 0;
    }

    if (chan == (enum sensor_channel)SHOCK_SENSOR_CHANNEL_MAIN_ZONE &&
        attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        if (val->val1 == 0) {
            data->main_zone_active = false;
        } else {
            int value = val->val1 / 10;
            data->current_main_zone = CLAMP(10 - value, 0, 9);
            data->selected_main_zone = data->current_main_zone;
            data->main_zone_active = true;
        }

        data->current_warn_zone = data->selected_warn_zone;
        data->last_tap_time_warn = current_time;
        data->last_tap_time_main = current_time;
        data->max_level_alert_warn = false;
        data->max_level_alert_main = false;
        reset_noise_tracking(data, current_time);
        set_zones(dev, data->current_warn_zone, data->current_main_zone);
        detector_reset(data);
        return 0;
    }

    if (chan == (enum sensor_channel)SHOCK_SENSOR_MODE &&
        attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        switch (val->val1) {
        case SHOCK_SENSOR_MODE_ARMED:
            if (data->mode == SHOCK_SENSOR_MODE_DISARMED) {
                return enter_armed(data, true);
            }
            return 0;

        case SHOCK_SENSOR_MODE_DISARMED:
            if (data->mode != SHOCK_SENSOR_MODE_DISARMED) {
                enter_disarmed(data);
            }
            return 0;

        case SHOCK_SENSOR_MODE_ALARM:
            if (data->mode == SHOCK_SENSOR_MODE_ARMED) {
                enter_alarm(data, val->val2);
            } else if (data->mode == SHOCK_SENSOR_MODE_ALARM && val->val2 <= 3000) {
                k_timer_start(&data->reset_timer_alarm,
                              K_MSEC(STOP_ALARM_INTERVAL_MS), K_NO_WAIT);
            }
            return 0;

        case SHOCK_SENSOR_MODE_ALARM_STOP:
            if (data->mode == SHOCK_SENSOR_MODE_ALARM) {
                k_timer_start(&data->reset_timer_alarm,
                              K_MSEC(STOP_ALARM_INTERVAL_MS), K_NO_WAIT);
                LOG_DBG("Alarm stop: re-arm in %d ms", STOP_ALARM_INTERVAL_MS);
            }
            return 0;

        default:
            return -ENOTSUP;
        }
    }

    return -ENOTSUP;
}

static int trigger_set(const struct device *dev,
                       const struct sensor_trigger *trig,
                       sensor_trigger_handler_t handler)
{
    struct sensor_data *data = dev->data;

    if (trig == NULL || handler == NULL) {
        return -EINVAL;
    }

    switch (trig->type) {
    case SENSOR_TRIG_TAP:
        data->warn_handler = handler;
        data->warn_trigger = trig;
        return 0;
    case SENSOR_TRIG_THRESHOLD:
        data->main_handler = handler;
        data->main_trigger = trig;
        return 0;
    default:
        return -ENOTSUP;
    }
}

#ifdef CONFIG_PM_DEVICE
static int pm_action(const struct device *dev, enum pm_device_action action)
{
    struct sensor_data *data = dev->data;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
    case PM_DEVICE_ACTION_TURN_OFF:
        shock_stream_stop(data);
        return 0;
    case PM_DEVICE_ACTION_RESUME:
    case PM_DEVICE_ACTION_TURN_ON:
        if (data->mode == SHOCK_SENSOR_MODE_ARMED) {
            return shock_stream_start(data, false);
        }
        return 0;
    default:
        return -ENOTSUP;
    }
}
#endif

static void detector_reset(struct sensor_data *data)
{
    /* The baseline is fixed for the complete armed cycle. Reset only the
     * transient event-detector state and queued DMA chunks.
     */
    data->lockout_samples = 0U;
    k_msgq_purge(&data->chunk_queue);
}

static int startup_median(uint16_t *values, size_t count)
{
    for (size_t i = 1U; i < count; ++i) {
        const uint16_t value = values[i];
        size_t j = i;

        while (j > 0U && values[j - 1U] > value) {
            values[j] = values[j - 1U];
            --j;
        }
        values[j] = value;
    }

    if ((count & 1U) != 0U) {
        return values[count / 2U];
    }

    return ((int)values[(count / 2U) - 1U] +
            (int)values[count / 2U]) / 2;
}

static void startup_detection_prepare(struct sensor_data *data,
                                      bool recalibrate_baseline)
{
    data->startup_blanking_samples_remaining =
        data->startup_blanking_samples_target;
    data->startup_calibration_blocks_seen = 0U;

    /* Calibrate only for a new DISARMED -> ARMED cycle. A restart while
     * already armed (alarm re-arm, PM resume or DMA recovery) reuses the
     * fixed centre. If calibration was interrupted, however, it must restart.
     */
    data->startup_calibration_active =
        recalibrate_baseline || !data->baseline_calibrated;
    if (data->startup_calibration_active) {
        data->baseline_calibrated = false;
        data->adc_centered_value = ADC_BASELINE_CENTER * MULTIPLIER;
    }
}

static void update_noise_levels(struct sensor_data *data, int amplitude_abs, int64_t current_time)
{
    if (!data->main_zone_active && !data->warn_zone_active) {
        return;
    }

    if ((current_time - data->max_main_noise_level_time) > data->noise_sampling_interval_msec) {
        int prev = data->max_main_noise_level;
        data->max_main_noise_level =
            (int)((float)data->max_main_noise_level /
                  (koeff[data->selected_warn_zone] + little_val));
        if (prev == data->max_main_noise_level) {
            data->max_main_noise_level = 0;
        }
        data->max_main_noise_level_time = current_time;
    } else if (amplitude_abs >= data->max_main_noise_level) {
        data->max_main_noise_level = MIN(amplitude_abs, MAX_MAIN_TAP_LEVEL);
        data->max_main_noise_level_time = current_time;
    }

    if ((current_time - data->max_warn_noise_level_time) > data->noise_sampling_interval_msec) {
        int prev = data->max_warn_noise_level;
        data->max_warn_noise_level =
            (int)((float)data->max_warn_noise_level / warn_noise_divider);
        if (prev == data->max_warn_noise_level) {
            data->max_warn_noise_level = 0;
        }
        data->max_warn_noise_level_time = current_time;
    } else if (amplitude_abs >= data->max_warn_noise_level) {
        data->max_warn_noise_level = MIN(amplitude_abs, MAX_WARN_TAP_LEVEL);
        data->max_warn_noise_level_time = current_time;
    }
}

static bool warn_coarsening_reaches_main(const struct sensor_data *data)
{
    if (!data->main_zone_active || data->current_warn_zone >= 9) {
        return false;
    }

    return data->warn_zones[data->current_warn_zone + 1] >=
           data->treshold_main;
}

static void classify_event(struct sensor_data *data, uint32_t peak)
{
    const struct device *dev = data->dev;
    const int64_t now = k_uptime_get();

    if (data->mode != SHOCK_SENSOR_MODE_ARMED) {
        return;
    }


    if (data->main_zone_active && !data->max_level_alert_main &&
        peak >= (uint32_t)data->treshold_main) {
        LOG_INF("Shock peak=%u class=main warn=%d main=%d", peak,
                data->treshold_warn, data->treshold_main);
        if (data->main_handler != NULL) {
            data->mode = SHOCK_SENSOR_MODE_ALARM;
            data->main_handler(dev, data->main_trigger);
            register_tap_main(data);
            shock_stream_stop(data);
        } else {
            LOG_ERR("Problem with main_handler");
        }
        return;
    }

    if (data->warn_zone_active && !data->max_level_alert_warn &&
        peak >= (uint32_t)data->treshold_warn) {
        if ((now - data->last_tap_time_warn) <= MIN_TAP_INTERVAL_MS) {
            /* The physical event was detected; only the warning output is rate-limited. */
            LOG_DBG("Shock peak=%u class=warn-rate-limited warn=%d main=%d", peak,
                    data->treshold_warn, data->treshold_main);
            return;
        }

        LOG_INF("Shock peak=%u class=warn warn=%d main=%d", peak,
                data->treshold_warn, data->treshold_main);
        if (data->warn_handler != NULL) {
            data->warn_handler(dev, data->warn_trigger);
            register_tap_warn(data);
        } else {
            LOG_ERR("Problem with warn_handler");
        }
        return;
    }

    LOG_DBG("Shock peak=%u class=noise warn=%d main=%d", peak,
            data->treshold_warn, data->treshold_main);
    update_noise_levels(data, (int)peak, now);
}

static void process_samples(struct sensor_data *data,
                            const ADC_READING_TYPE *samples,
                            size_t count)
{
    if (data->mode != SHOCK_SENSOR_MODE_ARMED || count == 0U) {
        return;
    }

    uint64_t sum = samples[0];
    ADC_READING_TYPE raw_min = samples[0];
    ADC_READING_TYPE raw_max = samples[0];

    for (size_t i = 1U; i < count; ++i) {
        const ADC_READING_TYPE value = samples[i];
        sum += value;
        raw_min = MIN(raw_min, value);
        raw_max = MAX(raw_max, value);
    }

    const int chunk_mean = (int)(sum / count);

    /*
     * Suppress event detection while the analog path settles. On each explicit
     * DISARMED -> ARMED transition, derive the centre from the median of several
     * complete DMA half-buffer means. The median rejects an isolated startup
     * transient or a single accidental knock better than a plain average.
     * Alarm re-arm, PM resume and DMA recovery retain that centre and apply only
     * the short blanking interval.
     */
    const bool startup_blanked =
        data->startup_blanking_samples_remaining > 0U;
    if (data->startup_blanking_samples_remaining > count) {
        data->startup_blanking_samples_remaining -= (uint32_t)count;
    } else {
        data->startup_blanking_samples_remaining = 0U;
    }

    if (data->startup_calibration_active) {
        const struct sensor_config *config = data->dev->config;
        const uint32_t index = data->startup_calibration_blocks_seen;

        if (index < config->sensor.startup_calibration_blocks) {
            data->startup_block_means[index] = (uint16_t)chunk_mean;
            data->startup_calibration_blocks_seen = index + 1U;
        }

        if (data->startup_calibration_blocks_seen >=
            config->sensor.startup_calibration_blocks) {
            const int measured_center = startup_median(
                data->startup_block_means,
                config->sensor.startup_calibration_blocks);
            const int deviation = abs(measured_center - ADC_BASELINE_CENTER);
            int applied_center = measured_center;

            if ((uint32_t)deviation >
                config->sensor.baseline_max_deviation) {
                applied_center = ADC_BASELINE_CENTER;
                LOG_WRN("Shock baseline rejected: measured=%d deviation=%d "
                        "limit=%u; using %d",
                        measured_center, deviation,
                        config->sensor.baseline_max_deviation,
                        ADC_BASELINE_CENTER);
            } else {
                LOG_INF("Shock baseline calibrated: center=%d deviation=%d "
                        "blocks=%u blanking=%u ms",
                        applied_center, deviation,
                        config->sensor.startup_calibration_blocks,
                        config->sensor.startup_blanking_ms);
            }

            data->adc_centered_value = applied_center * MULTIPLIER;
            data->baseline_calibrated = true;
            data->startup_calibration_active = false;
        }

        return;
    }

    if (startup_blanked) {
        return;
    }

    const int center = data->adc_centered_value / MULTIPLIER;
    const int positive_peak = (int)raw_max - center;
    const int negative_peak = center - (int)raw_min;
    const int amplitude_abs = MAX(0, MAX(positive_peak, negative_peak));
    const int64_t now = k_uptime_get();

    /*
     * Keep the centre fixed for the entire armed cycle. This prevents
     * sustained asymmetric vibration, rain or other long-running noise from
     * being learned as a new zero level and silently reducing sensitivity.
     * A fresh centre is measured only on the next DISARMED -> ARMED transition.
     */

    /*
     * Main remains active during warning lockout, so a stronger continuation
     * can still escalate to the main event.
     */
    const bool in_lockout = data->lockout_samples > 0U;
    if (data->lockout_samples > count) {
        data->lockout_samples -= (uint32_t)count;
    } else {
        data->lockout_samples = 0U;
    }

    if (data->main_zone_active && !data->max_level_alert_main &&
        amplitude_abs >= data->treshold_main) {
        LOG_DBG("Shock block peak=%d min=%u max=%u center=%d class=main warn=%d main=%d",
                amplitude_abs, (unsigned)raw_min, (unsigned)raw_max, center,
                data->treshold_warn, data->treshold_main);
        classify_event(data, (uint32_t)amplitude_abs);
        if (data->mode != SHOCK_SENSOR_MODE_ARMED) {
            return;
        }
        data->lockout_samples = data->lockout_samples_target;
        return;
    }

    if (!in_lockout && data->warn_zone_active &&
        !data->max_level_alert_warn &&
        amplitude_abs >= data->treshold_warn) {
        LOG_DBG("Shock block peak=%d min=%u max=%u center=%d class=warn-candidate warn=%d main=%d",
                amplitude_abs, (unsigned)raw_min, (unsigned)raw_max, center,
                data->treshold_warn, data->treshold_main);
        classify_event(data, (uint32_t)amplitude_abs);
        data->lockout_samples = data->lockout_samples_target;
        return;
    }

    if (in_lockout) {
        return;
    }

    update_noise_levels(data, amplitude_abs, now);

    if (amplitude_abs >= SHAKE_MINIMUM_LEVEL) {
        LOG_DBG("Shock raw block peak=%d min=%u max=%u center=%d warn=%d main=%d",
                amplitude_abs, (unsigned)raw_min, (unsigned)raw_max, center,
                data->treshold_warn, data->treshold_main);
    }
}

#ifdef CONFIG_SENSOR_SHOCK_DMA_STATS
static void log_dma_stats(struct sensor_data *data, const char *reason)
{
    LOG_INF("Shock DMA stats[%s]: cb=%ld/%ld queued=%ld/%ld processed=%ld/%ld "
            "dropped=%ld/%ld seq_err=%ld unexpected=%ld q=%u/%u",
            reason,
            (long)atomic_get(&data->dma_half_callbacks[0]),
            (long)atomic_get(&data->dma_half_callbacks[1]),
            (long)atomic_get(&data->dma_half_queued[0]),
            (long)atomic_get(&data->dma_half_queued[1]),
            (long)atomic_get(&data->dma_half_processed[0]),
            (long)atomic_get(&data->dma_half_processed[1]),
            (long)atomic_get(&data->dma_half_dropped[0]),
            (long)atomic_get(&data->dma_half_dropped[1]),
            (long)atomic_get(&data->dma_sequence_errors),
            (long)atomic_get(&data->dma_unexpected_status),
            (unsigned)k_msgq_num_used_get(&data->chunk_queue),
            DMA_CHUNK_QUEUE_LEN);
}

static void stats_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct sensor_data *data = CONTAINER_OF(dwork, struct sensor_data, stats_work);

    if (!data->stream_running) {
        return;
    }

    log_dma_stats(data, "periodic");
    (void)k_work_reschedule_for_queue(
        &data->workq, &data->stats_work,
        K_SECONDS(CONFIG_SENSOR_SHOCK_STATS_INTERVAL_SEC));
}

#endif

static void process_work_handler(struct k_work *work)
{
    struct sensor_data *data = CONTAINER_OF(work, struct sensor_data, process_work);

    while (k_msgq_get(&data->chunk_queue, data->process_item, K_NO_WAIT) == 0) {
        const struct shock_dma_chunk_header *header =
            (const struct shock_dma_chunk_header *)data->process_item;

        if (header->half > 1U) {
            atomic_inc(&data->dma_unexpected_status);
            continue;
        }

        atomic_inc(&data->dma_half_processed[header->half]);
        if (!data->stream_running) {
            continue;
        }

        const ADC_READING_TYPE *samples =
            (const ADC_READING_TYPE *)(data->process_item +
                                       data->chunk_samples_offset);
        process_samples(data, samples, data->dma_half_samples);
    }

    atomic_val_t dropped = atomic_set(&data->dropped_chunks_pending, 0);
    if (dropped > 0) {
        LOG_WRN("Dropped %ld shock DMA chunks; detector timing state reset",
                (long)dropped);
#ifdef CONFIG_SENSOR_SHOCK_DMA_STATS
        log_dma_stats(data, "drop");
#endif
        detector_reset(data);
    }
}
static void schedule_maintenance(struct sensor_data *data, atomic_val_t flags)
{
    atomic_or(&data->control_flags, flags);
    (void)k_work_submit_to_queue(&data->workq, &data->maintenance_work);
}

static void dma_callback(const struct device *dma_dev,
                         void *user_data,
                         uint32_t channel,
                         int status)
{
    struct sensor_data *data = user_data;
    ARG_UNUSED(dma_dev);
    ARG_UNUSED(channel);

    /* Ignore a late IRQ after the stream was deliberately stopped. */
    if (!data->stream_running) {
        return;
    }

    if (status < 0) {
        const struct sensor_config *config = data->dev->config;

        LL_TIM_DisableCounter(config->timer);
        atomic_set(&data->last_dma_error, status);
        atomic_inc(&data->dma_error_count);
        (void)k_work_reschedule_for_queue(
            &data->workq, &data->restart_work, K_MSEC(DMA_RESTART_DELAY_MS));
        return;
    }

    atomic_set(&data->dma_error_count, 0);

    uint8_t half;
    if (status == DMA_STATUS_BLOCK) {
        /* Zephyr 4.3 STM32U5/H5 driver maps the half-transfer IRQ here. */
        half = 0U;
    } else if (status == DMA_STATUS_COMPLETE) {
        /* Transfer-complete IRQ: the second half is now stable. */
        half = 1U;
    } else {
        atomic_inc(&data->dma_unexpected_status);
        return;
    }

    atomic_inc(&data->dma_half_callbacks[half]);

    const atomic_val_t expected = atomic_get(&data->dma_expected_half);
    if ((uint8_t)expected != half) {
        atomic_inc(&data->dma_sequence_errors);
    }
    atomic_set(&data->dma_expected_half, (atomic_val_t)(half ^ 1U));

    ADC_READING_TYPE *chunk =
        data->dma_buffer + ((size_t)half * data->dma_half_samples);
    const size_t bytes = data->dma_half_samples * sizeof(ADC_READING_TYPE);

    (void)sys_cache_data_invd_range(chunk, bytes);

    struct shock_dma_chunk_header *header =
        (struct shock_dma_chunk_header *)data->isr_queue_item;
    header->half = half;
    memcpy(data->isr_queue_item + data->chunk_samples_offset, chunk, bytes);

    if (k_msgq_put(&data->chunk_queue, data->isr_queue_item, K_NO_WAIT) != 0) {
        atomic_inc(&data->dropped_chunks_pending);
        atomic_inc(&data->dma_half_dropped[half]);
        return;
    }

    atomic_inc(&data->dma_half_queued[half]);
    (void)k_work_submit_to_queue(&data->workq, &data->process_work);
}

static int configure_timer(const struct device *dev)
{
    const struct sensor_config *config = dev->config;
    const struct device *clock = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
    uint32_t timer_clock_hz;

    int ret = clock_control_on(clock,
        (clock_control_subsys_t)&config->timer_pclken);
    if (ret != 0) {
        LOG_ERR("Cannot enable sampling timer clock: %d", ret);
        return ret;
    }

    ret = clock_control_get_rate(clock,
        (clock_control_subsys_t)&config->timer_pclken,
        &timer_clock_hz);
    if (ret != 0 || timer_clock_hz == 0U) {
        LOG_ERR("Cannot read sampling timer clock: %d", ret);
        return ret != 0 ? ret : -EINVAL;
    }

    const uint32_t requested_hz = config->sensor.sampling_frequency_hz;
    const uint64_t max_period_hz = (uint64_t)requested_hz * 65536ULL;
    uint32_t prescaler_div = (uint32_t)MAX(
        1ULL, ((uint64_t)timer_clock_hz + max_period_hz - 1ULL) /
                  max_period_hz);

    uint32_t auto_reload = (uint32_t)(
        (uint64_t)timer_clock_hz /
        ((uint64_t)prescaler_div * requested_hz));
    if (auto_reload == 0U) {
        return -EINVAL;
    }
    auto_reload -= 1U;

    LL_TIM_DisableCounter(config->timer);
    LL_TIM_SetPrescaler(config->timer, prescaler_div - 1U);
    LL_TIM_SetAutoReload(config->timer, auto_reload);
    LL_TIM_SetCounter(config->timer, 0U);
    LL_TIM_SetCounterMode(config->timer, LL_TIM_COUNTERMODE_UP);
    LL_TIM_SetClockDivision(config->timer, LL_TIM_CLOCKDIVISION_DIV1);
    LL_TIM_SetTriggerOutput(config->timer, LL_TIM_TRGO_UPDATE);
    LL_TIM_GenerateEvent_UPDATE(config->timer);
    LL_TIM_ClearFlag_UPDATE(config->timer);

    const uint32_t actual_hz =
        timer_clock_hz / (prescaler_div * (auto_reload + 1U));
    LOG_INF("Shock timer: clock=%u Hz sampling=%u Hz (requested %u)",
            timer_clock_hz, actual_hz, requested_hz);
    return 0;
}

static int configure_adc(const struct device *dev)
{
    const struct sensor_config *config = dev->config;
    ADC_TypeDef *adc = config->adc;

    if (LL_ADC_REG_IsConversionOngoing(adc) != 0U) {
        LL_ADC_REG_StopConversion(adc);
        for (int i = 0; i < 1000 && LL_ADC_REG_IsConversionOngoing(adc) != 0U; ++i) {
            k_busy_wait(1);
        }
    }

    /* Resolution/alignment are writable only while the ADC is disabled. */
    if (LL_ADC_IsEnabled(adc) != 0U) {
        LL_ADC_Disable(adc);
        for (int i = 0; i < 2000 && LL_ADC_IsEnabled(adc) != 0U; ++i) {
            k_busy_wait(1);
        }
        if (LL_ADC_IsEnabled(adc) != 0U) {
            LOG_ERR("ADC disable timeout");
            return -ETIMEDOUT;
        }
    }

    LL_ADC_SetResolution(adc, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(adc, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_REG_SetSequencerLength(adc, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(
        adc, LL_ADC_REG_RANK_1,
        __LL_ADC_DECIMAL_NB_TO_CHANNEL(config->sensor.port.channel_id));
    LL_ADC_REG_SetContinuousMode(adc, LL_ADC_REG_CONV_SINGLE);
    LL_ADC_REG_SetTriggerSource(adc, LL_ADC_REG_TRIG_EXT_TIM15_TRGO);
    LL_ADC_REG_SetDMATransfer(adc, LL_ADC_REG_DMA_TRANSFER_UNLIMITED);
    LL_ADC_REG_SetOverrun(adc, LL_ADC_REG_OVR_DATA_OVERWRITTEN);

    LL_ADC_ClearFlag_EOC(adc);
    LL_ADC_ClearFlag_EOS(adc);
    LL_ADC_ClearFlag_OVR(adc);

    if (LL_ADC_IsEnabled(adc) == 0U) {
        LL_ADC_ClearFlag_ADRDY(adc);
        LL_ADC_Enable(adc);
        for (int i = 0; i < 2000 && LL_ADC_IsActiveFlag_ADRDY(adc) == 0U; ++i) {
            k_busy_wait(1);
        }
        if (LL_ADC_IsActiveFlag_ADRDY(adc) == 0U) {
            LOG_ERR("ADC ready timeout");
            return -ETIMEDOUT;
        }
    }

    return 0;
}

static int shock_backend_prepare(struct sensor_data *data)
{
    if (data->backend_ready) {
        return 0;
    }

    int ret = configure_timer(data->dev);
    if (ret != 0) {
        return ret;
    }

    /*
     * The Zephyr STM32 ADC device init has already enabled the regulator and
     * calibrated ADC2.  Do not perform adc_read() here: the synchronous ADC
     * path waits for an ADC interrupt and can block system startup on the
     * affected STM32H563/Zephyr 4.3 configuration.
     */
    ret = configure_adc(data->dev);
    if (ret != 0) {
        return ret;
    }

    data->backend_ready = true;
    return 0;
}

static int configure_dma(struct sensor_data *data)
{
    const struct sensor_config *config = data->dev->config;

    memset(&data->dma_block, 0, sizeof(data->dma_block));
    memset(&data->dma_cfg, 0, sizeof(data->dma_cfg));

    data->dma_block.source_address =
        LL_ADC_DMA_GetRegAddr(config->adc, LL_ADC_DMA_REG_REGULAR_DATA);
    data->dma_block.dest_address = (uint32_t)(uintptr_t)data->dma_buffer;
    data->dma_block.block_size =
        data->dma_buffer_samples * sizeof(ADC_READING_TYPE);
    data->dma_block.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
    data->dma_block.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
    /*
     * Zephyr 4.3.0 dma_stm32u5.c enters its cyclic linked-list path when
     * source_reload_en is set. For peripheral-to-memory with a fixed ADC data
     * register and incrementing RAM destination, the driver reloads only CDAR;
     * dest_reload_en is not consulted by this backend.
     */
    data->dma_block.source_reload_en = 1U;
    data->dma_block.dest_reload_en = 0U;

    data->dma_cfg.dma_slot = config->dma_slot;
    data->dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
    /*
     * Zephyr 4.3 has no half_complete_callback_en field.  On the STM32U5/H5
     * GPDMA backend, source_reload_en selects the linked-list cyclic path and
     * that path enables the HT interrupt internally.
     */
    data->dma_cfg.complete_callback_en = 1U;
    data->dma_cfg.cyclic = 1U;
    data->dma_cfg.error_callback_dis = 0U;
    data->dma_cfg.channel_priority = 3U;
    data->dma_cfg.source_data_size = sizeof(ADC_READING_TYPE);
    data->dma_cfg.dest_data_size = sizeof(ADC_READING_TYPE);
    data->dma_cfg.source_burst_length = 1U;
    data->dma_cfg.dest_burst_length = 1U;
    data->dma_cfg.block_count = 1U;
    data->dma_cfg.head_block = &data->dma_block;
    data->dma_cfg.dma_callback = dma_callback;
    data->dma_cfg.user_data = data;

    return dma_config(config->dma_dev, config->dma_channel, &data->dma_cfg);
}

static int shock_stream_start(struct sensor_data *data, bool recalibrate_baseline)
{
    const struct sensor_config *config = data->dev->config;

    if (data->stream_running) {
        return 0;
    }

    int ret = shock_backend_prepare(data);
    if (ret != 0) {
        LOG_ERR("Shock backend prepare failed: %d", ret);
        return ret;
    }

    detector_reset(data);
    startup_detection_prepare(data, recalibrate_baseline);
    atomic_set(&data->dma_expected_half, 0);
    (void)sys_cache_data_flush_and_invd_range(
        data->dma_buffer,
        data->dma_buffer_samples * sizeof(ADC_READING_TYPE));

    ret = configure_dma(data);
    if (ret != 0) {
        LOG_ERR("DMA configure failed: %d", ret);
        return ret;
    }

    ret = dma_start(config->dma_dev, config->dma_channel);
    if (ret != 0) {
        LOG_ERR("DMA start failed: %d", ret);
        return ret;
    }

    LL_ADC_ClearFlag_EOC(config->adc);
    LL_ADC_ClearFlag_EOS(config->adc);
    LL_ADC_ClearFlag_OVR(config->adc);

    /* Mark active before TIM15 can generate the first DMA callback. */
    data->stream_running = true;
    LL_ADC_REG_StartConversion(config->adc);
    LL_TIM_SetCounter(config->timer, 0U);
    LL_TIM_EnableCounter(config->timer);

    (void)k_work_cancel_delayable(&data->restart_work);
#ifdef CONFIG_SENSOR_SHOCK_DMA_STATS
    (void)k_work_reschedule_for_queue(
        &data->workq, &data->stats_work,
        K_SECONDS(CONFIG_SENSOR_SHOCK_STATS_INTERVAL_SEC));
#endif
    return 0;
}

static void shock_stream_stop(struct sensor_data *data)
{
    const struct sensor_config *config = data->dev->config;

    if (!data->stream_running) {
        return;
    }

    data->stream_running = false;
#ifdef CONFIG_SENSOR_SHOCK_DMA_STATS
    (void)k_work_cancel_delayable(&data->stats_work);
#endif
    LL_TIM_DisableCounter(config->timer);

    if (LL_ADC_REG_IsConversionOngoing(config->adc) != 0U) {
        LL_ADC_REG_StopConversion(config->adc);
        for (int i = 0;
             i < 1000 && LL_ADC_REG_IsConversionOngoing(config->adc) != 0U;
             ++i) {
            k_busy_wait(1);
        }
    }

    int ret = dma_stop(config->dma_dev, config->dma_channel);
    if (ret != 0) {
        LOG_WRN("DMA stop failed: %d", ret);
    }

#ifdef CONFIG_SENSOR_SHOCK_DMA_STATS
    log_dma_stats(data, "stop");
#endif
    k_msgq_purge(&data->chunk_queue);
}

static void restart_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct sensor_data *data = CONTAINER_OF(dwork, struct sensor_data, restart_work);
    const atomic_val_t errors = atomic_get(&data->dma_error_count);
    const atomic_val_t last_error = atomic_get(&data->last_dma_error);

    if (errors > DMA_RESTART_MAX_ERRORS) {
        LOG_ERR("Shock DMA disabled after %ld consecutive errors (last=%ld)",
                (long)errors, (long)last_error);
        shock_stream_stop(data);
        return;
    }

    LOG_WRN("Shock DMA error %ld; controlled restart %ld/%d",
            (long)last_error, (long)errors, DMA_RESTART_MAX_ERRORS);
    schedule_maintenance(data, CTRL_RESTART_STREAM);
}

static void maintenance_work_handler(struct k_work *work)
{
    struct sensor_data *data = CONTAINER_OF(work, struct sensor_data, maintenance_work);
    atomic_val_t flags;

    /* Flags may be added by a timer/DMA ISR while this handler is running. */
    while ((flags = atomic_set(&data->control_flags, 0)) != 0) {
        if ((flags & CTRL_REARM) != 0) {
            if (data->mode == SHOCK_SENSOR_MODE_ALARM) {
                int ret = enter_armed(data, false);
                if (ret != 0) {
                    (void)k_work_reschedule_for_queue(
                        &data->workq, &data->restart_work, K_MSEC(100));
                }
            }
        }

        if ((flags & CTRL_RESTORE_WARN) != 0) {
            if (data->mode == SHOCK_SENSOR_MODE_ARMED && data->warn_zone_active) {
                coarsering_warn(data, false);
            } else if (data->current_warn_zone != data->selected_warn_zone) {
                k_timer_start(&data->increase_sensivity_timer_warn,
                              K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            }
        }

        if ((flags & CTRL_RESTORE_MAIN) != 0) {
            if (data->mode == SHOCK_SENSOR_MODE_ARMED && data->main_zone_active) {
                coarsering_main(data, false);
            } else if (data->current_main_zone != data->selected_main_zone) {
                k_timer_start(&data->increase_sensivity_timer_main,
                              K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            }
        }

        if ((flags & CTRL_RESTART_STREAM) != 0 &&
            data->mode == SHOCK_SENSOR_MODE_ARMED) {
            shock_stream_stop(data);
            int ret = shock_stream_start(data, false);
            if (ret != 0) {
                atomic_set(&data->last_dma_error, ret);
                atomic_inc(&data->dma_error_count);
                LOG_ERR("Shock stream restart failed: %d", ret);
                (void)k_work_reschedule_for_queue(
                    &data->workq, &data->restart_work,
                    K_MSEC(DMA_RESTART_DELAY_MS));
            }
        }
    }
}

static const struct sensor_driver_api sensor_api = {
    .sample_fetch = fetch,
    .channel_get = get,
    .trigger_set = trigger_set,
    .attr_set = attr_set,
};

static int sensor_init(const struct device *dev)
{
    const struct sensor_config *config = dev->config;
    struct sensor_data *data = dev->data;

    data->dev = dev;
    data->mode = SHOCK_SENSOR_MODE_DISARMED;
    data->warn_handler = NULL;
    data->main_handler = NULL;
    data->treshold_warn = __TEMP_SENSOR_SHAKE_WARN_ZONE;
    data->treshold_main = __TEMP_SENSOR_SHAKE_MAIN_ZONE;
    data->adc_centered_value = MULTIPLIER * ADC_BASELINE_CENTER;
    data->increase_sensivity_interval = 60;
    data->noise_sampling_interval_sec = 60;
    data->noise_sampling_interval_msec = 60000;
    data->backend_ready = false;
    data->baseline_calibrated = false;
    data->startup_calibration_active = false;
    data->startup_calibration_blocks_seen = 0U;
    data->startup_blanking_samples_remaining = 0U;
    atomic_set(&data->dma_error_count, 0);
    atomic_set(&data->last_dma_error, 0);
    reset_dma_stats(data);

    if (!adc_is_ready_dt(&config->sensor.port)) {
        LOG_ERR("ADC is not ready");
        return -ENODEV;
    }
    if (!device_is_ready(config->dma_dev)) {
        LOG_ERR("DMA is not ready");
        return -ENODEV;
    }

    if (config->gpio_power.port != NULL) {
        if (!gpio_is_ready_dt(&config->gpio_power)) {
            LOG_ERR("Power GPIO is not ready");
            return -ENODEV;
        }
        int ret = gpio_pin_configure_dt(&config->gpio_power, GPIO_OUTPUT_ACTIVE);
        if (ret != 0) {
            return ret;
        }
    }

    int ret = adc_channel_setup_dt(&config->sensor.port);
    if (ret != 0) {
        LOG_ERR("ADC channel setup failed: %d", ret);
        return ret;
    }

    data->dma_half_samples = data->dma_buffer_samples / 2U;
    data->lockout_samples_target = MAX(
        1U,
        (uint32_t)(((uint64_t)config->sensor.sampling_frequency_hz *
                    config->sensor.event_lockout_ms) / 1000ULL));
    data->startup_blanking_samples_target = MAX(
        1U,
        (uint32_t)(((uint64_t)config->sensor.sampling_frequency_hz *
                    config->sensor.startup_blanking_ms) / 1000ULL));

    data->chunk_samples_offset = sizeof(struct shock_dma_chunk_header);
    data->chunk_queue_item_size =
        data->chunk_samples_offset +
        data->dma_half_samples * sizeof(ADC_READING_TYPE);
    k_msgq_init(&data->chunk_queue,
                data->chunk_queue_buffer,
                data->chunk_queue_item_size,
                DMA_CHUNK_QUEUE_LEN);
    k_work_init(&data->process_work, process_work_handler);
    k_work_init(&data->maintenance_work, maintenance_work_handler);
    k_work_init_delayable(&data->restart_work, restart_work_handler);
#ifdef CONFIG_SENSOR_SHOCK_DMA_STATS
    k_work_init_delayable(&data->stats_work, stats_work_handler);
#endif

    struct k_work_queue_config workq_cfg = {
        .name = "shock_sensor:WQ",
    };
    k_work_queue_init(&data->workq);
    k_work_queue_start(&data->workq,
                       config->work_q_stack,
                       config->work_q_stack_size,
                       CONFIG_SENSOR_SHOCK_THREAD_PRIORITY,
                       &workq_cfg);

    k_timer_init(&data->reset_timer_alarm, reset_timer_handler_alarm, NULL);
    k_timer_init(&data->increase_sensivity_timer_warn,
                 increase_sensivity_warn_handler, NULL);
    k_timer_init(&data->increase_sensivity_timer_main,
                 increase_sensivity_main_handler, NULL);

    set_warn_zones(dev);
    set_zones(dev, 0, 0);
    data->last_tap_time_warn = k_uptime_get();
    data->last_tap_time_main = data->last_tap_time_warn;
    reset_noise_tracking(data, data->last_tap_time_warn);
    detector_reset(data);

    LOG_INF("Shock backend deferred until arm: TIM15 + ADC2 + circular DMA, %u Hz, %u samples, half=%u",
            config->sensor.sampling_frequency_hz,
            (unsigned)data->dma_buffer_samples,
            (unsigned)data->dma_half_samples);
    const uint32_t half_buffer_us = (uint32_t)(
        ((uint64_t)data->dma_half_samples * 1000000ULL) /
        config->sensor.sampling_frequency_hz);
    LOG_INF("Raw block detector: half=%u samples (%u us), lockout=%u ms",
            (unsigned)data->dma_half_samples, half_buffer_us,
            config->sensor.event_lockout_ms);
    LOG_INF("Baseline: calibrate on arm, median=%u blocks, blanking=%u ms, "
            "max deviation=%u; fixed while armed",
            config->sensor.startup_calibration_blocks,
            config->sensor.startup_blanking_ms,
            config->sensor.baseline_max_deviation);
#ifdef CONFIG_SENSOR_SHOCK_DMA_STATS
    LOG_INF("DMA diagnostics enabled: stats=%d s, WQ prio=%d",
            CONFIG_SENSOR_SHOCK_STATS_INTERVAL_SEC,
            CONFIG_SENSOR_SHOCK_THREAD_PRIORITY);
#endif

#ifdef CONFIG_PM_DEVICE_RUNTIME
    return pm_device_driver_init(dev, pm_action);
#else
    return 0;
#endif
}

static void reset_timer_handler_alarm(struct k_timer *timer)
{
    struct sensor_data *data =
        CONTAINER_OF(timer, struct sensor_data, reset_timer_alarm);
    schedule_maintenance(data, CTRL_REARM);
}

static void increase_sensivity_warn_handler(struct k_timer *timer)
{
    struct sensor_data *data =
        CONTAINER_OF(timer, struct sensor_data, increase_sensivity_timer_warn);
    schedule_maintenance(data, CTRL_RESTORE_WARN);
}

static void increase_sensivity_main_handler(struct k_timer *timer)
{
    struct sensor_data *data =
        CONTAINER_OF(timer, struct sensor_data, increase_sensivity_timer_main);
    schedule_maintenance(data, CTRL_RESTORE_MAIN);
}

static void set_zones(const struct device *dev, int warn_zone, int main_zone)
{
    struct sensor_data *data = dev->data;

    warn_zone = CLAMP(warn_zone, 0, 9);
    main_zone = CLAMP(main_zone, 0, 9);

    data->selected_warn_zone = warn_zone;
    data->current_warn_zone = warn_zone;
    create_main_zones(dev, warn_zone);
    data->selected_main_zone = main_zone;
    data->current_main_zone = main_zone;

    data->treshold_warn = data->warn_zones[warn_zone];
    data->treshold_main = data->main_zones[main_zone];
}

static void set_warn_zones(const struct device *dev)
{
    struct sensor_data *data = dev->data;
    for (int i = 0; i < 10; ++i) {
        data->warn_zones[i] = warn_zones_initial[i];
    }
}

static void create_main_zones(const struct device *dev, int zone)
{
    struct sensor_data *data = dev->data;
    zone = CLAMP(zone, 0, 9);

    const float k = koeff[zone];
    const float base = warn_zones_initial[zone];

    data->main_zones[0] = (int)roundf(base * k);
    for (int i = 1; i < 10; ++i) {
        data->main_zones[i] =
            (int)roundf((float)data->main_zones[i - 1] * k);
    }
}

static void apply_current_thresholds(struct sensor_data *data)
{
    data->treshold_warn = data->warn_zones[data->current_warn_zone];
    data->treshold_main = data->main_zones[data->current_main_zone];
}

static void coarsering_warn(struct sensor_data *data, bool increase)
{
    if (increase) {
        if (data->current_warn_zone == 9) {
            data->max_level_alert_warn = true;
            k_timer_start(&data->increase_sensivity_timer_warn,
                          K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }

        /*
         * Warning and main are independent zones. Warning coarsening must
         * never move the warning threshold to or above the main threshold.
         * Keep the current warning level active at that boundary, emit the
         * current warning normally, and start the usual gradual restoration
         * timer. Repeated warnings remain limited by MIN_TAP_INTERVAL_MS.
         */
        if (warn_coarsening_reaches_main(data)) {
            const int next_warn =
                data->warn_zones[data->current_warn_zone + 1];

            data->max_level_alert_warn = false;
            LOG_INF("Shock WARN threshold capped at %d: next=%d main=%d",
                    data->treshold_warn, next_warn, data->treshold_main);
            k_timer_start(&data->increase_sensivity_timer_warn,
                          K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }

        data->current_warn_zone++;
    } else {
        if (data->current_warn_zone == data->selected_warn_zone) {
            if (data->max_level_alert_warn) {
                if (data->max_warn_noise_level <= data->treshold_warn) {
                    data->max_level_alert_warn = false;
                } else {
                    k_timer_start(&data->increase_sensivity_timer_warn,
                                  K_SECONDS(data->increase_sensivity_interval),
                                  K_NO_WAIT);
                }
            }
            return;
        }
        if (data->max_warn_noise_level <=
            data->warn_zones[data->current_warn_zone - 1]) {
            data->current_warn_zone--;
            data->max_level_alert_warn = false;
        } else {
            k_timer_start(&data->increase_sensivity_timer_warn,
                          K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }
    }

    apply_current_thresholds(data);
    k_timer_start(&data->increase_sensivity_timer_warn,
                  K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
}

static void coarsering_main(struct sensor_data *data, bool increase)
{
    if (increase) {
        if (data->current_main_zone == 9) {
            data->max_level_alert_main = true;
            k_timer_start(&data->increase_sensivity_timer_main,
                          K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }
        data->current_main_zone++;
    } else {
        if (data->current_main_zone == data->selected_main_zone) {
            return;
        }

        const int candidate = data->main_zones[data->current_main_zone - 1];
        if (data->max_main_noise_level <= candidate) {
            if (data->warn_zone_active && candidate <= data->treshold_warn) {
                k_timer_start(&data->increase_sensivity_timer_main,
                              K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
                return;
            }
            data->current_main_zone--;
            data->max_level_alert_main = false;
        } else {
            k_timer_start(&data->increase_sensivity_timer_main,
                          K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }
    }

    apply_current_thresholds(data);
    k_timer_start(&data->increase_sensivity_timer_main,
                  K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
}

static void register_tap_main(struct sensor_data *data)
{
    data->last_tap_time_main = k_uptime_get();
    coarsering_main(data, true);
}

static void register_tap_warn(struct sensor_data *data)
{
    data->last_tap_time_warn = k_uptime_get();
    coarsering_warn(data, true);
}

#define SHOCK_SENSOR_DT_SPEC_GET(node_id)                                      \
    {                                                                          \
        .port = ADC_DT_SPEC_GET(node_id),                                      \
        .sampling_frequency_hz =                                               \
            DT_PROP_OR(node_id, sampling_frequency_hz, 10000),                  \
        .event_lockout_ms = DT_PROP_OR(node_id, event_lockout_ms, 20),         \
        .startup_calibration_blocks =                                          \
            DT_PROP_OR(node_id, startup_calibration_blocks, 8),                \
        .startup_blanking_ms =                                                 \
            DT_PROP_OR(node_id, startup_blanking_ms, 200),                     \
        .baseline_max_deviation =                                              \
            DT_PROP_OR(node_id, baseline_max_deviation, 12),                  \
    }

#define _INIT(inst)                                                            \
    BUILD_ASSERT(DT_SAME_NODE(DT_INST_PHANDLE(inst, sampling_timer),           \
                              DT_NODELABEL(timers15)),                          \
                 "shock sensor currently requires TIM15 TRGO");              \
    BUILD_ASSERT(DT_SAME_NODE(                                                  \
                     DT_DMAS_CTLR_BY_NAME(DT_DRV_INST(inst), rx),               \
                     DT_NODELABEL(gpdma1)),                                     \
                 "shock sensor currently requires GPDMA1");                  \
    BUILD_ASSERT(DT_DMAS_CELL_BY_NAME(DT_DRV_INST(inst), rx, slot) ==          \
                     LL_GPDMA1_REQUEST_ADC2,                                    \
                 "shock DMA request must be ADC2");                           \
    BUILD_ASSERT(DT_SAME_NODE(                                                  \
                     DT_PHANDLE_BY_IDX(DT_DRV_INST(inst), io_channels, 0),      \
                     DT_NODELABEL(adc2)),                                       \
                 "shock sensor input must use ADC2");                         \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, sampling_frequency_hz, 10000) > 0,      \
                 "shock sampling frequency must be non-zero");                \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, startup_calibration_blocks, 8) >= 3,     \
                 "shock startup calibration needs at least three blocks");    \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, startup_calibration_blocks, 8) <=       \
                     STARTUP_CALIBRATION_MAX_BLOCKS,                            \
                 "shock startup calibration block count is too large");       \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, startup_blanking_ms, 200) > 0,          \
                 "shock startup blanking must be non-zero");                  \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, baseline_max_deviation, 12) > 0,        \
                 "shock baseline max deviation must be non-zero");            \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, baseline_max_deviation, 12) <=         \
                     ADC_BASELINE_CENTER,                                       \
                 "shock baseline max deviation exceeds ADC half-scale");      \
    BUILD_ASSERT((DT_INST_PROP_OR(inst, dma_buffer_samples, 256) % 2) == 0,    \
                 "shock DMA buffer must have an even number of samples");     \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, dma_buffer_samples, 256) >= 64,         \
                 "shock DMA buffer is too small");                            \
    BUILD_ASSERT((DT_INST_PROP_OR(inst, dma_buffer_samples, 256) *             \
                  sizeof(ADC_READING_TYPE)) <= UINT16_MAX,                     \
                 "shock DMA buffer exceeds GPDMA block length");             \
    BUILD_ASSERT((((DT_INST_PROP_OR(inst, dma_buffer_samples, 256) / 2) *      \
                   sizeof(ADC_READING_TYPE)) % DMA_BUFFER_ALIGNMENT) == 0,     \
                 "shock DMA half-buffer must be cache-line aligned");        \
    static ADC_READING_TYPE sensor_##inst##_dma_buffer                         \
        [DT_INST_PROP_OR(inst, dma_buffer_samples, 256)]                        \
        __aligned(DMA_BUFFER_ALIGNMENT);                                        \
    static uint8_t sensor_##inst##_chunk_queue_buffer                         \
        [DMA_CHUNK_QUEUE_LEN *                                                 \
         (sizeof(struct shock_dma_chunk_header) +                              \
          ((DT_INST_PROP_OR(inst, dma_buffer_samples, 256) / 2) *              \
           sizeof(ADC_READING_TYPE)))]                                         \
        __aligned(DMA_BUFFER_ALIGNMENT);                                        \
    static uint8_t sensor_##inst##_process_item                                \
        [sizeof(struct shock_dma_chunk_header) +                               \
         ((DT_INST_PROP_OR(inst, dma_buffer_samples, 256) / 2) *               \
          sizeof(ADC_READING_TYPE))]                                           \
        __aligned(DMA_BUFFER_ALIGNMENT);                                        \
    static uint8_t sensor_##inst##_isr_queue_item                              \
        [sizeof(struct shock_dma_chunk_header) +                               \
         ((DT_INST_PROP_OR(inst, dma_buffer_samples, 256) / 2) *               \
          sizeof(ADC_READING_TYPE))]                                           \
        __aligned(DMA_BUFFER_ALIGNMENT);                                        \
    static struct sensor_data sensor_##inst##_data = {                         \
        .dma_buffer = sensor_##inst##_dma_buffer,                              \
        .dma_buffer_samples = DT_INST_PROP_OR(inst, dma_buffer_samples, 256),  \
        .chunk_queue_buffer = sensor_##inst##_chunk_queue_buffer,              \
        .process_item = sensor_##inst##_process_item,                          \
        .isr_queue_item = sensor_##inst##_isr_queue_item,                      \
    };                                                                         \
    static K_THREAD_STACK_DEFINE(work_q_stack_##inst,                          \
                                  CONFIG_SENSOR_SHOCK_THREAD_STACK_SIZE);       \
    static const struct sensor_config sensor_##inst##_config = {               \
        .sensor = SHOCK_SENSOR_DT_SPEC_GET(DT_DRV_INST(inst)),                 \
        .gpio_power = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {0}),        \
        .dma_dev = DEVICE_DT_GET(DT_DMAS_CTLR_BY_NAME(DT_DRV_INST(inst), rx)), \
        .dma_channel =                                                         \
            DT_DMAS_CELL_BY_NAME(DT_DRV_INST(inst), rx, channel),              \
        .dma_slot = DT_DMAS_CELL_BY_NAME(DT_DRV_INST(inst), rx, slot),         \
        .adc = (ADC_TypeDef *)DT_REG_ADDR(                                     \
            DT_PHANDLE_BY_IDX(DT_DRV_INST(inst), io_channels, 0)),             \
        .timer = (TIM_TypeDef *)DT_REG_ADDR(                                   \
            DT_INST_PHANDLE(inst, sampling_timer)),                            \
        .timer_pclken = {                                                       \
            .bus = DT_CLOCKS_CELL(DT_INST_PHANDLE(inst, sampling_timer), bus), \
            .enr = DT_CLOCKS_CELL(DT_INST_PHANDLE(inst, sampling_timer), bits),\
        },                                                                      \
        .work_q_stack = (k_thread_stack_t *)&work_q_stack_##inst,              \
        .work_q_stack_size = K_THREAD_STACK_SIZEOF(work_q_stack_##inst),       \
    };                                                                         \
    PM_DEVICE_DT_INST_DEFINE(inst, pm_action);                                  \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, &sensor_init,                           \
        PM_DEVICE_DT_INST_GET(inst), &sensor_##inst##_data,                    \
        &sensor_##inst##_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,     \
        &sensor_api);

DT_INST_FOREACH_STATUS_OKAY(_INIT)
