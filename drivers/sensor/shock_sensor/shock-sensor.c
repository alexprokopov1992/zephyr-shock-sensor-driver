/*
 * Copyright (c) 2023 FTP Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_shock_sensor

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include "shock-sensor.h"
#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(shock_sensor, CONFIG_SENSOR_LOG_LEVEL);
// LOG_MODULE_REGISTER(shock_sensor, LOG_LEVEL_DBG);

#ifdef CONFIG_SEQUENCE_32BITS_REGISTERS
    #define ADC_READING_TYPE uint32_t
#else
    #define ADC_READING_TYPE uint16_t
#endif

#define MAX_MAIN_TAP_LEVEL 1201
#define MAX_WARN_TAP_LEVEL 241

#define CONFIG_SEQUENCE_SAMPLES 16
#define MIN_TAP_INTERVAL 2000 // ms
#define STOP_ALARM_INTERVAL 5000
#define DELAY 100
#define ADC_READ_MAX_ATTEMPTS 3

struct sensor_data {
    struct adc_sequence sequence;
    k_timeout_t earliest_sample;
    ADC_READING_TYPE raw[CONFIG_SEQUENCE_SAMPLES];

    const struct device *dev;   // self reference
    struct k_work_q workq;   /* work queue */
    struct k_work_delayable dwork;  /* workers */

    // #ifndef CONFIG_USE_SYS_WORK_Q
    //     K_KERNEL_STACK_MEMBER(workq_stack, CONFIG_SENSOR_SHOCK_THREAD_STACK_SIZE);
    // #endif
    // struct k_poll_signal async_sig;
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

    int shake_warn;
    int shake_main;
    int adc_centered_value;

    int min_tap_interval;

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

// static const int warn_zones_initial[16] = {4, 5, 6, 8, 10, 12, 14, 17, 20, 23, 27, 32, 37, 43, 50, 60};
static const int warn_zones_initial[10] = {4, 6, 10, 16, 25, 39, 61, 97, 152, 240};
static const float koeff[10] = {
    1.76893602,
    1.698646465,
    1.614054238,
    1.539948249,
    1.472733358,
    1.408677779,
    1.34705442,
    1.285999948,
    1.229514814,
    1.174618943
};

static const float little_val = 0.03;
static const float warn_noise_divider = 1.6;


struct shock_sensor_dt_spec {
	const struct adc_dt_spec port;
    uint32_t sampling_period_ms;
};

struct sensor_config {
    struct shock_sensor_dt_spec sensor;
    struct gpio_dt_spec gpio_power;

    k_thread_stack_t *work_q_stack;
	size_t work_q_stack_size;
};

static void reset_timer_handler_alarm(struct k_timer *);

static void increase_sensivity_warn_handler(struct k_timer *);
static void increase_sensivity_main_handler(struct k_timer *);

static void set_zones(const struct device *dev, int warn_zone, int main_zone);
static void set_warn_zones(const struct device *dev);
static void create_main_zones(const struct device *dev, int zone);
static void coarsering_warn(struct sensor_data *data, bool increase);
static void coarsering_main(struct sensor_data *data, bool increase);
static void register_tap_main(struct sensor_data *data);
static void register_tap_warn(struct sensor_data *data);

// #define CONFIG_USE_SYS_WORK_Q
// #define CONFIG_USE_ASYNC_ADC_READ

// struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);




#define CONFIG_SHAKE_CENTERED_COUNT (256 / CONFIG_SEQUENCE_SAMPLES)
// Можливо треба зробити вдвічі більше ніж CONFIG_SHAKE_CENTERED_COUNT.
// Хоча раз взагалі якось працює і при 1, то може значення 4 буде більш ніж достатньо.
#define MULTIPLIER 16

#define Secs(x) (x * 200 / CONFIG_SEQUENCE_SAMPLES)

// Кількість періодів опитування датчика для визначення спрацювання.
#define CONFIG_SHAKE_MAIN_TIME Secs(1)
#define CONFIG_SHAKE_WARN_TIME Secs(1)




static int fetch(const struct device *dev, enum sensor_channel chan)
{
    int ret = 0;
    #if 0
    const struct voltage_config *config = dev->config;
    struct voltage_data *data = dev->data;

    if ((chan != SENSOR_CHAN_VOLTAGE) && (chan != SENSOR_CHAN_ALL)) {
        return -ENOTSUP;
    }

    /* Wait until sampling is valid */
    k_sleep(data->earliest_sample);

    ret = adc_read(config->voltage.port.dev, &data->sequence);
    if (ret != 0) {
        LOG_ERR("adc_read: %d", ret);
    }

#endif
    return ret;
}

static int get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    int ret = 0;
#if 0
    const struct voltage_config *config = dev->config;
    struct voltage_data *data = dev->data;
    int32_t raw_val;
    int32_t v_mv;

    __ASSERT_NO_MSG(val != NULL);

    if (chan != SENSOR_CHAN_VOLTAGE) {
        return -ENOTSUP;
    }

    if (config->voltage.port.channel_cfg.differential) {
        raw_val = (int16_t)data->raw;
    } else {
        raw_val = data->raw;
    }

    ret = adc_raw_to_millivolts_dt(&config->voltage.port, &raw_val);
    if (ret != 0) {
        LOG_ERR("raw_to_mv: %d", ret);
        return ret;
    }

    v_mv = raw_val;

    /* Note if full_ohms is not specified then unscaled voltage is returned */
    (void)voltage_divider_scale_dt(&config->voltage, &v_mv);

    LOG_DBG("%d of %d, %dmV, voltage:%dmV", data->raw,
        (1 << data->sequence.resolution) - 1, raw_val, v_mv);
    val->val1 = v_mv / 1000;
    val->val2 = (v_mv * 1000) % 1000000;
#endif
    return ret;
}

// k_timer_stop(&data->increase_sensivity_timer_warn);
// k_timer_stop(&data->increase_sensivity_timer_main);

static int attr_set(const struct device *dev,
    enum sensor_channel chan,
    enum sensor_attribute attr,
    const struct sensor_value *val)
{
    struct sensor_data *data = dev->data;
    LOG_DBG("[%s] dev: %p, chan: %d, attr: %d, val1: %d, val2: %d", __func__, dev, chan, attr, val->val1, val->val2);
    if (chan == SENSOR_CHAN_PROX && attr == SENSOR_ATTR_UPPER_THRESH) {
        data->treshold_warn = val->val1;
        data->treshold_main = val->val2;
        LOG_DBG("Seted treshold_warn: %d, treshold_main: %d", data->treshold_warn, data->treshold_main);
        return 0;
    }

    if (chan == (enum sensor_channel)SHOCK_SENSOR_INCREASE_SENSIVITY_INTERVAL_SEC && attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        data->increase_sensivity_interval = val->val1;
        LOG_DBG("Seted increase_sensivity_interval: %d", data->increase_sensivity_interval);
        k_timer_start(&data->increase_sensivity_timer_warn, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
        k_timer_start(&data->increase_sensivity_timer_main, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
        return 0;
    }

    if (chan == (enum sensor_channel)SHOCK_SENSOR_NOISE_SAMPLING_TIME_SEC && attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        data->noise_sampling_interval_sec = val->val1;
        data->noise_sampling_interval_msec = data->noise_sampling_interval_sec * 1000;
        LOG_DBG("Seted noise_sampling_interval_sec: %lld", data->noise_sampling_interval_sec);
        data->max_main_noise_level = 0;
        data->max_main_noise_level_time = k_uptime_get();
        data->max_warn_noise_level = 0;
        data->max_warn_noise_level_time = k_uptime_get();
        return 0;
    }

    int64_t current_time = k_uptime_get();
    
    if (chan == (enum sensor_channel)SHOCK_SENSOR_CHANNEL_WARN_ZONE && attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        if (val->val1 == 0) {
            data->warn_zone_active = false;
            data->current_warn_zone = 9;
            data->selected_warn_zone = 9;
            data->current_main_zone = data->selected_main_zone;
            set_zones(dev, data->current_warn_zone, data->current_main_zone);
            data->last_tap_time_warn = current_time;
            data->last_tap_time_main = current_time;
            data->max_main_noise_level = 0;
            data->max_main_noise_level_time = current_time;
            data->max_warn_noise_level = 0;
            data->max_warn_noise_level_time = current_time;
            data->max_level_alert_warn = false;
            data->max_level_alert_main = false;
            LOG_DBG("WARN_ZONE disabled");
            return 0;
        }
        int value = val->val1 / 10;
        int64_t current_time = k_uptime_get();
        data->current_warn_zone = 10 - value; 
        if (data->current_warn_zone < 0) {
            data->current_warn_zone = 0;
        }
        if (data->current_warn_zone > 9) {
            data->current_warn_zone = 9;
        }
        data->warn_zone_active = true;
        data->selected_warn_zone = data->current_warn_zone;
        create_main_zones(dev, data->current_warn_zone);
        data->current_main_zone = data->selected_main_zone;
        data->last_tap_time_warn = current_time;
        data->last_tap_time_main = current_time;
        data->max_main_noise_level = 0;
        data->max_main_noise_level_time = current_time;
        data->max_warn_noise_level = 0;
        data->max_warn_noise_level_time = current_time;
        data->max_level_alert_warn = false;
        data->max_level_alert_main = false;
        LOG_DBG("Seted warn_zone: %d", data->current_warn_zone);
        set_zones(dev, data->current_warn_zone, data->current_main_zone);
        return 0;
    }

    if (chan == (enum sensor_channel)SHOCK_SENSOR_CHANNEL_MAIN_ZONE && attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        if (val->val1 == 0) {
            data->main_zone_active = false;
            data->current_main_zone = data->selected_main_zone;
            data->last_tap_time_warn = current_time;
            data->last_tap_time_main = current_time;
            data->max_main_noise_level = 0;
            data->max_main_noise_level_time = current_time;
            data->max_warn_noise_level = 0;
            data->max_warn_noise_level_time = current_time;
            data->max_level_alert_warn = false;
            data->max_level_alert_main = false;
            LOG_DBG("MAIN_ZONE disabled");
            return 0;
        }
        int value = val->val1 / 10;
        int64_t current_time = k_uptime_get();
        data->current_main_zone = 10 - value;
        if (data->current_main_zone < 0) {
            data->current_main_zone = 0;
        }
        if (data->current_main_zone > 9) {
            data->current_main_zone = 9;
        }
        data->main_zone_active = true;
        data->selected_main_zone = data->current_main_zone;
        data->current_warn_zone = data->selected_warn_zone;
        data->last_tap_time_warn = current_time;
        data->last_tap_time_main = current_time;
        data->max_main_noise_level = 0;
        data->max_main_noise_level_time = current_time;
        data->max_warn_noise_level = 0;
        data->max_warn_noise_level_time = current_time;
        data->max_level_alert_warn = false;
        data->max_level_alert_main = false;
        LOG_DBG("Seted main_zone: %d", data->current_main_zone);
        set_zones(dev, data->current_warn_zone, data->current_main_zone);
        return 0;
    }

    if (chan == (enum sensor_channel)SHOCK_SENSOR_MODE && attr == (enum sensor_attribute)SHOCK_SENSOR_SPECIAL_ATTRS) {
        // LOG_INF("Current mode: %d -> %d", data->mode, val->val1);
        switch (data->mode) {
            case SHOCK_SENSOR_MODE_ARMED:
                switch (val->val1) {
                    case SHOCK_SENSOR_MODE_ARMED:
                        return 0;
                    case SHOCK_SENSOR_MODE_DISARMED:
                        data->mode = SHOCK_SENSOR_MODE_DISARMED;
                        data->current_warn_zone = data->selected_warn_zone;
                        data->current_main_zone = data->selected_main_zone;
                        data->max_level_alert_warn = false;
                        data->max_level_alert_main = false;
                        k_timer_stop(&data->reset_timer_alarm);
                        k_timer_stop(&data->increase_sensivity_timer_warn);
                        k_timer_stop(&data->increase_sensivity_timer_main);
                        if(data->warn_zone_active){
                            set_zones(dev, data->current_warn_zone, data->current_main_zone);
                        } else {
                            data->current_warn_zone = 9;
                            data->selected_warn_zone = 9;
                            set_zones(dev, data->current_warn_zone, data->current_main_zone);
                        }
                        LOG_DBG("Sensor is forced to disarmed mode");
                        return 0;
                    case SHOCK_SENSOR_MODE_ALARM:
                        data->mode = SHOCK_SENSOR_MODE_ALARM;
                        if (val->val2 > 3000) {
                            LOG_DBG("Entering alarm mode infinity time");
                            return 0;
                        } else {
                            k_timer_start(&data->reset_timer_alarm, K_MSEC(STOP_ALARM_INTERVAL), K_NO_WAIT);
                            LOG_DBG("Entering alarm mode for %d ms", STOP_ALARM_INTERVAL);
                            return 0;
                        }     
                    case SHOCK_SENSOR_MODE_ALARM_STOP:
                        return 0;
                    default:
                        return -ENOTSUP;
                }
            case SHOCK_SENSOR_MODE_DISARMED:
                switch (val->val1) {
                    case SHOCK_SENSOR_MODE_ARMED:
                        data->mode = SHOCK_SENSOR_MODE_ARMED;
                        int64_t current_time = k_uptime_get();
                        data->last_tap_time_warn = current_time;
                        data->last_tap_time_main = current_time;
                        k_timer_stop(&data->reset_timer_alarm);
                        k_timer_stop(&data->increase_sensivity_timer_warn);
                        k_timer_stop(&data->increase_sensivity_timer_main);
                        data->max_main_noise_level = 0;
                        data->max_main_noise_level_time = current_time;
                        data->max_warn_noise_level = 0;
                        data->max_warn_noise_level_time = current_time;
                        LOG_DBG("Sensor is armed");
                        return 0;
                    case SHOCK_SENSOR_MODE_DISARMED:
                        return 0;
                    case SHOCK_SENSOR_MODE_ALARM_STOP:
                        return 0;
                    case SHOCK_SENSOR_MODE_ALARM:
                        return 0;
                    default:
                        return -ENOTSUP;
                }
            case SHOCK_SENSOR_MODE_ALARM:
                switch (val->val1) {
                    case SHOCK_SENSOR_MODE_ARMED:
                        return 0;
                    case SHOCK_SENSOR_MODE_DISARMED:
                        data->mode = SHOCK_SENSOR_MODE_DISARMED;
                        data->current_warn_zone = data->selected_warn_zone;
                        data->current_main_zone = data->selected_main_zone;
                        data->max_level_alert_warn = false;
                        data->max_level_alert_main = false;
                        k_timer_stop(&data->reset_timer_alarm);
                        k_timer_stop(&data->increase_sensivity_timer_warn);
                        k_timer_stop(&data->increase_sensivity_timer_main);
                        set_zones(dev, data->current_warn_zone, data->current_main_zone);
                        LOG_DBG("Sensor is forced to disarmed mode");
                        return 0;
                    case SHOCK_SENSOR_MODE_ALARM_STOP:
                        k_timer_start(&data->reset_timer_alarm, K_MSEC(STOP_ALARM_INTERVAL), K_NO_WAIT);
                        LOG_DBG("Alarm mode stoped in %d ms", STOP_ALARM_INTERVAL);
                        return 0;
                    case SHOCK_SENSOR_MODE_ALARM:
                        data->mode = SHOCK_SENSOR_MODE_ALARM;
                        if (val->val2 > 3000) {
                            LOG_DBG("Entering alarm mode infinity time");
                            return 0;
                        } else {
                            k_timer_start(&data->reset_timer_alarm, K_MSEC(STOP_ALARM_INTERVAL), K_NO_WAIT);
                            LOG_DBG("Entering alarm mode for %d ms", STOP_ALARM_INTERVAL);
                            return 0;
                        }
                    default:
                        return -ENOTSUP;
                }
            case SHOCK_SENSOR_MODE_ALARM_STOP:
                return 0;
            default:
                return -ENOTSUP;
        }


        // if (val->val1 == SHOCK_SENSOR_MODE_ALARM && data->mode == SHOCK_SENSOR_MODE_ARMED) { 
        //     data->mode = SHOCK_SENSOR_MODE_ALARM;
        //     k_timer_start(&data->reset_timer_alarm, K_MSEC(val->val2), K_NO_WAIT);
        //     LOG_INF("Entering alarm mode for %d ms", val->val2);
        // }

        // if (val->val1 == SHOCK_SENSOR_MODE_ALARM) {
        //     return 0;
        // }
        
        // data->mode = val->val1;

        // if (data->mode == SHOCK_SENSOR_MODE_ALARM_STOP) {
        //     // data->mode = SHOCK_SENSOR_MODE_ARMED;
        //     // k_timer_stop(&data->reset_timer_alarm);
        //     k_timer_start(&data->reset_timer_alarm, K_MSEC(STOP_ALARM_INTERVAL), K_NO_WAIT);
        //     // k_timer_stop(&data->increase_sensivity_timer_warn);
        //     // k_timer_stop(&data->increase_sensivity_timer_main);
        //     LOG_INF("Alarm mode stoped in %d ms", STOP_ALARM_INTERVAL);
        // }
        // if (data->mode == SHOCK_SENSOR_MODE_DISARMED) {
        //     data->current_warn_zone = data->selected_warn_zone;
        //     data->current_main_zone = data->selected_main_zone;
        //     data->max_level_alert_warn = false;
        //     data->max_level_alert_main = false;
        //     k_timer_stop(&data->reset_timer_alarm);
        //     k_timer_stop(&data->increase_sensivity_timer_warn);
        //     k_timer_stop(&data->increase_sensivity_timer_main);
        //     set_zones(dev, data->current_warn_zone, data->current_main_zone);
        //     LOG_INF("Sensor is forced to disarmed mode");
        // }
        // if (data->mode == SHOCK_SENSOR_MODE_ARMED) {
        //     int64_t current_time = k_uptime_get();
        //     data->last_tap_time_warn = current_time;
        //     data->last_tap_time_main = current_time;
        //     k_timer_stop(&data->reset_timer_alarm);
        //     k_timer_stop(&data->increase_sensivity_timer_warn);
        //     k_timer_stop(&data->increase_sensivity_timer_main);
        //     data->max_main_noise_level = 0;
        //     data->max_main_noise_level_time = current_time;
        //     data->max_warn_noise_level = 0;
        //     data->max_warn_noise_level_time = current_time;
        //     // k_timer_start(&data->increase_sensivity_timer_warn, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
        //     // k_timer_start(&data->increase_sensivity_timer_main, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
        //     LOG_INF("Sensor is armed");
        // }
        // return 0;
    }

    return -ENOTSUP;
}

static int trigger_set(const struct device *dev, const struct sensor_trigger *trig,
    sensor_trigger_handler_t handler)
{
    struct sensor_data *data = dev->data;
    // LOG_ERR("[%s - %s] dev: %p, trig->chan: %d, trig->type: %d, handler: %p", __func__,
    // (handler == NULL) ? "off" : "on", dev, trig->chan, trig->type, handler);

    if (trig == NULL || handler == NULL) return -EINVAL;

    if (trig->type != SENSOR_TRIG_THRESHOLD && trig->type != SENSOR_TRIG_TAP) {
		return -ENOTSUP;
	}

    switch (trig->type) {
        case SENSOR_TRIG_TAP:
            // LOG_ERR("----------------- Seted");
            data->warn_handler = handler;
            data->warn_trigger = trig;
        break;
        case SENSOR_TRIG_THRESHOLD:
            // LOG_ERR("----------------- Seted");
            data->main_handler = handler;
            data->main_trigger = trig;
        break;
        default:
            return -ENOTSUP;
    }

    return 0;
}

#ifdef CONFIG_PM_DEVICE

static int pm_action(const struct device *dev, enum pm_device_action action)
{
    int ret = 0;
    #if 0
    const struct voltage_config *config = dev->config;
    struct voltage_data *data = dev->data;

    if (config->gpio_power.port == NULL) {
        /* No work to do */
        return 0;
    }

    switch (action) {
    case PM_DEVICE_ACTION_TURN_ON:
        ret = gpio_pin_configure_dt(&config->gpio_power, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("failed to configure GPIO for PM on");
        }
        break;
    case PM_DEVICE_ACTION_RESUME:
        ret = gpio_pin_set_dt(&config->gpio_power, 1);
        if (ret != 0) {
            LOG_ERR("failed to set GPIO for PM resume");
        }
        data->earliest_sample = K_TIMEOUT_ABS_TICKS(
            k_uptime_ticks() + k_us_to_ticks_ceil32(config->sample_delay_us));
        break;
#ifdef CONFIG_PM_DEVICE
    case PM_DEVICE_ACTION_SUSPEND:
        ret = gpio_pin_set_dt(&config->gpio_power, 0);
        if (ret != 0) {
            LOG_ERR("failed to set GPIO for PM suspend");
        }
        break;
    case PM_DEVICE_ACTION_TURN_OFF:
        break;
#endif /* CONFIG_PM_DEVICE */
    default:
        return -ENOTSUP;
    }
    #endif

    return ret;
}
#endif

// static void adc_vbus_process()
// {
//     printk("w");
//     /* Запускаємо асинхронне читання */
//     int ret = adc_read_async(adc_dev, &adc_seq_async, &adc_done_signal);
// }


#define __TEMP_SENSOR_SHAKE_WARN_ZONE 14
#define __TEMP_SENSOR_SHAKE_MAIN_ZONE 100

// int debug_counter = 0;


static void adc_vbus_work_handler(struct k_work *work)
{
    // Шось це якось заплутано. Чи можна якось простіше?
    // Можна при ініціалізації скопіювати sampling_period_ms в data
    struct k_work_delayable *delayable = k_work_delayable_from_work(work);
    struct sensor_data *data = CONTAINER_OF(delayable, struct sensor_data, dwork);

    if (data->mode == SHOCK_SENSOR_MODE_DISARMED) {
        k_work_schedule_for_queue(&data->workq, &data->dwork, K_MSEC(DELAY));
        return;
    }
    // dev
    const struct device *dev = data->dev;
    // const struct sensor_config *config = data->dev->config;
    const struct sensor_config *config = dev->config;

    // adc_vbus_process();

    // printk("w");
    #if 0
    int ret = adc_read(config->sensor.port.dev, &data->sequence);
	if (ret != 0) {
		LOG_ERR("adc_read: %d", ret);
	}
    #endif

    // uint64_t timestart = k_ticks_to_us_floor64(k_uptime_ticks()); //k_uptime_get();
    int ret = 1;
    #if defined(CONFIG_USE_ASYNC_ADC_READ)
        struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
        struct k_poll_event async_evt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
        K_POLL_MODE_NOTIFY_ONLY, &async_sig);
        ret = adc_read_async(config->sensor.port.dev, &data->sequence, &async_sig);
    #else
    int attempts = 0;
    do {
        ret = adc_read(config->sensor.port.dev, &data->sequence);
        if (++attempts == ADC_READ_MAX_ATTEMPTS) break;
    } while(ret != 0);

    #endif
    if(ret != 0) {
        LOG_ERR("adc_read[_async]: %d", ret);
        goto end;
    }
    // debug_counter++;
    #if defined(CONFIG_USE_ASYNC_ADC_READ)
        ret = k_poll(&async_evt, 1, K_FOREVER);
        if(ret != 0) {
            LOG_ERR("k_poll: %d", ret);
            goto end;
        }
    #endif
    // LOG_ERR("Data: %d", data->raw);

    // uint64_t timestop = k_ticks_to_us_floor64(k_uptime_ticks()); //k_uptime_get();
    // LOG_ERR("ADC read time: %" PRIu64 " usec", timestop - timestart);

    // Час перетворення при interval_us=0:
    //   1 самплінг - 1200 usec
    //  10 самплінгів - 1700 usec
    // 100 самплінгів - 9200 usec

    #if 0
    #if CONFIG_SEQUENCE_SAMPLES == 1
        LOG_ERR("Data: %d", data->raw[0]);
    #elif CONFIG_SEQUENCE_SAMPLES == 10
        LOG_ERR("Data: "
            "%d %d %d %d %d %d %d %d %d %d",
            data->raw[0], data->raw[1], data->raw[2], data->raw[3], data->raw[4],
            data->raw[5], data->raw[6], data->raw[7], data->raw[8], data->raw[9]);
    #else
        uint32_t sum = 0;
        uint32_t min = data->raw[0];
        uint32_t max = data->raw[0];
        for(int i=0; i<CONFIG_SEQUENCE_SAMPLES; i++) {
            sum += data->raw[i];
            if(data->raw[i] < min) {
                min = data->raw[i];
            }
            if(data->raw[i] > max) {
                max = data->raw[i];
            }
        }
        LOG_ERR("Data: %d/%d/%d %d", min, sum/CONFIG_SEQUENCE_SAMPLES, max, max-min);
        // LOG_HEXDUMP_ERR(data->raw, sizeof(data->raw), "Data: ");
    #endif
    #endif

    // Збільшемо значення шоб подальша математика не втрачала точність
    #if CONFIG_SEQUENCE_SAMPLES == 1
        int new_val = data->raw[0] * MULTIPLIER;
    #else
        uint32_t sum = 0;
        uint32_t min = data->raw[0];
        uint32_t max = data->raw[0];
        for(int i=0; i<CONFIG_SEQUENCE_SAMPLES; i++) {
            sum += data->raw[i];
            if(data->raw[i] < min) {
                min = data->raw[i];
            }
            if(data->raw[i] > max) {
                max = data->raw[i];
            }
        }
        min *= MULTIPLIER;
        max *= MULTIPLIER;
        int new_val = sum * MULTIPLIER / CONFIG_SEQUENCE_SAMPLES;
    #endif

    // Set centered value. Middle of the last 16 samples
    // Save 16x value, but use normalised value later
    data->adc_centered_value = (data->adc_centered_value * (CONFIG_SHAKE_CENTERED_COUNT-1) + new_val) / CONFIG_SHAKE_CENTERED_COUNT;

    // LOG_ERR("Shake sensor sample_raw: %d", val.val1);

    if(data->shake_main) {
        data->shake_main--;
    }

    if(data->shake_warn) {
        data->shake_warn--;
    }

    if (data->mode)
    {
        goto end;
    }

    #if CONFIG_SEQUENCE_SAMPLES == 1
        int amplitude_x = new_val - data->adc_centered_value;
        int amplitude_abs = abs(amplitude_x) / MULTIPLIER;
    #else
        int amplitude_x1 = max - data->adc_centered_value;
        int amplitude_x2 = data->adc_centered_value - min;
        int amplitude_x = (amplitude_x1 > amplitude_x2) ? amplitude_x1 : amplitude_x2;
        int amplitude_abs = amplitude_x / MULTIPLIER;
    #endif

    // if(debug_counter >= 31) {
    //     LOG_ERR("Shake amplitude: %d  %d-%d-%d", 
    //         amplitude_abs,
    //         min, adc_centered_value, max
    //     );
    //     debug_counter = 0;
    // }

    int64_t current_time = k_uptime_get();

    if (amplitude_abs > data->treshold_main && data->shake_main == 0 && !data->max_level_alert_main && data->main_zone_active) {
        data->shake_main = CONFIG_SHAKE_MAIN_TIME;
        if (data->main_handler) {
            if (!data->max_level_alert_main)
            {
                data->mode = SHOCK_SENSOR_MODE_ALARM;
                data->main_handler(dev, data->main_trigger);  
                LOG_DBG("MAIN amplitude: %d", amplitude_abs);
                // LOG_INF("MAIN amplitude: %d %d", amplitude_abs, data->mode);
                register_tap_main(data);
            } else {
                LOG_DBG("MAIN trigger disabled amplitude: %d", amplitude_abs);
            }
        } else {
            LOG_ERR("Problem with main_handler");
        }
    } else if (amplitude_abs > data->treshold_warn && data->shake_warn == 0 && !data->max_level_alert_warn && data->warn_zone_active) {
        data->shake_warn = CONFIG_SHAKE_WARN_TIME;
        if (data->warn_handler) {
            if (current_time - data->last_tap_time_warn > MIN_TAP_INTERVAL){
                if (!data->max_level_alert_warn)
                {
                    data->warn_handler(dev, data->warn_trigger);
                    LOG_DBG("WARN amplitude: %d", amplitude_abs);
                    register_tap_warn(data);
                } else {
                    LOG_DBG("WARN trigger disabled amplitude: %d", amplitude_abs);
                }
            } 
        } else {
            LOG_ERR("Problem with warn_handler");
        }
    } else if (data->shake_warn == 0 && data->shake_main == 0) {
        data->shake_warn = CONFIG_SHAKE_MAIN_TIME;
        data->shake_main = CONFIG_SHAKE_WARN_TIME;
        if (!data->main_zone_active && !data->warn_zone_active) {
            goto end;
        }
        if ((current_time - data->max_main_noise_level_time) > data->noise_sampling_interval_msec) {
            int prev_level = data->max_main_noise_level;
            LOG_DBG("MAIN noise window reset. Previous max: %d", data->max_main_noise_level);
            data->max_main_noise_level = (int)((float)data->max_main_noise_level / (koeff[data->selected_warn_zone]+little_val));
            if (prev_level == data->max_main_noise_level) {
                data->max_main_noise_level = 0;
            }
            LOG_DBG("Decrease MAIN noise level to: %d", data->max_main_noise_level);
            data->max_main_noise_level_time = current_time;
        } else if (amplitude_abs >= data->max_main_noise_level) {
                    int old_noise_level = data->max_main_noise_level;
                    if (amplitude_abs > MAX_MAIN_TAP_LEVEL) {
                        data->max_main_noise_level = MAX_MAIN_TAP_LEVEL;
                    } else{
                        data->max_main_noise_level = amplitude_abs;
                    }
                    data->max_main_noise_level_time = current_time;
                    if (old_noise_level != data->max_main_noise_level) LOG_DBG("New MAIN max noise level: %d", data->max_main_noise_level);
        }

        if ((current_time - data->max_warn_noise_level_time) > data->noise_sampling_interval_msec) {
            int prev_level = data->max_warn_noise_level;
            LOG_DBG("WARN noise window reset. Previous max: %d", data->max_warn_noise_level);
            data->max_warn_noise_level = (int)((float)data->max_warn_noise_level / warn_noise_divider);
            if (prev_level == data->max_warn_noise_level) {
                data->max_warn_noise_level = 0;
            }
            LOG_DBG("Decrease WARN noise level to: %d", data->max_warn_noise_level);
            data->max_warn_noise_level_time = current_time;
        } else if (amplitude_abs >= data->max_warn_noise_level) {
                    int old_noise_level = data->max_warn_noise_level;
                    if (amplitude_abs > MAX_WARN_TAP_LEVEL) {
                        data->max_warn_noise_level = MAX_WARN_TAP_LEVEL;
                    } else{
                        data->max_warn_noise_level = amplitude_abs;
                    }
                    data->max_warn_noise_level_time = current_time;
                    if (old_noise_level != data->max_warn_noise_level) LOG_DBG("New WARN max noise level: %d", data->max_warn_noise_level);
        }

    }
    

end:
    #ifdef CONFIG_USE_SYS_WORK_Q
        k_work_schedule(&data->dwork, K_MSEC(config->sensor.sampling_period_ms));
    #else
        k_work_schedule_for_queue(&data->workq, &data->dwork, K_MSEC(config->sensor.sampling_period_ms));
    #endif
}

// static K_THREAD_STACK_DEFINE(workq_stack, CONFIG_SENSOR_SHOCK_THREAD_STACK_SIZE);

static const struct sensor_driver_api sensor_api = {
    .sample_fetch = fetch,
    .channel_get = get,
    .trigger_set = trigger_set,
	.attr_set = attr_set,
};

/* Options for the sequence sampling. */
const struct adc_sequence_options options = {
    .extra_samplings = CONFIG_SEQUENCE_SAMPLES - 1,
    .interval_us = 2000,
};

static int sensor_init(const struct device *dev)
{
    int ret = 0;

    LOG_DBG("==== shock_sensor_init(%s)\n", dev->name);

    const struct sensor_config *config = dev->config;
    struct sensor_data *data = dev->data;

    // self reference
    data->dev = dev;
    data->shake_main = 0;
    data->shake_warn = 0;
    data->adc_centered_value = 0;
    data->warn_handler = NULL;
    data->main_handler = NULL;
    data->treshold_warn = __TEMP_SENSOR_SHAKE_WARN_ZONE;
    data->treshold_main = __TEMP_SENSOR_SHAKE_MAIN_ZONE;

    // Перші 10 секунд треба пропустити спрацювання за для стабілізації датчика
    data->shake_main = 10 * 1000 / config->sensor.sampling_period_ms / CONFIG_SEQUENCE_SAMPLES;
    data->shake_warn = data->shake_main;

    // Середина живлення (десь 522513). Якось треба врахувати розрядність ADC. Поки це 12 біт.
    data->adc_centered_value = MULTIPLIER * 2048;

    /* Default value to use if `power-gpios` does not exist */
    data->earliest_sample = K_TIMEOUT_ABS_TICKS(0);

    if (!adc_is_ready_dt(&config->sensor.port)) {
        LOG_ERR("ADC is not ready");
        return -ENODEV;
    }

    if (config->gpio_power.port != NULL) {
        if (!gpio_is_ready_dt(&config->gpio_power)) {
            LOG_ERR("Power GPIO is not ready");
            return -ENODEV;
        }
    }

    ret = adc_channel_setup_dt(&config->sensor.port);
    if (ret != 0) {
        LOG_ERR("setup: %d", ret);
        return ret;
    }

    ret = adc_sequence_init_dt(&config->sensor.port, &data->sequence);
    if (ret != 0) {
        LOG_ERR("sequence init: %d", ret);
        return ret;
    }

    data->sequence.buffer = &data->raw[0];
    data->sequence.buffer_size = sizeof(data->raw); /* buffer size in bytes, not number of samples */
    LOG_DBG("Buffer size: %d", data->sequence.buffer_size);
    // data->sequence.resolution = 12;
    data->sequence.options = &options;


    // Use k_sys_work_q for the work queue
    data->workq = k_sys_work_q;

    LOG_DBG("==== shock_sensor period: %d ms\n", config->sensor.sampling_period_ms);

    #ifdef CONFIG_USE_SYS_WORK_Q
        k_work_init_delayable(&data->dwork, adc_vbus_work_handler);
        k_work_schedule(&data->dwork, K_MSEC(config->sensor.sampling_period_ms));
    #else
        // Init signal
        // k_poll_signal_init(&data->async_sig);

        struct k_work_queue_config workq_cfg = {
            .name = "shock_sensor:WQ",
            // .no_yield = true,
        };

        k_work_queue_init(&data->workq);
        /* initialize the sensor work queue */
        k_work_queue_start(&data->workq,
            config->work_q_stack, config->work_q_stack_size,
            // workq_stack, K_KERNEL_STACK_SIZEOF(workq_stack),
            // data->workq_stack,
            // K_KERNEL_STACK_SIZEOF(data->workq_stack),
            CONFIG_SENSOR_SHOCK_THREAD_PRIORITY,
            // K_PRIO_COOP(7),
            &workq_cfg);
        // k_thread_name_set(&data->workq.thread, "shock_sensor:WQ");
        k_work_init_delayable(&data->dwork, adc_vbus_work_handler);
        k_work_schedule_for_queue(&data->workq, &data->dwork, K_MSEC(config->sensor.sampling_period_ms));
    #endif

    // k_work_init_delayable(&data->dwork, adc_vbus_work_handler);
    // // Перший запуск зробимо з системної черги
    // k_work_schedule(&data->dwork, K_MSEC(config->sensor.sampling_period_ms));
            

    #ifdef CONFIG_PM_DEVICE_RUNTIME
        return pm_device_driver_init(dev, pm_action);
    #else
        data->mode = SHOCK_SENSOR_MODE_DISARMED;
        set_warn_zones(dev);
        set_zones(dev, 0, 0);

        data->last_tap_time_warn = k_uptime_get();
        data->last_tap_time_main = k_uptime_get();
        k_timer_init(&data->reset_timer_alarm, reset_timer_handler_alarm, NULL);
        k_timer_init(&data->increase_sensivity_timer_warn, increase_sensivity_warn_handler, NULL);
        k_timer_init(&data->increase_sensivity_timer_main, increase_sensivity_main_handler, NULL);

        return 0;
    #endif
}

static void reset_timer_handler_alarm(struct k_timer *timer)
{
    struct sensor_data *data = CONTAINER_OF(timer, struct sensor_data, reset_timer_alarm);

    if (data->mode != SHOCK_SENSOR_MODE_ALARM) return;

    data->mode = SHOCK_SENSOR_MODE_ARMED;
    LOG_DBG("Sensor is armed");
}

static void increase_sensivity_warn_handler(struct k_timer *timer)
{
    struct sensor_data *data = CONTAINER_OF(timer, struct sensor_data, increase_sensivity_timer_warn);

    if (data->mode != SHOCK_SENSOR_MODE_ARMED) return;
    if (data->warn_zone_active == false) return;
    coarsering_warn(data, false);
}

static void increase_sensivity_main_handler(struct k_timer *timer)
{
    struct sensor_data *data = CONTAINER_OF(timer, struct sensor_data, increase_sensivity_timer_main);

    if (data->mode != SHOCK_SENSOR_MODE_ARMED) return;
    if (data->main_zone_active == false) return;
    coarsering_main(data, false);
    
}

static void set_zones(const struct device *dev, int warn_zone, int main_zone)
{
    struct sensor_data *data = dev->data;
    data->selected_warn_zone = warn_zone;
    data->current_warn_zone = warn_zone;
    create_main_zones(dev, warn_zone);
    data->selected_main_zone = main_zone;
    data->current_main_zone = main_zone;
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[warn_zone], .val2 = data->main_zones[main_zone] });
}

static void set_warn_zones(const struct device *dev)
{
    struct sensor_data *data = dev->data;
    for (int i = 0; i < 10; i++) {
        data->warn_zones[i] = warn_zones_initial[i]; 
    }
}

static void create_main_zones(const struct device *dev, int zone)
{
    struct sensor_data *data = dev->data;
    float k = koeff[data->selected_warn_zone];
    float float_main_zones[10];
    float_main_zones[0] = (float)data->treshold_warn * k;
    data->main_zones[0] = (int)roundf(float_main_zones[0]);
    for (int i = 1; i < 10; i++) {
        float_main_zones[i] = float_main_zones[i-1] * k;
        data->main_zones[i] = (int)roundf(float_main_zones[i]);
    }
}

static void coarsering_warn(struct sensor_data *data, bool increase)
{
    if (increase) {
        if (data->current_warn_zone == 9) 
        {
            data->max_level_alert_warn = true;
            LOG_DBG("Warning: Minimum warn zone sensivity reached");
            k_timer_start(&data->increase_sensivity_timer_warn, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }
        // if (data->warn_zones[data->current_warn_zone+1] >= data->treshold_main)
        // {
        //     // LOG_INF("Warning: warn zone sensivity can`t be increased due to main zone");
        //     k_timer_start(&data->increase_sensivity_timer_warn, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
        //     return;
        // }
        data->current_warn_zone++;
    } else {
        if (data->current_warn_zone == data->selected_warn_zone)
        {
            LOG_DBG("Warning: Maximum or setted warn zone sensivity reached");
            // k_timer_stop(&data->increase_sensivity_timer_warn);
            return;
        }
        if (data->max_warn_noise_level <= data->warn_zones[data->current_warn_zone - 1])
        {
            data->current_warn_zone--; 
            data->max_level_alert_warn = false;
        } else {
            LOG_DBG("Warning: warn zone sensivity can`t be decreased due to noise level");
            k_timer_start(&data->increase_sensivity_timer_warn, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }
    }
    k_timer_start(&data->increase_sensivity_timer_warn, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
    sensor_attr_set(data->dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[data->current_warn_zone], .val2 = data->main_zones[data->current_main_zone] });
}

static void coarsering_main(struct sensor_data *data, bool increase)
{
    if (increase) {
        if (data->current_main_zone == 9) 
        {
            data->max_level_alert_main = true;
            LOG_DBG("Warning: Minimum main zone sensivity reached");
            k_timer_start(&data->increase_sensivity_timer_main, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }
        data->current_main_zone++;
    } else {
        if (data->current_main_zone == data->selected_main_zone) 
        {
            LOG_DBG("Warning: Maximum or setted main zone sensivity reached");
            // k_timer_start(&data->increase_sensivity_timer_main, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }
        if (data->max_main_noise_level <= data->main_zones[data->current_main_zone - 1])
        {
            if (data->main_zones[data->current_main_zone-1] <= data->treshold_warn){
                LOG_DBG("Warning: main zone sensivity can`t be decreased due to warn zone");
                k_timer_start(&data->increase_sensivity_timer_main, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
                return;
            }
            data->max_level_alert_main = false;
            data->current_main_zone--; 
        } else {
            LOG_DBG("Warning: main zone sensivity can`t be decreased due to noise level");
            k_timer_start(&data->increase_sensivity_timer_main, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
            return;
        }
    }
    k_timer_start(&data->increase_sensivity_timer_main, K_SECONDS(data->increase_sensivity_interval), K_NO_WAIT);
    sensor_attr_set(data->dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &(struct sensor_value){ .val1 = data->warn_zones[data->current_warn_zone], .val2 = data->main_zones[data->current_main_zone] });
}

static void register_tap_main(struct sensor_data *data)
{
    int64_t current_time = k_uptime_get();
    // if (current_time - data->last_tap_time_main < MIN_TAP_INTERVAL) {
    //     // LOG_INF("Warning: Possible abuse detected, taps too frequent");
    //     return;
    // }
    data->last_tap_time_main = current_time;
    coarsering_main(data, true);
    
}

static void register_tap_warn(struct sensor_data *data)
{
    int64_t current_time = k_uptime_get();
    // if (current_time - data->last_tap_time_warn < MIN_TAP_INTERVAL) {
    //     // LOG_INF("Warning: Possible abuse detected, taps too frequent");
    //     return;
    // } 
    data->last_tap_time_warn = current_time;
    coarsering_warn(data, true);
} 



#define _INIT(inst)                                                                         \
    static struct sensor_data sensor_##inst##_data;                                       \
    static K_THREAD_STACK_DEFINE(work_q_stack_##_inst, CONFIG_SENSOR_SHOCK_THREAD_STACK_SIZE);       \
                                                                                            \
    static const struct sensor_config sensor_##inst##_config = {                          \
        .sensor = SHOCK_SENSOR_DT_SPEC_GET(DT_DRV_INST(inst)),                          \
        .gpio_power = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {0}),                     \
		.work_q_stack = (k_thread_stack_t *)&work_q_stack_##_inst,                         \
		.work_q_stack_size = K_THREAD_STACK_SIZEOF(work_q_stack_##_inst),                  \
    };                                                                                      \
                                                                                            \
    PM_DEVICE_DT_INST_DEFINE(inst, pm_action);                                              \
                                                                                            \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, &sensor_init, PM_DEVICE_DT_INST_GET(inst),          \
                &sensor_##inst##_data, &sensor_##inst##_config, POST_KERNEL,              \
                CONFIG_SENSOR_INIT_PRIORITY, &sensor_api);

DT_INST_FOREACH_STATUS_OKAY(_INIT)
