#include <limits.h>
#include <stdint.h>

#include "encoder_ab.h"
#include "esp_idf_version.h"

#define TAG = "ENCODER_AB";

#define PCNT_H_LIM_VAL     INT16_MAX
#define PCNT_L_LIM_VAL     INT16_MIN

#if ESP_IDF_VERSION_MAJOR >= 6
#include "driver/pulse_cnt.h"

static pcnt_unit_handle_t unit;
static pcnt_channel_handle_t channel;

void encoder_ab_init(int16_t sig_pin, int16_t dir_pin) {
    if (unit) {
        encoder_ab_clear();
        return;
    }

    pcnt_unit_config_t unit_config = {
        .low_limit = PCNT_L_LIM_VAL,
        .high_limit = PCNT_H_LIM_VAL,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 12500,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filter_config));

    pcnt_chan_config_t channel_config = {
        .edge_gpio_num = sig_pin,
        .level_gpio_num = dir_pin,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &channel_config, &channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(channel,
                                                PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(channel,
                                                 PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                 PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
}

void encoder_ab_clear() {
    if (unit) {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    }
}

int16_t encoder_ab_get_count() {
    int count = 0;
    if (unit) {
        ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &count));
    }
    return (int16_t)count;
}

#else
#include "driver/pcnt.h"

static int unit = PCNT_UNIT_0;

void encoder_ab_init(int16_t sig_pin, int16_t dir_pin) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = sig_pin,
        .ctrl_gpio_num = dir_pin,
        .channel = PCNT_CHANNEL_0,
        .unit = unit,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };

    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(unit, 1000);
    pcnt_filter_enable(unit);

    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    pcnt_counter_resume(unit);
}

void encoder_ab_clear() {
    pcnt_counter_clear(unit);
}

int16_t encoder_ab_get_count() {
    int16_t count = 0;
    pcnt_get_counter_value(unit, &count);
    return count;
}
#endif
