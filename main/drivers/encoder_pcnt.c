#include "encoder_pcnt.h"

#include <stdbool.h>

#include "driver/pulse_cnt.h"
#include "esp_log.h"

#define LEFT_ENC_A_GPIO   5
#define LEFT_ENC_B_GPIO   18
#define RIGHT_ENC_A_GPIO  19
#define RIGHT_ENC_B_GPIO  21

#define ENCODER_HIGH_LIMIT  30000
#define ENCODER_LOW_LIMIT  -30000

#define GLITCH_FILTER_NS  2000

static const char *TAG = "encoder_pcnt";

static pcnt_unit_handle_t s_left_unit;
static pcnt_unit_handle_t s_right_unit;

static int32_t s_left_prev;
static int32_t s_right_prev;
static int32_t s_left_delta;
static int32_t s_right_delta;

static bool s_initialized;

/*
 * 4× quadrature decoding: two PCNT channels per unit.
 * flags.accum_count = true makes the driver accumulate across hardware
 * limit crossings automatically, so pcnt_unit_get_count always returns
 * the true total without any manual read-and-clear race window.
 */
static esp_err_t unit_init(pcnt_unit_handle_t *unit, int gpio_a, int gpio_b)
{
    const pcnt_unit_config_t unit_cfg = {
        .low_limit        = ENCODER_LOW_LIMIT,
        .high_limit       = ENCODER_HIGH_LIMIT,
        .flags.accum_count = true,
    };
    esp_err_t ret = pcnt_new_unit(&unit_cfg, unit);
    if (ret != ESP_OK) return ret;

    const pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = GLITCH_FILTER_NS,
    };
    ret = pcnt_unit_set_glitch_filter(*unit, &filter_cfg);
    if (ret != ESP_OK) return ret;

    /* Channel A: edge on A, level (direction gate) on B. */
    pcnt_channel_handle_t chan_a;
    const pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = gpio_a,
        .level_gpio_num = gpio_b,
    };
    ret = pcnt_new_channel(*unit, &chan_a_cfg, &chan_a);
    if (ret != ESP_OK) return ret;

    ret = pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (ret != ESP_OK) return ret;

    ret = pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (ret != ESP_OK) return ret;

    /* Channel B: edge on B, level (direction gate) on A. */
    pcnt_channel_handle_t chan_b;
    const pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = gpio_b,
        .level_gpio_num = gpio_a,
    };
    ret = pcnt_new_channel(*unit, &chan_b_cfg, &chan_b);
    if (ret != ESP_OK) return ret;

    ret = pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (ret != ESP_OK) return ret;

    ret = pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);
    if (ret != ESP_OK) return ret;

    /* Watchpoints are required for accum_count to trigger accumulation. */
    ret = pcnt_unit_add_watch_point(*unit, ENCODER_HIGH_LIMIT);
    if (ret != ESP_OK) return ret;
    ret = pcnt_unit_add_watch_point(*unit, ENCODER_LOW_LIMIT);
    if (ret != ESP_OK) return ret;

    ret = pcnt_unit_enable(*unit);
    if (ret != ESP_OK) return ret;
    ret = pcnt_unit_clear_count(*unit);
    if (ret != ESP_OK) return ret;
    return pcnt_unit_start(*unit);
}

esp_err_t encoder_pcnt_init(void)
{
    const esp_err_t left_ret = unit_init(&s_left_unit, LEFT_ENC_A_GPIO, LEFT_ENC_B_GPIO);
    if (left_ret != ESP_OK) {
        ESP_LOGE(TAG, "left encoder init failed: %s", esp_err_to_name(left_ret));
        return left_ret;
    }

    const esp_err_t right_ret = unit_init(&s_right_unit, RIGHT_ENC_A_GPIO, RIGHT_ENC_B_GPIO);
    if (right_ret != ESP_OK) {
        ESP_LOGE(TAG, "right encoder init failed: %s", esp_err_to_name(right_ret));
        return right_ret;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t encoder_pcnt_read_sample(encoder_pcnt_sample_t *sample)
{
    if (sample == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_initialized)  return ESP_ERR_INVALID_STATE;

    int left_raw = 0, right_raw = 0;

    esp_err_t ret = pcnt_unit_get_count(s_left_unit, &left_raw);
    if (ret != ESP_OK) return ret;

    ret = pcnt_unit_get_count(s_right_unit, &right_raw);
    if (ret != ESP_OK) return ret;

    const int32_t left_count  = (int32_t)left_raw;
    const int32_t right_count = (int32_t)right_raw;

    s_left_delta  += left_count  - s_left_prev;
    s_right_delta += right_count - s_right_prev;
    s_left_prev    = left_count;
    s_right_prev   = right_count;

    sample->left_count  = left_count;
    sample->right_count = right_count;
    sample->left_delta  = s_left_delta;
    sample->right_delta = s_right_delta;

    s_left_delta  = 0;
    s_right_delta = 0;

    return ESP_OK;
}
