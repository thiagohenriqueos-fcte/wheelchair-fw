#include "encoder_pcnt.h"

#include <stdbool.h>

#include "driver/pulse_cnt.h"
#include "esp_log.h"

/* GPIO assignments — differential line receivers connect before these pins. */
#define LEFT_ENC_A_GPIO   5
#define LEFT_ENC_B_GPIO   18
#define RIGHT_ENC_A_GPIO  19
#define RIGHT_ENC_B_GPIO  21

/*
 * Hardware counter limits.  Kept well below ±32767 so a single polling
 * interval (50 ms) cannot overflow even at high encoder speeds.
 */
#define ENCODER_HIGH_LIMIT  30000
#define ENCODER_LOW_LIMIT  -30000

/* Reject glitch pulses shorter than 1 µs. */
#define GLITCH_FILTER_NS  1000

static const char *TAG = "encoder_pcnt";

static pcnt_unit_handle_t s_left_unit;
static pcnt_unit_handle_t s_right_unit;

static int32_t s_left_count;
static int32_t s_right_count;
static int32_t s_left_delta;
static int32_t s_right_delta;

static bool s_initialized;

/*
 * 4× quadrature decoding: two PCNT channels per unit.
 *
 * Channel A  (edge = gpio_a, level = gpio_b):
 *   A rising  + B=0 → +1   A rising  + B=1 → -1
 *   A falling + B=0 → -1   A falling + B=1 → +1
 *
 * Channel B  (edge = gpio_b, level = gpio_a):
 *   B rising  + A=0 → -1   B rising  + A=1 → +1
 *   B falling + A=0 → +1   B falling + A=1 → -1
 */
static esp_err_t unit_init(
    pcnt_unit_handle_t *unit,
    int gpio_a,
    int gpio_b)
{
    const pcnt_unit_config_t unit_cfg = {
        .low_limit  = ENCODER_LOW_LIMIT,
        .high_limit = ENCODER_HIGH_LIMIT,
    };
    esp_err_t ret = pcnt_new_unit(&unit_cfg, unit);
    if (ret != ESP_OK) {
        return ret;
    }

    const pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = GLITCH_FILTER_NS,
    };
    ret = pcnt_unit_set_glitch_filter(*unit, &filter_cfg);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Channel A: edge on A, level (direction gate) on B. */
    pcnt_channel_handle_t chan_a;
    const pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = gpio_a,
        .level_gpio_num = gpio_b,
    };
    ret = pcnt_new_channel(*unit, &chan_a_cfg, &chan_a);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,   /* A falling → -1 (B=0) */
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);  /* A rising  → +1 (B=0) */
    if (ret != ESP_OK) {
        return ret;
    }
    ret = pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,   /* B=1 → invert edge action */
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);     /* B=0 → keep edge action   */
    if (ret != ESP_OK) {
        return ret;
    }

    /* Channel B: edge on B, level (direction gate) on A. */
    pcnt_channel_handle_t chan_b;
    const pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = gpio_b,
        .level_gpio_num = gpio_a,
    };
    ret = pcnt_new_channel(*unit, &chan_b_cfg, &chan_b);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   /* B falling → +1 (A=0) */
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);  /* B rising  → -1 (A=0) */
    if (ret != ESP_OK) {
        return ret;
    }
    ret = pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,   /* A=1 → invert edge action */
        PCNT_CHANNEL_LEVEL_ACTION_KEEP);     /* A=0 → keep edge action   */
    if (ret != ESP_OK) {
        return ret;
    }

    ret = pcnt_unit_enable(*unit);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = pcnt_unit_clear_count(*unit);
    if (ret != ESP_OK) {
        return ret;
    }
    return pcnt_unit_start(*unit);
}

/*
 * Read the hardware counter, add to the 32-bit accumulators, then clear the
 * hardware register so it stays well within its ±30000 range between polls.
 * A few counts may arrive between get and clear; this is acceptable for the
 * 50 ms polling rate used in v0.7.
 */
static esp_err_t accum_unit(
    pcnt_unit_handle_t unit,
    int32_t *total,
    int32_t *delta)
{
    int raw = 0;
    esp_err_t ret = pcnt_unit_get_count(unit, &raw);
    if (ret != ESP_OK) {
        return ret;
    }
    *total += (int32_t)raw;
    *delta += (int32_t)raw;
    return pcnt_unit_clear_count(unit);
}

esp_err_t encoder_pcnt_init(void)
{
    const esp_err_t left_ret  = unit_init(&s_left_unit,  LEFT_ENC_A_GPIO,  LEFT_ENC_B_GPIO);
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

esp_err_t encoder_pcnt_get_counts(int32_t *left_count, int32_t *right_count)
{
    if (!s_initialized || left_count == NULL || right_count == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = accum_unit(s_left_unit,  &s_left_count,  &s_left_delta);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = accum_unit(s_right_unit, &s_right_count, &s_right_delta);
    if (ret != ESP_OK) {
        return ret;
    }

    *left_count  = s_left_count;
    *right_count = s_right_count;
    return ESP_OK;
}

esp_err_t encoder_pcnt_get_and_clear_deltas(int32_t *left_delta, int32_t *right_delta)
{
    if (!s_initialized || left_delta == NULL || right_delta == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    *left_delta  = s_left_delta;
    *right_delta = s_right_delta;
    s_left_delta  = 0;
    s_right_delta = 0;
    return ESP_OK;
}

esp_err_t encoder_pcnt_read_sample(encoder_pcnt_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = encoder_pcnt_get_counts(&sample->left_count, &sample->right_count);
    if (ret != ESP_OK) {
        return ret;
    }
    return encoder_pcnt_get_and_clear_deltas(&sample->left_delta, &sample->right_delta);
}
