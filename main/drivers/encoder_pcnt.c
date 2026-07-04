#include "encoder_pcnt.h"

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_log.h"

/*
 * Quadrature encoder reader built on the new PCNT driver (driver/pulse_cnt.h).
 * One pcnt_unit per encoder, two channels for 4x decoding, a glitch filter, and
 * a ±limit watch point whose ISR folds every wrap into a 64-bit accumulator.
 * The 16-bit hardware counter therefore never loses counts over long runs.
 */

/* ── Per-encoder pin map (A, B). Adapt here to change wiring / quantity. ─────── */
typedef struct {
    int gpio_a;
    int gpio_b;
} encoder_pins_t;

static const encoder_pins_t s_pins[ENCODER_COUNT] = {
    { 32, 33 },   /* enc0 — left  wheel:  A=GPIO32 B=GPIO33 */
    { 25, 26 },   /* enc1 — right wheel:  A=GPIO25 B=GPIO26 */
};

/* PCNT hardware counter is 16-bit; wrap well before it saturates. */
#define ENCODER_HIGH_LIMIT    30000
#define ENCODER_LOW_LIMIT   (-30000)
#define ENCODER_GLITCH_NS     1000

static const char *TAG = "encoder_pcnt";

static pcnt_unit_handle_t s_units[ENCODER_COUNT];
/* 64-bit overflow accumulator per encoder, updated only from the watch ISR. */
static volatile int64_t   s_accum[ENCODER_COUNT];
static bool               s_initialized;

/* ISR context: fold each ±limit crossing into the 64-bit accumulator. */
static bool IRAM_ATTR on_reach(pcnt_unit_handle_t unit,
                               const pcnt_watch_event_data_t *edata,
                               void *user_ctx)
{
    (void)unit;
    volatile int64_t *accum = (volatile int64_t *)user_ctx;
    *accum += edata->watch_point_value;
    return false;  /* no higher-priority task to wake */
}

static esp_err_t unit_init(int index)
{
    const encoder_pins_t pins = s_pins[index];
    esp_err_t ret;

    const pcnt_unit_config_t unit_cfg = {
        .high_limit = ENCODER_HIGH_LIMIT,
        .low_limit  = ENCODER_LOW_LIMIT,
    };
    pcnt_unit_handle_t unit = NULL;
    ret = pcnt_new_unit(&unit_cfg, &unit);
    if (ret != ESP_OK) return ret;

    const pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = ENCODER_GLITCH_NS,
    };
    ret = pcnt_unit_set_glitch_filter(unit, &filter_cfg);
    if (ret != ESP_OK) return ret;

    /*
     * 4x quadrature (mirrors the official ESP-IDF rotary_encoder example):
     *   chanA: edge on A gated by level of B
     *   chanB: edge on B gated by level of A
     */
    const pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num  = pins.gpio_a,
        .level_gpio_num = pins.gpio_b,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ret = pcnt_new_channel(unit, &chan_a_cfg, &chan_a);
    if (ret != ESP_OK) return ret;

    const pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num  = pins.gpio_b,
        .level_gpio_num = pins.gpio_a,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ret = pcnt_new_channel(unit, &chan_b_cfg, &chan_b);
    if (ret != ESP_OK) return ret;

    ret = pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    if (ret != ESP_OK) return ret;
    ret = pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (ret != ESP_OK) return ret;
    ret = pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    if (ret != ESP_OK) return ret;
    ret = pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    if (ret != ESP_OK) return ret;

    ret = pcnt_unit_add_watch_point(unit, ENCODER_HIGH_LIMIT);
    if (ret != ESP_OK) return ret;
    ret = pcnt_unit_add_watch_point(unit, ENCODER_LOW_LIMIT);
    if (ret != ESP_OK) return ret;

    const pcnt_event_callbacks_t cbs = { .on_reach = on_reach };
    ret = pcnt_unit_register_event_callbacks(unit, &cbs,
                                             (void *)&s_accum[index]);
    if (ret != ESP_OK) return ret;

    /* Internal pull-ups so open-collector encoder outputs idle high. */
    gpio_pullup_en(pins.gpio_a);
    gpio_pullup_en(pins.gpio_b);

    ret = pcnt_unit_enable(unit);
    if (ret != ESP_OK) return ret;
    ret = pcnt_unit_clear_count(unit);
    if (ret != ESP_OK) return ret;
    ret = pcnt_unit_start(unit);
    if (ret != ESP_OK) return ret;

    s_units[index] = unit;
    ESP_LOGI(TAG, "encoder %d ready: A=GPIO%d B=GPIO%d",
             index, pins.gpio_a, pins.gpio_b);
    return ESP_OK;
}

esp_err_t encoder_pcnt_init(void)
{
    for (int i = 0; i < ENCODER_COUNT; i++) {
        s_accum[i] = 0;
        const esp_err_t ret = unit_init(i);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "encoder %d init failed: %s",
                     i, esp_err_to_name(ret));
            return ret;
        }
    }
    s_initialized = true;
    return ESP_OK;
}

esp_err_t encoder_pcnt_read(int64_t counts[ENCODER_COUNT])
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;
    if (counts == NULL)  return ESP_ERR_INVALID_ARG;

    for (int i = 0; i < ENCODER_COUNT; i++) {
        int raw = 0;
        const esp_err_t ret = pcnt_unit_get_count(s_units[i], &raw);
        if (ret != ESP_OK) return ret;
        /* Total = accumulated wraps + live hardware count. */
        counts[i] = s_accum[i] + raw;
    }
    return ESP_OK;
}
