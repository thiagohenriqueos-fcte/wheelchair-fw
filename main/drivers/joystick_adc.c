#include "joystick_adc.h"

#include <stdbool.h>
#include <stddef.h>

#include "esp_adc/adc_oneshot.h"

#define JOYSTICK_ADC_UNIT ADC_UNIT_1
#define JOYSTICK_X_CHANNEL ADC_CHANNEL_6
#define JOYSTICK_Y_CHANNEL ADC_CHANNEL_7
#define JOYSTICK_ADC_BITWIDTH ADC_BITWIDTH_DEFAULT
#define JOYSTICK_ADC_ATTEN ADC_ATTEN_DB_12

#define JOYSTICK_RAW_MIN 0
#define JOYSTICK_RAW_CENTER 2048
#define JOYSTICK_RAW_MAX 4095
#define JOYSTICK_DEADZONE 0.08f

#define JOYSTICK_INVERT_X 0
#define JOYSTICK_INVERT_Y 1

static adc_oneshot_unit_handle_t adc_handle;

static float clamp_axis(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }

    if (value < -1.0f) {
        return -1.0f;
    }

    return value;
}

static float normalize_axis(int raw, bool invert)
{
    float normalized;

    if (raw >= JOYSTICK_RAW_CENTER) {
        normalized = (float)(raw - JOYSTICK_RAW_CENTER) /
                     (float)(JOYSTICK_RAW_MAX - JOYSTICK_RAW_CENTER);
    } else {
        normalized = (float)(raw - JOYSTICK_RAW_CENTER) /
                     (float)(JOYSTICK_RAW_CENTER - JOYSTICK_RAW_MIN);
    }

    normalized = clamp_axis(normalized);
    if (normalized > -JOYSTICK_DEADZONE &&
        normalized < JOYSTICK_DEADZONE) {
        return 0.0f;
    }

    if (invert) {
        normalized = -normalized;
    }

    return normalized;
}

esp_err_t joystick_adc_init(void)
{
    if (adc_handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = JOYSTICK_ADC_UNIT,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_config, &adc_handle);
    if (err != ESP_OK) {
        return err;
    }

    const adc_oneshot_chan_cfg_t channel_config = {
        .atten = JOYSTICK_ADC_ATTEN,
        .bitwidth = JOYSTICK_ADC_BITWIDTH,
    };

    err = adc_oneshot_config_channel(adc_handle, JOYSTICK_X_CHANNEL,
                                     &channel_config);
    if (err != ESP_OK) {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        return err;
    }

    err = adc_oneshot_config_channel(adc_handle, JOYSTICK_Y_CHANNEL,
                                     &channel_config);
    if (err != ESP_OK) {
        adc_oneshot_del_unit(adc_handle);
        adc_handle = NULL;
        return err;
    }

    return ESP_OK;
}

esp_err_t joystick_adc_read_raw(int *raw_x, int *raw_y)
{
    if (adc_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (raw_x == NULL || raw_y == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = adc_oneshot_read(adc_handle, JOYSTICK_X_CHANNEL, raw_x);
    if (err != ESP_OK) {
        return err;
    }

    return adc_oneshot_read(adc_handle, JOYSTICK_Y_CHANNEL, raw_y);
}

esp_err_t joystick_adc_read_normalized(float *x, float *y)
{
    if (x == NULL || y == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int raw_x;
    int raw_y;
    esp_err_t err = joystick_adc_read_raw(&raw_x, &raw_y);
    if (err != ESP_OK) {
        return err;
    }

    *x = normalize_axis(raw_x, JOYSTICK_INVERT_X);
    *y = normalize_axis(raw_y, JOYSTICK_INVERT_Y);

    return ESP_OK;
}

esp_err_t joystick_adc_read(joystick_adc_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = joystick_adc_read_raw(&sample->raw_x, &sample->raw_y);
    if (err != ESP_OK) {
        return err;
    }

    sample->x = normalize_axis(sample->raw_x, JOYSTICK_INVERT_X);
    sample->y = normalize_axis(sample->raw_y, JOYSTICK_INVERT_Y);

    return ESP_OK;
}
