#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "joystick_adc.h"
#include "version.h"

#define JOYSTICK_SAMPLE_PERIOD_MS 50
#define HEARTBEAT_PERIOD_MS 1000

static const char *TAG = "wheelchair_fw";

void app_main(void)
{
    uint32_t heartbeat = 0;

    ESP_LOGI(TAG, "%s", FIRMWARE_NAME);
    ESP_LOGI(TAG, "Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "Target: %s", HARDWARE_TARGET);
    ESP_LOGI(TAG, "Status: boot_ok");

    const esp_err_t joystick_init_result = joystick_adc_init();
    const bool joystick_ready = joystick_init_result == ESP_OK;
    if (joystick_ready) {
        ESP_LOGI(TAG, "Joystick ADC: ready");
    } else {
        ESP_LOGE(TAG, "Joystick ADC init failed: %s",
                 esp_err_to_name(joystick_init_result));
    }

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_heartbeat_time = last_wake_time;
    const TickType_t sample_period =
        pdMS_TO_TICKS(JOYSTICK_SAMPLE_PERIOD_MS);
    const TickType_t heartbeat_period = pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS);

    while (1) {
        vTaskDelayUntil(&last_wake_time, sample_period);

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_heartbeat_time) >= heartbeat_period) {
            heartbeat++;
            ESP_LOGI(TAG, "heartbeat=%" PRIu32, heartbeat);
            last_heartbeat_time += heartbeat_period;
        }

        if (joystick_ready) {
            joystick_adc_sample_t sample;
            const esp_err_t read_result = joystick_adc_read(&sample);

            if (read_result == ESP_OK) {
                ESP_LOGI(TAG,
                         "joy raw_x=%d raw_y=%d x=%.2f y=%.2f",
                         sample.raw_x, sample.raw_y, sample.x, sample.y);
            } else {
                ESP_LOGE(TAG, "Joystick ADC read failed: %s",
                         esp_err_to_name(read_result));
            }
        }
    }
}
