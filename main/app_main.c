#include <inttypes.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "version.h"

static const char *TAG = "wheelchair_fw";

void app_main(void)
{
    uint32_t heartbeat = 0;

    ESP_LOGI(TAG, "%s", FIRMWARE_NAME);
    ESP_LOGI(TAG, "Version: %s", FIRMWARE_VERSION);
    ESP_LOGI(TAG, "Target: %s", HARDWARE_TARGET);
    ESP_LOGI(TAG, "Status: boot_ok");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        heartbeat++;
        ESP_LOGI(TAG, "heartbeat=%" PRIu32, heartbeat);
    }
}
