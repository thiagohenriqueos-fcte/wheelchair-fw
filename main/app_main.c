#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "json_command.h"
#include "json_telemetry.h"
#include "joystick_adc.h"
#include "serial_io.h"
#include "version.h"

#define JOYSTICK_SAMPLE_PERIOD_MS 50
#define HEARTBEAT_PERIOD_MS 1000

void app_main(void)
{
    uint32_t heartbeat = 0;
    uint32_t telemetry_sequence = 0;

    if (serial_io_init() != ESP_OK) {
        return;
    }

    const esp_err_t joystick_init_result = joystick_adc_init();
    const bool joystick_ready = joystick_init_result == ESP_OK;

    const esp_err_t command_init_result = json_command_init();
    const bool command_ready = command_init_result == ESP_OK;
    bool receiver_ready = false;
    if (command_ready) {
        receiver_ready = json_command_start_receiver() == ESP_OK;
    }

    json_telemetry_send_status("boot", "ok", FIRMWARE_NAME);
    json_telemetry_send_status(
        "joystick_adc",
        joystick_ready ? "ok" : "error",
        joystick_ready ? NULL : esp_err_to_name(joystick_init_result));
    json_telemetry_send_status(
        "command_receiver",
        receiver_ready ? "ok" : "error",
        receiver_ready ? NULL : "initialization_failed");

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
            json_telemetry_send_heartbeat(heartbeat);
            last_heartbeat_time += heartbeat_period;
        }

        if (joystick_ready) {
            joystick_adc_sample_t sample;
            const esp_err_t read_result = joystick_adc_read(&sample);

            if (read_result == ESP_OK) {
                motion_command_t command = {0};
                if (command_ready) {
                    json_command_get_state(&command);
                }

                telemetry_sequence++;
                json_telemetry_send_joystick(
                    telemetry_sequence,
                    &sample,
                    &command);
            }
        }
    }
}
