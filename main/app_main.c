#include <stdbool.h>
#include <stdint.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "encoder_pcnt.h"
#include "json_command.h"
#include "json_telemetry.h"
#include "joystick_adc.h"
#include "motor_pwm.h"
#include "serial_io.h"
#include "version.h"

#define JOYSTICK_SAMPLE_PERIOD_MS  50
#define HEARTBEAT_PERIOD_MS        1000

/* Motor test watchdog: stop all PWM if no fresh command arrives within this window. */
#define MOTOR_TEST_TIMEOUT_MS  500

/*
 * Firmware absolute duty-cycle cap.  Set to 1.0 so the GUI is the
 * operator-facing safety limit (default GUI limit: 0.30).  The 500 ms
 * watchdog and STOP command remain active regardless of this constant.
 */
#define MOTOR_TEST_MAX_DUTY    1.0f

void app_main(void)
{
    uint32_t heartbeat = 0;
    uint32_t telemetry_sequence = 0;

    if (serial_io_init() != ESP_OK) {
        return;
    }

    const esp_err_t joystick_init_result = joystick_adc_init();
    const bool joystick_ready = joystick_init_result == ESP_OK;

    const esp_err_t motor_init_result = motor_pwm_init();
    const bool motor_ready = motor_init_result == ESP_OK;

    const esp_err_t command_init_result = json_command_init();
    const bool command_ready = command_init_result == ESP_OK;
    bool receiver_ready = false;
    if (command_ready) {
        receiver_ready = json_command_start_receiver() == ESP_OK;
    }

    const esp_err_t encoder_init_result = encoder_pcnt_init();
    const bool encoder_ready = encoder_init_result == ESP_OK;

    json_telemetry_send_status("boot", "ok", FIRMWARE_NAME);
    json_telemetry_send_status(
        "joystick_adc",
        joystick_ready ? "ok" : "error",
        joystick_ready ? NULL : esp_err_to_name(joystick_init_result));
    json_telemetry_send_status(
        "motor_pwm",
        motor_ready ? "ok" : "error",
        motor_ready ? NULL : esp_err_to_name(motor_init_result));
    json_telemetry_send_status(
        "command_receiver",
        receiver_ready ? "ok" : "error",
        receiver_ready ? NULL : "initialization_failed");
    json_telemetry_send_status(
        "encoder_pcnt",
        encoder_ready ? "ok" : "error",
        encoder_ready ? NULL : esp_err_to_name(encoder_init_result));

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
                motor_test_command_t motor_test = {0};
                if (command_ready) {
                    json_command_get_state(&command);
                    json_command_get_motor_test(&motor_test);
                }

                if (motor_ready) {
                    bool drive = false;
                    if (motor_test.valid) {
                        const uint32_t now_ms =
                            (uint32_t)(esp_timer_get_time() / 1000);
                        const uint32_t age_ms =
                            now_ms - motor_test.last_update_ms;
                        if (age_ms < MOTOR_TEST_TIMEOUT_MS) {
                            float l = motor_test.left;
                            float r = motor_test.right;
                            if (l >  MOTOR_TEST_MAX_DUTY) l =  MOTOR_TEST_MAX_DUTY;
                            if (l < -MOTOR_TEST_MAX_DUTY) l = -MOTOR_TEST_MAX_DUTY;
                            if (r >  MOTOR_TEST_MAX_DUTY) r =  MOTOR_TEST_MAX_DUTY;
                            if (r < -MOTOR_TEST_MAX_DUTY) r = -MOTOR_TEST_MAX_DUTY;
                            motor_pwm_set_left(l);
                            motor_pwm_set_right(r);
                            drive = true;
                        } else {
                            /* Watchdog: stale command — mark inactive for telemetry. */
                            motor_test.valid = false;
                        }
                    }
                    if (!drive) {
                        motor_pwm_stop_all();
                    }
                }

                encoder_pcnt_sample_t enc_sample = {0};
                if (encoder_ready) {
                    encoder_pcnt_read_sample(&enc_sample);
                }

                telemetry_sequence++;
                json_telemetry_send_joystick(
                    telemetry_sequence,
                    &sample,
                    &command,
                    &motor_test,
                    encoder_ready ? &enc_sample : NULL,
                    encoder_ready);
            }
        }
    }
}
