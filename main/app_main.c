#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "json_command.h"
#include "json_telemetry.h"
#include "joystick_adc.h"
#include "motor_pwm.h"
#include "serial_io.h"
#include "version.h"

#define CONTROL_PERIOD_MS    50            /* differential-drive loop @ 20 Hz   */
#define HEARTBEAT_PERIOD_MS  1000
#define CONTROL_DT           (CONTROL_PERIOD_MS / 1000.0f)

/*
 * Dead-man watchdog: the host must keep re-sending `drive_cfg` (armed=true).
 * If the freshest config is older than this, the chair is treated as disarmed
 * and the motors are stopped — covers GUI crash / USB unplug.
 */
#define DRIVE_CFG_TIMEOUT_MS 400

/*
 * Host-side semi-assist watchdog.  Once a drive_cmd has been received, stale
 * assisted commands stop the chair instead of silently falling back to manual.
 */
#define DRIVE_CMD_TIMEOUT_MS 200

/* Joystick radial dead-zone (normalised units) to reject centre noise. */
#define JOYSTICK_DEADZONE    0.08f

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

/*
 * Slew-rate limit `current` toward `target`.  Uses the accel step when the
 * output magnitude is growing and the decel step when it is shrinking (which
 * also covers crossing through zero).  A non-positive step means "no limit".
 */
static float ramp_toward(float current, float target,
                         float accel_step, float decel_step)
{
    if (target == current) {
        return current;
    }
    const float step = (fabsf(target) > fabsf(current)) ? accel_step : decel_step;
    if (step <= 0.0f) {
        return target;
    }
    if (target > current) {
        const float next = current + step;
        return next > target ? target : next;
    }
    const float next = current - step;
    return next < target ? target : next;
}

void app_main(void)
{
    uint32_t heartbeat = 0;
    uint32_t telemetry_sequence = 0;

    float cur_left = 0.0f;
    float cur_right = 0.0f;

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

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_heartbeat_time = last_wake_time;
    const TickType_t control_period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
    const TickType_t heartbeat_period = pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS);

    while (1) {
        vTaskDelayUntil(&last_wake_time, control_period);

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_heartbeat_time) >= heartbeat_period) {
            heartbeat++;
            json_telemetry_send_heartbeat(heartbeat);
            last_heartbeat_time += heartbeat_period;
        }

        if (!joystick_ready) {
            continue;
        }

        joystick_adc_sample_t sample;
        if (joystick_adc_read(&sample) != ESP_OK) {
            continue;
        }

        drive_config_t config = {0};
        drive_command_t assist = {0};
        if (command_ready) {
            json_command_get_drive_config(&config);
            json_command_get_drive_command(&assist);
        }

        /* Armed only if the operator gate is set AND the config is fresh. */
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        const bool driving =
            config.valid && config.armed &&
            (now_ms - config.last_update_ms) < DRIVE_CFG_TIMEOUT_MS;
        const bool assist_active =
            assist.valid && (now_ms - assist.last_update_ms) < DRIVE_CMD_TIMEOUT_MS;

        if (driving) {
            float left = 0.0f;
            float right = 0.0f;

            if (assist_active) {
                left = assist.left;
                right = assist.right;
            } else if (!assist.valid) {
                float x = sample.x;
                float y = sample.y;
                if ((x * x + y * y) < (JOYSTICK_DEADZONE * JOYSTICK_DEADZONE)) {
                    x = 0.0f;
                    y = 0.0f;
                }

                /* Differential mix: forward = y, turn = x. */
                left = y + x;
                right = y - x;
                const float mag = fmaxf(fmaxf(fabsf(left), fabsf(right)), 1.0f);
                left /= mag;
                right /= mag;
            }

            const float target_left = left * config.max_duty;
            const float target_right = right * config.max_duty;

            const float accel_step = config.accel * CONTROL_DT;
            const float decel_step = config.decel * CONTROL_DT;
            cur_left = ramp_toward(cur_left, target_left, accel_step, decel_step);
            cur_right = ramp_toward(cur_right, target_right, accel_step, decel_step);

            cur_left = clampf(cur_left, -config.max_duty, config.max_duty);
            cur_right = clampf(cur_right, -config.max_duty, config.max_duty);

            if (motor_ready) {
                motor_pwm_set_left(cur_left);
                motor_pwm_set_right(cur_right);
            }
        } else {
            /* Disarmed / stale: immediate stop, ramp state reset. */
            cur_left = 0.0f;
            cur_right = 0.0f;
            if (motor_ready) {
                motor_pwm_stop_all();
            }
        }

        telemetry_sequence++;
        json_telemetry_send_drive(
            telemetry_sequence, &sample, &config, &assist, assist_active,
            driving, cur_left, cur_right);
    }
}
