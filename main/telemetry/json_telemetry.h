#ifndef WHEELCHAIR_JSON_TELEMETRY_H
#define WHEELCHAIR_JSON_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "encoder_pcnt.h"
#include "json_command.h"
#include "joystick_adc.h"

esp_err_t json_telemetry_send_status(
    const char *event,
    const char *status,
    const char *detail);
esp_err_t json_telemetry_send_heartbeat(uint32_t counter);
esp_err_t json_telemetry_send_joystick(
    uint32_t sequence,
    const joystick_adc_sample_t *sample,
    const motion_command_t *command,
    const motor_test_command_t *motor_test,
    const encoder_pcnt_sample_t *encoder,
    bool encoder_ok);

#endif
