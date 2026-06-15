#ifndef WHEELCHAIR_JSON_TELEMETRY_H
#define WHEELCHAIR_JSON_TELEMETRY_H

#include <stdint.h>

#include "esp_err.h"
#include "joystick_adc.h"

esp_err_t json_telemetry_send_joystick(
    uint32_t sequence,
    const joystick_adc_sample_t *sample);

#endif
