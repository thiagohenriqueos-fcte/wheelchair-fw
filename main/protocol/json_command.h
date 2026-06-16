#ifndef WHEELCHAIR_JSON_COMMAND_H
#define WHEELCHAIR_JSON_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    float v_linear;
    float w_angular;
    uint32_t host_seq;
    uint32_t last_update_ms;
    bool valid;
} motion_command_t;

typedef struct {
    float left;
    float right;
    uint32_t host_seq;
    uint32_t last_update_ms;
    bool valid;
} motor_test_command_t;

esp_err_t json_command_init(void);
esp_err_t json_command_start_receiver(void);
esp_err_t json_command_get_state(motion_command_t *state);
esp_err_t json_command_get_motor_test(motor_test_command_t *state);

#endif
