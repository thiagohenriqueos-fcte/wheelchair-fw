#ifndef WHEELCHAIR_JSON_COMMAND_H
#define WHEELCHAIR_JSON_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/*
 * Differential-drive configuration pushed by the host (GUI) via a `drive_cfg`
 * command.  `armed` is the operator safety gate; the host must keep re-sending
 * this command so `last_update_ms` stays fresh — the firmware treats a stale
 * config as disarmed (dead-man watchdog).
 */
typedef struct {
    float accel;            /* duty-cycle ramp-up rate   [duty / second] */
    float decel;            /* duty-cycle ramp-down rate [duty / second] */
    float max_duty;         /* output magnitude clamp    [0, 1]          */
    bool armed;             /* operator safety gate                      */
    uint32_t host_seq;
    uint32_t last_update_ms;
    bool valid;
} drive_config_t;

/*
 * Optional host-side assisted drive command.  `left` and `right` are
 * normalized wheel requests in [-1, 1]; the firmware still applies max_duty,
 * ramping, and freshness checks from drive_config_t.
 */
typedef struct {
    float left;
    float right;
    uint32_t host_seq;
    uint32_t last_update_ms;
    bool valid;
} drive_command_t;

esp_err_t json_command_init(void);
esp_err_t json_command_start_receiver(void);
esp_err_t json_command_get_drive_config(drive_config_t *config);
esp_err_t json_command_get_drive_command(drive_command_t *command);

#endif
