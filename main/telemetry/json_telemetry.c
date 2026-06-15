#include "json_telemetry.h"

#include <stdbool.h>

#include "cJSON.h"
#include "esp_timer.h"
#include "serial_io.h"
#include "version.h"

static uint32_t current_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static esp_err_t send_json(cJSON *packet)
{
    if (packet == NULL) {
        return ESP_ERR_NO_MEM;
    }

    char *line = cJSON_PrintUnformatted(packet);
    cJSON_Delete(packet);
    if (line == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const esp_err_t err = serial_io_write_line(line);
    cJSON_free(line);
    return err;
}

esp_err_t json_telemetry_send_status(
    const char *event,
    const char *status,
    const char *detail)
{
    if (event == NULL || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *packet = cJSON_CreateObject();
    if (packet == NULL) {
        return ESP_ERR_NO_MEM;
    }

    bool complete =
        cJSON_AddStringToObject(packet, "type", "status") != NULL &&
        cJSON_AddNumberToObject(packet, "t_ms", current_time_ms()) != NULL &&
        cJSON_AddStringToObject(packet, "fw", FIRMWARE_VERSION) != NULL &&
        cJSON_AddStringToObject(packet, "target", HARDWARE_TARGET) != NULL &&
        cJSON_AddStringToObject(packet, "event", event) != NULL &&
        cJSON_AddStringToObject(packet, "status", status) != NULL;

    if (detail != NULL) {
        complete =
            complete &&
            cJSON_AddStringToObject(packet, "detail", detail) != NULL;
    }

    if (!complete) {
        cJSON_Delete(packet);
        return ESP_ERR_NO_MEM;
    }

    return send_json(packet);
}

esp_err_t json_telemetry_send_heartbeat(uint32_t counter)
{
    cJSON *packet = cJSON_CreateObject();
    if (packet == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const bool complete =
        cJSON_AddStringToObject(packet, "type", "heartbeat") != NULL &&
        cJSON_AddNumberToObject(packet, "seq", counter) != NULL &&
        cJSON_AddNumberToObject(packet, "t_ms", current_time_ms()) != NULL &&
        cJSON_AddStringToObject(packet, "fw", FIRMWARE_VERSION) != NULL &&
        cJSON_AddStringToObject(packet, "status", "ok") != NULL;

    if (!complete) {
        cJSON_Delete(packet);
        return ESP_ERR_NO_MEM;
    }

    return send_json(packet);
}

esp_err_t json_telemetry_send_joystick(
    uint32_t sequence,
    const joystick_adc_sample_t *sample,
    const motion_command_t *command,
    const motor_test_command_t *motor_test)
{
    if (sample == NULL || command == NULL || motor_test == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *packet = cJSON_CreateObject();
    if (packet == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const uint32_t now_ms = current_time_ms();
    bool packet_complete =
        cJSON_AddStringToObject(packet, "type", "joy") != NULL &&
        cJSON_AddNumberToObject(packet, "seq", sequence) != NULL &&
        cJSON_AddNumberToObject(packet, "t_ms", now_ms) != NULL &&
        cJSON_AddStringToObject(packet, "fw", FIRMWARE_VERSION) != NULL &&
        cJSON_AddNumberToObject(packet, "raw_x", sample->raw_x) != NULL &&
        cJSON_AddNumberToObject(packet, "raw_y", sample->raw_y) != NULL &&
        cJSON_AddNumberToObject(packet, "x", sample->x) != NULL &&
        cJSON_AddNumberToObject(packet, "y", sample->y) != NULL &&
        cJSON_AddNumberToObject(packet, "cmd_v", command->v_linear) != NULL &&
        cJSON_AddNumberToObject(packet, "cmd_w", command->w_angular) != NULL &&
        cJSON_AddNumberToObject(packet, "cmd_seq", command->host_seq) != NULL &&
        cJSON_AddBoolToObject(packet, "cmd_valid", command->valid) != NULL &&
        cJSON_AddNumberToObject(packet, "motor_left", motor_test->valid ? motor_test->left : 0.0f) != NULL &&
        cJSON_AddNumberToObject(packet, "motor_right", motor_test->valid ? motor_test->right : 0.0f) != NULL &&
        cJSON_AddBoolToObject(packet, "motor_test_active", motor_test->valid) != NULL &&
        cJSON_AddStringToObject(packet, "status", "ok") != NULL;

    if (command->valid) {
        packet_complete =
            packet_complete &&
            cJSON_AddNumberToObject(
                packet,
                "last_cmd_age_ms",
                now_ms - command->last_update_ms) != NULL;
    } else {
        packet_complete =
            packet_complete &&
            cJSON_AddNullToObject(packet, "last_cmd_age_ms") != NULL;
    }

    if (!packet_complete) {
        cJSON_Delete(packet);
        return ESP_ERR_NO_MEM;
    }

    return send_json(packet);
}
