#include "json_telemetry.h"

#include <stdbool.h>
#include <stdio.h>

#include "cJSON.h"
#include "version.h"

esp_err_t json_telemetry_send_joystick(
    uint32_t sequence,
    const joystick_adc_sample_t *sample)
{
    if (sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *packet = cJSON_CreateObject();
    if (packet == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const bool packet_complete =
        cJSON_AddStringToObject(packet, "type", "joystick") != NULL &&
        cJSON_AddStringToObject(packet, "version", FIRMWARE_VERSION) != NULL &&
        cJSON_AddNumberToObject(packet, "seq", sequence) != NULL &&
        cJSON_AddNumberToObject(packet, "raw_x", sample->raw_x) != NULL &&
        cJSON_AddNumberToObject(packet, "raw_y", sample->raw_y) != NULL &&
        cJSON_AddNumberToObject(packet, "x", sample->x) != NULL &&
        cJSON_AddNumberToObject(packet, "y", sample->y) != NULL;

    if (!packet_complete) {
        cJSON_Delete(packet);
        return ESP_ERR_NO_MEM;
    }

    char *json_line = cJSON_PrintUnformatted(packet);
    cJSON_Delete(packet);
    if (json_line == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const int print_result = printf("%s\n", json_line);
    cJSON_free(json_line);
    if (print_result < 0 || fflush(stdout) != 0) {
        return ESP_FAIL;
    }

    return ESP_OK;
}
