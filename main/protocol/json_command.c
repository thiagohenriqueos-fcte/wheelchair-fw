#include "json_command.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "serial_io.h"

#define COMMAND_LINE_SIZE 256
#define COMMAND_RX_TASK_STACK_SIZE 4096
#define COMMAND_RX_TASK_PRIORITY 5

static SemaphoreHandle_t state_mutex;
static motion_command_t command_state;
static uint32_t response_sequence;

static uint32_t current_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static bool read_uint32(const cJSON *item, uint32_t *value)
{
    if (!cJSON_IsNumber(item) || item->valuedouble < 0.0 ||
        item->valuedouble > UINT32_MAX ||
        floor(item->valuedouble) != item->valuedouble) {
        return false;
    }

    *value = (uint32_t)item->valuedouble;
    return true;
}

static bool read_float(const cJSON *item, float *value)
{
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) {
        return false;
    }

    const float converted = (float)item->valuedouble;
    if (!isfinite(converted)) {
        return false;
    }

    *value = converted;
    return true;
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

static esp_err_t send_ack(uint32_t command_sequence)
{
    cJSON *packet = cJSON_CreateObject();
    if (packet == NULL) {
        return ESP_ERR_NO_MEM;
    }

    response_sequence++;
    const bool complete =
        cJSON_AddStringToObject(packet, "type", "ack") != NULL &&
        cJSON_AddNumberToObject(packet, "seq", response_sequence) != NULL &&
        cJSON_AddNumberToObject(packet, "cmd_seq", command_sequence) != NULL &&
        cJSON_AddStringToObject(packet, "status", "ok") != NULL;

    if (!complete) {
        cJSON_Delete(packet);
        return ESP_ERR_NO_MEM;
    }

    return send_json(packet);
}

static esp_err_t send_error(const char *code)
{
    cJSON *packet = cJSON_CreateObject();
    if (packet == NULL) {
        return ESP_ERR_NO_MEM;
    }

    response_sequence++;
    const bool complete =
        cJSON_AddStringToObject(packet, "type", "err") != NULL &&
        cJSON_AddNumberToObject(packet, "seq", response_sequence) != NULL &&
        cJSON_AddStringToObject(packet, "code", code) != NULL &&
        cJSON_AddStringToObject(packet, "status", "error") != NULL;

    if (!complete) {
        cJSON_Delete(packet);
        return ESP_ERR_NO_MEM;
    }

    return send_json(packet);
}

static esp_err_t store_command(
    uint32_t host_sequence,
    float v_linear,
    float w_angular)
{
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    command_state.v_linear = v_linear;
    command_state.w_angular = w_angular;
    command_state.host_seq = host_sequence;
    command_state.last_update_ms = current_time_ms();
    command_state.valid = true;

    xSemaphoreGive(state_mutex);
    return ESP_OK;
}

static void process_command_line(const char *line)
{
    cJSON *packet = cJSON_Parse(line);
    if (packet == NULL || !cJSON_IsObject(packet)) {
        cJSON_Delete(packet);
        send_error("invalid_json");
        return;
    }

    const cJSON *type = cJSON_GetObjectItemCaseSensitive(packet, "type");
    if (!cJSON_IsString(type) || type->valuestring == NULL) {
        cJSON_Delete(packet);
        send_error("invalid_type");
        return;
    }

    uint32_t host_sequence;
    const cJSON *sequence = cJSON_GetObjectItemCaseSensitive(packet, "seq");
    if (!read_uint32(sequence, &host_sequence)) {
        cJSON_Delete(packet);
        send_error("invalid_seq");
        return;
    }

    float v_linear = 0.0f;
    float w_angular = 0.0f;

    if (strcmp(type->valuestring, "cmd") == 0) {
        const cJSON *v = cJSON_GetObjectItemCaseSensitive(packet, "v");
        const cJSON *w = cJSON_GetObjectItemCaseSensitive(packet, "w");
        if (!read_float(v, &v_linear) || !read_float(w, &w_angular)) {
            cJSON_Delete(packet);
            send_error("invalid_command");
            return;
        }
    } else if (strcmp(type->valuestring, "stop") != 0) {
        cJSON_Delete(packet);
        send_error("unknown_type");
        return;
    }

    cJSON_Delete(packet);
    if (store_command(host_sequence, v_linear, w_angular) != ESP_OK) {
        send_error("state_update_failed");
        return;
    }

    send_ack(host_sequence);
}

static void comm_rx_task(void *argument)
{
    (void)argument;

    char line[COMMAND_LINE_SIZE];

    while (true) {
        if (fgets(line, sizeof(line), stdin) == NULL) {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (strchr(line, '\n') == NULL) {
            int character;
            do {
                character = fgetc(stdin);
            } while (character != '\n' && character != EOF);
            send_error("line_too_long");
            continue;
        }

        process_command_line(line);
    }
}

esp_err_t json_command_init(void)
{
    if (state_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    state_mutex = xSemaphoreCreateMutex();
    if (state_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    command_state = (motion_command_t){0};
    response_sequence = 0;
    return ESP_OK;
}

esp_err_t json_command_start_receiver(void)
{
    if (state_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const BaseType_t result = xTaskCreate(
        comm_rx_task,
        "comm_rx_task",
        COMMAND_RX_TASK_STACK_SIZE,
        NULL,
        COMMAND_RX_TASK_PRIORITY,
        NULL);

    return result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t json_command_get_state(motion_command_t *state)
{
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (state_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    *state = command_state;
    xSemaphoreGive(state_mutex);
    return ESP_OK;
}
