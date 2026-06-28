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

/* Hard ceilings the firmware enforces regardless of what the host requests. */
#define DRIVE_MAX_DUTY_LIMIT 1.0f
#define DRIVE_RATE_LIMIT     50.0f   /* ramp rate upper bound [duty / second] */

static SemaphoreHandle_t state_mutex;
static drive_config_t drive_config_state;
static drive_command_t drive_command_state;
static uint32_t response_sequence;

static uint32_t current_time_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static float clampf(float value, float lo, float hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
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

static esp_err_t store_drive_config(
    uint32_t host_sequence,
    float accel,
    float decel,
    float max_duty,
    bool armed)
{
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    drive_config_state.accel          = clampf(accel,    0.0f, DRIVE_RATE_LIMIT);
    drive_config_state.decel          = clampf(decel,    0.0f, DRIVE_RATE_LIMIT);
    drive_config_state.max_duty       = clampf(max_duty, 0.0f, DRIVE_MAX_DUTY_LIMIT);
    drive_config_state.armed          = armed;
    drive_config_state.host_seq       = host_sequence;
    drive_config_state.last_update_ms = current_time_ms();
    drive_config_state.valid          = true;

    xSemaphoreGive(state_mutex);
    return ESP_OK;
}

static esp_err_t store_drive_command(
    uint32_t host_sequence,
    float left,
    float right)
{
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    drive_command_state.left           = clampf(left,  -1.0f, 1.0f);
    drive_command_state.right          = clampf(right, -1.0f, 1.0f);
    drive_command_state.host_seq       = host_sequence;
    drive_command_state.last_update_ms = current_time_ms();
    drive_command_state.valid          = true;

    xSemaphoreGive(state_mutex);
    return ESP_OK;
}

/* `stop` keeps the tuning but forces the safety gate closed immediately. */
static esp_err_t store_disarm(uint32_t host_sequence)
{
    if (xSemaphoreTake(state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    drive_config_state.armed          = false;
    drive_config_state.host_seq       = host_sequence;
    drive_config_state.last_update_ms = current_time_ms();
    drive_config_state.valid          = true;
    drive_command_state               = (drive_command_t){0};

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

    if (strcmp(type->valuestring, "drive_cfg") == 0) {
        float accel = 0.0f;
        float decel = 0.0f;
        float max_duty = 0.0f;
        const cJSON *a = cJSON_GetObjectItemCaseSensitive(packet, "accel");
        const cJSON *d = cJSON_GetObjectItemCaseSensitive(packet, "decel");
        const cJSON *m = cJSON_GetObjectItemCaseSensitive(packet, "max_duty");
        const cJSON *armed_item =
            cJSON_GetObjectItemCaseSensitive(packet, "armed");
        if (!read_float(a, &accel) || !read_float(d, &decel) ||
            !read_float(m, &max_duty) || !cJSON_IsBool(armed_item)) {
            cJSON_Delete(packet);
            send_error("invalid_command");
            return;
        }
        const bool armed = cJSON_IsTrue(armed_item);
        cJSON_Delete(packet);
        if (store_drive_config(host_sequence, accel, decel, max_duty, armed)
                != ESP_OK) {
            send_error("state_update_failed");
            return;
        }
        send_ack(host_sequence);
        return;
    }

    if (strcmp(type->valuestring, "drive_cmd") == 0) {
        float left = 0.0f;
        float right = 0.0f;
        const cJSON *l = cJSON_GetObjectItemCaseSensitive(packet, "left");
        const cJSON *r = cJSON_GetObjectItemCaseSensitive(packet, "right");
        if (!read_float(l, &left) || !read_float(r, &right)) {
            cJSON_Delete(packet);
            send_error("invalid_command");
            return;
        }
        cJSON_Delete(packet);
        if (store_drive_command(host_sequence, left, right) != ESP_OK) {
            send_error("state_update_failed");
            return;
        }
        send_ack(host_sequence);
        return;
    }

    if (strcmp(type->valuestring, "stop") == 0) {
        cJSON_Delete(packet);
        if (store_disarm(host_sequence) != ESP_OK) {
            send_error("state_update_failed");
            return;
        }
        send_ack(host_sequence);
        return;
    }

    cJSON_Delete(packet);
    send_error("unknown_type");
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

    drive_config_state = (drive_config_t){0};
    drive_command_state = (drive_command_t){0};
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

esp_err_t json_command_get_drive_config(drive_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (state_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    *config = drive_config_state;
    xSemaphoreGive(state_mutex);
    return ESP_OK;
}

esp_err_t json_command_get_drive_command(drive_command_t *command)
{
    if (command == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (state_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(state_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    *command = drive_command_state;
    xSemaphoreGive(state_mutex);
    return ESP_OK;
}
