#include "serial_io.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define SERIAL_IO_BUFFER_SIZE 1024
#define SERIAL_IO_MAX_LINE_SIZE 512

static SemaphoreHandle_t output_mutex;

esp_err_t serial_io_init(void)
{
    if (output_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    output_mutex = xSemaphoreCreateMutex();
    if (output_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    usb_serial_jtag_driver_config_t driver_config = {
        .tx_buffer_size = SERIAL_IO_BUFFER_SIZE,
        .rx_buffer_size = SERIAL_IO_BUFFER_SIZE,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&driver_config);
    if (err != ESP_OK) {
        vSemaphoreDelete(output_mutex);
        output_mutex = NULL;
        return err;
    }

    usb_serial_jtag_vfs_use_driver();
    usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_LF);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);
    setvbuf(stdin, NULL, _IONBF, 0);
    fcntl(fileno(stdin), F_SETFL, 0);

    return ESP_OK;
}

esp_err_t serial_io_write_line(const char *line)
{
    if (line == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (output_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const size_t line_length = strlen(line);
    if (line_length + 1 > SERIAL_IO_MAX_LINE_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    char output[SERIAL_IO_MAX_LINE_SIZE];
    memcpy(output, line, line_length);
    output[line_length] = '\n';
    const size_t output_length = line_length + 1;

    if (xSemaphoreTake(output_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_FAIL;
    }

    const int written =
        usb_serial_jtag_write_bytes(output, output_length, 0);
    xSemaphoreGive(output_mutex);

    return written == (int)output_length ? ESP_OK : ESP_ERR_TIMEOUT;
}
