#include "serial_io.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define SERIAL_IO_BUFFER_SIZE   1024
#define SERIAL_IO_MAX_LINE_SIZE 512

#ifdef CONFIG_IDF_TARGET_ESP32S3
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#else
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#define UART_PORT_NUM  UART_NUM_0
#define UART_BAUD_RATE 115200
#define UART_TX_PIN    GPIO_NUM_1
#define UART_RX_PIN    GPIO_NUM_3
#endif

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

#ifdef CONFIG_IDF_TARGET_ESP32S3
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
#else
    const uart_config_t uart_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(UART_PORT_NUM, SERIAL_IO_BUFFER_SIZE,
                                        SERIAL_IO_BUFFER_SIZE, 0, NULL, 0);
    if (err != ESP_OK) {
        vSemaphoreDelete(output_mutex);
        output_mutex = NULL;
        return err;
    }

    err = uart_param_config(UART_PORT_NUM, &uart_config);
    if (err != ESP_OK) {
        uart_driver_delete(UART_PORT_NUM);
        vSemaphoreDelete(output_mutex);
        output_mutex = NULL;
        return err;
    }

    err = uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        uart_driver_delete(UART_PORT_NUM);
        vSemaphoreDelete(output_mutex);
        output_mutex = NULL;
        return err;
    }

    uart_vfs_dev_use_driver(UART_PORT_NUM);
    uart_vfs_dev_port_set_rx_line_endings(UART_PORT_NUM, ESP_LINE_ENDINGS_LF);
    uart_vfs_dev_port_set_tx_line_endings(UART_PORT_NUM, ESP_LINE_ENDINGS_LF);
    setvbuf(stdin, NULL, _IONBF, 0);
    fcntl(fileno(stdin), F_SETFL, 0);
#endif

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

#ifdef CONFIG_IDF_TARGET_ESP32S3
    const int written = usb_serial_jtag_write_bytes(output, output_length, 0);
#else
    const int written = uart_write_bytes(UART_PORT_NUM, output, output_length);
#endif
    xSemaphoreGive(output_mutex);

    return written == (int)output_length ? ESP_OK : ESP_ERR_TIMEOUT;
}
