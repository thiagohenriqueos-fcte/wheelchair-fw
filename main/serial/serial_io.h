#ifndef WHEELCHAIR_SERIAL_IO_H
#define WHEELCHAIR_SERIAL_IO_H

#include "esp_err.h"

esp_err_t serial_io_init(void);
esp_err_t serial_io_write_line(const char *line);

#endif
