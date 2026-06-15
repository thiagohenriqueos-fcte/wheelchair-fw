#ifndef WHEELCHAIR_JOYSTICK_ADC_H
#define WHEELCHAIR_JOYSTICK_ADC_H

#include "esp_err.h"

typedef struct {
    int raw_x;
    int raw_y;
    float x;
    float y;
} joystick_adc_sample_t;

esp_err_t joystick_adc_init(void);
esp_err_t joystick_adc_read_raw(int *raw_x, int *raw_y);
esp_err_t joystick_adc_read_normalized(float *x, float *y);
esp_err_t joystick_adc_read(joystick_adc_sample_t *sample);

#endif
