#ifndef WHEELCHAIR_ENCODER_PCNT_H
#define WHEELCHAIR_ENCODER_PCNT_H

#include <stdint.h>

#include "esp_err.h"

typedef struct {
    int32_t left_count;
    int32_t right_count;
    int32_t left_delta;
    int32_t right_delta;
} encoder_pcnt_sample_t;

esp_err_t encoder_pcnt_init(void);
esp_err_t encoder_pcnt_get_counts(int32_t *left_count, int32_t *right_count);
esp_err_t encoder_pcnt_get_and_clear_deltas(int32_t *left_delta, int32_t *right_delta);
esp_err_t encoder_pcnt_read_sample(encoder_pcnt_sample_t *sample);

#endif
