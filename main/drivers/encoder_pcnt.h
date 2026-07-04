#ifndef WHEELCHAIR_ENCODER_PCNT_H
#define WHEELCHAIR_ENCODER_PCNT_H

#include <stdint.h>

#include "esp_err.h"

/* Number of quadrature encoders wired to the board. */
#define ENCODER_COUNT 2

/*
 * Counts per mechanical revolution after 4x quadrature decoding.
 * CPR = encoder PPR * 4  (e.g. a 2000 PPR encoder -> 8000 counts/rev).
 * Adjust to match the physical encoders so the host can convert
 * counts -> revolutions / angle / RPM.
 */
#define ENCODER_CPR 8000

/* Initialise every encoder unit (PCNT + quadrature channels + overflow watch). */
esp_err_t encoder_pcnt_init(void);

/*
 * Read the total signed count of each encoder (64-bit, overflow-accumulated).
 * `counts` must have room for ENCODER_COUNT entries.
 */
esp_err_t encoder_pcnt_read(int64_t counts[ENCODER_COUNT]);

#endif
