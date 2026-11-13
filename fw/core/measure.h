#pragma once
#include <stdint.h>

typedef struct { int valid; float freq_hz, duty_pct; uint8_t vmin, vmax; } meas_t;

/* Frequency and duty from rising crossings of the mid level (with hysteresis). */
void measure(const uint8_t *buf, int n, uint32_t rate_hz, meas_t *m);
