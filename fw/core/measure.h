#pragma once
#include <stdint.h>

typedef struct { int valid; float freq_hz, duty_pct, vpp_codes; uint8_t vmin, vmax; } meas_t;

#define MEAS_AVG_N 8
/* Running average of successive measurements (displayed values). */
typedef struct { int n; float freq_hz, duty_pct, vpp_codes; } meas_avg_t;

/* Frequency and duty from rising crossings of the mid level (with hysteresis). */
void measure(const uint8_t *buf, int n, uint32_t rate_hz, meas_t *m);

/* Forget the history (call when timebase, range or offset change). */
void meas_avg_reset(meas_avg_t *a);
/* Average freq/duty/vpp over the last up to MEAS_AVG_N captures (exponential once full).
   Restarts from this capture when it is invalid or freq or Vpp moved more than 10 %
   (Vpp: 10 % + 2 codes) from the average. *out = *in with the averaged values. */
void meas_avg_update(meas_avg_t *a, const meas_t *in, meas_t *out);
