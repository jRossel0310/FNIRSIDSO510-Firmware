#pragma once
#include <stdint.h>

enum { MEAS_FREQ = 0, MEAS_PERIOD, MEAS_DUTY_P, MEAS_DUTY_N, MEAS_WIDTH_P, MEAS_WIDTH_N,
       MEAS_MAX, MEAS_MIN, MEAS_VPP, MEAS_AMPL, MEAS_RMS, MEAS_AVG, MEAS_NUM };
enum { MEAS_U_HZ = 0, MEAS_U_S, MEAS_U_PCT, MEAS_U_V };

/* All *_codes are in ADC codes. The statistics window is the samples between the first and the last
   rising crossing when valid, else the whole record. */
typedef struct {
    int valid; float freq_hz, duty_pct, vpp_codes; uint8_t vmin, vmax;
    float max_codes, min_codes;      /* extremes of the record (float so they can be averaged) */
    float top_codes, base_codes;     /* most frequent code at/above and below the mid level */
    float mean_codes, meansq_codes;  /* mean and mean square of the code over the statistics window */
} meas_t;

#define MEAS_AVG_N 8
/* Running average of successive measurements (displayed values). */
typedef struct { int n; float freq_hz, duty_pct, vpp_codes;
                 float max_codes, min_codes, top_codes, base_codes, mean_codes, meansq_codes; } meas_avg_t;

/* Frequency and duty from rising crossings of the mid level (with hysteresis). */
void measure(const uint8_t *buf, int n, uint32_t rate_hz, meas_t *m);

/* Forget the history (call when timebase, range or offset change). */
void meas_avg_reset(meas_avg_t *a);
/* Average freq/duty/vpp over the last up to MEAS_AVG_N captures (exponential once full).
   Restarts from this capture when it is invalid or freq or Vpp moved more than 10 %
   (Vpp: 10 % + 2 codes) from the average. *out = *in with the averaged values. */
void meas_avg_update(meas_avg_t *a, const meas_t *in, meas_t *out);

/* Value of one measurement in its unit. volts_per_code and zero_code (the ADC code of 0 V, may be fractional)
   convert codes to volts. Returns 1 and sets *value and *unit, or 0 if it is not available (time measurements
   without a full period; unknown kind). *unit is always set for a known kind. */
int meas_value(const meas_t *m, int kind, float volts_per_code, float zero_code, float *value, int *unit);
const char *meas_name(int kind);   /* "Frequency", "Period", ... "Average"; "?" if unknown */
const char *meas_tag(int kind);    /* "Freq", "Per", "Duty+", ... "Avg"; "?" */
