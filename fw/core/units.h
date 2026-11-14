#pragma once
#include <stdint.h>

#define NUM_VDIV      10
#define NUM_TDIV      19
#define TDIV_2US      5
#define TDIV_10US     7
#define TDIV_1MS      13
#define CODES_PER_DIV 25
#define ADC_CENTER    128
#define TIMER_CLK_HZ  240000000u
#define MAX_RATE_HZ   12000000u
#define ILV_RATE_HZ   48000000u
#define SAMPLES_PER_DIV 120

extern const uint16_t vdiv_mv[NUM_VDIV];
extern const uint32_t tdiv_ns[NUM_TDIV];

typedef struct { uint16_t psc, arr; uint32_t rate_hz; uint8_t streams; } rate_cfg_t;

/* Sample rate for a timebase: 120 samples/div where possible. Up to 12 MS/s a single stream
   (streams = 1, psc/arr = sample timer). Above that the capture is interleaved with the ADC clocked
   at 48 MHz (psc 0, arr 4): streams = 2 gives 24 MS/s, streams = 4 gives 48 MS/s. */
rate_cfg_t units_rate_for_tdiv(uint32_t tdiv_ns);
/* Exact timer setting for a sample rate: TIMER_CLK_HZ / ((psc+1)(arr+1)) == hz.
   rate_hz = 0 if hz is 0 or no exact setting exists. */
rate_cfg_t units_rate_for_hz(uint32_t hz);
/* Samples covering the 10-division screen. */
uint32_t units_samples_per_screen(uint32_t rate_hz, uint32_t tdiv_ns);
float units_volts_per_code(int vdiv_idx, int probe_x10);
/* 0 V is at code ADC_CENTER + offset (offset in codes, -100..100). */
float units_code_to_volts(int code, int vdiv_idx, int offset, int probe_x10);
/* Stock FNIRSI battery thresholds. */
int units_batt_pct(float vbat);

/* Front-end settle time after the offset moves by dcodes ADC codes. The PA7 offset PWM filter
   settles exponentially with tau ~= 0.33 s (measured on hardware, docs/autodbg): wait
   40 ms (relays) + 350 ms * ln(2*|dcodes|), i.e. until the residual error is < 0.5 code. */
uint32_t fe_settle_ms(int dcodes);
