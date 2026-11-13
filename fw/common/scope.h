// Analog front end + external ADC (3PEAK 3PA1030, top 8 bits on PB0..PB7, clock on PB8).
#pragma once
#include <stdint.h>

#define FE_NUM_RANGES 10
#define FE_OFFSET_MAX 0xFFE         // TMR14 ARR (12-bit PWM on PA7)
#define FE_OFFSET_DEFAULT 0x834     // stock power-on value

void fe_init(void);
void fe_set_range(int r);          // 0..9, pin table from stock FUN_08008834
void fe_set_coupling(int pc5);     // PC5 level (stock setting byte @ 0x200000E8)
void fe_set_offset(uint16_t pwm);  // PA7 PWM compare (TMR14 CCR1)

typedef struct { uint16_t psc, arr; const char *label; } samplerate_t;
extern const samplerate_t sample_rates[];
extern const int num_sample_rates;

void adc_init(void);
// Blocking single-shot capture of n bytes at the given rate (TMR4 clocks ADC + paces DMA1 ch7).
// Returns 0 on success, -1 on timeout.
int adc_capture(uint8_t *buf, uint16_t n, const samplerate_t *sr);

// ---- Outputs (all as configured by stock firmware) ----
void cal_out_init(void);       // PA3 = TMR9 CH2, 1 kHz 50 % square (calibration pad, presumed)
void dac2_bias_init(void);     // PA5 = DAC2 static 0xF46 (stock init; purpose unknown)
void gen_sine_1k_init(void);   // PA4 = DAC1, TMR6-triggered DMA2 ch3 from 100-point table -> 1 kHz sine
