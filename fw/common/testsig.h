// Test signals (stock configuration, at 240 MHz timer clocks). Generator UI is sub-project 3.
#pragma once
#include <stdint.h>

void cal_out_init(void);           // PA3 = TMR9 CH2: 1 kHz, 50 % square on the cal pad
void dac2_bias_init(void);         // PA5 = DAC2 held at 0xF46 like stock (purpose unknown)
void gen_sine_init(uint16_t arr);  // PA4 = DAC1 sine, 100 points: f = 240 MHz / (arr+1) / 100
void gen_set_arr(uint16_t arr);
