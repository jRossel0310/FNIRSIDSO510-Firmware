// External ADC (3PEAK 3PA1030, pipelined): top 8 bits on PB0..PB7, clock on PB8 = TMR4 CH3.
// The clock runs continuously so the pipeline always holds fresh conversions.
#pragma once
#include <stdint.h>

void adc_init(void);
void adc_set_rate(uint16_t psc, uint16_t arr);   // rate = 240 MHz / ((psc+1)(arr+1)); waits for pipeline flush
void adc_start(uint8_t *buf, uint16_t n);        // arm DMA1 ch7 for n samples (non-blocking)
int adc_done(void);                               // 1 once the armed capture is complete (or none armed)
void adc_abort(void);
