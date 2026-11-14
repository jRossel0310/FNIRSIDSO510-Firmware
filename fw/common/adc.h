// External ADC (3PEAK 3PA1030, pipelined): top 8 bits on PB0..PB7, clock on PB8 = TMR4 CH3.
// The clock runs continuously so the pipeline always holds fresh conversions.
#pragma once
#include <stdint.h>

void adc_init(void);
void adc_set_rate(uint16_t psc, uint16_t arr);   // rate = 240 MHz / ((psc+1)(arr+1)); waits for pipeline flush
void adc_start(uint8_t *buf, uint16_t n);        // arm DMA1 ch7 for n samples (non-blocking)
int adc_done(void);                               // 1 once the armed capture is complete (or none armed)
void adc_abort(void);

// Continuous capture into a circular buffer of n bytes (n even): DMA1 ch7 circular mode. The timer
// rate is whatever adc_set_rate() last set. Stop with adc_abort().
void adc_ring_start(uint8_t *buf, uint16_t n);
// Number of half-buffer completions since adc_ring_start (incremented by the DMA1 ch7 IRQ).
// Completion k (from 0) filled half k % 2. The consumer must read this once per pass and hand that
// snapshot to logger_pick(completed, &consumed, n) (fw/core/logger.h). Only the newest completed
// half is safe to copy; older ones may already be overwritten.
uint32_t adc_ring_completed(void);
