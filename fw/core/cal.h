#pragma once
#include <stdint.h>

#define CAL_PAGE_ADDR  0x08006000u   /* stock settings+calibration page: READ ONLY for us */
#define CAL_PAGE_WORDS 0x27
#define CAL_SET_SLOW   0             /* <= 2.4 MS/s  (stock timebase index >= 10) */
#define CAL_SET_MID    1             /* > 2.4 MS/s single stream (stock index 9) */
#define CAL_SET_FAST   2             /* 4-way interleaved (stock index < 9), sub-project 1b */
#define CAL_NSETS      3
#define CAL_NRANGES    10
#define CAL_PWM_MAX    0xFFE

typedef struct {
    uint16_t lo[CAL_NSETS][CAL_NRANGES];   /* offset PWM at offset -100 */
    uint16_t hi[CAL_NSETS][CAL_NRANGES];   /* offset PWM at offset +100 */
    uint8_t valid;                         /* 1 = factory data, 0 = defaults */
} cal_t;

int cal_parse(const uint32_t *page, cal_t *c);
void cal_defaults(cal_t *c);
int cal_set_for_rate(uint32_t rate_hz);
uint16_t cal_offset_pwm(const cal_t *c, int set, int range, int offset);

/* Auto-zero helper: returns the PWM in [a, b] whose measured mean code is closest to target.
 * mean_at must set the PWM, wait for settling, capture and return the mean ADC code. */
typedef int (*cal_mean_fn)(uint16_t pwm, void *ctx);
uint16_t cal_search_pwm(cal_mean_fn mean_at, void *ctx, int target, uint16_t a, uint16_t b);
/* Move a range's lo/hi so that offset 0 maps to pwm_at_zero (span unchanged). */
void cal_shift_zero(cal_t *c, int set, int range, uint16_t pwm_at_zero);

/* One auto-zero step: given the trace mean (in 1/16 ADC codes) measured with the offset PWM at
   pwm, return the PWM that puts the mean on ADC_CENTER, using the range's PWM span (hi - lo per
   200 codes). Clamped to 0..CAL_PWM_MAX; span <= 0 returns pwm unchanged. */
uint16_t cal_zero_step(uint16_t pwm, int mean_x16, int span);
