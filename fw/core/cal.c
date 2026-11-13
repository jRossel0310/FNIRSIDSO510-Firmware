#include "cal.h"

static const uint8_t lo_off[CAL_NSETS] = {0x00, 0x14, 0x28};
static const uint8_t hi_off[CAL_NSETS] = {0x3C, 0x50, 0x64};

void cal_defaults(cal_t *c) {
    for (int s = 0; s < CAL_NSETS; s++)
        for (int r = 0; r < CAL_NRANGES; r++) { c->lo[s][r] = 0x400; c->hi[s][r] = 0xC00; }
    c->valid = 0;
}

int cal_parse(const uint32_t *page, cal_t *c) {
    uint32_t mark = page[0] & 0xFF;
    const uint8_t *b = (const uint8_t *)&page[8];
    int ok = (mark == 0x55 || mark == 0xAA);
    for (int s = 0; s < CAL_NSETS && ok; s++)
        for (int r = 0; r < CAL_NRANGES && ok; r++) {
            uint16_t lo = (uint16_t)(b[lo_off[s] + 2 * r] | b[lo_off[s] + 2 * r + 1] << 8);
            uint16_t hi = (uint16_t)(b[hi_off[s] + 2 * r] | b[hi_off[s] + 2 * r + 1] << 8);
            if (lo > CAL_PWM_MAX || hi > CAL_PWM_MAX || lo == hi) ok = 0;
            c->lo[s][r] = lo; c->hi[s][r] = hi;
        }
    if (!ok) { cal_defaults(c); return 0; }
    c->valid = 1;
    return 1;
}

int cal_set_for_rate(uint32_t rate_hz) { return rate_hz > 2400000u ? CAL_SET_MID : CAL_SET_SLOW; }

uint16_t cal_offset_pwm(const cal_t *c, int set, int range, int offset) {
    int lo = c->lo[set][range], hi = c->hi[set][range];
    int v = lo + (hi - lo) * (offset + 100) / 200;
    if (v < 0) v = 0;
    if (v > CAL_PWM_MAX) v = CAL_PWM_MAX;
    return (uint16_t)v;
}

uint16_t cal_search_pwm(cal_mean_fn mean_at, void *ctx, int target, uint16_t a, uint16_t b) {
    int lo = a < b ? a : b, hi = a < b ? b : a;
    int up = mean_at((uint16_t)hi, ctx) > mean_at((uint16_t)lo, ctx);
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        int m = mean_at((uint16_t)mid, ctx);
        if ((m < target) == up) lo = mid; else hi = mid;
    }
    int dl = mean_at((uint16_t)lo, ctx) - target, dh = mean_at((uint16_t)hi, ctx) - target;
    if (dl < 0) dl = -dl;
    if (dh < 0) dh = -dh;
    return (uint16_t)(dl <= dh ? lo : hi);
}

void cal_shift_zero(cal_t *c, int set, int range, uint16_t pwm_at_zero) {
    int d = (int)pwm_at_zero - (int)cal_offset_pwm(c, set, range, 0);
    int lo = c->lo[set][range] + d, hi = c->hi[set][range] + d;
    if (lo < 0) lo = 0;
    if (hi < 0) hi = 0;
    if (lo > CAL_PWM_MAX) lo = CAL_PWM_MAX;
    if (hi > CAL_PWM_MAX) hi = CAL_PWM_MAX;
    c->lo[set][range] = (uint16_t)lo; c->hi[set][range] = (uint16_t)hi;
}

uint16_t cal_zero_step(uint16_t pwm, int mean_x16, int span) {
    if (span <= 0) return pwm;
    int32_t e = (int32_t)mean_x16 - 128 * 16;            /* error, 1/16 codes; + = trace too high */
    int32_t n = e * span, d = 200 * 16;
    int32_t dp = (n >= 0 ? n + d / 2 : n - d / 2) / d;    /* rounded PWM correction */
    int32_t v = (int32_t)pwm - dp;
    if (v < 0) v = 0;
    if (v > CAL_PWM_MAX) v = CAL_PWM_MAX;
    return (uint16_t)v;
}
