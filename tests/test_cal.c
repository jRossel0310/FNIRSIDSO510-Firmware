#include "test.h"
#include "cal.h"
#include <stdint.h>

static void put16(uint32_t *page, int byte_off, uint16_t v) {
    uint8_t *b = (uint8_t *)&page[8];
    b[byte_off] = (uint8_t)v; b[byte_off + 1] = (uint8_t)(v >> 8);
}

static int lin_up(uint16_t pwm, void *ctx) { (void)ctx; return 128 + ((int)pwm - 2000) / 10; }
static int lin_down(uint16_t pwm, void *ctx) { (void)ctx; return 128 - ((int)pwm - 2500) / 10; }

void test_cal(void) {
    uint32_t page[CAL_PAGE_WORDS];
    memset(page, 0, sizeof page);
    page[0] = 0x00000555;                       /* low byte 0x55 */
    static const int lo_off[3] = {0x00, 0x14, 0x28}, hi_off[3] = {0x3C, 0x50, 0x64};
    for (int s = 0; s < 3; s++)
        for (int r = 0; r < 10; r++) {
            put16(page, lo_off[s] + 2 * r, (uint16_t)(1000 + 10 * s + r));
            put16(page, hi_off[s] + 2 * r, (uint16_t)(3000 + 10 * s + r));
        }
    cal_t c;
    CHECK_EQ(cal_parse(page, &c), 1);
    CHECK_EQ(c.valid, 1);
    CHECK_EQ(c.lo[CAL_SET_SLOW][0], 1000);
    CHECK_EQ(c.hi[CAL_SET_MID][9], 3019);
    CHECK_EQ(c.lo[CAL_SET_FAST][5], 1025);
    CHECK_EQ(cal_offset_pwm(&c, CAL_SET_SLOW, 0, -100), 1000);
    CHECK_EQ(cal_offset_pwm(&c, CAL_SET_SLOW, 0, 100), 3000);
    CHECK_EQ(cal_offset_pwm(&c, CAL_SET_SLOW, 0, 0), 2000);

    page[0] = 0x000005AA;                       /* defaults marker is also accepted */
    CHECK_EQ(cal_parse(page, &c), 1);
    put16(page, 0x3C, 0xFFFF);                  /* erased value -> invalid */
    CHECK_EQ(cal_parse(page, &c), 0);
    CHECK_EQ(c.valid, 0);
    CHECK_EQ(c.lo[0][0], 0x400); CHECK_EQ(c.hi[0][0], 0xC00);   /* defaults */
    page[0] = 0;
    put16(page, 0x3C, 3000);
    CHECK_EQ(cal_parse(page, &c), 0);           /* bad marker */

    CHECK_EQ(cal_set_for_rate(2400000), CAL_SET_SLOW);
    CHECK_EQ(cal_set_for_rate(6000000), CAL_SET_MID);
    CHECK_EQ(cal_set_for_rate(12000000), CAL_SET_MID);
    CHECK_EQ(cal_set_for_rate(24000000), CAL_SET_FAST);
    CHECK_EQ(cal_set_for_rate(48000000), CAL_SET_FAST);

    uint16_t p = cal_search_pwm(lin_up, 0, 128, 1000, 3000);
    CHECK(p >= 1990 && p <= 2010);   /* integer model: mean is 128 for pwm 1991..2009 */
    p = cal_search_pwm(lin_down, 0, 128, 1000, 3000);
    CHECK(p >= 2490 && p <= 2510);

    cal_defaults(&c);
    cal_shift_zero(&c, CAL_SET_SLOW, 3, 0x900);     /* mid was 0x800 -> shift +0x100 */
    CHECK_EQ(c.lo[CAL_SET_SLOW][3], 0x500);
    CHECK_EQ(c.hi[CAL_SET_SLOW][3], 0xD00);
    CHECK_EQ(cal_offset_pwm(&c, CAL_SET_SLOW, 3, 0), 0x900);
    /* auto-zero step: span 1040 PWM per 200 codes = 5.2 PWM/code */
    CHECK_EQ(cal_zero_step(2000, 128 * 16, 1040), 2000);          /* already centred */
    CHECK_EQ(cal_zero_step(2000, 138 * 16, 1040), 1948);          /* 10 codes high -> -52 */
    CHECK_EQ(cal_zero_step(2000, 118 * 16, 1040), 2052);          /* 10 codes low -> +52 */
    CHECK_EQ(cal_zero_step(2000, 128 * 16 + 8, 1040), 1997);      /* half code -> -2.6 -> -3 */
    CHECK_EQ(cal_zero_step(10, 250 * 16, 1040), 0);               /* clamped low */
    CHECK_EQ(cal_zero_step(CAL_PWM_MAX - 5, 0, 1040), CAL_PWM_MAX);/* clamped high */
    CHECK_EQ(cal_zero_step(2000, 0, 0), 2000);                    /* bad span: unchanged */
}
