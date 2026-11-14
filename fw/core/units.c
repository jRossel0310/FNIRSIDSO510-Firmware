#include "units.h"

const uint16_t vdiv_mv[NUM_VDIV] = {10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000};
const uint32_t tdiv_ns[NUM_TDIV] = {
    50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000,
    1000000, 2000000, 5000000, 10000000, 20000000, 50000000,
};

rate_cfg_t units_rate_for_tdiv(uint32_t tdiv) {
    /* timer ticks per sample = 240e6 * tdiv[s] / 120 = tdiv_ns / 500 */
    uint32_t div = tdiv / 500;
    uint32_t min_div = TIMER_CLK_HZ / MAX_RATE_HZ;
    rate_cfg_t r;
    if (div < min_div) {                                  /* faster than one stream can go */
        r.psc = 0; r.arr = (uint16_t)(TIMER_CLK_HZ / ILV_RATE_HZ - 1u);
        r.streams = div >= min_div / 2 ? 2 : 4;
        r.rate_hz = ILV_RATE_HZ / 4u * r.streams;
        return r;
    }
    r.streams = 1;
    if (div <= 65536) { r.psc = 0; r.arr = (uint16_t)(div - 1); }
    else { r.psc = (uint16_t)(div / 1000 - 1); r.arr = 999; }
    r.rate_hz = TIMER_CLK_HZ / div;
    return r;
}

uint32_t units_samples_per_screen(uint32_t rate_hz, uint32_t tdiv) {
    return (uint32_t)((uint64_t)rate_hz * tdiv * 10u / 1000000000u);
}

float units_volts_per_code(int vdiv_idx, int probe_x10) {
    return (float)vdiv_mv[vdiv_idx] / (CODES_PER_DIV * 1000.0f) * (probe_x10 ? 10.0f : 1.0f);
}

float units_code_to_volts(int code, int vdiv_idx, int offset, int probe_x10) {
    return (float)(code - (ADC_CENTER + offset)) * units_volts_per_code(vdiv_idx, probe_x10);
}

int units_batt_pct(float v) {
    static const float th[] = {4.06f, 3.98f, 3.92f, 3.87f, 3.82f, 3.79f, 3.77f, 3.74f, 3.68f};
    for (int i = 0; i < 9; i++) if (v >= th[i]) return 100 - 10 * i;
    return 10;
}

uint32_t fe_settle_ms(int dcodes) {
    uint32_t d = (uint32_t)(dcodes < 0 ? -dcodes : dcodes);
    if (d <= 1) return 40;
    int l2 = 0;                                 /* ceil(log2(2d)) */
    while ((1u << l2) < 2 * d) l2++;
    return 40u + 243u * (uint32_t)l2;           /* 350 ms * ln 2 = 243 ms per doubling */
}

rate_cfg_t units_rate_for_hz(uint32_t hz) {
    rate_cfg_t r = {0, 0, 0, 1};
    if (hz == 0 || TIMER_CLK_HZ % hz) return r;
    uint32_t div = TIMER_CLK_HZ / hz;
    for (uint32_t p = 1; p <= 65536u; p++) {
        if (div % p == 0 && div / p <= 65536u) {
            r.psc = (uint16_t)(p - 1u); r.arr = (uint16_t)(div / p - 1u); r.rate_hz = hz;
            return r;
        }
    }
    return r;
}
