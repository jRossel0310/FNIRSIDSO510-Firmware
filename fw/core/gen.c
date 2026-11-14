#include "gen.h"

void gen_defaults(gen_cfg_t *g) {
    g->on = 1; g->wave = GEN_SINE; g->fine = 0; g->duty_pct = 50;
    g->amp_dv = 20; g->offset_dv = 15; g->freq_hz = 1000;
}

void gen_clamp(gen_cfg_t *g) {
    if (g->wave >= GEN_NUM_WAVES) g->wave = GEN_SINE;
    if (g->freq_hz < GEN_FREQ_MIN) g->freq_hz = GEN_FREQ_MIN;
    if (g->freq_hz > GEN_FREQ_MAX) g->freq_hz = GEN_FREQ_MAX;
    if (g->duty_pct < 1) g->duty_pct = 1;
    if (g->duty_pct > 99) g->duty_pct = 99;
    if (g->amp_dv < 1) g->amp_dv = 1;
    if (g->amp_dv > GEN_AMP_MAX_DV) g->amp_dv = GEN_AMP_MAX_DV;
    if (g->wave == GEN_DC) {
        if (g->offset_dv > GEN_OFF_MAX_DV) g->offset_dv = GEN_OFF_MAX_DV;
        return;
    }
    /* centre*100 - amp*50 >= 0 and centre*100 + amp*50 <= 3100 (mV) */
    int lo = (g->amp_dv + 1) / 2, hi = (GEN_FS_MV - g->amp_dv * 50) / 100;
    if (g->offset_dv < lo) g->offset_dv = (uint8_t)lo;
    if (g->offset_dv > hi) g->offset_dv = (uint8_t)hi;
}

uint32_t gen_freq_step(uint32_t f, int dir, int fine) {
    uint32_t dec = 1;
    while (dec * 10u <= f) dec *= 10u;                 /* largest power of ten <= f */
    uint32_t r;
    if (fine) {
        if (dir > 0) r = f + (dec >= 100u ? dec / 100u : 1u);
        else {
            if (f == dec && dec >= 10u) dec /= 10u;    /* stepping down from 1000 uses the 100s decade */
            uint32_t st = dec >= 100u ? dec / 100u : 1u;
            r = f > st ? f - st : GEN_FREQ_MIN;
        }
    } else if (dir > 0) {
        r = f < 2u * dec ? 2u * dec : f < 5u * dec ? 5u * dec : 10u * dec;
    } else {
        r = f > 5u * dec ? 5u * dec : f > 2u * dec ? 2u * dec : f > dec ? dec : dec / 2u;
    }
    if (r < GEN_FREQ_MIN) r = GEN_FREQ_MIN;
    if (r > GEN_FREQ_MAX) r = GEN_FREQ_MAX;
    return r;
}

gen_plan_t gen_plan(uint32_t f) {
    gen_plan_t best = {0, 0, 0, 0, 0};
    uint32_t best_ppm = 0xFFFFFFFFu;
    if (f < GEN_FREQ_MIN) f = GEN_FREQ_MIN;
    if (f > GEN_FREQ_MAX) f = GEN_FREQ_MAX;
    uint64_t want = (uint64_t)f * 1000u;               /* mHz */
    /* Prefer one period per table and the longest table; add periods only when that is not accurate. */
    for (uint32_t m = 1; m <= GEN_N_MAX / GEN_PPP_MIN; m++) {
        for (uint32_t n = GEN_N_MAX; n >= GEN_PPP_MIN * m; n--) {
            uint64_t fn = (uint64_t)f * n;
            uint64_t num = (uint64_t)GEN_TIMER_HZ * m;
            uint32_t d = (uint32_t)((num + fn / 2) / fn);          /* total timer divider */
            if (d < GEN_MIN_DIV) continue;
            uint32_t ps = (d + 65535u) / 65536u;                   /* prescaler factor so the reload fits 16 bits */
            uint32_t ar = (d + ps / 2) / ps;
            if (ar > 65536u) ar = 65536u;
            uint64_t den = (uint64_t)ps * ar * n;
            uint64_t act = (num * 1000u + den / 2) / den;          /* mHz */
            uint64_t diff = act > want ? act - want : want - act;
            uint32_t ppm = (uint32_t)(diff * 1000000u / want);
            if (ppm < best_ppm) {
                best_ppm = ppm;
                best.n = (uint16_t)n; best.m = (uint16_t)m;
                best.psc = (uint16_t)(ps - 1u); best.arr = (uint16_t)(ar - 1u);
                best.actual_mhz = (uint32_t)act;
            }
            if (ppm <= 200u) return best;                          /* good enough: stop at the smoothest table */
        }
    }
    return best;
}

uint16_t gen_code_mv(int32_t mv) {
    if (mv <= 0) return 0;
    if (mv >= GEN_FS_MV) return 4095;
    return (uint16_t)((mv * 4095 + GEN_FS_MV / 2) / GEN_FS_MV);
}

/* sin(2*pi*x) for x in [0, 1): odd 7th-order polynomial on [-pi/2, pi/2] (error < 2e-4). */
static float gsin(float x) {
    if (x >= 0.75f) x -= 1.0f;                          /* -> [-0.25, 0.75) turns */
    if (x > 0.25f) x = 0.5f - x;                        /* mirror into [-0.25, 0.25] */
    float a = x * 6.2831853f, a2 = a * a;
    return a * (1.0f - a2 / 6.0f * (1.0f - a2 / 20.0f * (1.0f - a2 / 42.0f)));
}

/* e^(-t) for t in [0, 5]: (e^(-t/16))^16 with a 4-term series for the small step. */
static float gexpn(float t) {
    float s = t / 16.0f;
    float e = 1.0f - s * (1.0f - s / 2.0f * (1.0f - s / 3.0f * (1.0f - s / 4.0f)));
    e *= e; e *= e; e *= e; e *= e;
    return e;
}

void gen_table(const gen_cfg_t *g, const gen_plan_t *p, uint16_t *out) {
    int32_t lo_mv = (int32_t)g->offset_dv * 100 - (int32_t)g->amp_dv * 50;
    int32_t hi_mv = (int32_t)g->offset_dv * 100 + (int32_t)g->amp_dv * 50;
    const float e5 = 0.006737947f;                      /* e^-5 */
    uint32_t n = p->n, m = p->m ? p->m : 1u;
    /* square: high while r*100 < duty*n, clamped so every period keeps at least one high and one low sample */
    uint32_t th = (uint32_t)g->duty_pct * n;
    if (th < 100u * m) th = 100u * m;
    if (th > 100u * (n - m)) th = 100u * (n - m);
    for (uint32_t i = 0; i < n; i++) {
        uint32_t r = (i * m) % n;                       /* phase numerator: x = r / n */
        float x = (float)r / (float)n;                  /* phase within the period, 0..1 */
        float s;                                        /* shape, 0..1 */
        switch (g->wave) {
        case GEN_SQUARE:   s = r * 100u < th ? 1.0f : 0.0f; break;
        case GEN_TRIANGLE: s = x < 0.5f ? 2.0f * x : 2.0f - 2.0f * x; break;
        case GEN_SAW:      s = x; break;
        case GEN_DC:       s = 0.5f; break;
        case GEN_HALF:     s = gsin(x); if (s < 0.0f) s = 0.0f; break;
        case GEN_FULL:     s = gsin(x * 0.5f); if (s < 0.0f) s = 0.0f; break;   /* one hump: sin(pi x) */
        case GEN_EXP_RISE: s = (1.0f - gexpn(5.0f * x)) / (1.0f - e5); break;
        case GEN_EXP_FALL: s = (gexpn(5.0f * x) - e5) / (1.0f - e5); break;
        default:           s = 0.5f + 0.5f * gsin(x); break;                   /* sine */
        }
        if (s < 0.0f) s = 0.0f;
        if (s > 1.0f) s = 1.0f;
        int32_t mv = g->wave == GEN_DC ? (int32_t)g->offset_dv * 100
                                       : lo_mv + (int32_t)(s * (float)(hi_mv - lo_mv) + 0.5f);
        out[i] = gen_code_mv(mv);
    }
}

const char *gen_wave_name(int wave) {
    static const char *names[GEN_NUM_WAVES] = {"Sine", "Square", "Triangle", "Sawtooth", "DC",
                                               "Half-wave", "Full-wave", "Exp rise", "Exp fall"};
    return wave >= 0 && wave < GEN_NUM_WAVES ? names[wave] : "?";
}
