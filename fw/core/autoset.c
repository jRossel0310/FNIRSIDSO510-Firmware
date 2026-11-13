#include "autoset.h"
#include "units.h"

static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

/* round(n / d) for d > 0 */
static int32_t div_round(int32_t n, int32_t d) { return (n >= 0 ? n + d / 2 : n - d / 2) / d; }

int autoset_fit(int *vdiv, int *offset, int vmin, int vmax) {
    int v = *vdiv, off = *offset;
    int top = vmax >= 253, bot = vmin <= 2;
    if (top || bot) {
        if (top && !bot && off > -100) off = clampi(off - 50, -100, 100);
        else if (bot && !top && off < 100) off = clampi(off + 50, -100, 100);
        else if (v < NUM_VDIV - 1) v++;
        else return 0;
        *vdiv = v; *offset = off;
        return 1;
    }
    if (vmax - vmin < 2) return 0;                      /* flat: nothing to scale on */
    /* Signal extents in units of mV*CODES_PER_DIV (exact integers). */
    int32_t hi = (int32_t)(vmax - ADC_CENTER - off) * vdiv_mv[v];
    int32_t lo = (int32_t)(vmin - ADC_CENTER - off) * vdiv_mv[v];
    int32_t pp = hi - lo;
    int nv = NUM_VDIV - 1, noff = clampi(off, -100, 100);
    for (int i = 0; i < NUM_VDIV; i++) {
        int32_t m = vdiv_mv[i];
        if (pp > 6 * CODES_PER_DIV * m) continue;
        int o = clampi((int)div_round(-(hi + lo), 2 * m), -100, 100);
        int32_t ctop = ADC_CENTER + o + div_round(hi, m), cbot = ADC_CENTER + o + div_round(lo, m);
        if (i == NUM_VDIV - 1 || (cbot >= SCREEN_CODE_LO && ctop <= SCREEN_CODE_HI)) { nv = i; noff = o; break; }
    }
    int again = nv != v || noff - off > 2 || off - noff > 2;
    *vdiv = nv; *offset = noff;
    return again;
}

int autoset_tdiv(float freq_hz) {
    if (freq_hz <= 0.0f) return -1;
    float need_ns = 2e9f / freq_hz;                       /* two periods */
    for (int i = 0; i < NUM_TDIV; i++) if ((float)tdiv_ns[i] * 10.0f >= need_ns * 0.999f) return i;
    return NUM_TDIV - 1;
}
