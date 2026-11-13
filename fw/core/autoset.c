#include "autoset.h"
#include "units.h"

int autoset_vdiv(int cur, int vmin, int vmax) {
    if (vmin <= 2 || vmax >= 253) return cur < NUM_VDIV - 1 ? cur + 1 : cur;
    int pp = vmax - vmin;
    if (pp < 2) return cur;
    uint32_t mv_pp = (uint32_t)pp * vdiv_mv[cur] / CODES_PER_DIV;
    for (int i = 0; i < NUM_VDIV; i++) if (mv_pp <= 6u * vdiv_mv[i]) return i;
    return NUM_VDIV - 1;
}

int autoset_tdiv(float freq_hz) {
    if (freq_hz <= 0.0f) return -1;
    float need_ns = 2e9f / freq_hz;                       /* two periods */
    for (int i = 0; i < NUM_TDIV; i++) if ((float)tdiv_ns[i] * 10.0f >= need_ns * 0.999f) return i;
    return NUM_TDIV - 1;
}
