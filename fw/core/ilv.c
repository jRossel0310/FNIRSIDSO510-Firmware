#include "ilv.h"

void ilv_weave(uint8_t *dst, const uint8_t *const *src, int streams, uint32_t per) {
    for (int k = 0; k < streams; k++) {
        const uint8_t *s = src[k];
        uint8_t *d = dst + k;
        for (uint32_t i = 0; i < per; i++, d += streams) *d = s[i];
    }
}

uint32_t ilv_despike(uint8_t *rec, uint32_t n) {
    uint32_t fixed = 0;
    for (uint32_t i = 1; i + 1 < n; i++) {
        int a = rec[i - 1], x = rec[i], b = rec[i + 1];
        int d1 = x - a, d2 = x - b, flat = a > b ? a - b : b - a;
        if (flat > ILV_SPIKE_FLAT) continue;
        if ((d1 >= ILV_SPIKE_MIN && d2 >= ILV_SPIKE_MIN) || (d1 <= -ILV_SPIKE_MIN && d2 <= -ILV_SPIKE_MIN)) {
            rec[i] = (uint8_t)((a + b + 1) / 2);
            fixed++;
            i++;                                 /* the next sample was a neighbour, not a spike */
        }
    }
    return fixed;
}
