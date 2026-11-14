#include "ilv.h"

void ilv_weave(uint8_t *dst, const uint8_t *const *src, int streams, uint32_t per) {
    for (int k = 0; k < streams; k++) {
        const uint8_t *s = src[k];
        uint8_t *d = dst + k;
        for (uint32_t i = 0; i < per; i++, d += streams) *d = s[i];
    }
}
