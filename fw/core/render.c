#include "render.h"

void render_spans(const uint8_t *buf, int n, int start, uint32_t spc_q8, span_t *out, int ncols) {
    int have_prev = 0, prev = 0;
    for (int x = 0; x < ncols; x++) {
        span_t s = {-1, -1};
        if (spc_q8 >= 256) {
            int a = start + (int)(((uint32_t)x * spc_q8) >> 8);
            int b = start + (int)(((uint32_t)(x + 1) * spc_q8) >> 8);
            if (b <= a) b = a + 1;
            if (a >= 0 && a < n) {
                if (b > n) b = n;
                int lo = buf[a], hi = buf[a];
                for (int i = a + 1; i < b; i++) { if (buf[i] < lo) lo = buf[i]; if (buf[i] > hi) hi = buf[i]; }
                if (a > 0) { int p = buf[a - 1]; if (p < lo) lo = p; if (p > hi) hi = p; }
                s.lo = (int16_t)lo; s.hi = (int16_t)hi;
            }
        } else {
            int32_t pos = start * 256 + (int32_t)((uint32_t)x * spc_q8);
            int i = pos >> 8, f = pos & 255;
            if (i >= 0 && i < n) {
                int v = (i + 1 < n) ? (buf[i] * (256 - f) + buf[i + 1] * f + 128) >> 8 : buf[i];
                int lo = v, hi = v;
                if (have_prev) { if (prev < lo) lo = prev; if (prev > hi) hi = prev; }
                s.lo = (int16_t)lo; s.hi = (int16_t)hi;
                prev = v; have_prev = 1;
            }
        }
        out[x] = s;
    }
}
