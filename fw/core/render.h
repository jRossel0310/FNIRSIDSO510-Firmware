#pragma once
#include <stdint.h>

typedef struct { int16_t lo, hi; } span_t;   /* ADC codes; lo == -1 means empty */

/* Map buf[start ...] onto ncols columns; spc_q8 = samples per column * 256. */
void render_spans(const uint8_t *buf, int n, int start, uint32_t spc_q8, span_t *out, int ncols);
