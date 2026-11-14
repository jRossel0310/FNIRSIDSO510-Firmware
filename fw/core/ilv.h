#pragma once
#include <stdint.h>
/* Interleaved capture: weave `streams` equally long sample streams into one record.
   dst[i * streams + k] = src[k][i] for i < per. src[k] already points at stream k's first valid sample. */
void ilv_weave(uint8_t *dst, const uint8_t *const *src, int streams, uint32_t per);
