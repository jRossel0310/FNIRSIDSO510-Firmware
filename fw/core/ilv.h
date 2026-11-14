#pragma once
#include <stdint.h>
/* Interleaved capture: weave `streams` equally long sample streams into one record.
   dst[i * streams + k] = src[k][i] for i < per. src[k] already points at stream k's first valid sample. */
void ilv_weave(uint8_t *dst, const uint8_t *const *src, int streams, uint32_t per);

/* Remove isolated one-sample spikes from an interleaved record (rare misreads of the ADC pins while its
   output changes, measured on hardware: about 1 in 1000 samples at the worst code boundaries).
   Sample i is replaced by the mean of its neighbours when it lies at least ILV_SPIKE_MIN codes away from
   both, on the same side, and the neighbours differ by at most ILV_SPIKE_FLAT. Edges are left alone.
   Returns the number of samples replaced. */
#define ILV_SPIKE_MIN  8
#define ILV_SPIKE_FLAT 4
uint32_t ilv_despike(uint8_t *rec, uint32_t n);
