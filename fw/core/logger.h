#pragma once
#include <stdint.h>
#include "bytefifo.h"
#include "stream_proto.h"

/* Continuous logger output: LOG packets of <= SP_LOG_MAX samples; samples that can't be queued are
   counted and reported as DROP(first_index, count) right before the next LOG. */
typedef struct {
    uint32_t index;          /* running sample index (includes lost samples) */
    uint32_t pending_drop;   /* lost samples not yet reported */
    uint32_t sent, dropped;  /* totals */
} logger_t;

void logger_init(logger_t *l);
/* Queue n consecutive samples (the next ones after l->index). */
void logger_push(logger_t *l, bytefifo_t *f, const uint8_t *s, uint32_t n);
/* n samples were lost before reaching the logger (DMA ring overrun). */
void logger_lost(logger_t *l, uint32_t n);

/* Which DMA ring halves to take, oldest first: 0 = first half, 1 = second half, -1 = none.
   lost = samples overwritten before they could be taken. flags: bit0 HT, bit1 TC (as read and
   cleared); remaining = the DMA's transfers still to go in this pass; n = ring length (even). */
typedef struct { int first, second; uint32_t lost; } ring_pick_t;
ring_pick_t logger_pick(int flags, uint32_t remaining, uint32_t n);
