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

/* DMA ring of n samples (n even) as two halves. `completed` = total half-buffer completions since the
   ring started (from the DMA interrupt: completion k, counting from 0, fills half k % 2). `*consumed` =
   completions already handled (start at 0). Returns the newest completed half to take (first = 0 or 1,
   -1 if nothing new) and how many samples were overwritten before they could be taken; advances
   *consumed to completed. Only the newest half is safe: with two or more pending, the DMA is already
   overwriting the older one. second is always -1 (kept for the caller's loop). */
typedef struct { int first, second; uint32_t lost; } ring_pick_t;
ring_pick_t logger_pick(uint32_t completed, uint32_t *consumed, uint32_t n);
