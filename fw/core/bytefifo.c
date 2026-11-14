#include "bytefifo.h"

#define BF_BARRIER() __asm__ __volatile__("" ::: "memory")   /* data before the index update */

void bf_init(bytefifo_t *f, uint8_t *buf, uint32_t size) {
    f->buf = buf; f->mask = size - 1u; f->head = 0; f->tail = 0;
}

uint32_t bf_used(const bytefifo_t *f) { return f->head - f->tail; }
uint32_t bf_space(const bytefifo_t *f) { return f->mask + 1u - (f->head - f->tail); }

uint32_t bf_write(bytefifo_t *f, const uint8_t *p, uint32_t n) {
    uint32_t s = bf_space(f), h = f->head;
    if (n > s) n = s;
    for (uint32_t i = 0; i < n; i++) f->buf[(h + i) & f->mask] = p[i];
    BF_BARRIER();
    f->head = h + n;
    return n;
}

uint32_t bf_read(bytefifo_t *f, uint8_t *p, uint32_t max) {
    uint32_t u = bf_used(f), t = f->tail;
    if (max > u) max = u;
    for (uint32_t i = 0; i < max; i++) p[i] = f->buf[(t + i) & f->mask];
    BF_BARRIER();
    f->tail = t + max;
    return max;
}

void bf_clear(bytefifo_t *f) { f->tail = f->head; }
