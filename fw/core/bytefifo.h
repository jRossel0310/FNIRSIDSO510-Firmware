#pragma once
#include <stdint.h>
/* Single-producer / single-consumer byte FIFO; one side may run in an interrupt. The producer only
   writes `head`, the consumer only writes `tail` (free-running counters, size a power of two). */
typedef struct {
    uint8_t *buf;
    uint32_t mask;              /* size - 1 */
    volatile uint32_t head;     /* total bytes ever written */
    volatile uint32_t tail;     /* total bytes ever read */
} bytefifo_t;

void bf_init(bytefifo_t *f, uint8_t *buf, uint32_t size);    /* size must be a power of two */
uint32_t bf_used(const bytefifo_t *f);
uint32_t bf_space(const bytefifo_t *f);
/* Producer: append up to n bytes (only what fits). Returns bytes written. */
uint32_t bf_write(bytefifo_t *f, const uint8_t *p, uint32_t n);
/* Consumer: remove up to max bytes. Returns bytes read. */
uint32_t bf_read(bytefifo_t *f, uint8_t *p, uint32_t max);
/* Consumer: discard everything currently queued. */
void bf_clear(bytefifo_t *f);
