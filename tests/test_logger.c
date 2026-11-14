#include "test.h"
#include "logger.h"

static uint8_t mem[4096];
static bytefifo_t F;

typedef struct { int type; uint32_t a, b; uint32_t n; uint8_t first; } pk_t;

/* Pop one packet from F (assumes well-formed). Returns 0 if the FIFO is empty. */
static int pop(pk_t *p) {
    uint8_t h[5];
    if (bf_used(&F) < 5) return 0;
    bf_read(&F, h, 5);
    uint32_t len = (uint32_t)(h[3] | h[4] << 8);
    static uint8_t pl[2048];
    bf_read(&F, pl, len + 2);
    p->type = h[2];
    p->a = pl[0] | pl[1] << 8 | pl[2] << 16 | (uint32_t)pl[3] << 24;
    p->b = len >= 8 ? (pl[4] | pl[5] << 8 | pl[6] << 16 | (uint32_t)pl[7] << 24) : 0;
    p->n = p->type == SP_LOG ? len - 4 : 0;
    p->first = p->type == SP_LOG ? pl[4] : 0;
    return 1;
}

void test_logger(void) {
    static uint8_t s[3000];
    for (int i = 0; i < 3000; i++) s[i] = (uint8_t)i;
    logger_t L;
    pk_t p;
    bf_init(&F, mem, sizeof mem);
    logger_init(&L);

    logger_push(&L, &F, s, 100);                         /* one LOG, index 0 */
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 0); CHECK_EQ(p.n, 100); CHECK_EQ(p.first, 0);
    CHECK(!pop(&p));
    CHECK_EQ(L.index, 100); CHECK_EQ(L.sent, 100); CHECK_EQ(L.dropped, 0);

    logger_push(&L, &F, s, 2500);                         /* chunks of <= SP_LOG_MAX */
    CHECK(pop(&p)); CHECK_EQ(p.a, 100); CHECK_EQ(p.n, 1024);
    CHECK(pop(&p)); CHECK_EQ(p.a, 1124); CHECK_EQ(p.n, 1024);
    CHECK(pop(&p)); CHECK_EQ(p.a, 2148); CHECK_EQ(p.n, 452);
    CHECK(!pop(&p));
    CHECK_EQ(L.index, 2600);

    /* FIFO nearly full: the chunk is lost and counted */
    uint8_t fill[4000] = {0};
    bf_write(&F, fill, 4000);                             /* 96 bytes left */
    logger_push(&L, &F, s, 200);
    CHECK_EQ(bf_used(&F), 4000);
    CHECK_EQ(L.index, 2800); CHECK_EQ(L.dropped, 200); CHECK_EQ(L.pending_drop, 200);
    uint8_t junk[4000];
    bf_read(&F, junk, 4000);
    /* next push: DROP(first_index = 2800, count = 200) then LOG at 2800 */
    logger_push(&L, &F, s, 50);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, 2800); CHECK_EQ(p.b, 200);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 2800); CHECK_EQ(p.n, 50);
    CHECK_EQ(L.pending_drop, 0);

    /* samples lost in the DMA ring */
    logger_lost(&L, 4096);
    CHECK_EQ(L.index, 2850 + 4096); CHECK_EQ(L.dropped, 200 + 4096);
    logger_push(&L, &F, s, 10);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, 6946); CHECK_EQ(p.b, 4096);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 6946); CHECK_EQ(p.n, 10);

    /* ring half picker: n = 1000, halves of 500. flags bit0 = HT, bit1 = TC */
    ring_pick_t r = logger_pick(0, 800, 1000);
    CHECK_EQ(r.first, -1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0);
    r = logger_pick(1, 400, 1000);                         /* HT only: first half done */
    CHECK_EQ(r.first, 0); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0);
    r = logger_pick(2, 900, 1000);                         /* TC only: second half done */
    CHECK_EQ(r.first, 1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0);
    r = logger_pick(3, 900, 1000);                         /* both, DMA in first half: first is being overwritten */
    CHECK_EQ(r.first, 1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 500);
    r = logger_pick(3, 300, 1000);                         /* both, DMA in second half: second is being overwritten */
    CHECK_EQ(r.first, 0); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 500);
}
