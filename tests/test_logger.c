#include "test.h"
#include "logger.h"

static uint8_t mem[4096];
static bytefifo_t F;

typedef struct { int type; uint32_t a, b; uint32_t n; uint8_t first; uint8_t d[SP_LOG_MAX]; } pk_t;

/* Pop one packet from F (assumes well-formed). Returns 0 if the FIFO is empty.
   For LOG packets the sample bytes are copied into p->d. */
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
    if (p->type == SP_LOG) memcpy(p->d, pl + 4, p->n);
    return 1;
}

void test_logger(void) {
    static uint8_t s[3000];
    for (int i = 0; i < 3000; i++) s[i] = (uint8_t)(i * 7 + i / 256);
    static uint8_t big[4096];
    logger_t L;
    pk_t p;
    bf_init(&F, mem, sizeof mem);
    logger_init(&L);

    logger_push(&L, &F, s, 100);                         /* one LOG, index 0 */
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 0); CHECK_EQ(p.n, 100); CHECK_EQ(p.first, 0);
    CHECK(memcmp(p.d, s, 100) == 0);
    CHECK(!pop(&p));
    CHECK_EQ(L.index, 100); CHECK_EQ(L.sent, 100); CHECK_EQ(L.dropped, 0);

    logger_push(&L, &F, s, 2500);                         /* chunks of <= SP_LOG_MAX */
    CHECK(pop(&p)); CHECK_EQ(p.a, 100); CHECK_EQ(p.n, 1024);
    CHECK(memcmp(p.d, s, 1024) == 0);
    CHECK(pop(&p)); CHECK_EQ(p.a, 1124); CHECK_EQ(p.n, 1024);
    CHECK(memcmp(p.d, s + 1024, 1024) == 0);
    CHECK(pop(&p)); CHECK_EQ(p.a, 2148); CHECK_EQ(p.n, 452);
    CHECK(memcmp(p.d, s + 2048, 452) == 0);
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
    CHECK(memcmp(p.d, s, 50) == 0);
    CHECK_EQ(L.pending_drop, 0);

    /* samples lost in the DMA ring */
    logger_lost(&L, 4096);
    CHECK_EQ(L.index, 2850 + 4096); CHECK_EQ(L.dropped, 200 + 4096);
    logger_push(&L, &F, s, 10);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, 6946); CHECK_EQ(p.b, 4096);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 6946); CHECK_EQ(p.n, 10);
    CHECK(memcmp(p.d, s, 10) == 0);

    /* a chunk lost while a DROP is already pending: the counts merge into one DROP */
    logger_lost(&L, 30);                                  /* pending 30 */
    CHECK_EQ(L.index, 6986); CHECK_EQ(L.pending_drop, 30);
    bf_write(&F, big, 4036);                              /* 60 bytes left */
    logger_push(&L, &F, s, 40);                           /* need 4+40+7+15 = 66 > 60: lost */
    CHECK_EQ(L.index, 7026); CHECK_EQ(L.pending_drop, 70); CHECK_EQ(L.dropped, 4326 + 40);
    bf_read(&F, big, 4036);
    logger_push(&L, &F, s, 5);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, 7026); CHECK_EQ(p.b, 70);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 7026); CHECK_EQ(p.n, 5);
    CHECK(memcmp(p.d, s, 5) == 0);
    CHECK(!pop(&p));
    CHECK_EQ(L.pending_drop, 0); CHECK_EQ(L.index, 7031);

    /* multi-chunk push: chunk 1 fits, chunk 2 is lost (need 1035 fits in 1100, not in 65 left) */
    uint32_t i1 = L.index;                                /* 7031 */
    bf_write(&F, big, 2996);                              /* 1100 bytes left */
    logger_push(&L, &F, s, 2048);
    CHECK_EQ(L.index, i1 + 1024 + 1024);
    CHECK_EQ(L.pending_drop, 1024);
    bf_read(&F, big, 2996);                               /* drop the filler */
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, i1); CHECK_EQ(p.n, 1024);
    CHECK(memcmp(p.d, s, 1024) == 0);
    CHECK(!pop(&p));
    logger_push(&L, &F, s, 3);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, i1 + 2048); CHECK_EQ(p.b, 1024);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, i1 + 2048); CHECK_EQ(p.n, 3);
    CHECK(memcmp(p.d, s, 3) == 0);
    CHECK(!pop(&p));
    CHECK_EQ(L.pending_drop, 0);

    /* need boundary, no pending DROP: a 100-sample LOG needs exactly 100 + 11 bytes */
    uint32_t i2 = L.index;
    bf_write(&F, big, 4096 - 111);                        /* 111 bytes left: exactly enough */
    logger_push(&L, &F, s, 100);
    CHECK_EQ(L.index, i2 + 100); CHECK_EQ(L.pending_drop, 0);
    bf_read(&F, big, 4096 - 111);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, i2); CHECK_EQ(p.n, 100);
    CHECK(memcmp(p.d, s, 100) == 0);
    CHECK(!pop(&p));
    bf_write(&F, big, 4096 - 110);                        /* 110 bytes left: one short */
    logger_push(&L, &F, s, 100);
    CHECK_EQ(L.index, i2 + 200); CHECK_EQ(L.pending_drop, 100);
    bf_read(&F, big, 4096 - 110);
    CHECK(!pop(&p));

    /* need boundary, with a pending DROP: 100 samples + DROP needs 100 + 26 bytes */
    uint32_t i3 = L.index;                                /* i2 + 200 */
    bf_write(&F, big, 4096 - 125);                        /* 125 bytes left: one short */
    logger_push(&L, &F, s, 100);
    CHECK_EQ(L.index, i3 + 100); CHECK_EQ(L.pending_drop, 200);
    bf_read(&F, big, 4096 - 125);
    CHECK(!pop(&p));
    bf_write(&F, big, 4096 - 126);                        /* 126 bytes left: exactly enough */
    logger_push(&L, &F, s, 100);
    CHECK_EQ(L.index, i3 + 200); CHECK_EQ(L.pending_drop, 0);
    bf_read(&F, big, 4096 - 126);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, i3 + 100); CHECK_EQ(p.b, 200);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, i3 + 100); CHECK_EQ(p.n, 100);
    CHECK(memcmp(p.d, s, 100) == 0);
    CHECK(!pop(&p));

    /* logger_push_fit: partial push returns what was consumed, never counts a loss */
    bf_init(&F, mem, sizeof mem);
    logger_init(&L);
    bf_write(&F, big, 4096 - (2 * (1024 + 11) + 500));     /* room for two full chunks, not a third */
    uint32_t k = logger_push_fit(&L, &F, s, 3000);
    CHECK_EQ(k, 2048); CHECK_EQ(L.index, 2048); CHECK_EQ(L.sent, 2048);
    CHECK_EQ(L.dropped, 0); CHECK_EQ(L.pending_drop, 0);
    bf_read(&F, big, 4096 - (2 * (1024 + 11) + 500));
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 0); CHECK_EQ(p.n, 1024);
    CHECK(memcmp(p.d, s, 1024) == 0);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 1024); CHECK_EQ(p.n, 1024);
    CHECK(memcmp(p.d, s + 1024, 1024) == 0);
    CHECK(!pop(&p));
    k = logger_push_fit(&L, &F, s + 2048, 3000 - 2048);    /* the rest: 952 samples, fits now */
    CHECK_EQ(k, 952); CHECK_EQ(L.index, 3000); CHECK_EQ(L.dropped, 0);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 2048); CHECK_EQ(p.n, 952);
    CHECK(memcmp(p.d, s + 2048, 952) == 0);
    CHECK(!pop(&p));
    /* nothing fits: 0 consumed, index unchanged, no loss */
    bf_write(&F, big, 4096 - 110);                        /* 100 samples need 111 */
    k = logger_push_fit(&L, &F, s, 100);
    CHECK_EQ(k, 0); CHECK_EQ(L.index, 3000); CHECK_EQ(L.pending_drop, 0); CHECK_EQ(L.dropped, 0);
    bf_read(&F, big, 4096 - 110);
    CHECK(!pop(&p));
    /* pending DROP goes out before the first chunk; its room counts in the fit */
    logger_lost(&L, 500);                                 /* index 3500, pending 500 */
    bf_write(&F, big, 4096 - 125);                        /* 100 samples + DROP need 126 */
    k = logger_push_fit(&L, &F, s, 100);
    CHECK_EQ(k, 0); CHECK_EQ(L.index, 3500); CHECK_EQ(L.pending_drop, 500);
    bf_read(&F, big, 4096 - 125);
    CHECK(!pop(&p));
    bf_write(&F, big, 4096 - 126);
    k = logger_push_fit(&L, &F, s, 100);
    CHECK_EQ(k, 100); CHECK_EQ(L.index, 3600); CHECK_EQ(L.pending_drop, 0); CHECK_EQ(L.dropped, 500);
    bf_read(&F, big, 4096 - 126);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, 3500); CHECK_EQ(p.b, 500);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 3500); CHECK_EQ(p.n, 100);
    CHECK(!pop(&p));
    CHECK_EQ(logger_push_fit(&L, &F, s, 0), 0);           /* n = 0: nothing queued */
    CHECK(!pop(&p));

    /* logger_put_drop: the pending DROP on its own (sent before a LOGMETA) */
    CHECK_EQ(logger_put_drop(&L, &F), 0);                 /* nothing pending: no packet */
    CHECK(!pop(&p));
    logger_lost(&L, 40);                                  /* index 3640, pending 40 */
    bf_write(&F, big, 4096 - 14);                         /* DROP needs 15 */
    CHECK_EQ(logger_put_drop(&L, &F), -1); CHECK_EQ(L.pending_drop, 40);
    bf_read(&F, big, 4096 - 14);
    CHECK(!pop(&p));
    CHECK_EQ(logger_put_drop(&L, &F), 0); CHECK_EQ(L.pending_drop, 0); CHECK_EQ(L.index, 3640);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, 3640); CHECK_EQ(p.b, 40);
    CHECK(!pop(&p));

    /* ring picker: completion counter. consumed starts at 0; completion k fills half k % 2 */
    uint32_t c = 0;
    ring_pick_t r = logger_pick(0, &c, 1000);            /* nothing completed */
    CHECK_EQ(r.first, -1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0); CHECK_EQ(c, 0);
    r = logger_pick(1, &c, 1000);                         /* first half done */
    CHECK_EQ(r.first, 0); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0); CHECK_EQ(c, 1);
    r = logger_pick(2, &c, 1000);                         /* second half done */
    CHECK_EQ(r.first, 1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0); CHECK_EQ(c, 2);
    r = logger_pick(4, &c, 1000);                         /* two more completions: newest is half 1, one older lost */
    CHECK_EQ(r.first, 1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 500); CHECK_EQ(c, 4);
    r = logger_pick(9, &c, 1000);                         /* five more: newest half 0, four older lost */
    CHECK_EQ(r.first, 0); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 2000); CHECK_EQ(c, 9);
    uint32_t w = 0xFFFFFFFFu;                             /* counter wrap: 0xFFFFFFFF -> 0 is one completion */
    r = logger_pick(0, &w, 1000);
    CHECK_EQ(r.first, 1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0); CHECK_EQ(w, 0);
}
