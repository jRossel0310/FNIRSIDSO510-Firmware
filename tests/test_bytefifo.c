#include "test.h"
#include "bytefifo.h"

void test_bytefifo(void) {
    static uint8_t mem[8];
    bytefifo_t f;
    uint8_t out[16];
    bf_init(&f, mem, sizeof mem);
    CHECK_EQ(bf_used(&f), 0); CHECK_EQ(bf_space(&f), 8);
    const uint8_t a[5] = {1, 2, 3, 4, 5};
    CHECK_EQ(bf_write(&f, a, 5), 5);
    CHECK_EQ(bf_used(&f), 5); CHECK_EQ(bf_space(&f), 3);
    CHECK_EQ(bf_read(&f, out, 3), 3);
    CHECK_EQ(out[0], 1); CHECK_EQ(out[2], 3);
    const uint8_t b[6] = {6, 7, 8, 9, 10, 11};
    CHECK_EQ(bf_write(&f, b, 6), 6);                  /* wraps around the end */
    CHECK_EQ(bf_used(&f), 8); CHECK_EQ(bf_space(&f), 0);
    CHECK_EQ(bf_write(&f, a, 1), 0);                  /* full: nothing written */
    CHECK_EQ(bf_read(&f, out, 16), 8);                /* only what is there */
    CHECK_EQ(out[0], 4); CHECK_EQ(out[1], 5); CHECK_EQ(out[2], 6); CHECK_EQ(out[7], 11);
    CHECK_EQ(bf_read(&f, out, 4), 0);
    CHECK_EQ(bf_write(&f, b, 6), 6);
    CHECK_EQ(bf_write(&f, a, 5), 2);                  /* partial: only what fits */
    bf_clear(&f);
    CHECK_EQ(bf_used(&f), 0); CHECK_EQ(bf_space(&f), 8);
    /* free-running counters wrap past 2^32 */
    f.head = f.tail = 0xFFFFFFFCu;
    CHECK_EQ(bf_write(&f, b, 6), 6);
    CHECK_EQ(bf_used(&f), 6);
    CHECK_EQ(bf_read(&f, out, 6), 6);
    CHECK_EQ(out[0], 6); CHECK_EQ(out[5], 11);
    CHECK_EQ(bf_used(&f), 0);
}
