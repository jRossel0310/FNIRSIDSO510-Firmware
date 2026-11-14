#include "test.h"
#include "ilv.h"

void test_ilv(void) {
    uint8_t s0[4] = {0, 4, 8, 12}, s1[4] = {1, 5, 9, 13}, s2[4] = {2, 6, 10, 14}, s3[5] = {99, 3, 7, 11, 15};
    uint8_t out[17];
    memset(out, 0xEE, sizeof out);
    const uint8_t *four[4] = {s0, s1, s2, s3 + 1};      /* stream 3 skips its stale first sample */
    ilv_weave(out, four, 4, 4);
    for (int i = 0; i < 16; i++) if (out[i] != i) { CHECK_EQ(out[i], i); break; }
    CHECK_EQ(out[16], 0xEE);                             /* nothing written past streams * per */
    memset(out, 0xEE, sizeof out);
    const uint8_t *two[2] = {s0, s2};
    ilv_weave(out, two, 2, 4);
    CHECK_EQ(out[0], 0); CHECK_EQ(out[1], 2); CHECK_EQ(out[2], 4); CHECK_EQ(out[7], 14); CHECK_EQ(out[8], 0xEE);
    memset(out, 0xEE, sizeof out);
    ilv_weave(out, four, 4, 0);
    CHECK_EQ(out[0], 0xEE);
}
