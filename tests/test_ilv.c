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

    /* despike: isolated spikes go, everything else stays */
    uint8_t r1[12] = {64, 64, 88, 63, 62, 62, 47, 64, 64, 64, 64, 64};
    CHECK_EQ(ilv_despike(r1, 12), 2);
    CHECK_EQ(r1[2], 64); CHECK_EQ(r1[6], 63); CHECK_EQ(r1[1], 64); CHECK_EQ(r1[3], 63); CHECK_EQ(r1[7], 64);
    uint8_t edge[8] = {50, 50, 50, 120, 190, 190, 190, 190};          /* a fast edge is not a spike */
    CHECK_EQ(ilv_despike(edge, 8), 0); CHECK_EQ(edge[3], 120);
    uint8_t small[6] = {100, 100, 107, 100, 100, 100};                 /* below the threshold */
    CHECK_EQ(ilv_despike(small, 6), 0); CHECK_EQ(small[2], 107);
    uint8_t slope[6] = {100, 104, 120, 109, 113, 117};                 /* neighbours 5 apart: left alone */
    CHECK_EQ(ilv_despike(slope, 6), 0);
    uint8_t ends[4] = {200, 60, 60, 200};                              /* first and last sample are never touched */
    CHECK_EQ(ilv_despike(ends, 4), 0); CHECK_EQ(ends[0], 200); CHECK_EQ(ends[3], 200);
    uint8_t pair[2] = {0, 255};
    CHECK_EQ(ilv_despike(pair, 2), 0); CHECK_EQ(ilv_despike(pair, 0), 0);
    uint8_t wide[7] = {64, 64, 88, 88, 64, 64, 64};                    /* two samples wide: a real pulse, kept */
    CHECK_EQ(ilv_despike(wide, 7), 0);
}
