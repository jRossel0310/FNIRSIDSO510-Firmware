#include "test.h"
#include "autoset.h"
#include "units.h"

void test_autoset(void) {
    /* 1 V/div (idx 6), 2 codes pk-pk = 80 mV -> needs 20 mV/div (80 mV <= 6 div * 20 mV) */
    CHECK_EQ(autoset_vdiv(6, 127, 129), 1);
    /* 3.3 V pk-pk at 1 V/div = 82.5 codes -> smallest with 3300 <= 6 * vdiv: 1 V (idx 6) */
    CHECK_EQ(autoset_vdiv(6, 90, 172), 6);
    /* clipped -> one step less sensitive */
    CHECK_EQ(autoset_vdiv(3, 0, 255), 4);
    CHECK_EQ(autoset_vdiv(9, 0, 255), 9);
    /* flat -> unchanged */
    CHECK_EQ(autoset_vdiv(4, 128, 128), 4);

    /* 1 kHz: need 10*tdiv >= 2 ms -> 200 us/div (idx 8) */
    CHECK_EQ(tdiv_ns[autoset_tdiv(1000.0f)], 200000);
    CHECK_EQ(tdiv_ns[autoset_tdiv(1e6f)], 500);
    CHECK_EQ(tdiv_ns[autoset_tdiv(10.0f)], 20000000);   /* 200 ms screen = 2 periods */
    CHECK_EQ(tdiv_ns[autoset_tdiv(1.0f)], 50000000);    /* clamped to slowest */
    CHECK_EQ(autoset_tdiv(0.0f), -1);
}
