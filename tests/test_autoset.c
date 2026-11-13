#include "test.h"
#include "autoset.h"
#include "units.h"

static int V, O;
static int fit(int v, int o, int mn, int mx) { V = v; O = o; return autoset_fit(&V, &O, mn, mx); }

void test_autoset(void) {
    /* 1 V/div (idx 6), 2 codes pk-pk around 0 V = 80 mV -> 20 mV/div, offset stays 0 */
    CHECK_EQ(fit(6, 0, 127, 129), 1); CHECK_EQ(V, 1); CHECK_EQ(O, 0);
    /* 3.3 V pk-pk centred-ish at 1 V/div: stays 1 V/div; mid 0.12 V -> offset -3 */
    CHECK_EQ(fit(6, 0, 90, 172), 1); CHECK_EQ(V, 6); CHECK_EQ(O, -3);
    /* DC-biased sine 0.4..2.88 V at 1 V/div: 500 mV/div, centred with offset -82 */
    CHECK_EQ(fit(6, 0, 138, 200), 1); CHECK_EQ(V, 5); CHECK_EQ(O, -82);
    /* ...and the next capture at that setting is converged */
    CHECK_EQ(fit(5, -82, 66, 190), 0); CHECK_EQ(V, 5); CHECK_EQ(O, -82);
    /* ~2.9 V DC with 120 mV ripple: 20 mV/div can't reach it with +-100 codes of offset;
       500 mV/div with offset -100 is the most sensitive range that keeps it on screen */
    CHECK_EQ(fit(6, 0, 200, 203), 1); CHECK_EQ(V, 5); CHECK_EQ(O, -100);
    /* ADC clipped at the top only -> shift offset down 50 codes, same range */
    CHECK_EQ(fit(6, 0, 100, 255), 1); CHECK_EQ(V, 6); CHECK_EQ(O, -50);
    CHECK_EQ(fit(6, -80, 100, 255), 1); CHECK_EQ(O, -100);
    /* top clipped with offset already at -100 -> one step less sensitive */
    CHECK_EQ(fit(6, -100, 100, 255), 1); CHECK_EQ(V, 7); CHECK_EQ(O, -100);
    /* bottom clipped only -> shift up */
    CHECK_EQ(fit(4, 10, 0, 90), 1); CHECK_EQ(V, 4); CHECK_EQ(O, 60);
    /* clipped both rails -> one step less sensitive; at 10 V/div nothing more to do */
    CHECK_EQ(fit(3, 0, 0, 255), 1); CHECK_EQ(V, 4);
    CHECK_EQ(fit(9, 0, 0, 255), 0); CHECK_EQ(V, 9);
    /* flat -> unchanged */
    CHECK_EQ(fit(4, 7, 128, 128), 0); CHECK_EQ(V, 4); CHECK_EQ(O, 7);
    /* offset jitter of <= 2 codes counts as converged */
    CHECK_EQ(fit(5, -80, 68, 192), 0); CHECK_EQ(O, -82);

    /* 1 kHz: need 10*tdiv >= 2 ms -> 200 us/div (idx 8) */
    CHECK_EQ(tdiv_ns[autoset_tdiv(1000.0f)], 200000);
    CHECK_EQ(tdiv_ns[autoset_tdiv(1e6f)], 500);
    CHECK_EQ(tdiv_ns[autoset_tdiv(10.0f)], 20000000);   /* 200 ms screen = 2 periods */
    CHECK_EQ(tdiv_ns[autoset_tdiv(1.0f)], 50000000);    /* clamped to slowest */
    CHECK_EQ(autoset_tdiv(0.0f), -1);
}
