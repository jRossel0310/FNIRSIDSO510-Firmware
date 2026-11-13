#include "test.h"
#include "autoset.h"
#include "units.h"

static int V, O;
static int fit(int v, int o, int mn, int mx) { V = v; O = o; return autoset_fit(&V, &O, mn, mx); }

static double sig_lo, sig_hi;
static int model_code(int off, int vi, double v) {
    double c = 128 + off + v * 25.0 / (vdiv_mv[vi] / 1000.0);
    return c < 0 ? 0 : c > 255 ? 255 : (int)lround(c);
}
/* Is there any range/offset that shows the whole signal inside the screen? */
static int achievable(void) {
    for (int i = 0; i < NUM_VDIV; i++) for (int o = -100; o <= 100; o++)
        if (model_code(o, i, sig_lo) >= SCREEN_CODE_LO && model_code(o, i, sig_hi) <= SCREEN_CODE_HI) return 1;
    return 0;
}
/* Closed-loop auto-set against an ideal front end, as do_autoset runs it. */
static void closed_loop(double lo, double hi) {
    sig_lo = lo; sig_hi = hi;
    for (int start = 0; start < NUM_VDIV; start++) {
        int v = start, o = 0, it, again = 1;
        for (it = 0; it < AUTOSET_MAX_ITER && again; it++)
            again = autoset_fit(&v, &o, model_code(o, v, lo), model_code(o, v, hi));
        CHECK(!again);
        int mn = model_code(o, v, lo), mx = model_code(o, v, hi);
        if (achievable()) { CHECK(mn >= SCREEN_CODE_LO); CHECK(mx <= SCREEN_CODE_HI); }
        if (!(mn >= SCREEN_CODE_LO && mx <= SCREEN_CODE_HI) && achievable())
            printf("  closed loop %g..%g from %d -> v%d o%d codes %d..%d\n", lo, hi, start, v, o, mn, mx);
    }
}

void test_autoset(void) {
    closed_loop(-3.5, 5.3);
    closed_loop(0.0, 12.0);
    closed_loop(0.4, 2.9);
    closed_loop(2.88, 3.0);
    closed_loop(-0.01, 0.01);
    closed_loop(-5.0, 5.0);
    closed_loop(-20.0, 20.0);

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
    /* any clip: jump 3 ranges less sensitive, offset back to 0 (clamped at 10 V/div) */
    CHECK_EQ(fit(6, 0, 100, 255), 1); CHECK_EQ(V, 9); CHECK_EQ(O, 0);
    CHECK_EQ(fit(4, -80, 100, 255), 1); CHECK_EQ(V, 7); CHECK_EQ(O, 0);
    CHECK_EQ(fit(4, 10, 0, 90), 1); CHECK_EQ(V, 7); CHECK_EQ(O, 0);
    CHECK_EQ(fit(3, 0, 0, 255), 1); CHECK_EQ(V, 6); CHECK_EQ(O, 0);
    CHECK_EQ(fit(9, 50, 0, 255), 1); CHECK_EQ(V, 9); CHECK_EQ(O, 0);
    CHECK_EQ(fit(9, 0, 0, 255), 0); CHECK_EQ(V, 9);
    /* flat at 0 V -> most sensitive range, centred; flat DC level is fitted like any other */
    CHECK_EQ(fit(4, 0, 128, 128), 1); CHECK_EQ(V, 0); CHECK_EQ(O, 0);
    CHECK_EQ(fit(4, 7, 128, 128), 1); CHECK_EQ(V, 0); CHECK_EQ(O, 100);
    /* offset jitter of <= 2 codes counts as converged */
    CHECK_EQ(fit(5, -80, 68, 192), 0); CHECK_EQ(O, -82);

    /* 1 kHz: need 10*tdiv >= 2 ms -> 200 us/div (idx 8) */
    CHECK_EQ(tdiv_ns[autoset_tdiv(1000.0f)], 200000);
    CHECK_EQ(tdiv_ns[autoset_tdiv(1e6f)], 500);
    CHECK_EQ(tdiv_ns[autoset_tdiv(10.0f)], 20000000);   /* 200 ms screen = 2 periods */
    CHECK_EQ(tdiv_ns[autoset_tdiv(1.0f)], 50000000);    /* clamped to slowest */
    CHECK_EQ(autoset_tdiv(0.0f), -1);
}
