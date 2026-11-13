#include "test.h"
#include "units.h"

void test_units(void) {
    rate_cfg_t r;
    r = units_rate_for_tdiv(500);          /* capped at 12 MS/s */
    CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 19); CHECK_EQ(r.rate_hz, 12000000);
    r = units_rate_for_tdiv(10000);        /* 120 samples/div at 12 MS/s */
    CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 19); CHECK_EQ(r.rate_hz, 12000000);
    r = units_rate_for_tdiv(20000);
    CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 39); CHECK_EQ(r.rate_hz, 6000000);
    r = units_rate_for_tdiv(20000000);
    CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 39999); CHECK_EQ(r.rate_hz, 6000);
    r = units_rate_for_tdiv(50000000);
    CHECK_EQ(r.psc, 99); CHECK_EQ(r.arr, 999); CHECK_EQ(r.rate_hz, 2400);

    CHECK_EQ(units_samples_per_screen(12000000, 500), 60);
    CHECK_EQ(units_samples_per_screen(12000000, 10000), 1200);
    CHECK_EQ(units_samples_per_screen(2400, 50000000), 1200);

    CHECK_EQ(vdiv_mv[0], 10); CHECK_EQ(vdiv_mv[NUM_VDIV - 1], 10000);
    CHECK_EQ(tdiv_ns[TDIV_10US], 10000); CHECK_EQ(tdiv_ns[TDIV_1MS], 1000000);

    CHECK_NEAR(units_code_to_volts(153, 6, 0, 0), 1.0, 1e-6);      /* +1 div at 1 V/div */
    CHECK_NEAR(units_code_to_volts(153, 6, 0, 1), 10.0, 1e-5);     /* probe x10 */
    CHECK_NEAR(units_code_to_volts(153, 6, 25, 0), 0.0, 1e-6);     /* offset moves 0 V */
    CHECK_NEAR(units_code_to_volts(103, 0, 0, 0), -0.010, 1e-7);   /* -1 div at 10 mV/div */
    CHECK_NEAR(units_volts_per_code(6, 0), 0.04, 1e-7);

    CHECK_EQ(units_batt_pct(4.10f), 100);
    CHECK_EQ(units_batt_pct(3.95f), 80);
    CHECK_EQ(units_batt_pct(3.80f), 50);
    CHECK_EQ(units_batt_pct(3.50f), 10);
    /* offset settle time: tau ~0.33 s, until < 0.5 code residual */
    CHECK_EQ(fe_settle_ms(0), 40);
    CHECK_EQ(fe_settle_ms(1), 40);
    CHECK_EQ(fe_settle_ms(-1), 40);
    CHECK_EQ(fe_settle_ms(2), 40 + 2 * 243);          /* 2d = 4 -> 2 doublings */
    CHECK_EQ(fe_settle_ms(100), 40 + 8 * 243);        /* 200 -> 8 doublings: ~2 s */
    CHECK_EQ(fe_settle_ms(-100), fe_settle_ms(100));
    CHECK_EQ(fe_settle_ms(200), 40 + 9 * 243);        /* full -100..+100 swing */
}
