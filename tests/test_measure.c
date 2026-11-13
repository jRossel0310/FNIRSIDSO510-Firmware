#include "test.h"
#include "measure.h"
#include <stdint.h>

void test_measure(void) {
    static uint8_t b[1000];
    meas_t m;
    for (int i = 0; i < 1000; i++) b[i] = (i % 100) < 30 ? 200 : 50;    /* 1 kHz @ 100 kS/s, 30 % */
    measure(b, 1000, 100000, &m);
    CHECK(m.valid);
    CHECK_NEAR(m.freq_hz, 1000.0, 0.01);
    CHECK_NEAR(m.duty_pct, 30.0, 0.01);
    CHECK_EQ(m.vmin, 50); CHECK_EQ(m.vmax, 200);

    for (int i = 0; i < 1000; i++) b[i] = (uint8_t)(128 + 90 * sin(2 * M_PI * i / 77.0));
    measure(b, 1000, 1000000, &m);
    CHECK(m.valid);
    CHECK_NEAR(m.freq_hz, 1000000.0 / 77.0, 1000000.0 / 77.0 * 0.01);
    CHECK_NEAR(m.duty_pct, 50.0, 2.0);

    memset(b, 128, sizeof b);
    measure(b, 1000, 100000, &m);
    CHECK(!m.valid);
    CHECK_EQ(m.vmin, 128); CHECK_EQ(m.vmax, 128);

    /* averaging */
    meas_avg_t a; meas_t in, out;
    meas_avg_reset(&a);
    memset(&in, 0, sizeof in);
    in.valid = 1; in.freq_hz = 1000; in.duty_pct = 50; in.vpp_codes = 100;
    meas_avg_update(&a, &in, &out);
    CHECK_NEAR(out.freq_hz, 1000.0, 1e-3);                 /* first capture passes through */
    in.freq_hz = 1050; in.vpp_codes = 104;
    meas_avg_update(&a, &in, &out);
    CHECK_NEAR(out.freq_hz, 1025.0, 1e-3);                 /* mean of two */
    CHECK_NEAR(out.vpp_codes, 102.0, 1e-3);
    in.freq_hz = 1000; in.vpp_codes = 100;
    for (int i = 0; i < 20; i++) meas_avg_update(&a, &in, &out);
    CHECK_NEAR(out.freq_hz, 1000.0, 1.0);                  /* converges */
    CHECK_EQ(a.n, MEAS_AVG_N);
    in.freq_hz = 1080;                                     /* 8 % jitter: averaged, 1/8 weight */
    meas_avg_update(&a, &in, &out);
    CHECK(out.freq_hz > 1005.0 && out.freq_hz < 1015.0);
    in.freq_hz = 2000;                                     /* real change: restart */
    meas_avg_update(&a, &in, &out);
    CHECK_NEAR(out.freq_hz, 2000.0, 1e-3); CHECK_EQ(a.n, 1);
    in.vpp_codes = 150;                                    /* amplitude jump: restart */
    meas_avg_update(&a, &in, &out);
    CHECK_NEAR(out.vpp_codes, 150.0, 1e-3); CHECK_EQ(a.n, 1);
    in.valid = 0;                                          /* invalid: pass through, forget */
    meas_avg_update(&a, &in, &out);
    CHECK(!out.valid); CHECK_EQ(a.n, 0);
}
