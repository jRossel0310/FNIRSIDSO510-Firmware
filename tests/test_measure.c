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

    /* ---- twelve measurements ---- */
    static uint8_t r[4800];
    float v; int u;
    const float vpc = 0.02f, zero = 28.0f;

    /* square wave, 30 % high, starting low; first rising edge at index 20 */
    for (int i = 0; i < 4800; i++) r[i] = ((i + 80) % 100) < 30 ? 200 : 50;
    measure(r, 4800, 100000, &m);
    CHECK(m.valid);
    CHECK_NEAR(m.freq_hz, 1000.0, 10.0);
    CHECK_NEAR(m.max_codes, 200.0, 0.01); CHECK_NEAR(m.min_codes, 50.0, 0.01);
    CHECK_NEAR(m.top_codes, 200.0, 0.01); CHECK_NEAR(m.base_codes, 50.0, 0.01);
    CHECK(meas_value(&m, MEAS_FREQ, vpc, zero, &v, &u));    CHECK_NEAR(v, 1000.0, 10.0);   CHECK_EQ(u, MEAS_U_HZ);
    CHECK(meas_value(&m, MEAS_PERIOD, vpc, zero, &v, &u));  CHECK_NEAR(v, 1e-3, 1e-5);     CHECK_EQ(u, MEAS_U_S);
    CHECK(meas_value(&m, MEAS_DUTY_P, vpc, zero, &v, &u));  CHECK_NEAR(v, 30.0, 0.3);      CHECK_EQ(u, MEAS_U_PCT);
    CHECK(meas_value(&m, MEAS_DUTY_N, vpc, zero, &v, &u));  CHECK_NEAR(v, 70.0, 0.7);      CHECK_EQ(u, MEAS_U_PCT);
    CHECK(meas_value(&m, MEAS_WIDTH_P, vpc, zero, &v, &u)); CHECK_NEAR(v, 3e-4, 3e-6);    CHECK_EQ(u, MEAS_U_S);
    CHECK(meas_value(&m, MEAS_WIDTH_N, vpc, zero, &v, &u)); CHECK_NEAR(v, 7e-4, 7e-6);    CHECK_EQ(u, MEAS_U_S);
    CHECK(meas_value(&m, MEAS_MAX, vpc, zero, &v, &u));     CHECK_NEAR(v, 3.44, 0.0344);   CHECK_EQ(u, MEAS_U_V);
    CHECK(meas_value(&m, MEAS_MIN, vpc, zero, &v, &u));     CHECK_NEAR(v, 0.44, 0.0044);   CHECK_EQ(u, MEAS_U_V);
    CHECK(meas_value(&m, MEAS_VPP, vpc, zero, &v, &u));     CHECK_NEAR(v, 3.0, 0.03);      CHECK_EQ(u, MEAS_U_V);
    CHECK(meas_value(&m, MEAS_AMPL, vpc, zero, &v, &u));    CHECK_NEAR(v, 3.0, 0.03);      CHECK_EQ(u, MEAS_U_V);
    CHECK(meas_value(&m, MEAS_AVG, vpc, zero, &v, &u));     CHECK_NEAR(v, 1.34, 0.0134);   CHECK_EQ(u, MEAS_U_V);
    CHECK(meas_value(&m, MEAS_RMS, vpc, zero, &v, &u));     CHECK_NEAR(v, 1.9198, 0.01);   CHECK_EQ(u, MEAS_U_V);

    /* sine: 60 codes around 128, 48 samples per period */
    for (int i = 0; i < 4800; i++) r[i] = (uint8_t)(128.5 + 60 * sin(2 * M_PI * i / 48.0));
    measure(r, 4800, 48000, &m);
    CHECK(m.valid);
    CHECK(meas_value(&m, MEAS_AVG, vpc, 128.0f, &v, &u)); CHECK_NEAR(v, 0.0, 0.02);
    CHECK(meas_value(&m, MEAS_RMS, vpc, 128.0f, &v, &u)); CHECK_NEAR(v, 60 / sqrt(2.0) * 0.02, 0.02);
    CHECK(meas_value(&m, MEAS_VPP, vpc, 128.0f, &v, &u)); CHECK_NEAR(v, 2.4, 0.05);
    CHECK(meas_value(&m, MEAS_AMPL, vpc, 128.0f, &v, &u)); CHECK_NEAR(v, 2.4, 0.1);

    /* flat record */
    memset(r, 130, sizeof r);
    measure(r, 4800, 100000, &m);
    CHECK(!m.valid);
    float flat = (130 - zero) * vpc;
    for (int k = MEAS_FREQ; k <= MEAS_WIDTH_N; k++) {
        v = 123; u = -1;
        CHECK(!meas_value(&m, k, vpc, zero, &v, &u));
        CHECK_EQ(u, k == MEAS_FREQ ? MEAS_U_HZ : k == MEAS_PERIOD ? MEAS_U_S :
                    (k == MEAS_DUTY_P || k == MEAS_DUTY_N) ? MEAS_U_PCT : MEAS_U_S);
    }
    CHECK(meas_value(&m, MEAS_MAX, vpc, zero, &v, &u)); CHECK_NEAR(v, flat, 1e-4);
    CHECK(meas_value(&m, MEAS_MIN, vpc, zero, &v, &u)); CHECK_NEAR(v, flat, 1e-4);
    CHECK(meas_value(&m, MEAS_AVG, vpc, zero, &v, &u)); CHECK_NEAR(v, flat, 1e-4);
    CHECK(meas_value(&m, MEAS_VPP, vpc, zero, &v, &u)); CHECK_NEAR(v, 0.0, 1e-6);
    CHECK(meas_value(&m, MEAS_AMPL, vpc, zero, &v, &u)); CHECK_NEAR(v, 0.0, 1e-6);
    CHECK(meas_value(&m, MEAS_RMS, vpc, zero, &v, &u)); CHECK_NEAR(v, flat, 1e-3);

    /* half a period: one ramp through the mid level */
    for (int i = 0; i < 200; i++) r[i] = (uint8_t)(50 + i * 150 / 199);
    measure(r, 200, 100000, &m);
    CHECK(!m.valid);
    CHECK(!meas_value(&m, MEAS_PERIOD, vpc, zero, &v, &u));
    CHECK(!meas_value(&m, MEAS_WIDTH_P, vpc, zero, &v, &u));
    CHECK(meas_value(&m, MEAS_MAX, vpc, zero, &v, &u)); CHECK_NEAR(v, 3.44, 0.01);
    CHECK(meas_value(&m, MEAS_MIN, vpc, zero, &v, &u)); CHECK_NEAR(v, 0.44, 0.01);
    CHECK(meas_value(&m, MEAS_VPP, vpc, zero, &v, &u)); CHECK_NEAR(v, 3.0, 0.01);

    /* empty record: new fields are 0 */
    measure(r, 0, 100000, &m);
    CHECK(!m.valid);
    CHECK_NEAR(m.max_codes, 0, 0); CHECK_NEAR(m.min_codes, 0, 0);
    CHECK_NEAR(m.top_codes, 0, 0); CHECK_NEAR(m.base_codes, 0, 0);
    CHECK_NEAR(m.mean_codes, 0, 0); CHECK_NEAR(m.meansq_codes, 0, 0);

    /* unknown kinds, names, tags */
    CHECK(!meas_value(&m, -1, vpc, zero, &v, &u));
    CHECK(!meas_value(&m, MEAS_NUM, vpc, zero, &v, &u));
    CHECK_STR(meas_name(-1), "?"); CHECK_STR(meas_name(MEAS_NUM), "?");
    CHECK_STR(meas_tag(-1), "?");  CHECK_STR(meas_tag(MEAS_NUM), "?");
    static const char *const nm[MEAS_NUM] = { "Frequency", "Period", "Duty +", "Duty -", "Width +", "Width -",
        "Maximum", "Minimum", "Peak-peak", "Amplitude", "RMS", "Average" };
    static const char *const tg[MEAS_NUM] = { "Freq", "Per", "Duty+", "Duty-", "Wid+", "Wid-",
        "Max", "Min", "Vpp", "Ampl", "RMS", "Avg" };
    for (int k = 0; k < MEAS_NUM; k++) { CHECK_STR(meas_name(k), nm[k]); CHECK_STR(meas_tag(k), tg[k]); }

    /* averaging of the new fields */
    meas_avg_reset(&a);
    memset(&in, 0, sizeof in);
    in.valid = 1; in.freq_hz = 1000; in.duty_pct = 50; in.vpp_codes = 100;
    in.mean_codes = 100; in.max_codes = 180; in.top_codes = 170; in.meansq_codes = 10000;
    meas_avg_update(&a, &in, &out);
    CHECK_NEAR(out.mean_codes, 100.0, 1e-3);
    in.mean_codes = 110; in.max_codes = 190; in.top_codes = 180; in.meansq_codes = 12000;
    meas_avg_update(&a, &in, &out);
    CHECK_NEAR(out.mean_codes, 105.0, 1e-3);
    CHECK_NEAR(out.max_codes, 185.0, 1e-3);
    CHECK_NEAR(out.top_codes, 175.0, 1e-3);
    CHECK_NEAR(out.meansq_codes, 11000.0, 1e-2);
    in.freq_hz = 1500; in.mean_codes = 90;
    meas_avg_update(&a, &in, &out);
    CHECK_NEAR(out.mean_codes, 90.0, 1e-3); CHECK_EQ(a.n, 1);
}
