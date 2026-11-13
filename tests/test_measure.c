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
}
