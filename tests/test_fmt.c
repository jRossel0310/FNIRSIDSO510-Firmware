#include "test.h"
#include "fmt.h"

void test_fmt(void) {
    char b[16];
    CHECK_STR(fmt_si(b, 1000.0f, "Hz", 0), "1.00kHz");
    CHECK_STR(fmt_si(b, 3.3f, "V", 0), "3.30V");
    CHECK_STR(fmt_si(b, -0.025f, "V", 0), "-25.0mV");
    CHECK_STR(fmt_si(b, 0.0f, "V", 0), "0.00V");
    CHECK_STR(fmt_si(b, 999.6f, "Hz", 0), "1.00kHz");
    CHECK_STR(fmt_si(b, 123456.0f, "Hz", 0), "123kHz");
    CHECK_STR(fmt_si(b, 500e-9f, "s", 1), "500ns");
    CHECK_STR(fmt_si(b, 1e-6f, "s", 1), "1us");
    CHECK_STR(fmt_si(b, 2.5e-3f, "s", 1), "2.5ms");
    CHECK_STR(fmt_si(b, 10.0f, "V", 1), "10V");
    CHECK_STR(fmt_si(b, 0.01f, "V", 1), "10mV");
    CHECK_STR(fmt_si(b, 100.0f, "V", 1), "100V");
}
