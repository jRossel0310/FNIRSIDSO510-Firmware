#include "test.h"
#include "csv.h"

static csv_meta_t base(void) {
    csv_meta_t m;
    memset(&m, 0, sizeof m);
    m.rate_hz = 1000000; m.vdiv_mv = 1000; m.coupling_dc = 1; m.probe_x10 = 0;
    m.trig_mode = 0; m.edge = 0; m.zero_code = 128; m.trig_level_code = 153;
    m.trig_index = 100; m.triggered = 1; m.offset_pwm = 2033;
    return m;
}

void test_csv(void) {
    char b[600];
    csv_meta_t m = base();
    int n = csv_header(b, sizeof b, &m);
    CHECK(n > 0);
    b[n > 0 ? n : 0] = 0;
    CHECK_STR(b,
        "# DSO-510 custom firmware 0.3.0\n"
        "# sample_rate_hz=1000000\n"
        "# volts_per_div=1.0000\n"
        "# coupling=DC\n"
        "# probe=1\n"
        "# trigger=Auto,rising,1.0000\n"
        "# trigger_index=100\n"
        "# triggered=1\n"
        "# cal=offset_pwm=2033,volts_per_code=0.040000\n"
        "time_s,volts,adc_code\n");
    CHECK_EQ(csv_header(b, 10, &m), -1);                       /* too small */

    m.coupling_dc = 0; m.probe_x10 = 1; m.trig_mode = 2; m.edge = 1; m.vdiv_mv = 10;
    m.trig_level_code = 103;
    n = csv_header(b, sizeof b, &m); b[n] = 0;
    CHECK(strstr(b, "# volts_per_div=0.1000\n") != 0);        /* 10 mV x10 probe */
    CHECK(strstr(b, "# coupling=AC\n") != 0);
    CHECK(strstr(b, "# probe=10\n") != 0);
    CHECK(strstr(b, "# trigger=Single,falling,-0.1000\n") != 0);   /* -25 codes * 4 mV */
    CHECK(strstr(b, "volts_per_code=0.004000\n") != 0);
    m = base(); m.triggered = 0;                                /* untriggered Auto capture */
    n = csv_header(b, sizeof b, &m); b[n] = 0;
    CHECK(strstr(b, "# trigger_index=100\n# triggered=0\n# cal=") != 0);
    m = base(); m.trig_mode = 1;
    n = csv_header(b, sizeof b, &m); b[n] = 0;
    CHECK(strstr(b, "# trigger=Normal,rising,") != 0);

    m = base();
    n = csv_row(b, &m, 100, 128); b[n] = 0; CHECK_STR(b, "0.000000000,0.0000,128\n");
    n = csv_row(b, &m, 99, 153);  b[n] = 0; CHECK_STR(b, "-0.000001000,1.0000,153\n");
    n = csv_row(b, &m, 101, 103); b[n] = 0; CHECK_STR(b, "0.000001000,-1.0000,103\n");
    n = csv_row(b, &m, 1100, 0);  b[n] = 0; CHECK_STR(b, "0.001000000,-5.1200,0\n");
    m.rate_hz = 12000000;                                       /* 83.33 ns per sample */
    n = csv_row(b, &m, 101, 128); b[n] = 0; CHECK_STR(b, "0.000000083,0.0000,128\n");
    n = csv_row(b, &m, 99, 128);  b[n] = 0; CHECK_STR(b, "-0.000000083,0.0000,128\n");
    m.rate_hz = 2400; m.trig_index = 4800;
    n = csv_row(b, &m, 0, 128);   b[n] = 0; CHECK_STR(b, "-2.000000000,0.0000,128\n");
    m = base(); m.vdiv_mv = 20;                                 /* 800 uV per code */
    n = csv_row(b, &m, 100, 129); b[n] = 0; CHECK_STR(b, "0.000000000,0.0008,129\n");
    m.vdiv_mv = 10;                                             /* 400 uV per code: 0.0004 */
    n = csv_row(b, &m, 100, 127); b[n] = 0; CHECK_STR(b, "0.000000000,-0.0004,127\n");
    m = base(); m.vdiv_mv = 10000; m.probe_x10 = 1; m.zero_code = 255; /* worst-case width */
    n = csv_row(b, &m, -4700, 0);
    CHECK(n <= CSV_ROW_MAX);
    b[n] = 0; CHECK_STR(b, "-0.004800000,-1020.0000,0\n");
}
