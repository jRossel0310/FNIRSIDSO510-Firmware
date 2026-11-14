#include "test.h"
#include "stream_proto.h"
#include <stdlib.h>

static uint8_t mem[8192];
static bytefifo_t F;
static uint8_t out[8192];

/* Canonical inputs, also hard-coded in tools/tests/test_dsoproto.py. */
static const uint8_t FRAME_S[8] = {46, 71, 96, 121, 146, 121, 96, 71};
static csv_meta_t frame_meta(void) {
    csv_meta_t m;
    memset(&m, 0, sizeof m);
    m.rate_hz = 600000; m.vdiv_mv = 500; m.coupling_dc = 1; m.probe_x10 = 0; m.trig_mode = 0;
    m.edge = 0; m.triggered = 1; m.zero_code = 46; m.trig_level_code = 75; m.trig_index = 3;
    m.offset_pwm = 1630;
    return m;
}

static uint32_t drain(void) { return bf_read(&F, out, sizeof out); }

/* Compare out[0..n) with tests/vectors/<name>; with WRITE_VECTORS=1 in the environment, write it. */
static void vector(const char *name, const uint8_t *p, uint32_t n) {
    char path[96];
    snprintf(path, sizeof path, "vectors/%s", name);
    if (getenv("WRITE_VECTORS")) {
        FILE *f = fopen(path, "wb");
        CHECK(f != 0);
        if (f) { fwrite(p, 1, n, f); fclose(f); }
        return;
    }
    static uint8_t ref[16384];
    FILE *f = fopen(path, "rb");
    CHECK(f != 0);
    if (!f) return;
    size_t m = fread(ref, 1, sizeof ref, f);
    fclose(f);
    CHECK_EQ(m, n);
    CHECK(m == n && memcmp(ref, p, n) == 0);
}

static uint32_t put_info(void) {
    char t[160];
    uint32_t n = sp_info(t, sizeof t, "idle");
    CHECK_EQ(sp_put(&F, SP_INFO, (const uint8_t *)t, n, 0, 0), 0);
    return drain();
}
static uint32_t put_frame(void) {
    uint8_t h[SP_FRAME_META + 2];
    csv_meta_t m = frame_meta();
    uint32_t hl = sp_frame_meta(h, &m, 8);
    CHECK_EQ(hl, SP_FRAME_META + 2);
    CHECK_EQ(sp_put(&F, SP_FRAME, h, hl, FRAME_S, 8), 0);
    return drain();
}
static uint32_t put_logmeta(void) {
    uint8_t h[SP_LOGMETA_LEN];
    uint32_t hl = sp_logmeta(h, 100000, 1000, 0, 1, 128, 2033);
    CHECK_EQ(hl, SP_LOGMETA_LEN);
    CHECK_EQ(sp_put(&F, SP_LOGMETA, h, hl, 0, 0), 0);
    return drain();
}
static uint32_t put_log(uint8_t flip) {
    uint8_t h[8], s[16];
    sp_u32x2(h, 1000, 0);
    for (int i = 0; i < 16; i++) s[i] = (uint8_t)(120 + i);
    CHECK_EQ(sp_put(&F, SP_LOG, h, 4, s, 16), 0);
    uint32_t n = drain();
    out[9] ^= flip;                                    /* corrupt one payload byte if asked */
    return n;
}
static uint32_t put_drop(void) {
    uint8_t h[8];
    sp_u32x2(h, 2048, 512);
    CHECK_EQ(sp_put(&F, SP_DROP, h, 8, 0, 0), 0);
    return drain();
}
static uint32_t put_text(uint8_t type, const char *t) {
    CHECK_EQ(sp_put(&F, type, (const uint8_t *)t, (uint32_t)strlen(t), 0, 0), 0);
    return drain();
}

void test_stream_proto(void) {
    bf_init(&F, mem, sizeof mem);
    /* CRC-16/CCITT-FALSE check value */
    CHECK_EQ(sp_crc16(0xFFFF, (const uint8_t *)"123456789", 9), 0x29B1);

    /* exact framing of a tiny packet: A5 5A 06 02 00 'O' 'K' crc_lo crc_hi */
    uint32_t n = put_text(SP_OK, "OK");
    CHECK_EQ(n, 9);
    CHECK_EQ(out[0], 0xA5); CHECK_EQ(out[1], 0x5A); CHECK_EQ(out[2], SP_OK);
    CHECK_EQ(out[3], 2); CHECK_EQ(out[4], 0); CHECK_EQ(out[5], 'O'); CHECK_EQ(out[6], 'K');
    uint16_t c = sp_crc16(0xFFFF, out + 2, 5);
    CHECK_EQ(out[7], c & 0xFF); CHECK_EQ(out[8], c >> 8);

    /* all or nothing */
    static uint8_t small_mem[16];
    bytefifo_t s;
    bf_init(&s, small_mem, sizeof small_mem);
    uint8_t big[10] = {0};
    CHECK_EQ(sp_put(&s, SP_LOG, big, 10, 0, 0), -1);   /* needs 17 > 16 */
    CHECK_EQ(bf_used(&s), 0);
    CHECK_EQ(sp_put(&s, SP_LOG, big, 9, 0, 0), 0);     /* exactly 16 */
    CHECK_EQ(bf_used(&s), 16);

    /* golden vectors */
    vector("info.bin", out, put_info());
    vector("frame.bin", out, put_frame());
    vector("logmeta.bin", out, put_logmeta());
    vector("log.bin", out, put_log(0));
    vector("drop.bin", out, put_drop());
    vector("ok.bin", out, put_text(SP_OK, "LOG 100000"));
    vector("err.bin", out, put_text(SP_ERR, "bad rate"));
    /* a stream: junk, then every packet, with one corrupted LOG before the good one */
    static uint8_t st[4096];
    uint32_t k = 0;
    st[k++] = 0x00; st[k++] = 0xA5; st[k++] = 0x13;
    n = put_info(); memcpy(st + k, out, n); k += n;
    n = put_frame(); memcpy(st + k, out, n); k += n;
    n = put_log(0x40); memcpy(st + k, out, n); k += n;
    n = put_log(0); memcpy(st + k, out, n); k += n;
    n = put_drop(); memcpy(st + k, out, n); k += n;
    n = put_text(SP_OK, "LOG 100000"); memcpy(st + k, out, n); k += n;
    n = put_text(SP_ERR, "bad rate"); memcpy(st + k, out, n); k += n;
    vector("stream.bin", st, k);
    /* the firmware's CSV of the frame, for the Python CSV writer test */
    static char csvt[2048];
    csv_meta_t m = frame_meta();
    int cl = csv_header(csvt, sizeof csvt, &m);
    CHECK(cl > 0);
    for (int i = 0; i < 8; i++) cl += csv_row(csvt + cl, &m, i, FRAME_S[i]);
    vector("frame.csv", (const uint8_t *)csvt, (uint32_t)cl);

    /* info text */
    char t[160];
    sp_info(t, sizeof t, "log");
    CHECK(strstr(t, "fw=" FW_VERSION "\n") == t);
    CHECK(strstr(t, "mode=log\n") != 0);
    CHECK(strstr(t, "rates=1000,2000,5000,10000,20000,50000,100000,200000,500000\n") != 0);
    CHECK_EQ(sp_info(t, 10, "idle"), 0);                   /* too small */

    /* command lines */
    sp_line_t l;
    memset(&l, 0, sizeof l);
    const char *in = "INFO?\r\nLOG 200000\nSTOP\nFRAMES\nfoo\n\nLOG x\nLOG\n";
    sp_cmd_t got[10]; uint32_t args[10]; int ng = 0;
    for (const char *p = in; *p; p++)
        if (sp_line_feed(&l, (uint8_t)*p) && ng < 10) { got[ng] = sp_parse(l.buf, &args[ng]); ng++; }
    CHECK_EQ(ng, 8);
    CHECK_EQ(got[0], SP_CMD_INFO);
    CHECK_EQ(got[1], SP_CMD_LOG); CHECK_EQ(args[1], 200000);
    CHECK_EQ(got[2], SP_CMD_STOP);
    CHECK_EQ(got[3], SP_CMD_FRAMES);
    CHECK_EQ(got[4], SP_CMD_BAD);
    CHECK_EQ(got[5], SP_CMD_NONE);                          /* empty line ignored */
    CHECK_EQ(got[6], SP_CMD_BAD);
    CHECK_EQ(got[7], SP_CMD_BAD);
    /* an over-long line is reported once as BAD, then parsing recovers */
    memset(&l, 0, sizeof l);
    int lines = 0; sp_cmd_t last = SP_CMD_NONE; uint32_t a;
    for (int i = 0; i < 50; i++) if (sp_line_feed(&l, 'A')) lines++;
    if (sp_line_feed(&l, '\n')) { lines++; last = sp_parse(l.buf, &a); }
    CHECK_EQ(lines, 1); CHECK_EQ(last, SP_CMD_BAD);
    const char *ok = "STOP\n";
    for (const char *p = ok; *p; p++) if (sp_line_feed(&l, (uint8_t)*p)) last = sp_parse(l.buf, &a);
    CHECK_EQ(last, SP_CMD_STOP);

    /* rates */
    CHECK_EQ(sp_rate_ok(100000), 1); CHECK_EQ(sp_rate_ok(500000), 1); CHECK_EQ(sp_rate_ok(1000), 1);
    CHECK_EQ(sp_rate_ok(3000), 0); CHECK_EQ(sp_rate_ok(0), 0); CHECK_EQ(sp_rate_ok(1000000), 0);
}
