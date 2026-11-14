#include "test.h"
#include "settings.h"
#include "stream_proto.h"
#include <stdint.h>

static void fix_crc(uint8_t *b) {
    uint16_t c = sp_crc16(0xFFFF, b, SETTINGS_LEN - 2);
    b[SETTINGS_LEN - 2] = (uint8_t)c; b[SETTINGS_LEN - 1] = (uint8_t)(c >> 8);
}

static int tdiv_index(uint32_t ns) {
    for (int i = 0; i < NUM_TDIV; i++) if (tdiv_ns[i] == ns) return i;
    return -1;
}

static void make_state(scope_state_t *s) {
    state_defaults(s);
    s->tdiv = (int8_t)tdiv_index(2000); s->vdiv = 3; s->offset = -40;
    s->trig_level = 200; s->trig_pos = 60; s->trig_mode = TRIG_NORMAL;
    s->edge = 1; s->coupling_dc = 0; s->probe_x10 = 1; s->ctl = CTL_GEN;
    s->gen.on = 0; s->gen.wave = GEN_SQUARE; s->gen.freq_hz = 12300; s->gen.amp_dv = 10;
    s->gen.offset_dv = 12; s->gen.dc_dv = 7; s->gen.duty_pct = 30; s->gen.fine = 1;
}

static void make_cal(cal_t *c) {
    cal_defaults(c);
    for (int s = 0; s < CAL_NSETS; s++)
        for (int r = 0; r < CAL_NRANGES; r++) {
            c->lo[s][r] = (uint16_t)(100 + 10 * s + r);
            c->hi[s][r] = (uint16_t)(2000 + 10 * s + r);
        }
}

/* Rejection helper: unpack must return 0 and leave sentinels byte-for-byte unchanged. */
static void expect_reject(const uint8_t *buf, uint32_t len) {
    scope_state_t s, s0; cal_t c, c0; int z = 1;
    memset(&s, 0, sizeof s); memset(&c, 0, sizeof c);
    state_defaults(&s); s.offset = 17; cal_defaults(&c); c.lo[1][2] = 777;
    memcpy(&s0, &s, sizeof s); memcpy(&c0, &c, sizeof c);
    CHECK_EQ(settings_unpack(buf, len, &s, &c, &z), 0);
    CHECK(memcmp(&s, &s0, sizeof s) == 0);
    CHECK(memcmp(&c, &c0, sizeof c) == 0);
    CHECK_EQ(z, 1);
}

void test_settings(void) {
    scope_state_t src, dst; cal_t cs, cd; int z;
    uint8_t b[SETTINGS_LEN + 2], t[SETTINGS_LEN + 2];
    memset(&src, 0, sizeof src); memset(&dst, 0, sizeof dst);
    memset(&cs, 0, sizeof cs); memset(&cd, 0, sizeof cd);
    make_state(&src); make_cal(&cs);
    settings_pack(b, &src, &cs, 1);

    /* 1. round trip */
    state_defaults(&dst); dst.run = 0; dst.menu_open = 0; dst.menu_item = 3;
    dst.gen_open = 1; dst.gen_item = 4; cal_defaults(&cd); z = 0;
    CHECK_EQ(settings_unpack(b, SETTINGS_LEN, &dst, &cd, &z), 1);
    CHECK_EQ(dst.tdiv, src.tdiv); CHECK_EQ(dst.vdiv, 3); CHECK_EQ(dst.offset, -40);
    CHECK_EQ(dst.trig_level, 200); CHECK_EQ(dst.trig_pos, 60);
    CHECK_EQ(dst.trig_mode, TRIG_NORMAL); CHECK_EQ(dst.edge, 1);
    CHECK_EQ(dst.coupling_dc, 0); CHECK_EQ(dst.probe_x10, 1); CHECK_EQ(dst.ctl, CTL_GEN);
    CHECK_EQ(dst.gen.on, 0); CHECK_EQ(dst.gen.wave, GEN_SQUARE); CHECK_EQ(dst.gen.fine, 1);
    CHECK_EQ(dst.gen.duty_pct, 30); CHECK_EQ(dst.gen.amp_dv, 10);
    CHECK_EQ(dst.gen.offset_dv, 12); CHECK_EQ(dst.gen.dc_dv, 7);
    CHECK_EQ(dst.gen.freq_hz, 12300);
    CHECK(memcmp(cd.lo, cs.lo, sizeof cs.lo) == 0);
    CHECK(memcmp(cd.hi, cs.hi, sizeof cs.hi) == 0);
    CHECK_EQ(z, 1);
    CHECK_EQ(dst.run, 0); CHECK_EQ(dst.menu_open, 0); CHECK_EQ(dst.menu_item, 3);
    CHECK_EQ(dst.gen_open, 1); CHECK_EQ(dst.gen_item, 4);
    CHECK_EQ(cd.valid, 0);

    /* 2. layout pins */
    CHECK(b[0] == 'D' && b[1] == 'S' && b[2] == 'S' && b[3] == '1');
    CHECK_EQ(b[4], 1); CHECK_EQ(b[5] & 1, 1);
    CHECK_EQ(b[6] | (b[7] << 8) | (b[8] << 16) | ((uint32_t)b[9] << 24), 2000);
    CHECK_EQ(b[10] | (b[11] << 8), vdiv_mv[3]);
    CHECK_EQ(b[29], 7);
    CHECK_EQ(b[34], 100); CHECK_EQ(b[94] | (b[95] << 8), 2000);
    CHECK_EQ(b[154] | (b[155] << 8), sp_crc16(0xFFFF, b, 154));

    /* 3. zeroed = 0 */
    settings_pack(t, &src, &cs, 0);
    for (int z0 = 0; z0 <= 1; z0++) {
        state_defaults(&dst); cal_defaults(&cd); cd.lo[0][0] = 5; z = z0;
        CHECK_EQ(settings_unpack(t, SETTINGS_LEN, &dst, &cd, &z), 1);
        CHECK_EQ(cd.lo[0][0], 5);
        CHECK(cd.lo[1][1] != cs.lo[1][1]);
        CHECK_EQ(z, z0);
        CHECK_EQ(dst.vdiv, 3);
    }

    /* 4. rejections */
    expect_reject(b, SETTINGS_LEN - 1);
    memcpy(t, b, SETTINGS_LEN); t[SETTINGS_LEN] = 0;
    expect_reject(t, SETTINGS_LEN + 1);
    memcpy(t, b, SETTINGS_LEN); t[0] = 'X'; fix_crc(t); expect_reject(t, SETTINGS_LEN);
    memcpy(t, b, SETTINGS_LEN); t[4] = 2; fix_crc(t); expect_reject(t, SETTINGS_LEN);
    memcpy(t, b, SETTINGS_LEN); t[12] ^= 0x10; expect_reject(t, SETTINGS_LEN);
    memcpy(t, b, SETTINGS_LEN); t[6] = 123; t[7] = t[8] = t[9] = 0; fix_crc(t);
    expect_reject(t, SETTINGS_LEN);
    memcpy(t, b, SETTINGS_LEN); t[10] = 7; t[11] = 0; fix_crc(t);
    expect_reject(t, SETTINGS_LEN);

    /* 5. clamping */
    memcpy(t, b, SETTINGS_LEN);
    t[12] = 0xF4; t[13] = 0x01;          /* offset 500 */
    t[14] = 0xFB; t[15] = 0xFF;          /* trig_level -5 */
    t[16] = 0x88; t[17] = 0x13;          /* trig_pos 5000 */
    t[18] = 9; t[22] = 77; t[24] = 200;
    t[30] = 0x3F; t[31] = 0x42; t[32] = 0x0F; t[33] = 0; /* 999999 */
    t[27] = 99;
    fix_crc(t);
    state_defaults(&dst);
    CHECK_EQ(settings_unpack(t, SETTINGS_LEN, &dst, &cd, &z), 1);
    CHECK_EQ(dst.offset, 100); CHECK_EQ(dst.trig_level, 0);
    CHECK_EQ(dst.trig_pos, PLOT_COLS - 1); CHECK_EQ(dst.trig_mode, 0);
    CHECK_EQ(dst.ctl, 0); CHECK_EQ(dst.gen.wave, GEN_SINE);
    CHECK_EQ(dst.gen.freq_hz, 50000); CHECK_EQ(dst.gen.amp_dv, 30);

    /* 6. bad zero table */
    memcpy(t, b, SETTINGS_LEN); t[94 + 2 * 7] = 0x00; t[94 + 2 * 7 + 1] = 0x10; fix_crc(t);
    state_defaults(&dst); cal_defaults(&cd); cd.lo[0][0] = 5; z = 0;
    CHECK_EQ(settings_unpack(t, SETTINGS_LEN, &dst, &cd, &z), 1);
    CHECK_EQ(dst.vdiv, 3); CHECK_EQ(dst.offset, -40);
    CHECK_EQ(cd.lo[0][0], 5); CHECK(cd.lo[1][1] != cs.lo[1][1]);
    CHECK_EQ(z, 0);
}
