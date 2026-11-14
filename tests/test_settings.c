#include "test.h"
#include "settings.h"
#include "stream_proto.h"
#include <stdint.h>

/* CRC over b[0..off-1], stored at b[off], b[off+1] */
static void fix_crc(uint8_t *b, uint32_t off) {
    uint16_t c = sp_crc16(0xFFFF, b, off);
    b[off] = (uint8_t)c; b[off + 1] = (uint8_t)(c >> 8);
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
    s->meas_sel[0] = MEAS_RMS; s->meas_sel[1] = MEAS_PERIOD; s->meas_sel[2] = MEAS_MIN; s->auto_off = 3;
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
    CHECK_EQ(dst.meas_sel[0], MEAS_RMS); CHECK_EQ(dst.meas_sel[1], MEAS_PERIOD); CHECK_EQ(dst.meas_sel[2], MEAS_MIN);
    CHECK_EQ(dst.auto_off, 3);
    CHECK(memcmp(cd.lo, cs.lo, sizeof cs.lo) == 0);
    CHECK(memcmp(cd.hi, cs.hi, sizeof cs.hi) == 0);
    CHECK_EQ(z, 1);
    CHECK_EQ(dst.run, 0); CHECK_EQ(dst.menu_open, 0); CHECK_EQ(dst.menu_item, 3);
    CHECK_EQ(dst.gen_open, 1); CHECK_EQ(dst.gen_item, 4);
    CHECK_EQ(cd.valid, 0);

    /* 2. layout pins */
    CHECK(b[0] == 'D' && b[1] == 'S' && b[2] == 'S' && b[3] == '1');
    CHECK_EQ(b[4], 2); CHECK_EQ(b[5] & 1, 1);
    CHECK_EQ(b[6] | (b[7] << 8) | (b[8] << 16) | ((uint32_t)b[9] << 24), 2000);
    CHECK_EQ(b[10] | (b[11] << 8), vdiv_mv[3]);
    CHECK_EQ(b[29], 7);
    CHECK_EQ(b[34], 100); CHECK_EQ(b[94] | (b[95] << 8), 2000);
    CHECK_EQ(b[154], MEAS_RMS); CHECK_EQ(b[155], MEAS_PERIOD); CHECK_EQ(b[156], MEAS_MIN); CHECK_EQ(b[157], 3);
    CHECK_EQ(b[158] | (b[159] << 8), sp_crc16(0xFFFF, b, 158));

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
    memcpy(t, b, SETTINGS_LEN); t[0] = 'X'; fix_crc(t, 158); expect_reject(t, SETTINGS_LEN);
    memcpy(t, b, SETTINGS_LEN); t[4] = 3; fix_crc(t, 158); expect_reject(t, SETTINGS_LEN);
    memcpy(t, b, SETTINGS_LEN); t[12] ^= 0x10; expect_reject(t, SETTINGS_LEN);
    memcpy(t, b, SETTINGS_LEN); t[6] = 123; t[7] = t[8] = t[9] = 0; fix_crc(t, 158);
    expect_reject(t, SETTINGS_LEN);
    memcpy(t, b, SETTINGS_LEN); t[10] = 7; t[11] = 0; fix_crc(t, 158);
    expect_reject(t, SETTINGS_LEN);

    memcpy(t, b, SETTINGS_LEN); t[4] = 1; fix_crc(t, 158); expect_reject(t, SETTINGS_LEN);   /* 160 bytes, version 1 */
    memcpy(t, b, 156); fix_crc(t, 154); expect_reject(t, 156);                              /* 156 bytes, version 2 */
    memcpy(t, b, SETTINGS_LEN); expect_reject(t, 158);

    /* 4b. version 1 file still read; meas_sel / auto_off untouched */
    memcpy(t, b, 156); t[4] = 1; fix_crc(t, 154);
    state_defaults(&dst); cal_defaults(&cd); z = 0;
    dst.meas_sel[0] = MEAS_AVG; dst.meas_sel[1] = MEAS_AVG; dst.meas_sel[2] = MEAS_AVG; dst.auto_off = 1;
    CHECK_EQ(settings_unpack(t, 156, &dst, &cd, &z), 1);
    CHECK_EQ(dst.vdiv, 3); CHECK_EQ(dst.offset, -40); CHECK_EQ(dst.gen.freq_hz, 12300); CHECK_EQ(z, 1);
    CHECK_EQ(dst.meas_sel[0], MEAS_AVG); CHECK_EQ(dst.meas_sel[1], MEAS_AVG); CHECK_EQ(dst.meas_sel[2], MEAS_AVG);
    CHECK_EQ(dst.auto_off, 1);

    /* 5. clamping */
    memcpy(t, b, SETTINGS_LEN);
    t[12] = 0xF4; t[13] = 0x01;          /* offset 500 */
    t[14] = 0xFB; t[15] = 0xFF;          /* trig_level -5 */
    t[16] = 0x88; t[17] = 0x13;          /* trig_pos 5000 */
    t[18] = 9; t[22] = 77; t[24] = 200;
    t[30] = 0x3F; t[31] = 0x42; t[32] = 0x0F; t[33] = 0; /* 999999 */
    t[27] = 99;
    fix_crc(t, 158);
    state_defaults(&dst);
    CHECK_EQ(settings_unpack(t, SETTINGS_LEN, &dst, &cd, &z), 1);
    CHECK_EQ(dst.offset, 100); CHECK_EQ(dst.trig_level, 0);
    CHECK_EQ(dst.trig_pos, PLOT_COLS - 1); CHECK_EQ(dst.trig_mode, 0);
    CHECK_EQ(dst.ctl, 0); CHECK_EQ(dst.gen.wave, GEN_SINE);
    CHECK_EQ(dst.gen.freq_hz, 50000); CHECK_EQ(dst.gen.amp_dv, 30);
    memcpy(t, b, SETTINGS_LEN); t[155] = 200; t[157] = 9; fix_crc(t, 158);
    state_defaults(&dst); dst.meas_sel[1] = MEAS_AVG; dst.auto_off = 0;
    CHECK_EQ(settings_unpack(t, SETTINGS_LEN, &dst, &cd, &z), 1);
    CHECK_EQ(dst.meas_sel[0], MEAS_RMS); CHECK_EQ(dst.meas_sel[1], MEAS_DUTY_P); CHECK_EQ(dst.meas_sel[2], MEAS_MIN);
    CHECK_EQ(dst.auto_off, 2);

    /* 6. bad zero table */
    memcpy(t, b, SETTINGS_LEN); t[94 + 2 * 7] = 0x00; t[94 + 2 * 7 + 1] = 0x10; fix_crc(t, 158);
    state_defaults(&dst); cal_defaults(&cd); cd.lo[0][0] = 5; z = 0;
    CHECK_EQ(settings_unpack(t, SETTINGS_LEN, &dst, &cd, &z), 1);
    CHECK_EQ(dst.vdiv, 3); CHECK_EQ(dst.offset, -40);
    CHECK_EQ(cd.lo[0][0], 5); CHECK(cd.lo[1][1] != cs.lo[1][1]);
    CHECK_EQ(z, 0);

    /* 7. degenerate table (lo == hi): tables ignored, rest of the file used */
    memcpy(t, b, SETTINGS_LEN);
    t[94 + 2 * 7] = t[34 + 2 * 7]; t[94 + 2 * 7 + 1] = t[34 + 2 * 7 + 1]; fix_crc(t, 158);
    state_defaults(&dst); cal_defaults(&cd); cd.lo[0][0] = 5; z = 0;
    CHECK_EQ(settings_unpack(t, SETTINGS_LEN, &dst, &cd, &z), 1);
    CHECK_EQ(dst.vdiv, 3); CHECK_EQ(dst.offset, -40);
    CHECK_EQ(cd.lo[0][0], 5); CHECK(cd.lo[1][1] != cs.lo[1][1]);
    CHECK_EQ(z, 0);

    /* 8. stability: pack -> unpack -> pack is byte-identical (no flash write at every power-off) */
    state_defaults(&dst); cal_defaults(&cd); z = 0;
    CHECK_EQ(settings_unpack(b, SETTINGS_LEN, &dst, &cd, &z), 1);
    settings_pack(t, &dst, &cd, z);
    CHECK(memcmp(t, b, SETTINGS_LEN) == 0);
    {
        scope_state_t d0; cal_t c0; uint8_t u[SETTINGS_LEN], v[SETTINGS_LEN];
        state_defaults(&d0); cal_defaults(&c0);
        settings_pack(u, &d0, &c0, 0);
        state_defaults(&dst); cal_defaults(&cd); z = 0;
        CHECK_EQ(settings_unpack(u, SETTINGS_LEN, &dst, &cd, &z), 1);
        settings_pack(v, &dst, &cd, z);
        CHECK(memcmp(u, v, SETTINGS_LEN) == 0);
    }

    /* 9. remaining layout bytes, distinct non-default values */
    {
        scope_state_t p; cal_t c; uint8_t o[SETTINGS_LEN];
        state_defaults(&p); cal_defaults(&c);
        p.trig_mode = TRIG_SINGLE; p.edge = 1; p.coupling_dc = 1; p.probe_x10 = 1; p.ctl = CTL_GEN;
        p.gen.on = 1; p.gen.fine = 1; p.gen.duty_pct = 35; p.gen.amp_dv = 14; p.gen.offset_dv = 9;
        p.gen.freq_hz = 12345;
        c.lo[0][0] = 0x0123; c.hi[0][0] = 0x0456;
        settings_pack(o, &p, &c, 1);
        CHECK_EQ(o[18], TRIG_SINGLE); CHECK_EQ(o[19], 1); CHECK_EQ(o[20], 1); CHECK_EQ(o[21], 1);
        CHECK_EQ(o[22], CTL_GEN); CHECK_EQ(o[23], 1); CHECK_EQ(o[25], 1);
        CHECK_EQ(o[26], 35); CHECK_EQ(o[27], 14); CHECK_EQ(o[28], 9);
        CHECK_EQ(o[30] | (o[31] << 8) | (o[32] << 16) | ((uint32_t)o[33] << 24), 12345);
        CHECK_EQ(o[34] | (o[35] << 8), 0x0123); CHECK_EQ(o[94] | (o[95] << 8), 0x0456);
    }

    /* 10. 0/1 normalisation and trig_mode range */
    memcpy(t, b, SETTINGS_LEN);
    t[19] = 7; t[20] = 9; t[21] = 2; t[23] = 5; t[25] = 3; t[18] = 2; fix_crc(t, 158);
    state_defaults(&dst); cal_defaults(&cd); z = 0;
    CHECK_EQ(settings_unpack(t, SETTINGS_LEN, &dst, &cd, &z), 1);
    CHECK_EQ(dst.edge, 1); CHECK_EQ(dst.coupling_dc, 1); CHECK_EQ(dst.probe_x10, 1);
    CHECK_EQ(dst.gen.on, 1); CHECK_EQ(dst.gen.fine, 1); CHECK_EQ(dst.trig_mode, 2);
    t[18] = 3; fix_crc(t, 158);
    CHECK_EQ(settings_unpack(t, SETTINGS_LEN, &dst, &cd, &z), 1);
    CHECK_EQ(dst.trig_mode, 0);
}
