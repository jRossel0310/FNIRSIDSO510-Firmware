#include "settings.h"
#include "stream_proto.h"

#define OFF_FLAGS 5
#define OFF_TDIV  6
#define OFF_VDIV  10
#define OFF_LO    34
#define OFF_HI    94
#define OFF_CRC   154

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int16_t geti16(const uint8_t *p) { return (int16_t)get16(p); }

static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

void settings_pack(uint8_t *out, const scope_state_t *s, const cal_t *cal, int zeroed) {
    for (uint32_t i = 0; i < SETTINGS_LEN; i++) out[i] = 0;
    out[0] = 'D'; out[1] = 'S'; out[2] = 'S'; out[3] = '1';
    out[4] = 1;
    out[OFF_FLAGS] = zeroed ? 1 : 0;
    put32(out + OFF_TDIV, tdiv_ns[s->tdiv]);
    put16(out + OFF_VDIV, vdiv_mv[s->vdiv]);
    put16(out + 12, (uint16_t)s->offset);
    put16(out + 14, (uint16_t)s->trig_level);
    put16(out + 16, (uint16_t)s->trig_pos);
    out[18] = s->trig_mode; out[19] = s->edge; out[20] = s->coupling_dc;
    out[21] = s->probe_x10; out[22] = s->ctl;
    out[23] = s->gen.on; out[24] = s->gen.wave; out[25] = s->gen.fine;
    out[26] = s->gen.duty_pct; out[27] = s->gen.amp_dv; out[28] = s->gen.offset_dv;
    out[29] = s->gen.dc_dv;
    put32(out + 30, s->gen.freq_hz);
    for (int k = 0; k < CAL_NSETS; k++)
        for (int r = 0; r < CAL_NRANGES; r++) {
            put16(out + OFF_LO + 2 * (k * CAL_NRANGES + r), cal->lo[k][r]);
            put16(out + OFF_HI + 2 * (k * CAL_NRANGES + r), cal->hi[k][r]);
        }
    put16(out + OFF_CRC, sp_crc16(0xFFFF, out, OFF_CRC));
}

int settings_unpack(const uint8_t *in, uint32_t len, scope_state_t *s, cal_t *cal, int *zeroed) {
    if (len != SETTINGS_LEN) return 0;
    if (in[0] != 'D' || in[1] != 'S' || in[2] != 'S' || in[3] != '1') return 0;
    if (in[4] != 1) return 0;
    if (get16(in + OFF_CRC) != sp_crc16(0xFFFF, in, OFF_CRC)) return 0;

    uint32_t ns = get32(in + OFF_TDIV);
    uint16_t mv = get16(in + OFF_VDIV);
    int ti = -1, vi = -1;
    for (int i = 0; i < NUM_TDIV; i++) if (tdiv_ns[i] == ns) ti = i;
    for (int i = 0; i < NUM_VDIV; i++) if (vdiv_mv[i] == mv) vi = i;
    if (ti < 0 || vi < 0) return 0;

    /* parse into locals */
    int8_t tdiv = (int8_t)ti, vdiv = (int8_t)vi;
    int16_t offset = (int16_t)clampi(geti16(in + 12), -100, 100);
    int16_t trig_level = (int16_t)clampi(geti16(in + 14), 0, 255);
    int16_t trig_pos = (int16_t)clampi(geti16(in + 16), 0, PLOT_COLS - 1);
    uint8_t trig_mode = in[18] > TRIG_SINGLE ? TRIG_AUTO : in[18];
    uint8_t edge = in[19] ? 1 : 0, coupling_dc = in[20] ? 1 : 0, probe_x10 = in[21] ? 1 : 0;
    uint8_t ctl = in[22] < CTL_NUM ? in[22] : CTL_TIME_VOLT;
    gen_cfg_t g;
    g.on = in[23] ? 1 : 0; g.wave = in[24]; g.fine = in[25] ? 1 : 0;
    g.duty_pct = in[26]; g.amp_dv = in[27]; g.offset_dv = in[28]; g.dc_dv = in[29];
    g.freq_hz = get32(in + 30);
    gen_clamp(&g);

    int take_cal = (in[OFF_FLAGS] & 1) != 0;
    uint16_t lo[CAL_NSETS][CAL_NRANGES], hi[CAL_NSETS][CAL_NRANGES];
    for (int k = 0; k < CAL_NSETS; k++)
        for (int r = 0; r < CAL_NRANGES; r++) {
            lo[k][r] = get16(in + OFF_LO + 2 * (k * CAL_NRANGES + r));
            hi[k][r] = get16(in + OFF_HI + 2 * (k * CAL_NRANGES + r));
            if (lo[k][r] > CAL_PWM_MAX || hi[k][r] > CAL_PWM_MAX) take_cal = 0;
        }

    /* commit */
    s->tdiv = tdiv; s->vdiv = vdiv; s->offset = offset;
    s->trig_level = trig_level; s->trig_pos = trig_pos; s->trig_mode = trig_mode;
    s->edge = edge; s->coupling_dc = coupling_dc; s->probe_x10 = probe_x10; s->ctl = ctl;
    s->gen = g;
    if (take_cal) {
        for (int k = 0; k < CAL_NSETS; k++)
            for (int r = 0; r < CAL_NRANGES; r++) { cal->lo[k][r] = lo[k][r]; cal->hi[k][r] = hi[k][r]; }
        *zeroed = 1;
    }
    return 1;
}
