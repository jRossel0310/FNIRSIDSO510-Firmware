#include "csv.h"

static int64_t rdiv(int64_t n, int64_t d) { return n >= 0 ? (n + d / 2) / d : -((-n + d / 2) / d); }

/* v in units of 10^-dec, printed as a decimal with exactly dec decimals. Returns length. */
static int put_fixed(char *p, int64_t v, int dec) {
    char t[24];
    int n = 0, k = 0;
    uint64_t u = v < 0 ? (uint64_t)(-v) : (uint64_t)v;
    do { t[n++] = (char)('0' + u % 10); u /= 10; } while (u || n <= dec);
    if (v < 0) p[k++] = '-';
    while (n--) {
        p[k++] = t[n];
        if (dec && n == dec) p[k++] = '.';
    }
    return k;
}

static int32_t uv_per_code(const csv_meta_t *m) { return (int32_t)m->vdiv_mv * 40 * (m->probe_x10 ? 10 : 1); }

typedef struct { char *p; int n, cap, err; } sb_t;
static void sb_s(sb_t *b, const char *s) {
    while (*s) { if (b->n >= b->cap) { b->err = 1; return; } b->p[b->n++] = *s++; }
}
static void sb_fx(sb_t *b, int64_t v, int dec) { char t[32]; int k = put_fixed(t, v, dec); t[k] = 0; sb_s(b, t); }

int csv_header(char *buf, int cap, const csv_meta_t *m) {
    static const char *modes[3] = {"Auto", "Normal", "Single"};
    sb_t b = {buf, 0, cap, 0};
    int32_t uvpc = uv_per_code(m);
    sb_s(&b, "# DSO-510 custom firmware " FW_VERSION "\n# sample_rate_hz=");
    sb_fx(&b, m->rate_hz, 0);
    sb_s(&b, "\n# volts_per_div=");
    sb_fx(&b, (int64_t)m->vdiv_mv * 10 * (m->probe_x10 ? 10 : 1), 4);   /* mV -> 1e-4 V */
    sb_s(&b, m->coupling_dc ? "\n# coupling=DC" : "\n# coupling=AC");
    sb_s(&b, m->probe_x10 ? "\n# probe=10" : "\n# probe=1");
    sb_s(&b, "\n# trigger=");
    sb_s(&b, modes[m->trig_mode < 3 ? m->trig_mode : 0]);
    sb_s(&b, m->edge ? ",falling," : ",rising,");
    sb_fx(&b, rdiv((int64_t)(m->trig_level_code - m->zero_code) * uvpc, 100), 4);
    sb_s(&b, "\n# trigger_index=");
    sb_fx(&b, m->trig_index, 0);
    sb_s(&b, "\n# cal=offset_pwm=");
    sb_fx(&b, m->offset_pwm, 0);
    sb_s(&b, ",volts_per_code=");
    sb_fx(&b, uvpc, 6);
    sb_s(&b, "\ntime_s,volts,adc_code\n");
    return b.err ? -1 : b.n;
}

int csv_row(char *buf, const csv_meta_t *m, int32_t i, uint8_t code) {
    int k = put_fixed(buf, rdiv((int64_t)(i - m->trig_index) * 1000000000, (int64_t)m->rate_hz), 9);
    buf[k++] = ',';
    k += put_fixed(buf + k, rdiv((int64_t)((int32_t)code - m->zero_code) * uv_per_code(m), 100), 4);
    buf[k++] = ',';
    k += put_fixed(buf + k, code, 0);
    buf[k++] = '\n';
    return k;
}
