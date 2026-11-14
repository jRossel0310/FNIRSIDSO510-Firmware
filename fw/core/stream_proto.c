#include "stream_proto.h"

const uint32_t sp_rates[SP_NUM_RATES] = {1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000};

uint16_t sp_crc16(uint16_t crc, const uint8_t *p, uint32_t n) {
    while (n--) {
        crc ^= (uint16_t)(*p++ << 8);
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

int sp_put(bytefifo_t *f, uint8_t type, const uint8_t *a, uint32_t alen, const uint8_t *b, uint32_t blen) {
    uint32_t len = alen + blen;
    if (len > 65535u || bf_space(f) < len + SP_OVERHEAD) return -1;
    uint8_t h[5] = {0xA5, 0x5A, type, (uint8_t)len, (uint8_t)(len >> 8)};
    uint16_t crc = sp_crc16(0xFFFF, h + 2, 3);
    crc = sp_crc16(crc, a, alen);
    crc = sp_crc16(crc, b, blen);
    uint8_t t[2] = {(uint8_t)crc, (uint8_t)(crc >> 8)};
    bf_write(f, h, 5);
    bf_write(f, a, alen);
    bf_write(f, b, blen);
    bf_write(f, t, 2);
    return 0;
}

static uint8_t *le16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); return p + 2; }
static uint8_t *le32(uint8_t *p, uint32_t v) { p = le16(p, v & 0xFFFFu); return le16(p, v >> 16); }

uint32_t sp_frame_meta(uint8_t *out, const csv_meta_t *m, uint16_t n) {
    uint8_t *p = le32(out, m->rate_hz);
    p = le16(p, m->vdiv_mv);
    *p++ = m->coupling_dc; *p++ = m->probe_x10; *p++ = m->trig_mode; *p++ = m->edge; *p++ = m->triggered;
    p = le16(p, (uint16_t)m->zero_code);
    p = le16(p, (uint16_t)m->trig_level_code);
    p = le32(p, (uint32_t)m->trig_index);
    p = le16(p, m->offset_pwm);
    p = le16(p, n);
    return (uint32_t)(p - out);
}

uint32_t sp_logmeta(uint8_t *out, uint32_t rate_hz, uint16_t vdiv_mv, uint8_t probe_x10,
                    uint8_t coupling_dc, int16_t zero_code, uint16_t offset_pwm) {
    uint8_t *p = le32(out, rate_hz);
    p = le16(p, vdiv_mv);
    *p++ = probe_x10; *p++ = coupling_dc;
    p = le16(p, (uint16_t)zero_code);
    p = le16(p, offset_pwm);
    return (uint32_t)(p - out);
}

uint32_t sp_u32x2(uint8_t *out, uint32_t a, uint32_t b) { le32(le32(out, a), b); return 8; }

static uint32_t put_s(char *out, uint32_t cap, uint32_t k, const char *s) {
    while (*s) { if (k + 1 >= cap) return cap; out[k++] = *s++; }
    return k;
}
static uint32_t put_u(char *out, uint32_t cap, uint32_t k, uint32_t v) {
    char t[12]; int n = 0;
    do { t[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n--) { if (k + 1 >= cap) return cap; out[k++] = t[n]; }
    return k;
}

uint32_t sp_info(char *out, uint32_t cap, const char *mode) {
    uint32_t k = put_s(out, cap, 0, "fw=" FW_VERSION "\nmode=");
    k = put_s(out, cap, k, mode);
    k = put_s(out, cap, k, "\nrates=");
    for (int i = 0; i < SP_NUM_RATES && k < cap; i++) {
        if (i) k = put_s(out, cap, k, ",");
        k = put_u(out, cap, k, sp_rates[i]);
    }
    k = put_s(out, cap, k, "\n");
    if (k >= cap) return 0;
    out[k] = 0;
    return k;
}

int sp_rate_ok(uint32_t hz) {
    for (int i = 0; i < SP_NUM_RATES; i++) if (sp_rates[i] == hz) return 1;
    return 0;
}

int sp_line_feed(sp_line_t *l, uint8_t c) {
    if (c == '\r') return 0;
    if (c == '\n') {
        if (l->overflow) { l->buf[0] = '?'; l->buf[1] = 0; }
        else l->buf[l->n] = 0;
        l->n = 0; l->overflow = 0;
        return 1;
    }
    if (l->n < SP_LINE_MAX) l->buf[l->n++] = (char)c; else l->overflow = 1;
    return 0;
}

static int eq(const char *a, const char *b) { while (*a && *a == *b) { a++; b++; } return *a == *b; }

sp_cmd_t sp_parse(const char *s, uint32_t *arg) {
    *arg = 0;
    if (!s[0]) return SP_CMD_NONE;
    if (eq(s, "INFO?")) return SP_CMD_INFO;
    if (eq(s, "FRAMES")) return SP_CMD_FRAMES;
    if (eq(s, "STOP")) return SP_CMD_STOP;
    if (s[0] == 'L' && s[1] == 'O' && s[2] == 'G' && s[3] == ' ' && s[4]) {
        uint32_t v = 0;
        for (const char *p = s + 4; *p; p++) {
            if (*p < '0' || *p > '9' || v > 100000000u) return SP_CMD_BAD;
            v = v * 10 + (uint32_t)(*p - '0');
        }
        *arg = v;
        return SP_CMD_LOG;
    }
    return SP_CMD_BAD;
}
