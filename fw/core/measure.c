#include "measure.h"
#include "trigger.h"

static void stats(const uint8_t *buf, int from, int to, int mn, int mx, meas_t *m) {
    static uint16_t hist[256];   /* single-threaded; counts saturate only for windows > 65535 samples */
    uint64_t sum = 0, sq = 0;
    for (int i = 0; i < 256; i++) hist[i] = 0;
    for (int i = from; i < to; i++) {
        unsigned c = buf[i];
        sum += c; sq += (uint64_t)c * c;
        if (hist[c] < 65535) hist[c]++;
    }
    m->mean_codes = (float)((double)sum / (double)(to - from));
    m->meansq_codes = (float)((double)sq / (double)(to - from));
    if (mx - mn < 8) { m->top_codes = m->base_codes = m->mean_codes; return; }
    int level = (mn + mx) / 2, best = 0, bc = -1;
    for (int c = level; c <= 255; c++) if (hist[c] && (int)hist[c] >= bc) { bc = hist[c]; best = c; }
    m->top_codes = (float)best;
    bc = -1; best = mn;
    for (int c = level - 1; c >= 0; c--) if (hist[c] && (int)hist[c] >= bc) { bc = hist[c]; best = c; }
    m->base_codes = (float)best;
}

void measure(const uint8_t *buf, int n, uint32_t rate_hz, meas_t *m) {
    int mn = 255, mx = 0;
    for (int i = 0; i < n; i++) { if (buf[i] < mn) mn = buf[i]; if (buf[i] > mx) mx = buf[i]; }
    m->vmin = (uint8_t)mn; m->vmax = (uint8_t)mx; m->vpp_codes = (float)(mx - mn);
    m->valid = 0; m->freq_hz = 0; m->duty_pct = 0;
    m->max_codes = m->min_codes = m->top_codes = m->base_codes = m->mean_codes = m->meansq_codes = 0;
    if (n <= 0) return;
    m->max_codes = (float)mx; m->min_codes = (float)mn;
    int from = 0, to = n;
    if (mx - mn >= 8) {
        int level = (mn + mx) / 2, hyst = (mx - mn) / 8;
        if (hyst < 4) hyst = 4;
        int first = trigger_find(buf, 0, n, level, hyst, EDGE_RISING);
        if (first >= 0) {
            int last = first, periods = 0;
            for (;;) {
                int j = trigger_find(buf, last + 1, n, level, hyst, EDGE_RISING);
                if (j < 0) break;
                last = j; periods++;
            }
            if (periods > 0) {
                int high = 0;
                for (int i = first; i < last; i++) if (buf[i] >= level) high++;
                m->freq_hz = (float)periods * (float)rate_hz / (float)(last - first);
                m->duty_pct = 100.0f * (float)high / (float)(last - first);
                m->valid = 1;
                from = first; to = last;
            }
        }
    }
    stats(buf, from, to, mn, mx, m);
}

void meas_avg_reset(meas_avg_t *a) { a->n = 0; }

static float absf(float x) { return x < 0 ? -x : x; }

void meas_avg_update(meas_avg_t *a, const meas_t *in, meas_t *out) {
    *out = *in;
    if (!in->valid || a->n == 0 ||
        absf(in->freq_hz - a->freq_hz) > 0.1f * a->freq_hz ||
        absf(in->vpp_codes - a->vpp_codes) > 0.1f * a->vpp_codes + 2.0f) {
        a->n = in->valid ? 1 : 0;
        a->freq_hz = in->freq_hz; a->duty_pct = in->duty_pct; a->vpp_codes = in->vpp_codes;
        a->max_codes = in->max_codes; a->min_codes = in->min_codes;
        a->top_codes = in->top_codes; a->base_codes = in->base_codes;
        a->mean_codes = in->mean_codes; a->meansq_codes = in->meansq_codes;
        return;
    }
    if (a->n < MEAS_AVG_N) a->n++;
    float k = 1.0f / (float)a->n;
    a->freq_hz += (in->freq_hz - a->freq_hz) * k;
    a->duty_pct += (in->duty_pct - a->duty_pct) * k;
    a->vpp_codes += (in->vpp_codes - a->vpp_codes) * k;
    a->max_codes += (in->max_codes - a->max_codes) * k;
    a->min_codes += (in->min_codes - a->min_codes) * k;
    a->top_codes += (in->top_codes - a->top_codes) * k;
    a->base_codes += (in->base_codes - a->base_codes) * k;
    a->mean_codes += (in->mean_codes - a->mean_codes) * k;
    a->meansq_codes += (in->meansq_codes - a->meansq_codes) * k;
    out->freq_hz = a->freq_hz; out->duty_pct = a->duty_pct; out->vpp_codes = a->vpp_codes;
    out->max_codes = a->max_codes; out->min_codes = a->min_codes;
    out->top_codes = a->top_codes; out->base_codes = a->base_codes;
    out->mean_codes = a->mean_codes; out->meansq_codes = a->meansq_codes;
}

static float sqrt_f(float x) {
    if (x <= 0) return 0;
    float g = x > 1 ? x * 0.5f + 0.5f : 1.0f;
    for (int i = 0; i < 40; i++) {
        float ng = 0.5f * (g + x / g);
        if (absf(ng - g) <= 1e-6f * ng) { g = ng; break; }
        g = ng;
    }
    return g;
}

static const char *const names[MEAS_NUM] = { "Frequency", "Period", "Duty +", "Duty -", "Width +", "Width -",
    "Maximum", "Minimum", "Peak-peak", "Amplitude", "RMS", "Average" };
static const char *const tags[MEAS_NUM] = { "Freq", "Per", "Duty+", "Duty-", "Wid+", "Wid-",
    "Max", "Min", "Vpp", "Ampl", "RMS", "Avg" };

const char *meas_name(int kind) { return kind >= 0 && kind < MEAS_NUM ? names[kind] : "?"; }
const char *meas_tag(int kind) { return kind >= 0 && kind < MEAS_NUM ? tags[kind] : "?"; }

int meas_value(const meas_t *m, int kind, float vpc, float zero, float *value, int *unit) {
    float v = 0;
    int ok = 1;
    switch (kind) {
    case MEAS_FREQ: *unit = MEAS_U_HZ; ok = m->valid; if (ok) v = m->freq_hz; break;
    case MEAS_PERIOD: *unit = MEAS_U_S; ok = m->valid; if (ok) v = 1.0f / m->freq_hz; break;
    case MEAS_DUTY_P: *unit = MEAS_U_PCT; ok = m->valid; if (ok) v = m->duty_pct; break;
    case MEAS_DUTY_N: *unit = MEAS_U_PCT; ok = m->valid; if (ok) v = 100.0f - m->duty_pct; break;
    case MEAS_WIDTH_P: *unit = MEAS_U_S; ok = m->valid; if (ok) v = m->duty_pct / 100.0f / m->freq_hz; break;
    case MEAS_WIDTH_N: *unit = MEAS_U_S; ok = m->valid; if (ok) v = (100.0f - m->duty_pct) / 100.0f / m->freq_hz; break;
    case MEAS_MAX: *unit = MEAS_U_V; v = (m->max_codes - zero) * vpc; break;
    case MEAS_MIN: *unit = MEAS_U_V; v = (m->min_codes - zero) * vpc; break;
    case MEAS_VPP: *unit = MEAS_U_V; v = m->vpp_codes * vpc; break;
    case MEAS_AMPL: *unit = MEAS_U_V; v = (m->top_codes - m->base_codes) * vpc; break;
    case MEAS_AVG: *unit = MEAS_U_V; v = (m->mean_codes - zero) * vpc; break;
    case MEAS_RMS: {
        *unit = MEAS_U_V;
        float r = m->meansq_codes - 2.0f * zero * m->mean_codes + zero * zero;
        v = vpc * sqrt_f(r);
        break; }
    default: return 0;
    }
    if (ok) *value = v;
    return ok;
}
