#include "measure.h"
#include "trigger.h"

void measure(const uint8_t *buf, int n, uint32_t rate_hz, meas_t *m) {
    int mn = 255, mx = 0;
    for (int i = 0; i < n; i++) { if (buf[i] < mn) mn = buf[i]; if (buf[i] > mx) mx = buf[i]; }
    m->vmin = (uint8_t)mn; m->vmax = (uint8_t)mx; m->vpp_codes = (float)(mx - mn);
    m->valid = 0; m->freq_hz = 0; m->duty_pct = 0;
    if (n <= 0 || mx - mn < 8) return;
    int level = (mn + mx) / 2, hyst = (mx - mn) / 8;
    if (hyst < 4) hyst = 4;
    int first = trigger_find(buf, 0, n, level, hyst, EDGE_RISING);
    if (first < 0) return;
    int last = first, periods = 0;
    for (;;) {
        int j = trigger_find(buf, last + 1, n, level, hyst, EDGE_RISING);
        if (j < 0) break;
        last = j; periods++;
    }
    if (periods == 0) return;
    int high = 0;
    for (int i = first; i < last; i++) if (buf[i] >= level) high++;
    m->freq_hz = (float)periods * (float)rate_hz / (float)(last - first);
    m->duty_pct = 100.0f * (float)high / (float)(last - first);
    m->valid = 1;
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
        return;
    }
    if (a->n < MEAS_AVG_N) a->n++;
    float k = 1.0f / (float)a->n;
    a->freq_hz += (in->freq_hz - a->freq_hz) * k;
    a->duty_pct += (in->duty_pct - a->duty_pct) * k;
    a->vpp_codes += (in->vpp_codes - a->vpp_codes) * k;
    out->freq_hz = a->freq_hz; out->duty_pct = a->duty_pct; out->vpp_codes = a->vpp_codes;
}
