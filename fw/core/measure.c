#include "measure.h"
#include "trigger.h"

void measure(const uint8_t *buf, int n, uint32_t rate_hz, meas_t *m) {
    int mn = 255, mx = 0;
    for (int i = 0; i < n; i++) { if (buf[i] < mn) mn = buf[i]; if (buf[i] > mx) mx = buf[i]; }
    m->vmin = (uint8_t)mn; m->vmax = (uint8_t)mx;
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
