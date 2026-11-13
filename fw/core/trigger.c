#include "trigger.h"

int trigger_hysteresis(const uint8_t *buf, int n) {
    int mn = 255, mx = 0;
    for (int i = 0; i < n; i++) { if (buf[i] < mn) mn = buf[i]; if (buf[i] > mx) mx = buf[i]; }
    int h = (mx - mn) / 8;
    return h < 4 ? 4 : h;
}

int trigger_find(const uint8_t *buf, int from, int to, int level, int hyst, edge_t edge) {
    int armed = 0;
    for (int i = from < 0 ? 0 : from; i < to; i++) {
        int v = buf[i];
        if (edge == EDGE_RISING) {
            if (v <= level - hyst) armed = 1;
            else if (armed && v >= level) return i;
        } else {
            if (v >= level + hyst) armed = 1;
            else if (armed && v <= level) return i;
        }
    }
    return -1;
}
