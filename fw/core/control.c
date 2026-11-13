#include "control.h"

static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

void state_defaults(scope_state_t *s) {
    s->tdiv = TDIV_1MS; s->vdiv = 6; s->offset = 0;
    s->trig_level = 128; s->trig_pos = PLOT_COLS / 2;
    s->trig_mode = TRIG_AUTO; s->edge = 0; s->coupling_dc = 1; s->probe_x10 = 0;
    s->run = 1; s->ctl = CTL_TIME_VOLT; s->menu_open = 0; s->menu_item = 0;
}

static uint32_t menu_key(scope_state_t *s, int k, int t, int tap) {
    if ((k == KEY_MODE || k == KEY_WHEEL_PRESS) && t == KEV_SHORT) { s->menu_open = 0; return CHG_SCREEN; }
    if (k == KEY_UP && tap) { s->menu_item = (uint8_t)((s->menu_item + MENU_ITEMS - 1) % MENU_ITEMS); return CHG_MENU; }
    if (k == KEY_DOWN && tap) { s->menu_item = (uint8_t)((s->menu_item + 1) % MENU_ITEMS); return CHG_MENU; }
    if ((k == KEY_WHEEL_L || k == KEY_WHEEL_R) && tap) {
        int d = k == KEY_WHEEL_R ? 1 : -1;
        switch (s->menu_item) {
        case 0: s->coupling_dc ^= 1; return CHG_MENU | CHG_FE | CHG_BARS;
        case 1: s->probe_x10 ^= 1; return CHG_MENU | CHG_BARS;
        case 2: s->trig_mode = (uint8_t)((s->trig_mode + 3 + d) % 3); return CHG_MENU | CHG_BARS | CHG_TRIG;
        case 3: s->edge ^= 1; return CHG_MENU | CHG_BARS | CHG_TRIG;
        }
    }
    return 0;
}

uint32_t control_handle(scope_state_t *s, kev_t ev) {
    int k = ev.key, t = ev.type, tap = (t == KEV_SHORT || t == KEV_REPEAT);
    if (k == KEY_POWER) return t == KEV_LONG ? ACT_POWEROFF : 0;
    if (k == KEY_MODE && t == KEV_LONG) { s->menu_open ^= 1; s->menu_item = 0; return CHG_SCREEN; }
    if (s->menu_open) return menu_key(s, k, t, tap);
    if (k == KEY_MODE && t == KEV_SHORT) { s->ctl = (uint8_t)((s->ctl + 1) % 3); return CHG_BARS; }
    if (k == KEY_RUN && t == KEV_SHORT) { s->run ^= 1; return CHG_RUN | CHG_BARS; }
    if (k == KEY_RUN && t == KEV_LONG) return ACT_SAVE;
    if (k == KEY_AUTO && t == KEV_SHORT) return ACT_AUTOSET;
    if (k == KEY_AUTO && t == KEV_LONG) return ACT_AUTOZERO;
    if (!tap) return 0;
    int h = k == KEY_WHEEL_R ? 1 : k == KEY_WHEEL_L ? -1 : 0;
    int v = k == KEY_UP ? 1 : k == KEY_DOWN ? -1 : 0;
    if (s->ctl == CTL_TIME_VOLT) {
        if (h) {
            int n = clampi(s->tdiv + h, 0, NUM_TDIV - 1);
            if (n == s->tdiv) return 0;
            s->tdiv = (int8_t)n; return CHG_TIME | CHG_BARS;
        }
        if (v) {
            int n = clampi(s->vdiv - v, 0, NUM_VDIV - 1);
            if (n == s->vdiv) return 0;
            /* keep the trigger level at the same voltage on the new scale */
            int zero = ADC_CENTER + s->offset;
            int32_t rel = (int32_t)(s->trig_level - zero) * vdiv_mv[s->vdiv];
            int32_t m = vdiv_mv[n];
            s->trig_level = (int16_t)clampi(zero + (int)((rel >= 0 ? rel + m / 2 : rel - m / 2) / m), 0, 255);
            s->vdiv = (int8_t)n; return CHG_FE | CHG_BARS | CHG_TRIG;
        }
        return 0;
    }
    if (h) { s->trig_pos = (int16_t)clampi(s->trig_pos + 10 * h, 0, PLOT_COLS - 1); return CHG_TRIG; }
    if (v && s->ctl == CTL_POS_OFFSET) {
        int n = clampi(s->offset + 5 * v, -100, 100), d = n - s->offset;
        if (!d) return 0;
        s->offset = (int16_t)n;
        s->trig_level = (int16_t)clampi(s->trig_level + d, 0, 255);   /* level moves with the trace */
        return CHG_FE | CHG_TRIG;
    }
    if (v && s->ctl == CTL_POS_LEVEL) { s->trig_level = (int16_t)clampi(s->trig_level + 2 * v, 0, 255); return CHG_TRIG; }
    return 0;
}
