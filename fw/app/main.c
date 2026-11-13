// DSO-510 custom firmware: core scope (sub-project 1).
#include "board.h"
#include "lcd.h"
#include "adc.h"
#include "frontend.h"
#include "testsig.h"
#include "units.h"
#include "trigger.h"
#include "render.h"
#include "measure.h"
#include "cal.h"
#include "control.h"
#include "autoset.h"
#include "keys.h"
#include "ui.h"

#define REC_MAX (4 * 1200)

static uint8_t rec[REC_MAX];
static span_t spans[PLOT_W];
static cal_t cal;
static scope_state_t st;
static keys_state_t ks;
static rate_cfg_t rc;
static uint32_t sps;          /* samples per screen */
static int rec_n, armed;
static uint32_t toast_until, armed_at;

static void toast(const char *msg, uint32_t ms) { ui_toast(msg); toast_until = millis() + ms; }

static void apply_fe(void) {
    fe_set_range(st.vdiv);
    fe_set_coupling_dc(st.coupling_dc);
    fe_set_offset_pwm(cal_offset_pwm(&cal, cal_set_for_rate(rc.rate_hz), st.vdiv, st.offset));
}

static void apply_time(void) {
    rc = units_rate_for_tdiv(tdiv_ns[st.tdiv]);
    adc_set_rate(rc.psc, rc.arr);
    sps = units_samples_per_screen(rc.rate_hz, tdiv_ns[st.tdiv]);
    rec_n = (int)(4 * sps);
    if (rec_n > REC_MAX) rec_n = REC_MAX;
    apply_fe();
}

/* Expected capture time plus 1 s margin. */
static uint32_t capture_timeout_ms(void) { return (uint32_t)((uint64_t)rec_n * 1000u / rc.rate_hz) + 1000u; }

static int capture_blocking(void) {
    adc_start(rec, (uint16_t)rec_n);
    uint32_t t0 = millis(), lim = capture_timeout_ms();
    while (!adc_done()) {
        WDT_CMD = 0xAAAA;
        if (millis() - t0 > lim) { adc_abort(); toast(" ADC timeout  ", 2000); return -1; }
    }
    return 0;
}

static void redraw_all(void) {
    ui_draw_static();
    ui_draw_bars(&st, !cal.valid);
    ui_draw_markers(&st);
    if (st.menu_open) ui_draw_menu(&st);
}

static void process_capture(void) {
    int pre = (int)((uint32_t)st.trig_pos * sps / PLOT_W);
    int post = (int)sps - pre;
    int t = trigger_find(rec, pre, rec_n - post, st.trig_level, trigger_hysteresis(rec, rec_n), (edge_t)st.edge);
    int start;
    if (t >= 0) start = t - pre;
    else if (st.trig_mode == TRIG_AUTO) start = 0;
    else return;                                    /* Normal/Single: keep previous trace */
    if (st.trig_mode == TRIG_SINGLE) { st.run = 0; ui_draw_bars(&st, !cal.valid); }
    render_spans(rec, rec_n, start, (sps << 8) / PLOT_W, spans, PLOT_W);
    ui_draw_trace(spans);
    meas_t m;
    measure(rec + start, (int)sps, rc.rate_hz, &m);
    ui_draw_meas(&st, &m);
}

static void minmax(int *mn, int *mx) {
    *mn = 255; *mx = 0;
    for (int i = 0; i < rec_n; i++) { if (rec[i] < *mn) *mn = rec[i]; if (rec[i] > *mx) *mx = rec[i]; }
}

static void do_autoset(void) {
    toast(" AUTO...      ", 60000);
    adc_abort(); armed = 0;
    st.offset = 0; st.trig_mode = TRIG_AUTO; st.trig_pos = PLOT_W / 2; st.run = 1;
    st.tdiv = TDIV_1MS;
    apply_time();
    for (int it = 0; it < 5; it++) {
        capture_blocking();
        int mn, mx; minmax(&mn, &mx);
        int nv = autoset_vdiv(st.vdiv, mn, mx);
        if (nv == st.vdiv) break;
        st.vdiv = (int8_t)nv; apply_fe(); delay_ms(30);
    }
    meas_t m;
    capture_blocking();
    measure(rec, rec_n, rc.rate_hz, &m);
    if (m.valid && m.freq_hz > 20000.0f) {
        st.tdiv = TDIV_10US; apply_time(); capture_blocking();
        measure(rec, rec_n, rc.rate_hz, &m);
    }
    if (m.valid) { int nt = autoset_tdiv(m.freq_hz); if (nt >= 0) st.tdiv = (int8_t)nt; }
    apply_time();
    capture_blocking();
    int mn, mx; minmax(&mn, &mx);
    st.trig_level = (int16_t)((mn + mx) / 2);
    redraw_all();
    ui_toast_clear();
}

static int mean_at_pwm(uint16_t pwm, void *ctx) {
    (void)ctx;
    fe_set_offset_pwm(pwm);
    delay_ms(40);                               /* let the PA7 PWM filter settle */
    capture_blocking();
    uint32_t sum = 0;
    for (int i = 0; i < rec_n; i++) sum += rec[i];
    return (int)(sum / (uint32_t)rec_n);
}

static void do_autozero(void) {
    toast(" ZERO: short! ", 60000);
    adc_abort(); armed = 0;
    int set = cal_set_for_rate(rc.rate_hz), keep = st.vdiv;
    for (int r = 0; r < NUM_VDIV; r++) {
        char b[4];
        ui_toast(" ZERO range   ");
        lcd_text(31, 0, fmt_u(b, (uint32_t)r, 1), C_BLACK, C_YELLOW);
        fe_set_range(r);
        uint16_t p = cal_search_pwm(mean_at_pwm, 0, ADC_CENTER, cal.lo[set][r], cal.hi[set][r]);
        cal_shift_zero(&cal, set, r, p);
    }
    cal.valid = 1;
    st.vdiv = (int8_t)keep;
    apply_fe();
    redraw_all();
    toast(" ZERO done    ", 2000);
}

static void power_down(void) {
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(0);
    power_off();
    for (;;) WDT_CMD = 0xAAAA;   /* on USB power the scope stays alive; keep the watchdog quiet */
}

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    batt_adc_init();
    cal_parse((const uint32_t *)CAL_PAGE_ADDR, &cal);
    state_defaults(&st);
    keys_reset(&ks, KEY_BIT(KEY_UP) | KEY_BIT(KEY_DOWN) | KEY_BIT(KEY_WHEEL_L) | KEY_BIT(KEY_WHEEL_R));
    fe_init();
    adc_init();
    dac2_bias_init();
    cal_out_init();
    gen_sine_init(2399);          /* 1 kHz test sine until sub-project 3 adds the generator UI */
    apply_time();
    redraw_all();
    backlight(1);
    if (!cal.valid) toast(" no factory cal", 3000);

    uint32_t next_batt = 0;
    for (;;) {
        kev_t ev[8];
        int ne = keys_update(&ks, keys_read(), millis(), ev, 8);
        uint32_t chg = 0;
        for (int i = 0; i < ne; i++) chg |= control_handle(&st, ev[i]);

        if (chg & ACT_POWEROFF) power_down();
        if (chg & CHG_TIME) { adc_abort(); armed = 0; apply_time(); }
        if (chg & CHG_FE) { adc_abort(); armed = 0; apply_fe(); }
        if (chg & CHG_RUN) { adc_abort(); armed = 0; }
        if (chg & CHG_SCREEN) redraw_all();
        else {
            if (chg & CHG_BARS) ui_draw_bars(&st, !cal.valid);
            if (chg & (CHG_TRIG | CHG_FE)) ui_draw_markers(&st);
            if ((chg & CHG_MENU) && st.menu_open) ui_draw_menu(&st);
        }
        if (chg & ACT_SAVE) toast(" CSV: step 2  ", 1500);
        if (chg & ACT_AUTOSET) do_autoset();
        if (chg & ACT_AUTOZERO) do_autozero();

        if (st.run && !st.menu_open) {
            if (!armed) { adc_start(rec, (uint16_t)rec_n); armed = 1; armed_at = millis(); }
            else if (adc_done()) { armed = 0; process_capture(); }
            else if (millis() - armed_at > capture_timeout_ms()) {
                adc_abort(); armed = 0; toast(" ADC timeout  ", 2000);
            }
        }

        uint32_t now = millis();
        if ((int32_t)(now - next_batt) >= 0) {
            next_batt = now + 2000;
            float v = (float)batt_adc_read() * 3.3f / 4095.0f * 2.0f;
            ui_draw_batt(units_batt_pct(v));
        }
        if (toast_until && (int32_t)(now - toast_until) >= 0) { toast_until = 0; ui_toast_clear(); }
        WDT_CMD = 0xAAAA;
    }
}
