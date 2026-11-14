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
#include "spiflash.h"
#include "flashmap.h"
#include "vol.h"
#include "msc.h"
#include "usbd.h"
#include "save.h"

#define REC_MAX (4 * 1200)

static uint8_t recbuf[2][REC_MAX];
static uint8_t *rec = recbuf[0];        /* buffer being captured into */
static uint8_t *shown;                  /* last processed capture (on screen) */
static int shown_n;
static csv_meta_t shown_meta;           /* settings the shown capture was taken with */
static msc_t msc;
static const msc_disk_t vol_disk = {vol_read, vol_write, USER_VOL_SECTORS};
void board_idle(void) { usbd_poll(); }
static span_t spans[PLOT_W];
static cal_t cal;
static scope_state_t st;
static keys_state_t ks;
static rate_cfg_t rc;
static uint32_t sps;          /* samples per screen */
static int rec_n, armed;
static uint32_t toast_until, armed_at, next_batt, settle_until;
#define FE_SETTLE_MS 40       /* range relays / PA7 offset PWM filter */
static int have_trace;        /* spans[] / last_meas hold a drawn trace */
static meas_t last_meas;
static int8_t trace_vdiv; static uint8_t trace_x10;   /* scale last_meas was taken at */
static meas_avg_t meas_avg;
static uint8_t zeroed, zero_failed;
#define UNCAL() (!cal.valid && !zeroed)

static const char *toast_msg;
static void toast(const char *msg, uint32_t ms) { toast_msg = msg; ui_toast(msg); toast_until = millis() + ms; }
/* Status bars; re-show an active toast, since the mode label shares its columns. */
static void draw_bars(void) { ui_draw_bars(&st, UNCAL()); if (toast_until) ui_toast(toast_msg); }

static uint16_t fe_pwm;          /* offset PWM last applied */
static uint32_t fe_settle_need;  /* settle time owed by the last apply_fe() */

static void apply_fe(void) {
    meas_avg_reset(&meas_avg);
    fe_set_range(st.vdiv);
    fe_set_coupling_dc(st.coupling_dc);
    int set = cal_set_for_rate(rc.rate_hz);
    uint16_t pwm = cal_offset_pwm(&cal, set, st.vdiv, st.offset);
    int span = (int)cal.hi[set][st.vdiv] - (int)cal.lo[set][st.vdiv];   /* PWM per 200 codes */
    if (span <= 0) span = 1000;
    uint32_t need = fe_settle_ms(((int)pwm - (int)fe_pwm) * 200 / span);
    if (need > fe_settle_need) fe_settle_need = need;   /* keep the longest pending wait */
    fe_pwm = pwm;
    fe_set_offset_pwm(pwm);
}

static void set_time(int tdiv) {
    rc = units_rate_for_tdiv(tdiv_ns[tdiv]);
    adc_set_rate(rc.psc, rc.arr);
    sps = units_samples_per_screen(rc.rate_hz, tdiv_ns[tdiv]);
    rec_n = (int)(4 * sps);
    if (rec_n > REC_MAX) rec_n = REC_MAX;
}

static void apply_time(void) {
    set_time(st.tdiv);
    apply_fe();
}

/* Block until the offset PWM filter has settled after the last front-end change(s). */
static void fe_settle(void) {
    delay_ms(fe_settle_need > FE_SETTLE_MS ? fe_settle_need : FE_SETTLE_MS);
    fe_settle_need = 0;
}

/* Expected capture time plus 1 s margin. */
static uint32_t capture_timeout_ms(void) { return (uint32_t)((uint64_t)rec_n * 1000u / rc.rate_hz) + 1000u; }

static int capture_blocking(void) {
    adc_start(rec, (uint16_t)rec_n);
    uint32_t t0 = millis(), lim = capture_timeout_ms();
    while (!adc_done()) {
        WDT_CMD = 0xAAAA;
        board_idle();
        if (millis() - t0 > lim) { adc_abort(); toast(" ADC timeout  ", 2000); return -1; }
    }
    return 0;
}

static void redraw_all(void) {
    ui_draw_static();
    draw_bars();
    ui_draw_markers(&st);
    if (have_trace) { ui_draw_trace(spans); ui_draw_meas(&st, &last_meas); }
    next_batt = millis();
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
    shown = rec; shown_n = rec_n;
    shown_meta.rate_hz = rc.rate_hz;
    shown_meta.vdiv_mv = vdiv_mv[st.vdiv];
    shown_meta.coupling_dc = st.coupling_dc;
    shown_meta.probe_x10 = st.probe_x10;
    shown_meta.trig_mode = st.trig_mode;
    shown_meta.edge = st.edge;
    shown_meta.zero_code = (int16_t)(ADC_CENTER + st.offset);
    shown_meta.trig_level_code = st.trig_level;
    shown_meta.trig_index = t >= 0 ? t : start + pre;   /* untriggered Auto: the screen trigger position */
    shown_meta.triggered = t >= 0;
    shown_meta.offset_pwm = fe_pwm;
    rec = (rec == recbuf[0]) ? recbuf[1] : recbuf[0];
    if (st.trig_mode == TRIG_SINGLE) { st.run = 0; draw_bars(); }
    render_spans(shown, shown_n, start, ((sps << 8) + PLOT_W / 2) / PLOT_W, spans, PLOT_W);
    ui_draw_trace(spans);
    meas_t raw;
    measure(shown, shown_n, rc.rate_hz, &raw);         /* whole record (4 screens): more periods, steadier */
    meas_avg_update(&meas_avg, &raw, &last_meas);
    have_trace = 1; trace_vdiv = st.vdiv; trace_x10 = st.probe_x10;
    ui_draw_meas(&st, &last_meas);
}

static void minmax(int *mn, int *mx) {
    *mn = 255; *mx = 0;
    for (int i = 0; i < rec_n; i++) { if (rec[i] < *mn) *mn = rec[i]; if (rec[i] > *mx) *mx = rec[i]; }
}

/* On capture failure: restore a sane state and leave the timeout toast visible. */
#define CAP() do { if (capture_blocking() < 0) { apply_time(); redraw_all(); \
                    toast(" ADC timeout  ", 2000); return; } } while (0)

#ifdef AUTOSET_DEBUG
/* Temporary hardware diagnostics: log each auto-set step, then watch the trace settle. */
static int dbg_n; static int16_t dbg[16][5];
static void dbg_log(int mn, int mx) {
    if (dbg_n < 16) { dbg[dbg_n][0] = st.vdiv; dbg[dbg_n][1] = st.offset; dbg[dbg_n][2] = (int16_t)mn;
                      dbg[dbg_n][3] = (int16_t)mx;
                      dbg[dbg_n][4] = (int16_t)cal_offset_pwm(&cal, cal_set_for_rate(rc.rate_hz), st.vdiv, st.offset); dbg_n++; }
}
static void dbg_num(int *c, int row, int v) {
    char b[8]; char *p = b;
    if (v < 0) { *p++ = '-'; v = -v; }
    fmt_u(p, (uint32_t)v, 1);
    *c = lcd_text(*c, row, b, C_WHITE, C_BLACK);
    *c = lcd_text(*c, row, " ", C_WHITE, C_BLACK);
}
static void dbg_show(void) {
    lcd_fill(0, 16, LCD_W, 16 * 13, C_BLACK);
    lcd_text(0, 1, "it vdiv off min max pwm", C_CYAN, C_BLACK);
    for (int i = 0; i < dbg_n && i < 12; i++) {
        int c = 0;
        dbg_num(&c, 2 + i, i);
        for (int k = 0; k < 5; k++) dbg_num(&c, 2 + i, dbg[i][k]);
    }
    /* settle watch: same settings, capture at +0.1/0.3/1/2 s */
    static const uint16_t at[4] = {100, 300, 1000, 2000};
    uint32_t t0 = millis();
    int c = lcd_text(24, 1, "settle mn/mx", C_CYAN, C_BLACK);
    (void)c;
    for (int k = 0; k < 4; k++) {
        while (millis() - t0 < at[k]) WDT_CMD = 0xAAAA;
        if (capture_blocking() < 0) break;
        int mn, mx; mn = 255; mx = 0;
        for (int i = 0; i < rec_n; i++) { if (rec[i] < mn) mn = rec[i]; if (rec[i] > mx) mx = rec[i]; }
        c = 24; dbg_num(&c, 2 + k, at[k]); dbg_num(&c, 2 + k, mn); dbg_num(&c, 2 + k, mx);
    }
    lcd_text(24, 7, "photo, any key", C_YELLOW, C_BLACK);
    while (keys_read()) WDT_CMD = 0xAAAA;
    while (!keys_read()) WDT_CMD = 0xAAAA;
    while (keys_read()) WDT_CMD = 0xAAAA;
    keys_reset(&ks, KEY_BIT(KEY_UP) | KEY_BIT(KEY_DOWN) | KEY_BIT(KEY_WHEEL_L) | KEY_BIT(KEY_WHEEL_R));
}
#define DBG_LOG(mn, mx) dbg_log(mn, mx)
#else
#define DBG_LOG(mn, mx) ((void)0)
#endif

static void do_autoset(void) {
    toast(" AUTO...      ", 60000);
    adc_abort(); armed = 0;
    fe_settle_need = 0;          /* only the moves made by auto-set itself count from here */
    st.offset = 0; st.trig_mode = TRIG_AUTO; st.trig_pos = PLOT_W / 2; st.run = 1;
    st.tdiv = TDIV_1MS;
    apply_time();
    fe_settle();
    for (int it = 0; it < AUTOSET_MAX_ITER; it++) {
        CAP();
        int mn, mx; minmax(&mn, &mx);
        DBG_LOG(mn, mx);
        int v = st.vdiv, o = st.offset;
        int again = autoset_fit(&v, &o, mn, mx);
        st.vdiv = (int8_t)v; st.offset = (int16_t)o;
        apply_fe();
        fe_settle();
        if (!again) break;
    }
    meas_t m;
    CAP();
    measure(rec, rec_n, rc.rate_hz, &m);
    if (m.valid && m.freq_hz > 20000.0f) {
        st.tdiv = TDIV_10US; apply_time(); fe_settle(); CAP();
        measure(rec, rec_n, rc.rate_hz, &m);
    }
    if (m.valid) { int nt = autoset_tdiv(m.freq_hz); if (nt >= 0) st.tdiv = (int8_t)nt; }
    apply_time();
    fe_settle();
    CAP();
    int mn, mx; minmax(&mn, &mx);
    st.trig_level = (int16_t)((mn + mx) / 2);
    DBG_LOG(mn, mx);
    toast_until = 0;
    ui_toast_clear();
#ifdef AUTOSET_DEBUG
    dbg_show(); dbg_n = 0;
#endif
    redraw_all();
}

static void power_down(void);
static uint32_t pwr_since;

/* Called from long blocking operations: hold Power ~1 s to switch off. */
static void poll_power(void) {
    if (keys_read() & KEY_BIT(KEY_POWER)) {
        if (!pwr_since) pwr_since = millis() | 1u;
        else if (millis() - pwr_since >= 1000) power_down();
    } else pwr_since = 0;
}

static void wait_power_aware(uint32_t ms) {
    for (uint32_t t0 = millis(); millis() - t0 < ms; ) { WDT_CMD = 0xAAAA; board_idle(); poll_power(); }
}

/* Trace mean in 1/16 ADC codes with the offset PWM at pwm, after the PA7 filter has settled
   (wait scaled to the PWM move; span = PWM per 200 codes). */
static int mean_x16_at_pwm(uint16_t pwm, int span) {
    if (zero_failed) return ADC_CENTER * 16;
    int dcodes = span > 0 ? ((int)pwm - (int)fe_pwm) * 200 / span : 200;
    fe_set_offset_pwm(pwm);
    fe_pwm = pwm;
    wait_power_aware(fe_settle_ms(dcodes));
    poll_power();
    if (capture_blocking() < 0) { zero_failed = 1; return ADC_CENTER * 16; }
    uint32_t sum = 0;
    for (int i = 0; i < rec_n; i++) sum += rec[i];
    return (int)(sum * 16u / (uint32_t)rec_n);
}

static void do_autozero(void) {
    static const struct { uint8_t set, tdiv; } plan[2] = {
        { CAL_SET_SLOW, TDIV_1MS }, { CAL_SET_MID, TDIV_10US },
    };
    toast(" ZERO: short! ", 60000);
    adc_abort(); armed = 0;
    wait_power_aware(2000);
    zero_failed = 0;
    cal_t tmp = cal;                            /* shifts are committed only if every range succeeds */
    for (int p = 0; p < 2; p++) {
        set_time(plan[p].tdiv);
        for (int r = 0; r < NUM_VDIV; r++) {
            char b[4];
            ui_toast(" ZERO range   ");
            lcd_text(31, 0, fmt_u(b, (uint32_t)r, 1), C_BLACK, C_YELLOW);
            fe_set_range(r);
            int set = plan[p].set;
            int span = (int)tmp.hi[set][r] - (int)tmp.lo[set][r];
            uint16_t pw = cal_offset_pwm(&tmp, set, r, 0);
            /* measure settled, correct directly; repeat until within half a code (max 3) */
            for (int k = 0; k < 3 && !zero_failed; k++) {
                int m = mean_x16_at_pwm(pw, span);
                if (m - ADC_CENTER * 16 >= -8 && m - ADC_CENTER * 16 <= 8) break;
                pw = cal_zero_step(pw, m, span);
            }
            if (zero_failed) {
                apply_time();
                redraw_all();
                toast(" ZERO failed  ", 3000);
                return;
            }
            cal_shift_zero(&tmp, plan[p].set, r, pw);
        }
    }
    cal = tmp;
    zeroed = 1;
    apply_time();
    redraw_all();
    toast(" ZERO done    ", 2000);
}

static void power_down(void) {
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(0);
    usbd_detach();               /* a host sees an unplug, not a dead drive */
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
    spiflash_init();
    msc_init(&msc, &vol_disk);
    usbd_init(&msc);
    gen_sine_init(2399);          /* 1 kHz test sine until sub-project 3 adds the generator UI */
    apply_time();
    redraw_all();
    backlight(1);
    if (!cal.valid) toast("No factory cal", 3000);

    for (;;) {
        usbd_poll();
        kev_t ev[8];
        int ne = keys_update(&ks, keys_read(), millis(), ev, 8);
        uint32_t chg = 0;
        for (int i = 0; i < ne; i++) chg |= control_handle(&st, ev[i]);

        if (chg & ACT_POWEROFF) power_down();
        if (chg & CHG_TIME) { adc_abort(); armed = 0; apply_time(); have_trace = 0; settle_until = millis() + FE_SETTLE_MS; }
        if (chg & CHG_FE) {
            adc_abort(); armed = 0; apply_fe(); settle_until = millis() + FE_SETTLE_MS;
            if (st.vdiv != trace_vdiv || st.probe_x10 != trace_x10) have_trace = 0;   /* held Vpp would be mis-scaled */
        }
        if (chg & CHG_RUN) { adc_abort(); armed = 0; }
        if (chg & CHG_SCREEN) redraw_all();
        else {
            if (chg & CHG_BARS) draw_bars();
            if (chg & (CHG_TRIG | CHG_FE)) ui_draw_markers(&st);
            if ((chg & CHG_MENU) && st.menu_open) ui_draw_menu(&st);
        }
        if (chg & ACT_SAVE) {
            if (!have_trace || !shown) toast(" No capture   ", 1500);
            else {
                adc_abort(); armed = 0;
                toast(" Saving...    ", 60000);
                const char *r = save_capture(shown, shown_n, &shown_meta, &msc, poll_power);
                toast(r, 2500);
            }
        }
        if (chg & ACT_AUTOSET) do_autoset();
        if (chg & ACT_AUTOZERO) do_autozero();

        if (st.run && !st.menu_open) {
            if (!armed) {
                if ((int32_t)(millis() - settle_until) >= 0) {   /* not while the front end settles */
                    adc_start(rec, (uint16_t)rec_n); armed = 1; armed_at = millis();
                }
            }
            else if (adc_done()) { armed = 0; process_capture(); }
            else if (millis() - armed_at > capture_timeout_ms()) {
                adc_abort(); armed = 0; toast(" ADC timeout  ", 2000);
            }
        }

        uint32_t now = millis();
        if ((int32_t)(now - next_batt) >= 0) {
            next_batt = now + 2000;
            uint16_t raw = batt_adc_read();
            if (raw != 0xFFFF) {                    /* 0xFFFF = ADC timeout: keep the old reading */
                float v = (float)raw * 3.3f / 4095.0f * 2.0f;
                ui_draw_batt(units_batt_pct(v));
            }
        }
        if (toast_until && (int32_t)(now - toast_until) >= 0) {
            toast_until = 0; ui_toast_clear(); draw_bars();   /* restore the mode label under it */
        }
        WDT_CMD = 0xAAAA;
    }
}
