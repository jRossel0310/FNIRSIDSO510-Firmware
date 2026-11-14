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
#include <string.h>
#include "stream.h"
#include "logger.h"
#include "gen.h"
#include "gendac.h"
#include "ilv.h"
#include "settings.h"
#include "cfgfile.h"

#define REC_MAX (4 * 1200)

static uint8_t recbuf[2][REC_MAX];
static uint8_t ilv_raw[4 * (REC_MAX / 4 + ADC_ILV_PAD)];   /* per-stream buffers of an interleaved capture */
_Static_assert(REC_MAX % 4 == 0, "an interleaved record splits evenly over 4 streams");
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
static uint8_t cfg_saved[SETTINGS_LEN];   /* settings blob as stored on the flash */
static uint8_t cfg_have;                  /* cfg_saved is valid */
static uint8_t fs_busy;                   /* a FatFs operation of the app is in progress */
#define UNCAL() (!cal.valid && !zeroed)

static uint8_t usb_marker;      /* "USB" shown in the top bar (frames streaming) */
static const char *toast_msg;
static void toast(const char *msg, uint32_t ms) { toast_msg = msg; ui_toast(msg); toast_until = millis() + ms; }
/* Status bars; re-show an active toast, since the mode label shares its columns. */
static void draw_bars(void) { ui_draw_bars(&st, UNCAL()); if (toast_until) ui_toast(toast_msg);
                         if (!toast_until) ui_draw_gen_marker(st.gen.on);   /* a toast covers cols 19..32 */
                         if (usb_marker) ui_draw_usb(1); }

static uint16_t gen_tab[GEN_N_MAX];      /* played by DMA: must stay static */
static gen_plan_t gen_pl;

/* Push the generator settings to the hardware. */
static void gen_apply(void) {
    gen_clamp(&st.gen);
    gen_pl = gen_plan(st.gen.wave == GEN_DC ? 1000u : st.gen.freq_hz);
    if (!st.gen.on) { gendac_off(); return; }
    gendac_hold();                        /* DMA must not read the table while it is rewritten */
    gen_table(&st.gen, &gen_pl, gen_tab);
    gendac_load(gen_tab, gen_pl.n, gen_pl.psc, gen_pl.arr);
}

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
    if (rc.streams > 1) adc_ilv_enter(); else adc_set_rate(rc.psc, rc.arr);
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

/* Arm one capture of rec_n samples into rec. */
static void cap_start(void) {
    if (rc.streams > 1) {
        adc_ilv_start(ilv_raw, (uint16_t)(rec_n / rc.streams), rc.streams);
        /* An interleaved capture lasts at most ~100 us and each ADC value is on the pins for only 5 CPU
           clocks: wait for it here, so no other bus traffic from the main loop can delay a DMA read. */
        for (uint32_t spin = 0; spin < 200000u && !adc_ilv_done(); spin++) { }
    } else adc_start(rec, (uint16_t)rec_n);
}

/* 1 once the armed capture is complete and rec holds the record. */
static int cap_done(void) {
    if (rc.streams == 1) return adc_done();
    if (!adc_ilv_done()) return 0;
    const uint8_t *src[4];
    for (int k = 0; k < rc.streams; k++) src[k] = adc_ilv_stream(k);
    ilv_weave(rec, src, rc.streams, (uint32_t)(rec_n / rc.streams));
    ilv_despike(rec, (uint32_t)rec_n);
    return 1;
}

static int capture_blocking(void) {
    cap_start();
    uint32_t t0 = millis(), lim = capture_timeout_ms();
    while (!cap_done()) {
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
    if (st.gen_open) ui_draw_gen(&st, gen_pl.actual_mhz);
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
    stream_frame(shown, shown_n, &shown_meta);
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
    static const struct { uint8_t set, tdiv; } plan[3] = {
        { CAL_SET_SLOW, TDIV_1MS }, { CAL_SET_MID, TDIV_10US }, { CAL_SET_FAST, TDIV_2US },
    };
    toast(" ZERO: short! ", 60000);
    adc_abort(); armed = 0;
    wait_power_aware(2000);
    zero_failed = 0;
    cal_t tmp = cal;                            /* shifts are committed only if every range succeeds */
    for (int p = 0; p < 3; p++) {
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

/* Store the settings if they changed. Only from power_down(), after the USB detach. */
static void settings_store(void) {
    static uint8_t b[SETTINGS_LEN];
    if (fs_busy) return;                         /* power-off in the middle of a capture save */
    settings_pack(b, &st, &cal, zeroed);
    if (cfg_have && !memcmp(b, cfg_saved, SETTINGS_LEN)) return;
    cfgfile_save(b, SETTINGS_LEN);               /* best effort: power-off continues either way */
}

static void power_down(void) {
    gendac_off();                /* no signal on the BNC while the scope is off */
    adc_abort();                 /* stop the logger ring DMA before power-off */
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(0);
    usbd_detach();               /* a host sees an unplug, not a dead drive */
    settings_store();
    power_off();
    for (;;) WDT_CMD = 0xAAAA;   /* on USB power the scope stays alive; keep the watchdog quiet */
}

static uint8_t ring[16384];
static uint8_t stage[sizeof ring / 2];   /* stable copy of the half being sent */
static uint32_t stage_len, stage_off;    /* stage[stage_off..stage_len) is still to be sent */
static uint32_t ring_n;                 /* even, <= sizeof ring: halves of ~100 ms */
static uint32_t ring_consumed;
static uint32_t log_rate, log_next_ui;
static int log_vmin = 255, log_vmax, log_cnt; static uint32_t log_sum;

/* Front-end settle while logging (V/div or offset change, and logger start). A sample taken before the
   change, or while the range relays and the PA7 offset-PWM filter settle, must never go out under the
   new LOGMETA. The rule:
   - discard_until = now + max(fe_settle_need, FE_SETTLE_MS) (the sub-project 1 settle model);
   - on a change while running: the unsent rest of the stage is lost, and so is every ring half up to
     the first one that started filling after discard_until. At the first pass at or after
     discard_until (time read before the completion counter, count c), that is the half whose
     completion makes the count c + 2: the half completing at c + 1 was already filling at that pass.
     This also drops the half that was in progress at the change. All of it is counted lost (DROP);
   - at logger start the ring DMA is simply not started until discard_until, so no sample exists from
     before the change and the stream starts at index 0 without a gap;
   - LOGMETA is queued just before the first clean half is staged (stream.c sends the DROP for the
     lost samples first), so the first LOG after a LOGMETA holds only samples taken at that setting. */
static uint8_t log_ring_wait;           /* start: ring DMA not started yet */
static uint8_t log_hold, log_clean_set; /* change while running: halves discarded until log_clean_from */
static uint32_t log_discard_until, log_clean_from;

static uint32_t take_settle(void) {
    uint32_t need = fe_settle_need > FE_SETTLE_MS ? fe_settle_need : FE_SETTLE_MS;
    fe_settle_need = 0;
    return need;
}

static void log_send_meta(void) {
    stream_logmeta(log_rate, vdiv_mv[st.vdiv], st.probe_x10, st.coupling_dc,
                   (int16_t)(ADC_CENTER + st.offset), fe_pwm);
}

static void log_fe_changed(void) {      /* call right after apply_fe() */
    log_discard_until = millis() + take_settle();
    if (log_ring_wait) return;          /* not started yet: just start later */
    if (stage_off < stage_len) stream_log_lost(stage_len - stage_off);
    stage_len = stage_off = 0;
    log_hold = 1; log_clean_set = 0;
}

static int hook_log_start(uint32_t rate) {
    rate_cfg_t q = units_rate_for_hz(rate);
    if (!q.rate_hz) return -1;
    adc_abort(); armed = 0;
    st.menu_open = 0; st.gen_open = 0;
    rc = q;                              /* apply_fe uses rc.rate_hz for the cal set */
    adc_set_rate(q.psc, q.arr);
    apply_fe();
    log_rate = rate;
    ring_n = rate / 5;                   /* two halves of ~100 ms */
    if (ring_n > sizeof ring) ring_n = sizeof ring;
    if (ring_n < 256) ring_n = 256;
    ring_n &= ~1u;
    log_vmin = 255; log_vmax = 0; log_sum = 0; log_cnt = 0;
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    log_next_ui = millis();
    stage_len = stage_off = 0;
    log_hold = 0; log_clean_set = 0;
    log_ring_wait = 1;                   /* ring + LOGMETA once the front end has settled */
    log_discard_until = millis() + take_settle();
    return 0;
}

static void hook_log_stop(void) {
    adc_abort(); armed = 0;
    log_ring_wait = 0; log_hold = 0; stage_len = stage_off = 0;
    apply_time();                         /* back to the scope's own timebase */
    settle_until = millis() + FE_SETTLE_MS;
    have_trace = 0;
    redraw_all();
}

static const stream_hooks_t hooks = {hook_log_start, hook_log_stop};

/* Take the newest completed ring half (if any) into the stage. */
static void log_take_half(void) {
    uint32_t t = millis();                             /* before the counter: see the settle rule */
    uint32_t done = adc_ring_completed();              /* one snapshot per pass */
    if (log_hold && !log_clean_set && (int32_t)(t - log_discard_until) >= 0) {
        log_clean_from = done + 2; log_clean_set = 1;
    }
    ring_pick_t pk = logger_pick(done, &ring_consumed, ring_n);
    if (pk.first < 0) return;
    /* a newer half is here: what is left of the stage was not sent in time */
    if (stage_off < stage_len) stream_log_lost(stage_len - stage_off);
    stage_len = stage_off = 0;
    if (pk.lost) stream_log_lost(pk.lost);
    uint32_t hl = ring_n / 2;
    if (log_hold && !(log_clean_set && (int32_t)(done - log_clean_from) >= 0)) { stream_log_lost(hl); return; }
    memcpy(stage, ring + (uint32_t)pk.first * hl, hl);
    if (adc_ring_completed() != done) { stream_log_lost(hl); return; }   /* DMA may have wrapped into it */
    if (log_hold) { log_hold = 0; log_send_meta(); }   /* first clean half: its settings go first */
    stage_len = hl;
    for (uint32_t i = 0; i < hl; i++) {
        if (stage[i] < log_vmin) log_vmin = stage[i];
        if (stage[i] > log_vmax) log_vmax = stage[i];
        log_sum += stage[i];
    }
    log_cnt += (int)hl;
}

/* Logger mode, one main-loop step: move completed ring halves to USB (as much of the staged half as
   fits in the TX FIFO each pass), keys, status screen. */
static void logger_step(void) {
    if (log_ring_wait) {
        if ((int32_t)(millis() - log_discard_until) >= 0) {
            log_ring_wait = 0;
            log_send_meta();
            ring_consumed = 0;
            adc_ring_start(ring, (uint16_t)ring_n);
        }
    } else {
        log_take_half();
        if (stage_off < stage_len) stage_off += stream_log_fit(stage + stage_off, stage_len - stage_off);
    }
    kev_t ev[8];
    int ne = keys_update(&ks, keys_read(), millis(), ev, 8);
    for (int i = 0; i < ne; i++) {
        int tap = ev[i].type == KEV_SHORT || ev[i].type == KEV_REPEAT;
        if (ev[i].key == KEY_POWER && ev[i].type == KEV_LONG) power_down();
        else if (ev[i].key == KEY_RUN && ev[i].type == KEV_SHORT) { stream_stop_log(); return; }
        else if (tap && (ev[i].key == KEY_UP || ev[i].key == KEY_DOWN)) {
            int nv = st.vdiv + (ev[i].key == KEY_UP ? -1 : 1);       /* Up = more sensitive */
            if (nv >= 0 && nv < NUM_VDIV) { st.vdiv = (int8_t)nv; apply_fe(); log_fe_changed(); }
        }
    }
    uint32_t now = millis();
    if ((int32_t)(now - log_next_ui) >= 0) {
        log_next_ui = now + 250;
        uint32_t sent, dropped;
        stream_log_stats(&sent, &dropped);
        float vpc = units_volts_per_code(st.vdiv, st.probe_x10);
        int zero = ADC_CENTER + st.offset;
        float mean = log_cnt ? (float)log_sum / (float)log_cnt : (float)zero;
        ui_draw_logger(log_rate, sent, dropped,
                       log_cnt ? (float)(log_vmin - zero) * vpc : 0.0f,
                       log_cnt ? (float)(log_vmax - zero) * vpc : 0.0f,
                       (mean - (float)zero) * vpc,
                       (float)vdiv_mv[st.vdiv] * 1e-3f * (st.probe_x10 ? 10.0f : 1.0f));
        log_vmin = 255; log_vmax = 0; log_sum = 0; log_cnt = 0;
    }
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
    {   /* stored settings (before USB starts, so nobody else is using the volume) */
        static uint8_t b[SETTINGS_LEN];
        int z = 0;
        if (cfgfile_load(b, sizeof b) == (int)SETTINGS_LEN && settings_unpack(b, SETTINGS_LEN, &st, &cal, &z)) {
            memcpy(cfg_saved, b, SETTINGS_LEN); cfg_have = 1;
            if (z) zeroed = 1;
        }
    }
    msc_init(&msc, &vol_disk);
    usbd_init(&msc);
    stream_init(&hooks);
    gendac_init(); gen_apply();
    apply_time();
    redraw_all();
    backlight(1);
    if (!cal.valid) toast("No factory cal", 3000);

    for (;;) {
        usbd_poll();
        stream_poll();
        if (stream_mode() == STREAM_LOG) { logger_step(); WDT_CMD = 0xAAAA; continue; }
        int usb_on = stream_mode() == STREAM_FRAMES;
        if (usb_on != usb_marker) { usb_marker = (uint8_t)usb_on; ui_draw_usb(usb_on); }
        kev_t ev[8];
        int ne = keys_update(&ks, keys_read(), millis(), ev, 8);
        uint32_t chg = 0;
        for (int i = 0; i < ne; i++) chg |= control_handle(&st, ev[i]);

        if (chg & ACT_POWEROFF) power_down();
        if (chg & CHG_TIME) { adc_abort(); armed = 0; apply_time(); have_trace = 0;
                         uint32_t need = fe_settle_need > FE_SETTLE_MS ? fe_settle_need : FE_SETTLE_MS;
                         fe_settle_need = 0; settle_until = millis() + need; }
        if (chg & CHG_FE) {
            adc_abort(); armed = 0; apply_fe(); settle_until = millis() + FE_SETTLE_MS;
            if (st.vdiv != trace_vdiv || st.probe_x10 != trace_x10) have_trace = 0;   /* held Vpp would be mis-scaled */
        }
        if (chg & CHG_RUN) { adc_abort(); armed = 0; }
        if (chg & CHG_GEN) gen_apply();
        if (chg & CHG_SCREEN) redraw_all();
        else {
            if (chg & CHG_BARS) draw_bars();
            if (chg & (CHG_TRIG | CHG_FE)) ui_draw_markers(&st);
            if ((chg & CHG_MENU) && st.menu_open) ui_draw_menu(&st);
            if ((chg & CHG_GENPAGE) && st.gen_open) ui_draw_gen(&st, gen_pl.actual_mhz);
        }
        if (chg & ACT_SAVE) {
            if (!have_trace || !shown) toast(" No capture   ", 1500);
            else {
                adc_abort(); armed = 0;
                toast(" Saving...    ", 60000);
                fs_busy = 1;
                const char *r = save_capture(shown, shown_n, &shown_meta, &msc, poll_power);
                fs_busy = 0;
                toast(r, 2500);
            }
        }
        if (chg & ACT_AUTOSET) do_autoset();
        if (chg & ACT_AUTOZERO) do_autozero();

        if (st.run && !st.menu_open && !st.gen_open) {
            if (!armed) {
                if ((int32_t)(millis() - settle_until) >= 0) {   /* not while the front end settles */
                    cap_start(); armed = 1; armed_at = millis();
                }
            }
            else if (cap_done()) { armed = 0; process_capture(); }
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
