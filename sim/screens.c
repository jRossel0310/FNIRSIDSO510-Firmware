/* Screen emulator: renders the firmware's real UI code (fw/app/ui.c and fw/core) into a framebuffer on the
   PC and writes the README screenshots. The input signal is the firmware's own generator table (gen.c),
   sampled the way the scope samples it. Only the analog front end is idealised (exact gain, small noise).

   usage: screens OUTDIR      (writes OUTDIR/<name>.ppm, 640x480) */
#include <stdio.h>
#include <string.h>
#include "ui.h"
#include "lcd.h"
#include "units.h"
#include "trigger.h"
#include "render.h"
#include "measure.h"
#include "gen.h"

int sim_save_ppm(const char *path, int scale);

static uint8_t rec[4 * 1200];
static span_t spans[PLOT_W];
static uint16_t gtab[GEN_N_MAX];

static int tdiv_index(uint32_t ns) { for (int i = 0; i < NUM_TDIV; i++) if (tdiv_ns[i] == ns) return i; return 0; }
static int vdiv_index(uint16_t mv) { for (int i = 0; i < NUM_VDIV; i++) if (vdiv_mv[i] == mv) return i; return 0; }

/* Capture the generator output as the scope would at the state's timebase and draw a full screen. */
static void scene(const scope_state_t *st, int batt, const char *dir, const char *name) {
    rate_cfg_t rc = units_rate_for_tdiv(tdiv_ns[st->tdiv]);
    uint32_t sps = units_samples_per_screen(rc.rate_hz, tdiv_ns[st->tdiv]);
    int n = (int)(4 * sps);
    if (n > (int)sizeof rec) n = (int)sizeof rec;

    /* DAC output: the real table, played at the real planned rate */
    gen_cfg_t g = st->gen;
    gen_clamp(&g);
    gen_plan_t p = gen_plan(g.wave == GEN_DC ? 1000u : g.freq_hz);
    gen_table(&g, &p, gtab);
    double dac_rate = 240e6 / ((double)(p.psc + 1) * (p.arr + 1));
    float vpc = units_volts_per_code(st->vdiv, st->probe_x10);
    int zero = ADC_CENTER + st->offset;
    uint32_t lcg = 12345;
    for (int i = 0; i < n; i++) {
        double t = (double)i / rc.rate_hz + 0.37e-3;                 /* arbitrary phase */
        uint32_t k = (uint32_t)(t * dac_rate) % p.n;
        double v = g.on ? gtab[k] * 3.1 / 4095.0 : 0.0;
        if (!st->coupling_dc) v -= g.wave == GEN_DC ? g.dc_dv / 10.0 : g.offset_dv / 10.0;
        lcg = lcg * 1103515245u + 12345u;
        int noise = (int)((lcg >> 16) % 3) - 1;
        int c = (int)(v / vpc + (v >= 0 ? 0.5 : -0.5)) + zero + noise;
        rec[i] = (uint8_t)(c < 0 ? 0 : c > 255 ? 255 : c);
    }

    /* same steps as process_capture() in fw/app/main.c */
    int pre = (int)((uint32_t)st->trig_pos * sps / PLOT_W), post = (int)sps - pre;
    int t = trigger_find(rec, pre, n - post, st->trig_level, trigger_hysteresis(rec, n), (edge_t)st->edge);
    int start = t >= 0 ? t - pre : 0;
    render_spans(rec, n, start, ((sps << 8) + PLOT_W / 2) / PLOT_W, spans, PLOT_W);
    meas_t m;
    measure(rec, n, rc.rate_hz, &m);

    /* same order as redraw_all() */
    ui_draw_static();
    ui_draw_bars(st, 0);
    ui_draw_gen_marker(st->gen.on);
    ui_draw_batt(batt);
    ui_draw_markers(st);
    ui_draw_trace(spans);
    ui_draw_meas(st, &m);
    if (st->menu_open) ui_draw_menu(st);
    if (st->gen_open) ui_draw_gen(st, p.actual_mhz);

    char path[512];
    snprintf(path, sizeof path, "%s/%s.ppm", dir, name);
    if (sim_save_ppm(path, 2) == 0) printf("wrote %s\n", path);
}

int main(int argc, char **argv) {
    const char *dir = argc > 1 ? argv[1] : ".";
    scope_state_t st;

    /* 1 kHz sine from the generator, 500 mV/div, 500 us/div */
    state_defaults(&st);
    st.tdiv = (int8_t)tdiv_index(500000); st.vdiv = (int8_t)vdiv_index(500); st.offset = -100;
    st.trig_level = (int16_t)(ADC_CENTER - 100 + 75);               /* 1.5 V */
    scene(&st, 90, dir, "scope_sine");

    /* 1 kHz square, 30 % duty, three other measurements */
    st.gen.wave = GEN_SQUARE; st.gen.duty_pct = 30;
    st.meas_sel[0] = MEAS_FREQ; st.meas_sel[1] = MEAS_WIDTH_P; st.meas_sel[2] = MEAS_RMS;
    scene(&st, 90, dir, "scope_square");

    /* settings menu over the trace */
    st.menu_open = 1; st.menu_item = 5;
    scene(&st, 90, dir, "menu");
    st.menu_open = 0;

    /* generator page */
    st.gen.wave = GEN_TRIANGLE; st.gen.freq_hz = 5000; st.tdiv = (int8_t)tdiv_index(50000);
    st.meas_sel[0] = MEAS_FREQ; st.meas_sel[1] = MEAS_DUTY_P; st.meas_sel[2] = MEAS_VPP;
    st.gen_open = 1; st.gen_item = 1;
    scene(&st, 90, dir, "generator");
    st.gen_open = 0;

    /* quick Gen control mode in the top bar, triangle on screen */
    st.ctl = CTL_GEN;
    scene(&st, 90, dir, "quick_gen");
    st.ctl = CTL_TIME_VOLT;

    /* 48 MS/s interleaved capture: 50 kHz sine at 2 us/div */
    st.gen.wave = GEN_SINE; st.gen.freq_hz = 50000; st.gen.amp_dv = 30; st.tdiv = (int8_t)tdiv_index(2000);
    st.meas_sel[2] = MEAS_AMPL;
    scene(&st, 90, dir, "fast_48msps");

    /* logger status screen (shown while the PC logs over USB) */
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    state_defaults(&st);
    ui_draw_bars(&st, 0); ui_draw_gen_marker(1); ui_draw_usb(1); ui_draw_batt(90);
    ui_draw_logger(100000, 1234944, 0, 0.48f, 2.52f, 1.50f, 1.0f);
    char path[512];
    snprintf(path, sizeof path, "%s/logger.ppm", dir);
    if (sim_save_ppm(path, 2) == 0) printf("wrote %s\n", path);
    return 0;
}
