#include "ui.h"
#include "lcd.h"
#include "fmt.h"
#include "units.h"
#include "gen.h"

#define C_ORANGE RGB565(255, 140, 0)

static int code_to_y(int code) {
    int y = 227 - code;
    return y < 0 ? 0 : y > PLOT_H - 1 ? PLOT_H - 1 : y;
}

static inline uint16_t bg_px(int x, int y) {
    if ((x % 30) == 0 && (y % 4) == 0) return C_GRAY;     /* 10 vertical divisions */
    if ((y % 25) == 0 && (x % 6) == 0) return C_GRAY;     /* 8 horizontal divisions */
    if (x == PLOT_W / 2 && (y % 2) == 0) return C_GRAY;   /* centre lines */
    if (y == PLOT_H / 2 && (x % 2) == 0) return C_GRAY;
    return C_BLACK;
}

static void draw_col(int x, int y0, int y1) {
    lcd_window(PLOT_X + x, PLOT_Y, PLOT_X + x, PLOT_Y + PLOT_H - 1);
    for (int y = 0; y < PLOT_H; y++) lcd_push((y >= y0 && y <= y1) ? C_YELLOW : bg_px(x, y));
}

void ui_draw_static(void) {
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    for (int x = 0; x < PLOT_W; x++) draw_col(x, -1, -1);
}

void ui_draw_trace(const span_t *sp) {
    for (int x = 0; x < PLOT_W; x++) {
        if (sp[x].lo < 0) { draw_col(x, -1, -1); continue; }
        draw_col(x, code_to_y(sp[x].hi), code_to_y(sp[x].lo));
    }
}

/* Top row of a 3-row marker centred on y, kept inside the PLOT_H rows that get erased. */
static int marker_y0(int y) { y = y > 1 ? y - 1 : 0; return y > PLOT_H - 3 ? PLOT_H - 3 : y; }

void ui_draw_markers(const scope_state_t *s) {
    /* 0 V baseline: left margin */
    lcd_fill(0, PLOT_Y, PLOT_X - 1, PLOT_H, C_BLACK);
    int y = code_to_y(ADC_CENTER + s->offset);
    lcd_fill(0, PLOT_Y + marker_y0(y), PLOT_X - 2, 3, C_YELLOW);
    /* trigger level: right margin */
    lcd_fill(PLOT_X + PLOT_W + 1, PLOT_Y, LCD_W - PLOT_X - PLOT_W - 1, PLOT_H, C_BLACK);
    y = code_to_y(s->trig_level);
    lcd_fill(PLOT_X + PLOT_W + 2, PLOT_Y + marker_y0(y), 7, 3, C_CYAN);
    /* trigger position: strip above the plot */
    lcd_fill(PLOT_X, 16, PLOT_W + 2, 4, C_BLACK);   /* marker at trig_pos 299 reaches x = PLOT_W + 1 */
    int x = s->trig_pos - 2 < 0 ? 0 : s->trig_pos - 2;
    lcd_fill(PLOT_X + x, 16, 5, 4, C_ORANGE);
}

void ui_draw_gen_marker(int on) { lcd_text(29, 0, on ? "G" : " ", C_BLACK, on ? C_GREEN : C_BLACK); }

/* "2.0V" from tenths of a volt */
static char *fmt_dv(char *b, int dv) {
    b[0] = (char)('0' + dv / 10); b[1] = '.'; b[2] = (char)('0' + dv % 10); b[3] = 'V'; b[4] = 0;
    return b;
}

/* millihertz -> "999.001Hz" / "1000.00Hz" / "49868.8Hz" (truncated), padded with spaces to width w */
static char *fmt_hz_mhz(char *b, uint32_t mhz, int w) {
    uint32_t hz = mhz / 1000u, fr = mhz % 1000u;
    int dec = hz < 1000u ? 3 : hz < 10000u ? 2 : 1;
    char t[12];
    int k = 0;
    const char *p = fmt_u(t, hz, 1);
    while (*p) b[k++] = *p++;
    b[k++] = '.';
    if (dec == 2) fr /= 10u; else if (dec == 1) fr /= 100u;
    p = fmt_u(t, fr, dec);
    for (; *p; p++) b[k++] = *p == ' ' ? '0' : *p;
    b[k++] = 'H'; b[k++] = 'z';
    while (k < w) b[k++] = ' ';
    b[k] = 0;
    return b;
}

void ui_draw_gen(const scope_state_t *s, uint32_t actual_mhz) {
    static const char *labels[GEN_ROWS] = {"Output", "Waveform", "Frequency", "Amplitude", "Offset", "Duty"};
    const gen_cfg_t *g = &s->gen;
    char v[GEN_ROWS][16], b[16];
    const char *p;
    int k;
    /* row values */
    p = g->on ? "On" : "Off";            for (k = 0; p[k]; k++) v[0][k] = p[k]; v[0][k] = 0;
    p = gen_wave_name(g->wave);          for (k = 0; p[k]; k++) v[1][k] = p[k]; v[1][k] = 0;
    p = fmt_si(b, (float)g->freq_hz, "Hz", 1);
    for (k = 0; p[k]; k++) v[2][k] = p[k];
    if (g->fine) { v[2][k++] = ' '; v[2][k++] = 'f'; v[2][k++] = 'i'; v[2][k++] = 'n'; v[2][k++] = 'e'; }
    v[2][k] = 0;
    fmt_dv(v[3], g->amp_dv);
    fmt_dv(v[4], g->offset_dv);
    p = fmt_u(b, g->duty_pct, 1);        for (k = 0; p[k]; k++) v[5][k] = p[k]; v[5][k++] = '%'; v[5][k] = 0;

    lcd_fill(6 * 8 - 4, 2 * 16 - 4, 28 * 8 + 8, 11 * 16 + 8, C_NAVY);
    lcd_text(6, 2, " GENERATOR      Mode=close  ", C_BLACK, C_CYAN);
    for (int i = 0; i < GEN_ROWS; i++) {
        uint16_t bg = i == s->gen_item ? C_GRAY : C_NAVY;
        int dim = (i == 5 && g->wave != GEN_SQUARE) || (i == 3 && g->wave == GEN_DC);   /* row has no effect */
        lcd_fill(6 * 8, (4 + i) * 16, 28 * 8, 16, bg);
        lcd_text(7, 4 + i, labels[i], C_WHITE, bg);
        lcd_text(18, 4 + i, "<             >", C_CYAN, bg);
        lcd_text(20, 4 + i, v[i], dim ? C_GRAY : C_YELLOW, bg);
    }
    /* what the hardware really produces */
    lcd_text(7, 11, "actual ", C_GRAY, C_NAVY);
    if (g->wave == GEN_DC) lcd_text(14, 11, "DC level     ", C_WHITE, C_NAVY);
    else {
        char a[24];
        lcd_text(14, 11, fmt_hz_mhz(a, actual_mhz, 14), C_WHITE, C_NAVY);   /* cols 14..27 */
    }
    lcd_text(7, 12, "press wheel on Freq: fine", C_GRAY, C_NAVY);
}

void ui_draw_bars(const scope_state_t *s, int uncal) {
    /* what the wheel | Up/Down adjust; padded to 16 columns (cols 12..27); col 28 is a gap, 29 the G marker, 31..33 the USB marker, 35 the uncalibrated mark */
    static const char *ctl_names[] = {"Time | Volts    ", "TrigPos | Offset", "TrigPos | Level ", "Gen Freq | Amp  "};
    static const char *trig_names[] = {"Auto  ", "Normal", "Single"};
    char b[16];
    lcd_fill(0, 0, 12 * 8, 16, C_BLACK);
    int c = lcd_text(0, 0, " ", C_WHITE, C_BLACK);
    c = lcd_text(c, 0, fmt_si(b, (float)tdiv_ns[s->tdiv] * 1e-9f, "s", 1), C_WHITE, C_BLACK);
    lcd_text(7, 0, s->run ? "RUN " : "STOP", C_BLACK, s->run ? C_GREEN : C_RED);
    if (s->ctl == CTL_GEN) {
        char l[24], t[8];
        int k = 0;
        const char *p = "Gen ";
        while (*p) l[k++] = *p++;
        p = fmt_si(b, (float)s->gen.freq_hz, "Hz", 1);
        while (*p && k < 11) l[k++] = *p++;                 /* at most 7 characters: "10.1kHz" */
        l[k++] = s->gen.fine ? '~' : ' ';
        p = fmt_dv(t, s->gen.amp_dv);
        while (*p) l[k++] = *p++;
        while (k < 16) l[k++] = ' ';
        l[k] = 0;
        lcd_text(12, 0, l, C_CYAN, C_BLACK);
    } else lcd_text(12, 0, ctl_names[s->ctl], C_CYAN, C_BLACK);
    lcd_text(35, 0, uncal ? "!" : " ", C_BLACK, uncal ? C_RED : C_BLACK);   /* no factory calibration */

    lcd_fill(0, 224, 21 * 8, 16, C_BLACK);
    c = lcd_text(0, 14, " ", C_WHITE, C_BLACK);
    c = lcd_text(c, 14, fmt_si(b, (float)vdiv_mv[s->vdiv] * 1e-3f * (s->probe_x10 ? 10.0f : 1.0f), "V", 1),
                 C_YELLOW, C_BLACK);
    lcd_text(7, 14, trig_names[s->trig_mode], C_WHITE, C_BLACK);
    lcd_text(14, 14, s->probe_x10 ? "x10" : "x1 ", C_WHITE, C_BLACK);
    lcd_text(17, 14, s->coupling_dc ? "DC" : "AC", C_WHITE, C_BLACK);
    lcd_text(19, 14, s->edge ? "F" : "R", C_CYAN, C_BLACK);
}

void ui_draw_meas(const scope_state_t *s, const meas_t *m) {
    char b[16];
    lcd_fill(21 * 8, 224, LCD_W - 21 * 8, 16, C_BLACK);
    if (!m->valid) { lcd_text(21, 14, "---", C_GRAY, C_BLACK); }
    else {
        int c = lcd_text(21, 14, fmt_si(b, m->freq_hz, "Hz", 0), C_WHITE, C_BLACK);
        c = lcd_text(c + 1, 14, fmt_u(b, (uint32_t)(m->duty_pct + 0.5f), 2), C_WHITE, C_BLACK);
        lcd_text(c, 14, "%", C_WHITE, C_BLACK);
    }
    float vpp = m->vpp_codes * units_volts_per_code(s->vdiv, s->probe_x10);
    lcd_text(33, 14, fmt_si(b, vpp, "V", 0), C_YELLOW, C_BLACK);
}

void ui_draw_menu(const scope_state_t *s) {
    static const char *labels[MENU_ITEMS] = {"Coupling", "Probe", "Trigger", "Edge", "Generator"};
    static const char *trig_names[] = {"Auto", "Normal", "Single"};
    const char *vals[MENU_ITEMS] = {
        s->coupling_dc ? "DC" : "AC", s->probe_x10 ? "x10" : "x1",
        trig_names[s->trig_mode], s->edge ? "Falling" : "Rising", "open >",
    };
    lcd_fill(8 * 8 - 4, 3 * 16 - 4, 24 * 8 + 8, 8 * 16 + 8, C_NAVY);
    lcd_text(8, 3, " SETTINGS   Mode=close  ", C_BLACK, C_CYAN);
    for (int i = 0; i < MENU_ITEMS; i++) {
        uint16_t bg = i == s->menu_item ? C_GRAY : C_NAVY;
        lcd_fill(8 * 8, (5 + i) * 16, 24 * 8, 16, bg);
        lcd_text(9, 5 + i, labels[i], C_WHITE, bg);
        lcd_text(20, 5 + i, "<       >", C_CYAN, bg);
        lcd_text(21, 5 + i, vals[i], C_YELLOW, bg);
    }
}

void ui_draw_batt(int pct) {
    char b[8];
    uint16_t col = pct <= 20 ? C_RED : C_GREEN;
    int c = lcd_text(36, 0, fmt_u(b, (uint32_t)pct, 3), col, C_BLACK);
    lcd_text(c, 0, "%", col, C_BLACK);
}

void ui_toast(const char *msg) {
    lcd_fill(19 * 8, 0, 14 * 8, 16, C_BLACK);
    lcd_text(19, 0, msg, C_BLACK, C_YELLOW);
}

void ui_toast_clear(void) { lcd_fill(19 * 8, 0, 14 * 8, 16, C_BLACK); }

void ui_draw_usb(int on) { lcd_text(31, 0, on ? "USB" : "   ", C_BLACK, on ? C_CYAN : C_BLACK); }

static void kv_line(int row, const char *k, const char *v, uint16_t col) {
    int c = lcd_text(2, row, k, C_GRAY, C_BLACK);
    c = lcd_text(c, row, v, col, C_BLACK);
    lcd_fill(c * 8, row * 16, LCD_W - c * 8, 16, C_BLACK);   /* clear the rest of the row */
}

void ui_draw_logger(uint32_t rate_hz, uint32_t sent, uint32_t dropped, float vmin, float vmax, float vmean,
                    float vdiv_v) {
    char b[16];
    lcd_text(2, 2, "LOGGING -> USB      Run = stop", C_CYAN, C_BLACK);
    kv_line(4, "rate      ", fmt_si(b, (float)rate_hz, "S/s", 0), C_WHITE);
    kv_line(5, "V/div     ", fmt_si(b, vdiv_v, "V", 1), C_YELLOW);
    kv_line(7, "sent      ", fmt_u(b, sent, 1), C_WHITE);
    kv_line(8, "dropped   ", fmt_u(b, dropped, 1), dropped ? C_RED : C_GREEN);
    kv_line(10, "min       ", fmt_si(b, vmin, "V", 0), C_WHITE);
    kv_line(11, "max       ", fmt_si(b, vmax, "V", 0), C_WHITE);
    kv_line(12, "mean      ", fmt_si(b, vmean, "V", 0), C_WHITE);
}
