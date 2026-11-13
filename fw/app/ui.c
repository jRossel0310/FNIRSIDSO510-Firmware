#include "ui.h"
#include "lcd.h"
#include "fmt.h"
#include "units.h"

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

void ui_draw_markers(const scope_state_t *s) {
    /* 0 V baseline: left margin */
    lcd_fill(0, PLOT_Y, PLOT_X - 1, PLOT_H, C_BLACK);
    int y = code_to_y(ADC_CENTER + s->offset);
    lcd_fill(0, PLOT_Y + (y > 1 ? y - 1 : 0), PLOT_X - 2, 3, C_YELLOW);
    /* trigger level: right margin */
    lcd_fill(PLOT_X + PLOT_W + 1, PLOT_Y, LCD_W - PLOT_X - PLOT_W - 1, PLOT_H, C_BLACK);
    y = code_to_y(s->trig_level);
    lcd_fill(PLOT_X + PLOT_W + 2, PLOT_Y + (y > 1 ? y - 1 : 0), 7, 3, C_CYAN);
    /* trigger position: strip above the plot */
    lcd_fill(PLOT_X, 16, PLOT_W, 4, C_BLACK);
    int x = s->trig_pos - 2 < 0 ? 0 : s->trig_pos - 2;
    lcd_fill(PLOT_X + x, 16, 5, 4, C_ORANGE);
}

void ui_draw_bars(const scope_state_t *s, int uncal) {
    /* what the wheel | Up/Down adjust; padded to the full 21-column slot (cols 12..32) */
    static const char *ctl_names[] = {"Time | Volts         ", "TrigPos | Offset     ", "TrigPos | Level      "};
    static const char *trig_names[] = {"Auto  ", "Normal", "Single"};
    char b[16];
    lcd_fill(0, 0, 12 * 8, 16, C_BLACK);
    int c = lcd_text(0, 0, " ", C_WHITE, C_BLACK);
    c = lcd_text(c, 0, fmt_si(b, (float)tdiv_ns[s->tdiv] * 1e-9f, "s", 1), C_WHITE, C_BLACK);
    lcd_text(7, 0, s->run ? "RUN " : "STOP", C_BLACK, s->run ? C_GREEN : C_RED);
    lcd_text(12, 0, ctl_names[s->ctl], C_CYAN, C_BLACK);
    lcd_text(33, 0, uncal ? "UNC" : "   ", C_RED, C_BLACK);

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
    static const char *labels[MENU_ITEMS] = {"Coupling", "Probe", "Trigger", "Edge"};
    static const char *trig_names[] = {"Auto", "Normal", "Single"};
    const char *vals[MENU_ITEMS] = {
        s->coupling_dc ? "DC" : "AC", s->probe_x10 ? "x10" : "x1",
        trig_names[s->trig_mode], s->edge ? "Falling" : "Rising",
    };
    lcd_fill(8 * 8 - 4, 3 * 16 - 4, 24 * 8 + 8, 7 * 16 + 8, C_NAVY);
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
