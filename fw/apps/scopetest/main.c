// scopetest @ 240 MHz: raw capture test. Wheel L/R = range, Up/Down = offset PWM,
// Run = timebase, Mode = coupling, Auto = freeze, wheel press = generator frequency, hold Power = off.
#include "board.h"
#include "lcd.h"
#include "adc.h"
#include "frontend.h"
#include "testsig.h"
#include "units.h"
#include "trigger.h"

#define PLOT_X 10
#define PLOT_Y 32
#define PLOT_W 300
#define PLOT_H 176
#define CAP_N  (PLOT_W * 2)

static uint8_t cap[CAP_N];
static int range = 5, tdiv = 7, dc = 1, frozen = 0, gen = 2;
static uint16_t offset = 0x834;
static uint32_t notrig;

static const struct { uint16_t arr; const char *label; } gen_freqs[] = {
    {23999, "100Hz"}, {7199, "333Hz"}, {2399, "1kHz"}, {1799, "1.33kHz"},
    {479, "5kHz"}, {329, "7.27kHz"}, {239, "10kHz"}, {119, "20kHz"},
};
#define NUM_GEN (int)(sizeof gen_freqs / sizeof gen_freqs[0])

static inline uint16_t bg_px(int x, int y) {
    if ((x % 30) == 0 && (y % 4) == 0) return C_GRAY;
    if ((y % 22) == 0 && (x % 6) == 0) return C_GRAY;
    return C_BLACK;
}

static void draw_col(int x, int y0, int y1) {
    lcd_window(PLOT_X + x, PLOT_Y, PLOT_X + x, PLOT_Y + PLOT_H - 1);
    for (int y = 0; y < PLOT_H; y++) lcd_push((y >= y0 && y <= y1) ? C_YELLOW : bg_px(x, y));
}

static int to_y(uint8_t v) { return PLOT_H - 1 - (v * (PLOT_H - 1)) / 255; }

static void apply_rate(void) {
    rate_cfg_t r = units_rate_for_tdiv(tdiv_ns[tdiv]);
    adc_set_rate(r.psc, r.arr);
}

static void draw_status(void) {
    char b[16];
    int c = lcd_text(0, 0, " R", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, fmt_u(b, range, 1), C_WHITE, C_BLACK);
    c = lcd_text(c, 0, "  rate ", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, fmt_u(b, units_rate_for_tdiv(tdiv_ns[tdiv]).rate_hz, 8), C_YELLOW, C_BLACK);
    c = lcd_text(c, 0, " OFS ", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, fmt_u(b, offset, 4), C_WHITE, C_BLACK);
    lcd_text(c, 0, dc ? " DC" : " AC", C_WHITE, C_BLACK);
    c = lcd_text(0, 14, " GEN sine ", C_GRAY, C_BLACK);
    c = lcd_text(c, 14, gen_freqs[gen].label, C_CYAN, C_BLACK);
    lcd_text(c, 14, frozen ? "  HOLD          " : "                ", C_RED, C_BLACK);
}

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    for (int x = 0; x < PLOT_W; x++) draw_col(x, -1, -1);
    backlight(1);
    fe_init();
    fe_set_range(range); fe_set_coupling_dc(dc); fe_set_offset_pwm(offset);
    adc_init();
    dac2_bias_init();
    cal_out_init();
    gen_sine_init(gen_freqs[gen].arr);
    apply_rate();
    draw_status();

    uint32_t prev = 0;
    char b[16];
    for (;;) {
        uint32_t k = keys_read(), press = k & ~prev;
        prev = k;
        int changed = 0;
        if (press & KEY_BIT(KEY_WHEEL_R) && range < FE_NUM_RANGES - 1) { range++; changed = 1; }
        if (press & KEY_BIT(KEY_WHEEL_L) && range > 0) { range--; changed = 1; }
        if (k & KEY_BIT(KEY_UP))   { offset = offset + 16 > FE_PWM_MAX ? FE_PWM_MAX : offset + 16; changed = 1; }
        if (k & KEY_BIT(KEY_DOWN)) { offset = offset < 16 ? 0 : offset - 16; changed = 1; }
        if (press & KEY_BIT(KEY_RUN)) { tdiv = (tdiv + 1) % NUM_TDIV; apply_rate(); changed = 1; }
        if (press & KEY_BIT(KEY_MODE)) { dc ^= 1; changed = 1; }
        if (press & KEY_BIT(KEY_AUTO)) { frozen ^= 1; changed = 1; }
        if (press & KEY_BIT(KEY_WHEEL_PRESS)) { gen = (gen + 1) % NUM_GEN; gen_set_arr(gen_freqs[gen].arr); changed = 1; }
        if (k & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER))
                if (millis() - t0 > 1500) { backlight(0); power_off(); for (;;) { } }
        }
        if (changed) {
            fe_set_range(range); fe_set_coupling_dc(dc); fe_set_offset_pwm(offset);
            draw_status();
        }
        if (frozen) { delay_ms(20); continue; }

        adc_start(cap, CAP_N);
        uint32_t t0 = millis();
        while (!adc_done()) if (millis() - t0 > 3000) { adc_abort(); break; }

        int mn = 255, mx = 0; uint32_t sum = 0;
        for (int i = 0; i < CAP_N; i++) { sum += cap[i]; if (cap[i] < mn) mn = cap[i]; if (cap[i] > mx) mx = cap[i]; }
        int lvl = (int)(sum / CAP_N);
        int t = trigger_find(cap, 0, PLOT_W, lvl, trigger_hysteresis(cap, CAP_N), EDGE_RISING);
        if (t < 0) { notrig++; if (mx - mn > 8) continue; t = 0; }

        int last = to_y(cap[t]);
        for (int x = 0; x < PLOT_W; x++) {
            int y = to_y(cap[t + x]);
            draw_col(x, y < last ? y : last, y < last ? last : y);
            last = y;
        }
        int c = lcd_text(0, 1, " min ", C_GRAY, C_BLACK);
        c = lcd_text(c, 1, fmt_u(b, mn, 3), C_WHITE, C_BLACK);
        c = lcd_text(c, 1, " max ", C_GRAY, C_BLACK);
        c = lcd_text(c, 1, fmt_u(b, mx, 3), C_WHITE, C_BLACK);
        c = lcd_text(c, 1, " notrig ", C_GRAY, C_BLACK);
        lcd_text(c, 1, fmt_u(b, notrig, 5), C_WHITE, C_BLACK);
    }
}
