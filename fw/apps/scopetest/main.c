// scopetest: first live capture from the external ADC.
// Controls: wheel L/R = range, Up/Down = offset PWM, Run = sample rate, Mode = PC5 coupling,
//           Auto = freeze/unfreeze, hold Power = off.
#include "board.h"
#include "lcd.h"
#include "scope.h"

#define PLOT_X 10
#define PLOT_Y 32
#define PLOT_W 300
#define PLOT_H 176
// The 3PA1030 is pipelined and its clock stops between captures, so the first samples of a
// capture are stale conversions from the end of the previous one. Discard them.
#define CAP_SKIP 16
#define CAP_N  (PLOT_W * 2 + CAP_SKIP)

static uint8_t cap_raw[CAP_N];
static uint8_t *const cap = cap_raw + CAP_SKIP;
#define CAP_USE (CAP_N - CAP_SKIP)

static int range = 5, rate = 3, coupling = 0, frozen = 0;
static uint32_t glitches, glitch_frames, notrig;

// Generator frequencies: f = 8 MHz / (arr+1) / 100. "Odd" ones are not integer divisors of the
// sample rates, so the sine drifts relative to the sample clock (tests the trigger properly).
static const struct { uint16_t arr; const char *label; } gen_freqs[] = {
    {799, "100Hz"}, {239, "333Hz"}, {79, "1kHz"}, {59, "1.33kHz"},
    {15, "5kHz"},  {10, "7.27kHz"}, {7, "10kHz"}, {3, "20kHz"},
};
#define NUM_GEN (int)(sizeof gen_freqs / sizeof gen_freqs[0])
static int gen = 2;
static uint16_t offset = FE_OFFSET_DEFAULT;

static void draw_status(void) {
    char b[12];
    int c = lcd_text(0, 0, " R", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, fmt_u(b, range, 1), C_WHITE, C_BLACK);
    c = lcd_text(c, 0, "  ", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, sample_rates[rate].label, C_YELLOW, C_BLACK);
    c = lcd_text(c, 0, "        ", C_BLACK, C_BLACK);
    c = lcd_text(19, 0, "OFS ", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, fmt_u(b, offset, 4), C_WHITE, C_BLACK);
    c = lcd_text(c, 0, " PC5=", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, coupling ? "1" : "0", C_WHITE, C_BLACK);
    lcd_text(c, 0, frozen ? " H" : "  ", C_RED, C_BLACK);
    c = lcd_text(0, 14, " GEN sine ", C_GRAY, C_BLACK);
    c = lcd_text(c, 14, gen_freqs[gen].label, C_CYAN, C_BLACK);
    lcd_text(c, 14, "  (wheel-press)          ", C_GRAY, C_BLACK);
}

static inline uint16_t bg_px(int x, int y) {
    if ((x % 30) == 0 && (y % 4) == 0) return C_GRAY;     // vertical grid (10 div)
    if ((y % 22) == 0 && (x % 6) == 0) return C_GRAY;     // horizontal grid (8 div)
    return C_BLACK;
}

// Draw one plot column in a single window: grid background + trace segment [y0, y1].
static void draw_col(int x, int y0, int y1) {
    lcd_window(PLOT_X + x, PLOT_Y, PLOT_X + x, PLOT_Y + PLOT_H - 1);
    for (int y = 0; y < PLOT_H; y++) lcd_push((y >= y0 && y <= y1) ? C_YELLOW : bg_px(x, y));
}

static int to_y(uint8_t v) { return PLOT_H - 1 - (v * (PLOT_H - 1)) / 255; }

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    for (int x = 0; x < PLOT_W; x++) draw_col(x, -1, -1);
    lcd_fill(PLOT_X - 1, PLOT_Y - 1, PLOT_W + 2, 1, C_GRAY);
    lcd_fill(PLOT_X - 1, PLOT_Y + PLOT_H, PLOT_W + 2, 1, C_GRAY);
    backlight(1);

    fe_init();
    fe_set_range(range);
    fe_set_coupling(coupling);
    fe_set_offset(offset);
    adc_init();
    dac2_bias_init();
    cal_out_init();
    gen_sine_1k_init();
    draw_status();

    uint32_t prev_keys = 0, held = 0;
    char b[12];

    for (;;) {
        // ---- keys ----
        uint32_t k = keys_read(), press = k & ~prev_keys;
        held = (k == prev_keys && k) ? held + 1 : 0;
        int rep = held > 15 && (held % 3) == 0;   // auto-repeat for up/down
        prev_keys = k;
        int changed = 0;
        if (press & KEY_BIT(KEY_WHEEL_R) && range < FE_NUM_RANGES - 1) { range++; changed = 1; }
        if (press & KEY_BIT(KEY_WHEEL_L) && range > 0) { range--; changed = 1; }
        if ((press | (rep ? k : 0)) & KEY_BIT(KEY_UP))   { offset = offset + 32 > FE_OFFSET_MAX ? FE_OFFSET_MAX : offset + 32; changed = 1; }
        if ((press | (rep ? k : 0)) & KEY_BIT(KEY_DOWN)) { offset = offset < 32 ? 0 : offset - 32; changed = 1; }
        if (press & KEY_BIT(KEY_RUN))  { rate = (rate + 1) % num_sample_rates; changed = 1; }
        if (press & KEY_BIT(KEY_MODE)) { coupling ^= 1; changed = 1; }
        if (press & KEY_BIT(KEY_AUTO)) { frozen ^= 1; changed = 1; }
        if (press & KEY_BIT(KEY_WHEEL_PRESS)) {
            gen = (gen + 1) % NUM_GEN;
            gen_set_arr(gen_freqs[gen].arr);
            changed = 1;
        }
        if (k & KEY_BIT(KEY_POWER)) {   // hold ~2-3 s (time-based, independent of capture speed)
            int t = 0;
            while (keys_read() & KEY_BIT(KEY_POWER)) {
                delay_ms(10);
                if (++t == 150) { lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK); backlight(0); power_off(); for (;;) { } }
            }
        }
        if (changed) {
            fe_set_range(range); fe_set_coupling(coupling); fe_set_offset(offset);
            draw_status();
        }
        if (frozen) { delay_ms(20); continue; }

        // ---- capture ----
        if (adc_capture(cap_raw, CAP_N, &sample_rates[rate]) != 0) {
            lcd_text(0, 1, " capture timeout (DMA never finished)  ", C_BLACK, C_RED);
            delay_ms(200);
            continue;
        }
        uint32_t sum = 0; uint8_t mn = 255, mx = 0;
        for (int i = 0; i < CAP_USE; i++) { sum += cap[i]; if (cap[i] < mn) mn = cap[i]; if (cap[i] > mx) mx = cap[i]; }

        // Software trigger with hysteresis. Level = mean (robust to a single bad sample, unlike
        // (min+max)/2). Arm once clearly below the level, fire at the first sample >= level.
        int t0 = 0, trig = 0;
        int lvl = sum / CAP_USE, hyst = (mx - mn) / 8;
        if (mx - mn > 8) {
            int armed = 0;
            for (int i = 0; i < PLOT_W; i++) {
                if (cap[i] <= lvl - hyst) armed = 1;
                else if (armed && cap[i] >= lvl) { t0 = i; trig = 1; break; }
            }
        }
        // Glitch detector: a sample far from both neighbours that agree with each other.
        uint32_t g = 0;
        int gthr = (mx - mn) / 3 + 6;
        for (int i = 1; i < CAP_USE - 1; i++) {
            int a = cap[i - 1], b = cap[i], c2 = cap[i + 1];
            int d1 = b - a, d2 = b - c2, dn = a - c2;
            if (d1 < 0) d1 = -d1;
            if (d2 < 0) d2 = -d2;
            if (dn < 0) dn = -dn;
            if (d1 > gthr && d2 > gthr && dn < gthr / 2) g++;
        }
        glitches += g;
        if (g) { glitch_frames++; }
        if (!trig) notrig++;
        lcd_text(36, 0, trig ? "TRG" : "---", trig ? C_GREEN : C_RED, C_BLACK);
        {
            char b2[12];
            int c = lcd_text(0, 13, " glitch smp ", C_GRAY, C_BLACK);
            c = lcd_text(c, 13, fmt_u(b2, glitches, 5), g ? C_RED : C_WHITE, C_BLACK);
            c = lcd_text(c, 13, " frm ", C_GRAY, C_BLACK);
            c = lcd_text(c, 13, fmt_u(b2, glitch_frames, 4), C_WHITE, C_BLACK);
            c = lcd_text(c, 13, " notrig ", C_GRAY, C_BLACK);
            lcd_text(c, 13, fmt_u(b2, notrig, 4), C_WHITE, C_BLACK);
        }
        // Normal-mode behaviour: keep the previous trace if this capture didn't trigger.
        if (!trig && (mx - mn) > 8) continue;

        // ---- draw trace (per-column erase + vertical segment) ----
        int last = to_y(cap[t0]);
        for (int x = 0; x < PLOT_W; x++) {
            int y = to_y(cap[t0 + x]);
            int y0 = y < last ? y : last, y1 = y < last ? last : y;
            draw_col(x, y0, y1);
            last = y;
        }

        int c = lcd_text(0, 1, " min ", C_GRAY, C_BLACK);
        c = lcd_text(c, 1, fmt_u(b, mn, 3), C_WHITE, C_BLACK);
        c = lcd_text(c, 1, " max ", C_GRAY, C_BLACK);
        c = lcd_text(c, 1, fmt_u(b, mx, 3), C_WHITE, C_BLACK);
        c = lcd_text(c, 1, " avg ", C_GRAY, C_BLACK);
        c = lcd_text(c, 1, fmt_u(b, sum / CAP_USE, 3), C_WHITE, C_BLACK);
        c = lcd_text(c, 1, "  s0 0x", C_GRAY, C_BLACK);
        lcd_text(c, 1, fmt_x(b, cap[0], 2), C_WHITE, C_BLACK);
    }
}
