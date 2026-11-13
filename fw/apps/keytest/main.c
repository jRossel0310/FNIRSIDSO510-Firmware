// Key test: shows the 9 front-panel key inputs live, press counts, battery ADC, and lets
// a long press of K1 (PD2, believed power key) release the PD3 power latch.
#include "board.h"
#include "lcd.h"

#define ROW_KEYS 2

static void draw_static(void) {
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    lcd_text(0, 0, " DSO-510 key test        custom fw ", C_BLACK, C_CYAN);
    lcd_text(0, 1, " key pin   state  presses", C_GRAY, C_BLACK);
    char b[12];
    for (int i = 0; i < NUM_KEYS; i++) {
        int c = lcd_text(0, ROW_KEYS + i, " K", C_WHITE, C_BLACK);
        c = lcd_text(c, ROW_KEYS + i, fmt_u(b, i + 1, 1), C_WHITE, C_BLACK);
        lcd_text(5, ROW_KEYS + i, board_keys[i].name, C_YELLOW, C_BLACK);
    }
    lcd_text(0, 14, " Hold K1 3 s: release power latch PD3 ", C_GRAY, C_BLACK);
}

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    batt_adc_init();
    draw_static();
    backlight(1);

    uint32_t prev = 0xFFFFFFFF, counts[NUM_KEYS] = {0}, k1_held = 0, tick = 0;
    int latch_released = 0;
    char b[12];

    for (;;) {
        uint32_t k = keys_read();
        for (int i = 0; i < NUM_KEYS; i++) {
            uint32_t bit = 1u << i;
            if ((k & bit) && !(prev & bit) && prev != 0xFFFFFFFF) counts[i]++;
            if (((k ^ prev) & bit) || prev == 0xFFFFFFFF) {
                int down = (k & bit) != 0;
                lcd_text(11, ROW_KEYS + i, down ? " DOWN " : "  up  ",
                         down ? C_BLACK : C_GRAY, down ? C_GREEN : C_BLACK);
                lcd_text(18, ROW_KEYS + i, fmt_u(b, counts[i], 5), C_WHITE, C_BLACK);
            }
        }
        prev = k;

        // Long-press K1 -> release power latch.
        if (k & 1) {
            if (++k1_held == 150 && !latch_released) {   // ~3 s (loop is ~10-20 ms)
                lcd_text(0, 13, " PD3 low: releasing power latch...    ", C_BLACK, C_RED);
                power_off();
                latch_released = 1;
            }
        } else {
            k1_held = 0;
            if (latch_released) {
                // Still alive after release -> powered via USB or PD3 isn't the latch.
                delay_ms(500);
                lcd_text(0, 13, " Still on after PD3 low (USB power?) ", C_BLACK, C_YELLOW);
                GPIO_SCR(PWR_LATCH_PORT) = 1u << PWR_LATCH_PIN;
                latch_released = 0;
            }
        }

        // Battery ADC + raw port snapshot, ~4x per second.
        if (++tick % 25 == 0) {
            uint16_t raw = batt_adc_read();
            int c = lcd_text(0, 12, " BAT PC3 raw ", C_GRAY, C_BLACK);
            c = lcd_text(c, 12, fmt_u(b, raw, 4), C_WHITE, C_BLACK);
            c = lcd_text(c, 12, "  pin mV ", C_GRAY, C_BLACK);
            lcd_text(c, 12, fmt_u(b, raw * 3300u / 4095u, 4), C_WHITE, C_BLACK);
            c = lcd_text(30, 2, "PA8 ", C_GRAY, C_BLACK);
            lcd_text(c, 2, (GPIO_IDT(GPIOA) & (1u << 8)) ? "1" : "0", C_WHITE, C_BLACK);
            c = lcd_text(30, 3, "PA0 ", C_GRAY, C_BLACK);
            lcd_text(c, 3, (GPIO_IDT(GPIOA) & 1u) ? "1" : "0", C_WHITE, C_BLACK);
        }
        delay_ms(10);
    }
}
