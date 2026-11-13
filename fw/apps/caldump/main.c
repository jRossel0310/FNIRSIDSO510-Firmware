// caldump: show the factory settings/calibration page at 0x08006000 (read-only).
#include "board.h"
#include "lcd.h"
#include "cal.h"

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(1);
    const uint32_t *page = (const uint32_t *)CAL_PAGE_ADDR;
    char b[12];
    lcd_text(0, 0, " caldump 0x08006000 (words 00..26)", C_CYAN, C_BLACK);
    for (int row = 0; row < 10; row++) {
        int c = lcd_text(0, row + 1, fmt_x(b, row * 4, 2), C_GRAY, C_BLACK);
        for (int w = 0; w < 4 && row * 4 + w < CAL_PAGE_WORDS; w++) {
            c = lcd_text(c, row + 1, " ", C_BLACK, C_BLACK);
            c = lcd_text(c, row + 1, fmt_x(b, page[row * 4 + w], 8), C_WHITE, C_BLACK);
        }
    }
    cal_t cal;
    int ok = cal_parse(page, &cal);
    lcd_text(0, 12, ok ? " parse: VALID factory data" : " parse: INVALID -> defaults", ok ? C_GREEN : C_RED, C_BLACK);
    int c = lcd_text(0, 13, " slow r6 lo ", C_GRAY, C_BLACK);
    c = lcd_text(c, 13, fmt_x(b, cal.lo[CAL_SET_SLOW][6], 3), C_WHITE, C_BLACK);
    c = lcd_text(c, 13, " hi ", C_GRAY, C_BLACK);
    lcd_text(c, 13, fmt_x(b, cal.hi[CAL_SET_SLOW][6], 3), C_WHITE, C_BLACK);
    lcd_text(0, 14, " Photo this screen. Hold POWER = off", C_GRAY, C_BLACK);
    for (;;) {
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER))
                if (millis() - t0 > 1000) { backlight(0); power_off(); for (;;) { } }
        }
        delay_ms(20);
    }
}
