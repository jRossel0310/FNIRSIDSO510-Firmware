// faulttest: press RUN to execute an undefined instruction; the HardFault screen must appear.
#include "board.h"
#include "lcd.h"

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    lcd_text(1, 1, "faulttest @ 240 MHz", C_CYAN, C_BLACK);
    lcd_text(1, 3, "RUN: trigger a HardFault", C_WHITE, C_BLACK);
    lcd_text(1, 4, "Hold POWER: off", C_WHITE, C_BLACK);
    backlight(1);
    char b[12];
    for (;;) {
        uint32_t k = keys_read();
        if (k & KEY_BIT(KEY_RUN)) __asm volatile (".short 0xde00");   // UDF #0
        if (k & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER))
                if (millis() - t0 > 1000) { backlight(0); power_off(); }
        }
        int c = lcd_text(1, 6, "uptime s ", C_GRAY, C_BLACK);
        lcd_text(c, 6, fmt_u(b, millis() / 1000, 5), C_WHITE, C_BLACK);
        delay_ms(50);
    }
}
