// flashprobe: identify the external SPI NOR flash and find FAT boot sectors on it (READ-ONLY).
#include "board.h"
#include "lcd.h"
#include "spiflash.h"

static uint8_t sec[512];

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | p[1] << 8); }
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static int text_n(int c, int row, const uint8_t *p, int n, uint16_t col) {
    char s[12];
    for (int i = 0; i < n && i < 11; i++) s[i] = (p[i] >= 32 && p[i] < 127) ? (char)p[i] : '.';
    s[n < 11 ? n : 11] = 0;
    return lcd_text(c, row, s, col, C_BLACK);
}

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(1);
    char b[12];
    lcd_text(0, 0, " flashprobe (read-only)", C_CYAN, C_BLACK);
    spiflash_init();
    uint32_t id = spiflash_jedec_id();
    int c = lcd_text(0, 1, " JEDEC ", C_GRAY, C_BLACK);
    c = lcd_text(c, 1, fmt_x(b, id, 6), C_WHITE, C_BLACK);
    uint32_t size = 0;
    uint8_t cap = (uint8_t)id;
    if (id != 0xFFFFFFFFu && id != 0 && cap >= 0x10 && cap <= 0x18) size = 1u << cap;   /* 24-bit addressing only */
    c = lcd_text(c, 1, "  size ", C_GRAY, C_BLACK);
    lcd_text(c, 1, size ? fmt_x(b, size, 7) : "??", size ? C_WHITE : C_RED, C_BLACK);

    /* offset 0: first bytes regardless */
    if (spiflash_read(0, sec, 512) == 0) {
        c = lcd_text(0, 2, " @0:", C_GRAY, C_BLACK);
        for (int i = 0; i < 8; i++) { c = lcd_text(c, 2, " ", C_BLACK, C_BLACK); c = lcd_text(c, 2, fmt_x(b, sec[i], 2), C_WHITE, C_BLACK); }
        c = lcd_text(c, 2, " ..", C_GRAY, C_BLACK);
        c = lcd_text(c, 2, fmt_x(b, sec[510], 2), C_WHITE, C_BLACK);
        lcd_text(c, 2, fmt_x(b, sec[511], 2), C_WHITE, C_BLACK);
    } else lcd_text(0, 2, " read error", C_RED, C_BLACK);

    /* scan every 4 KiB for a FAT boot sector */
    lcd_text(0, 3, "offset OEM      B/S  SC rsv total  type", C_GRAY, C_BLACK);
    int row = 4, found = 0;
    uint32_t lim = size ? size : 0x1000000u;
    for (uint32_t a = 0; a < lim && row < 14; a += 4096) {
        WDT_CMD = 0xAAAA;
        if (spiflash_read(a, sec, 512) < 0) { lcd_text(0, row++, " SPI timeout", C_RED, C_BLACK); break; }
        if (sec[510] != 0x55 || sec[511] != 0xAA || (sec[0] != 0xEB && sec[0] != 0xE9)) continue;
        found++;
        c = lcd_text(0, row, fmt_x(b, a, 6), C_YELLOW, C_BLACK);
        c = lcd_text(c, row, " ", C_BLACK, C_BLACK);
        c = text_n(c, row, sec + 3, 8, C_WHITE);
        c = lcd_text(c, row, " ", C_BLACK, C_BLACK);
        c = lcd_text(c, row, fmt_x(b, le16(sec + 11), 4), C_WHITE, C_BLACK);
        c = lcd_text(c, row, " ", C_BLACK, C_BLACK);
        c = lcd_text(c, row, fmt_x(b, sec[13], 2), C_WHITE, C_BLACK);
        c = lcd_text(c, row, " ", C_BLACK, C_BLACK);
        c = lcd_text(c, row, fmt_x(b, le16(sec + 14), 3), C_WHITE, C_BLACK);
        c = lcd_text(c, row, " ", C_BLACK, C_BLACK);
        uint32_t tot = le16(sec + 19) ? le16(sec + 19) : le32(sec + 32);
        c = lcd_text(c, row, fmt_x(b, tot, 6), C_WHITE, C_BLACK);
        c = lcd_text(c, row, " ", C_BLACK, C_BLACK);
        text_n(c, row, le16(sec + 22) ? sec + 54 : sec + 82, 5, C_GREEN);   /* FAT12/16 vs FAT32 */
        row++;
    }
    c = lcd_text(0, 14, " found ", C_GRAY, C_BLACK);
    c = lcd_text(c, 14, fmt_x(b, (uint32_t)found, 2), C_WHITE, C_BLACK);
    lcd_text(c, 14, "  Photo this. Hold POWER=off", C_GRAY, C_BLACK);
    for (;;) {
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER))
                if (millis() - t0 > 1000) { backlight(0); power_off(); for (;;) { } }
        }
        WDT_CMD = 0xAAAA;
        delay_ms(20);
    }
}
