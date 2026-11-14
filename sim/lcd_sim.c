/* Host stand-in for fw/common/lcd.c: the same drawing API, into a 320x240 RGB565 framebuffer.
   lcd_text / lcd_text8 / fmt_u / fmt_x mirror fw/common/lcd.c line for line; keep them in step. */
#include <stdio.h>
#include "lcd.h"
#include "font8x16.h"
#include "font8x8.h"

uint16_t sim_fb[LCD_H][LCD_W];
volatile int lcd_ready = 1;
static int wx0, wy0, wx1, wy1, cx, cy;

void lcd_init(void) {}

void lcd_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    wx0 = x0; wy0 = y0; wx1 = x1; wy1 = y1; cx = x0; cy = y0;
}

void lcd_push(uint16_t c) {
    if (cx < LCD_W && cy < LCD_H) sim_fb[cy][cx] = c;
    if (++cx > wx1) { cx = wx0; if (++cy > wy1) cy = wy0; }
}

void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c) {
    lcd_window(x, y, x + w - 1, y + h - 1);
    for (uint32_t n = (uint32_t)w * h; n; n--) lcd_push(c);
}

int lcd_text(int col, int row, const char *s, uint16_t fg, uint16_t bg) {
    for (; *s && col < LCD_W / FONT_W; s++, col++) {
        unsigned ch = (uint8_t)*s;
        if (ch < FONT_FIRST || ch > FONT_LAST) ch = '?';
        const uint8_t *g = font8x16[ch - FONT_FIRST];
        lcd_window(col * FONT_W, row * FONT_H, col * FONT_W + FONT_W - 1, row * FONT_H + FONT_H - 1);
        for (int y = 0; y < FONT_H; y++)
            for (int x = 0; x < FONT_W; x++)
                lcd_push((g[y] & (0x80 >> x)) ? fg : bg);
    }
    return col;
}

int lcd_text8(int col, int y, const char *s, uint16_t fg, uint16_t bg) {
    for (; *s && col < LCD_W / FONT_W; s++, col++) {
        unsigned ch = (uint8_t)*s;
        if (ch < FONT_FIRST || ch > FONT_LAST) ch = '?';
        const uint8_t *g = font8x8[ch - FONT_FIRST];
        lcd_window(col * FONT_W, y, col * FONT_W + FONT_W - 1, y + FONT8_H - 1);
        for (int r = 0; r < FONT8_H; r++)
            for (int x = 0; x < FONT_W; x++)
                lcd_push((g[r] & (0x80 >> x)) ? fg : bg);
    }
    return col;
}

char *fmt_u(char *buf, uint32_t v, int width) {
    char t[11]; int n = 0;
    do { t[n++] = '0' + v % 10; v /= 10; } while (v);
    int i = 0;
    while (width-- > n) buf[i++] = ' ';
    while (n) buf[i++] = t[--n];
    buf[i] = 0;
    return buf;
}

char *fmt_x(char *buf, uint32_t v, int digits) {
    for (int i = digits - 1; i >= 0; i--, v >>= 4) buf[i] = "0123456789ABCDEF"[v & 15];
    buf[digits] = 0;
    return buf;
}

/* Write the framebuffer as a binary PPM, scaled up by `scale` (nearest neighbour). */
int sim_save_ppm(const char *path, int scale) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    fprintf(f, "P6\n%d %d\n255\n", LCD_W * scale, LCD_H * scale);
    for (int y = 0; y < LCD_H * scale; y++)
        for (int x = 0; x < LCD_W * scale; x++) {
            uint16_t c = sim_fb[y / scale][x / scale];
            unsigned char px[3] = {(unsigned char)(((c >> 11) & 31) * 255 / 31), (unsigned char)(((c >> 5) & 63) * 255 / 63),
                                   (unsigned char)((c & 31) * 255 / 31)};
            fwrite(px, 1, 3, f);
        }
    return fclose(f);
}
