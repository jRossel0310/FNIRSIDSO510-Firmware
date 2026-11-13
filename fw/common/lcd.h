#pragma once
#include <stdint.h>

#define LCD_W 320
#define LCD_H 240
#define RGB565(r, g, b) (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

#define C_BLACK  RGB565(0, 0, 0)
#define C_WHITE  RGB565(255, 255, 255)
#define C_GRAY   RGB565(110, 110, 110)
#define C_RED    RGB565(255, 40, 40)
#define C_GREEN  RGB565(40, 220, 40)
#define C_YELLOW RGB565(255, 220, 0)
#define C_CYAN   RGB565(0, 220, 255)
#define C_NAVY   RGB565(16, 16, 48)

void lcd_init(void);
void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c);
// Text on an 8x16 grid; col/row are character cells (40 x 15). Returns next col.
int lcd_text(int col, int row, const char *s, uint16_t fg, uint16_t bg);
// Format helpers (write into buf, return buf).
char *fmt_u(char *buf, uint32_t v, int width);   // right-aligned decimal
char *fmt_x(char *buf, uint32_t v, int digits);  // hex, zero-padded
