// ST7789 320x240 on XMC bank 1 (16-bit 8080). Values from stock firmware V2.0.5.
#include "lcd.h"
#include "board.h"
#include "font8x16.h"

#define XMC_BK1CTRL1   REG32(0xA0000000)
#define XMC_BK1TMG1    REG32(0xA0000004)
#define XMC_BK1TMGWR1  REG32(0xA0000104)
#define XMC_EXT1       REG32(0xA0000220)
#define LCD_CMD  REG16(0x6001FFFE)   // A16 = 0
#define LCD_DATA REG16(0x60020000)   // A16 = 1

static void cmd(uint8_t c) { LCD_CMD = c; }
static void dat(uint8_t d) { LCD_DATA = d; }

void lcd_init(void) {
    static const uint8_t pd_xmc[] = {0, 1, 4, 5, 7, 8, 9, 10, 11, 14, 15};
    IOMUX_REMAP2 |= 0x08000000;
    for (unsigned i = 0; i < sizeof pd_xmc; i++) gpio_mode(GPIOD, pd_xmc[i], CFG_AF_PP);
    for (unsigned p = 7; p <= 15; p++) gpio_mode(GPIOE, p, CFG_AF_PP);
    gpio_mode(LCD_RST_PORT, LCD_RST_PIN, CFG_OUT_PP);

    XMC_BK1CTRL1 = 0x5010;
    XMC_BK1TMG1 = 0x02020424;
    XMC_BK1TMGWR1 = 0x202;
    XMC_EXT1 = (XMC_EXT1 & 0xFFFF0000u) | 8;
    XMC_BK1CTRL1 |= 1;

    GPIO_SCR(LCD_RST_PORT) = 1u << LCD_RST_PIN; delay_ms(10);
    GPIO_CLR(LCD_RST_PORT) = 1u << LCD_RST_PIN; delay_ms(20);
    GPIO_SCR(LCD_RST_PORT) = 1u << LCD_RST_PIN; delay_ms(20);

    cmd(0x11); delay_ms(120);
    cmd(0x36); dat(0x00);
    cmd(0x3A); dat(0x05);
    cmd(0xB2); dat(0x0C); dat(0x0C); dat(0x00); dat(0x33); dat(0x33);
    cmd(0xB7); dat(0x35);
    cmd(0xBB); dat(0x28);
    cmd(0xC0); dat(0x2C);
    cmd(0xC2); dat(0x01);
    cmd(0xC3); dat(0x0B);
    cmd(0xC4); dat(0x20);
    cmd(0xC6); dat(0x0F);
    cmd(0xD0); dat(0xA4); dat(0xA1);
    static const uint8_t gp[] = {0xD0,0x01,0x08,0x0F,0x11,0x2A,0x36,0x55,0x44,0x3A,0x0B,0x06,0x11,0x20};
    static const uint8_t gn[] = {0xD0,0x02,0x07,0x0A,0x0B,0x18,0x34,0x43,0x4A,0x2B,0x1B,0x1C,0x22,0x1F};
    cmd(0xE0); for (unsigned i = 0; i < sizeof gp; i++) dat(gp[i]);
    cmd(0xE1); for (unsigned i = 0; i < sizeof gn; i++) dat(gn[i]);
    cmd(0x29);
    cmd(0x36); dat(0x60);
}

static void window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    cmd(0x2A); dat(x0 >> 8); dat(x0); dat(x1 >> 8); dat(x1);
    cmd(0x2B); dat(y0 >> 8); dat(y0); dat(y1 >> 8); dat(y1);
    cmd(0x2C);
}

void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c) {
    window(x, y, x + w - 1, y + h - 1);
    for (uint32_t n = (uint32_t)w * h; n; n--) LCD_DATA = c;
}

int lcd_text(int col, int row, const char *s, uint16_t fg, uint16_t bg) {
    for (; *s && col < LCD_W / FONT_W; s++, col++) {
        unsigned ch = (uint8_t)*s;
        if (ch < FONT_FIRST || ch > FONT_LAST) ch = '?';
        const uint8_t *g = font8x16[ch - FONT_FIRST];
        window(col * FONT_W, row * FONT_H, col * FONT_W + FONT_W - 1, row * FONT_H + FONT_H - 1);
        for (int y = 0; y < FONT_H; y++)
            for (int x = 0; x < FONT_W; x++)
                LCD_DATA = (g[y] & (0x80 >> x)) ? fg : bg;
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
