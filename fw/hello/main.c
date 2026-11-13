// DSO-510 "hello" test app: power latch, LCD init (ST7789 on XMC), backlight, color bars.
// Linked at 0x08007000 (after FNIRSI's IAP bootloader). Flash by copying hello.bin to the IAP drive.
// Recovery: hold the update button at power-on, copy the official APP_*.bin.

#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define REG16(a) (*(volatile uint16_t *)(a))

// CRM (clock & reset)
#define CRM_CTRL     REG32(0x40021000)
#define CRM_CFG      REG32(0x40021004)
#define CRM_CLKINT   REG32(0x40021008)
#define CRM_AHBEN    REG32(0x40021014)
#define CRM_APB2EN   REG32(0x40021018)
#define CRM_MISC1    REG32(0x40021030)

#define IOMUX_REMAP2 REG32(0x40010030)
#define WDT_CMD      REG32(0x40003000)
#define SCB_VTOR     REG32(0xE000ED08)

// GPIO (STM32F1-style)
#define GPIOA 0x40010800u
#define GPIOD 0x40011400u
#define GPIOE 0x40011800u
#define GPIO_CFGLR(p) REG32((p) + 0x00)
#define GPIO_CFGHR(p) REG32((p) + 0x04)
#define GPIO_SCR(p)   REG32((p) + 0x10)  // set
#define GPIO_CLR(p)   REG32((p) + 0x14)  // reset

#define CFG_IN_FLOAT 0x4u
#define CFG_OUT_PP   0x3u   // push-pull, 50 MHz
#define CFG_AF_PP    0xBu   // alternate function push-pull, 50 MHz

// XMC bank 1 (LCD)
#define XMC_BK1CTRL1   REG32(0xA0000000)
#define XMC_BK1TMG1    REG32(0xA0000004)
#define XMC_BK1TMGWR1  REG32(0xA0000104)
#define XMC_EXT1       REG32(0xA0000220)
#define LCD_CMD  REG16(0x6001FFFE)   // A16 = 0
#define LCD_DATA REG16(0x60020000)   // A16 = 1

#define LCD_W 320
#define LCD_H 240

static void gpio_mode(uint32_t port, unsigned pin, uint32_t cfg) {
    volatile uint32_t *r = pin < 8 ? &GPIO_CFGLR(port) : &GPIO_CFGHR(port);
    unsigned sh = (pin & 7) * 4;
    *r = (*r & ~(0xFu << sh)) | (cfg << sh);
}

static void delay_ms(uint32_t ms) {
    // ~8 MHz HICK, ~4 cycles per iteration
    while (ms--) {
        for (volatile uint32_t i = 0; i < 2000; i++) { }
        WDT_CMD = 0xAAAA;   // reload watchdog in case the bootloader started it
    }
}

static void clock_reset_to_hick(void) {
    // Mirrors the stock firmware's SystemInit: run from 8 MHz HICK, PLL off.
    CRM_CTRL |= 1;                       // HICKEN
    while (!(CRM_CTRL & 2)) { }          // HICKSTBL
    CRM_CFG &= ~3u;                      // SCLKSEL = HICK
    while (CRM_CFG & 0xC) { }            // SCLKSTS == HICK
    CRM_CTRL &= 0xFEF2FFFFu;             // HEXT/PLL/CFD off
    CRM_CFG = 0;
    CRM_MISC1 = 0;
    CRM_CLKINT = 0x009F0000;             // clear clock interrupt flags
}

static void lcd_cmd(uint8_t c) { LCD_CMD = c; }
static void lcd_dat(uint8_t d) { LCD_DATA = d; }

static void lcd_init(void) {
    // Pins: PD6 = LCD reset. XMC: PD0,1,4,5,7,8,9,10,11(A16),14,15 + PE7..15.
    static const uint8_t pd_xmc[] = {0, 1, 4, 5, 7, 8, 9, 10, 11, 14, 15};
    IOMUX_REMAP2 |= 0x08000000;          // as stock firmware does before XMC pin setup
    for (unsigned i = 0; i < sizeof pd_xmc; i++) gpio_mode(GPIOD, pd_xmc[i], CFG_AF_PP);
    for (unsigned p = 7; p <= 15; p++) gpio_mode(GPIOE, p, CFG_AF_PP);
    gpio_mode(GPIOD, 6, CFG_OUT_PP);

    // XMC bank 1: SRAM-type, 16-bit, write enable, extended (separate write timing) — stock values.
    XMC_BK1CTRL1 = 0x5010;
    XMC_BK1TMG1 = 0x02020424;
    XMC_BK1TMGWR1 = 0x202;
    XMC_EXT1 = (XMC_EXT1 & 0xFFFF0000u) | 8;
    XMC_BK1CTRL1 |= 1;

    // Hardware reset, same timing as stock.
    GPIO_SCR(GPIOD) = 1u << 6; delay_ms(10);
    GPIO_CLR(GPIOD) = 1u << 6; delay_ms(20);
    GPIO_SCR(GPIOD) = 1u << 6; delay_ms(20);

    // ST7789 init sequence copied from stock firmware V2.0.5 (main init @ 0x080290F8).
    lcd_cmd(0x11); delay_ms(120);                      // SLPOUT
    lcd_cmd(0x36); lcd_dat(0x00);                      // MADCTL
    lcd_cmd(0x3A); lcd_dat(0x05);                      // COLMOD: RGB565
    lcd_cmd(0xB2); lcd_dat(0x0C); lcd_dat(0x0C); lcd_dat(0x00); lcd_dat(0x33); lcd_dat(0x33);
    lcd_cmd(0xB7); lcd_dat(0x35);
    lcd_cmd(0xBB); lcd_dat(0x28);
    lcd_cmd(0xC0); lcd_dat(0x2C);
    lcd_cmd(0xC2); lcd_dat(0x01);
    lcd_cmd(0xC3); lcd_dat(0x0B);
    lcd_cmd(0xC4); lcd_dat(0x20);
    lcd_cmd(0xC6); lcd_dat(0x0F);
    lcd_cmd(0xD0); lcd_dat(0xA4); lcd_dat(0xA1);
    static const uint8_t gp[] = {0xD0,0x01,0x08,0x0F,0x11,0x2A,0x36,0x55,0x44,0x3A,0x0B,0x06,0x11,0x20};
    static const uint8_t gn[] = {0xD0,0x02,0x07,0x0A,0x0B,0x18,0x34,0x43,0x4A,0x2B,0x1B,0x1C,0x22,0x1F};
    lcd_cmd(0xE0); for (unsigned i = 0; i < sizeof gp; i++) lcd_dat(gp[i]);
    lcd_cmd(0xE1); for (unsigned i = 0; i < sizeof gn; i++) lcd_dat(gn[i]);
    lcd_cmd(0x29);                                     // DISPON
    lcd_cmd(0x36); lcd_dat(0x60);                      // MADCTL: landscape 320x240
}

static void lcd_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_cmd(0x2A); lcd_dat(x0 >> 8); lcd_dat(x0); lcd_dat(x1 >> 8); lcd_dat(x1);
    lcd_cmd(0x2B); lcd_dat(y0 >> 8); lcd_dat(y0); lcd_dat(y1 >> 8); lcd_dat(y1);
    lcd_cmd(0x2C);
}

static void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c) {
    lcd_window(x, y, x + w - 1, y + h - 1);
    for (uint32_t n = (uint32_t)w * h; n; n--) LCD_DATA = c;
}

#define RGB565(r, g, b) (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

int main(void) {
    __asm volatile ("cpsid i");
    SCB_VTOR = 0x08007000;
    clock_reset_to_hick();

    // Clocks: IOMUX, GPIOA, GPIOD, GPIOE; XMC.
    CRM_APB2EN |= (1u << 0) | (1u << 2) | (1u << 5) | (1u << 6);
    CRM_AHBEN |= (1u << 8);

    // Power latch first: PD3 high keeps the scope on after the button is released.
    GPIO_SCR(GPIOD) = 1u << 3;
    gpio_mode(GPIOD, 3, CFG_OUT_PP);

    lcd_init();

    // Color bars.
    static const uint16_t bars[8] = {
        RGB565(255,255,255), RGB565(255,255,0), RGB565(0,255,255), RGB565(0,255,0),
        RGB565(255,0,255),   RGB565(255,0,0),   RGB565(0,0,255),   RGB565(0,0,0),
    };
    for (int i = 0; i < 8; i++) lcd_fill(i * 40, 0, 40, 200, bars[i]);
    lcd_fill(0, 200, LCD_W, 40, RGB565(16, 16, 48));

    // Backlight on: PA2 (stock drives it with TMR9 CH1 PWM; plain high = 100%).
    GPIO_SCR(GPIOA) = 1u << 2;
    gpio_mode(GPIOA, 2, CFG_OUT_PP);

    // Heartbeat: a square bouncing along the bottom strip.
    int x = 0, dx = 4;
    for (;;) {
        lcd_fill(x, 210, 20, 20, RGB565(16, 16, 48));
        x += dx;
        if (x <= 0 || x >= LCD_W - 20) dx = -dx;
        lcd_fill(x, 210, 20, 20, RGB565(255, 160, 0));
        delay_ms(15);
    }
}
