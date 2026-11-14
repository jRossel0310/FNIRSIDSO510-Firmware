// HardFault: show what happened instead of freezing. Strong override of the weak handler.
#include "board.h"
#include "lcd.h"

/* Drop the D+ pull-up (USB_CFG bit 1) directly so a USB host sees an unplug rather than a dead device.
   Not usbd_detach(): fault context must not take locks or touch msc state. */
#define USB_DP_OFF() (REG32(0x40005C60) |= 2u)

void hardfault_report(uint32_t *frame) {
    char b[12];
    if (!lcd_ready) {                       /* fault before lcd_init: LCD access would fault again */
        for (;;) {
            WDT_CMD = 0xAAAA;
            if (keys_read() & KEY_BIT(KEY_POWER)) { USB_DP_OFF(); power_off(); }
        }
    }
    backlight(1);
    lcd_fill(0, 0, LCD_W, LCD_H, C_RED);
    lcd_text(1, 1, "HARD FAULT", C_WHITE, C_RED);
    int c = lcd_text(1, 3, "PC   0x", C_WHITE, C_RED);
    lcd_text(c, 3, fmt_x(b, frame[6], 8), C_WHITE, C_RED);
    c = lcd_text(1, 4, "LR   0x", C_WHITE, C_RED);
    lcd_text(c, 4, fmt_x(b, frame[5], 8), C_WHITE, C_RED);
    c = lcd_text(1, 5, "CFSR 0x", C_WHITE, C_RED);
    lcd_text(c, 5, fmt_x(b, REG32(0xE000ED28), 8), C_WHITE, C_RED);
    c = lcd_text(1, 6, "HFSR 0x", C_WHITE, C_RED);
    lcd_text(c, 6, fmt_x(b, REG32(0xE000ED2C), 8), C_WHITE, C_RED);
    c = lcd_text(1, 7, "BFAR 0x", C_WHITE, C_RED);
    lcd_text(c, 7, fmt_x(b, REG32(0xE000ED38), 8), C_WHITE, C_RED);
    lcd_text(1, 10, "Photo this, then press POWER", C_WHITE, C_RED);
    lcd_text(1, 11, "to switch off.", C_WHITE, C_RED);
    for (;;) {
        WDT_CMD = 0xAAAA;
        if (keys_read() & KEY_BIT(KEY_POWER)) { USB_DP_OFF(); power_off(); }
    }
}

__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile (
        "tst lr, #4      \n"
        "ite eq          \n"
        "mrseq r0, msp   \n"
        "mrsne r0, psp   \n"
        "b hardfault_report \n");
}
