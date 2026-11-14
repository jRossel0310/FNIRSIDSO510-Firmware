// usbdisk: the 5 MiB user volume as a USB drive, nothing else. Status on screen; hold POWER = off.
#include "board.h"
#include "lcd.h"
#include "spiflash.h"
#include "flashmap.h"
#include "vol.h"
#include "msc.h"
#include "usbd.h"

static msc_t msc;
static uint32_t nread, nwrite;
static int rd(uint32_t lba, uint8_t *b) { nread++; return vol_read(lba, b); }
static int wr(uint32_t lba, const uint8_t *b) { nwrite++; return vol_write(lba, b); }
static const msc_disk_t disk = {rd, wr, USER_VOL_SECTORS};

void board_idle(void) { usbd_poll(); }

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(1);
    lcd_text(0, 0, " usbdisk: plug USB into a computer", C_CYAN, C_BLACK);
    spiflash_init();
    msc_init(&msc, &disk);
    usbd_init(&msc);
    uint32_t next = 0;
    char b[12];
    for (;;) {
        usbd_poll();
        uint32_t now = millis();
        if ((int32_t)(now - next) >= 0) {
            next = now + 250;
            lcd_text(0, 2, usbd_configured() ? " host: CONFIGURED " : " host: none       ",
                     usbd_configured() ? C_GREEN : C_GRAY, C_BLACK);
            int c = lcd_text(0, 3, " blocks read  ", C_GRAY, C_BLACK);
            lcd_text(c, 3, fmt_x(b, nread, 6), C_WHITE, C_BLACK);
            c = lcd_text(0, 4, " blocks write ", C_GRAY, C_BLACK);
            lcd_text(c, 4, fmt_x(b, nwrite, 6), C_WHITE, C_BLACK);
            lcd_text(0, 14, " Hold POWER = off", C_GRAY, C_BLACK);
        }
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER)) {
                usbd_poll();
                if (millis() - t0 > 1000) { backlight(0); usbd_detach(); power_off(); for (;;) WDT_CMD = 0xAAAA; }
            }
        }
        WDT_CMD = 0xAAAA;
    }
}
