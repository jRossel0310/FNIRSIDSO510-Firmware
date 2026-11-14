// flashtest: guarded SPI NOR write/erase on the LAST user-volume sector, with backup + restore.
// The system volume (0x500000+) must stay byte-identical and the guard must refuse it.
#include "board.h"
#include "lcd.h"
#include "spiflash.h"
#include "flashmap.h"

#define TEST_ADDR (USER_VOL_BYTES - FLASH_SECTOR)   /* 0x4FF000 */
static uint8_t backup[FLASH_SECTOR], pat[FLASH_SECTOR], rd[FLASH_SECTOR], sys0[256], sys1[256];

static int row = 1;
static void line(const char *what, int ok, uint32_t val) {
    char b[12];
    int c = lcd_text(0, row, what, C_WHITE, C_BLACK);
    c = lcd_text(c, row, ok ? " PASS " : " FAIL ", ok ? C_GREEN : C_RED, C_BLACK);
    lcd_text(c, row, fmt_x(b, val, 8), C_GRAY, C_BLACK);
    row++;
}

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(1);
    lcd_text(0, 0, " flashtest 0x4FF000 (backup+restore)", C_CYAN, C_BLACK);
    spiflash_init();
    line(" jedec      ", spiflash_jedec_id() == 0x856017u, spiflash_jedec_id());
    int ok = spiflash_read(TEST_ADDR, backup, FLASH_SECTOR) == 0;
    line(" backup read", ok, 0);
    int sys0_ok = spiflash_read(SYS_VOL_BASE, sys0, sizeof sys0) == 0;
    if (ok) {
        for (uint32_t i = 0; i < FLASH_SECTOR; i++) pat[i] = (uint8_t)(i * 7 + 3);
        uint32_t t0 = millis();
        int r = spiflash_write_sector(TEST_ADDR, pat);
        uint32_t dt = millis() - t0;
        line(" write ms   ", r == 0, dt);
        int same = spiflash_read(TEST_ADDR, rd, FLASH_SECTOR) == 0;
        for (uint32_t i = 0; i < FLASH_SECTOR; i++) if (rd[i] != pat[i]) same = 0;
        line(" readback   ", same, 0);
        line(" guard sys  ", spiflash_write_sector(SYS_VOL_BASE, pat) == -2, 0);
        line(" guard erase", spiflash_erase_sector(SYS_VOL_BASE) == -2, 0);
        r = spiflash_write_sector(TEST_ADDR, backup);
        line(" restore    ", r == 0, (uint32_t)r);
        int sys_same = sys0_ok && spiflash_read(SYS_VOL_BASE, sys1, sizeof sys1) == 0;
        for (uint32_t i = 0; i < sizeof sys0; i++) if (sys0[i] != sys1[i]) sys_same = 0;
        line(" sys intact ", sys_same, 0);
    }
    lcd_text(0, 14, " Photo this. Hold POWER=off", C_GRAY, C_BLACK);
    for (;;) {
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER))
                if (millis() - t0 > 1000) { backlight(0); power_off(); for (;;) WDT_CMD = 0xAAAA; }
        }
        WDT_CMD = 0xAAAA;
        delay_ms(20);
    }
}
