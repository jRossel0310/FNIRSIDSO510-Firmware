// cdctest: the composite device. Serial port: replies "OK <line>" to every line; "BENCH" floods
// 5 s of LOG packets for a throughput measurement. Drive works as usual. Hold POWER = off.
#include "board.h"
#include "lcd.h"
#include "spiflash.h"
#include "flashmap.h"
#include "vol.h"
#include "msc.h"
#include "usbd.h"
#include "stream_proto.h"

static msc_t msc;
static const msc_disk_t disk = {vol_read, vol_write, USER_VOL_SECTORS};
void board_idle(void) { usbd_poll(); }

int main(void) {
    board_init(); keys_init(); lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK); backlight(1);
    lcd_text(0, 0, " cdctest: drive + serial port", C_CYAN, C_BLACK);
    spiflash_init();
    msc_init(&msc, &disk);
    usbd_init(&msc);
    sp_line_t line = {{0}, 0, 0};
    uint32_t next = 0, bench_until = 0, bench_bytes = 0, bench_t0 = 0, seq = 0, lines = 0;
    static uint8_t blk[1024];
    char b[12];
    for (;;) {
        usbd_poll();
        uint8_t in[32];
        uint32_t n = usbd_cdc_read(in, sizeof in);
        for (uint32_t i = 0; i < n; i++) {
            if (!sp_line_feed(&line, in[i])) continue;
            lines++;
            if (line.buf[0] == 'B' && line.buf[1] == 'E' && line.buf[2] == 'N' && line.buf[3] == 'C' && line.buf[4] == 'H') {
                bench_t0 = millis(); bench_until = bench_t0 + 5000; bench_bytes = 0;
            }
            uint32_t len = 0; while (line.buf[len]) len++;
            if (usbd_cdc_open()) {
                sp_put(usbd_cdc_txfifo(), SP_OK, (const uint8_t *)line.buf, len, 0, 0);
                usbd_cdc_kick();
            }
        }
        if (bench_until) {
            if (!usbd_cdc_open() || (int32_t)(millis() - bench_until) >= 0) bench_until = 0;
            else {
                uint8_t h[8];
                for (int i = 0; i < 1024; i++) blk[i] = (uint8_t)(seq + (uint32_t)i);
                sp_u32x2(h, seq, 0);
                if (sp_put(usbd_cdc_txfifo(), SP_LOG, h, 4, blk, 1024) == 0) { seq += 1024; bench_bytes += 1024; }
                usbd_cdc_kick();
            }
        }
        uint32_t now = millis();
        if ((int32_t)(now - next) >= 0) {
            next = now + 250;
            lcd_text(0, 2, usbd_host() ? " host: CONFIGURED " : " host: none       ", usbd_host() ? C_GREEN : C_GRAY, C_BLACK);
            lcd_text(0, 3, usbd_cdc_open() ? " serial: OPEN  " : " serial: closed", usbd_cdc_open() ? C_GREEN : C_GRAY, C_BLACK);
            int c = lcd_text(0, 4, " lines ", C_GRAY, C_BLACK); lcd_text(c, 4, fmt_x(b, lines, 6), C_WHITE, C_BLACK);
            c = lcd_text(0, 5, " bench bytes ", C_GRAY, C_BLACK); lcd_text(c, 5, fmt_x(b, bench_bytes, 8), C_WHITE, C_BLACK);
            uint32_t ms = (bench_until ? now : (bench_t0 ? bench_t0 + 5000 : now)) - bench_t0;
            c = lcd_text(0, 6, " bench KB/s  ", C_GRAY, C_BLACK);
            lcd_text(c, 6, fmt_x(b, ms ? bench_bytes / ms : 0, 6), C_WHITE, C_BLACK);
            lcd_text(0, 14, " Hold POWER = off", C_GRAY, C_BLACK);
        }
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER)) {
                usbd_poll(); WDT_CMD = 0xAAAA;
                if (millis() - t0 > 1000) { backlight(0); usbd_detach(); power_off(); for (;;) WDT_CMD = 0xAAAA; }
            }
        }
        WDT_CMD = 0xAAAA;
    }
}
