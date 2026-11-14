#include "save.h"
#include "capsave.h"
#include "usbd.h"
#include "board.h"

const char *save_capture(const uint8_t *rec, int n, const csv_meta_t *m, msc_t *msc) {
    static char msg[15] = " Saved CAP0000";
    int host = usbd_configured();
    if (host) {
        uint32_t t0 = millis();
        while (msc_busy(msc, millis())) {
            usbd_poll();
            WDT_CMD = 0xAAAA;
            if (millis() - t0 > 5000) return " Busy, retry  ";
        }
        msc_set_present(msc, 0);                        /* host sees MEDIUM NOT PRESENT */
    }
    int num = 0;
    cap_res_t r = capsave_write(rec, n, m, &num);
    if (host) msc_set_present(msc, 1);                  /* host sees UNIT ATTENTION, re-reads */
    if (r == CAP_FULL) return " Drive full   ";
    if (r != CAP_OK) return " Save failed  ";
    for (int i = 13; i >= 10; i--) { msg[i] = (char)('0' + num % 10); num /= 10; }
    return msg;
}
