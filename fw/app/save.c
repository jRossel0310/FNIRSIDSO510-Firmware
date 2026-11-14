#include "save.h"
#include "capsave.h"
#include "usbd.h"
#include "board.h"

#define IDLE_WAIT_MS 5000u
#define UNPLUG_MS    500u          /* D+ off long enough for every host to register the unplug */

static void tick(void (*poll)(void)) { WDT_CMD = 0xAAAA; if (poll) poll(); }

const char *save_capture(const uint8_t *rec, int n, const csv_meta_t *m, msc_t *msc, void (*poll)(void)) {
    static char msg[15] = " Saved CAP0000";
    int host = usbd_host();                             /* configured, also while suspended */
    if (host) {
        uint32_t t0 = millis();
        while (msc_busy(msc, millis()) && usbd_configured()) {   /* let the host finish; stop if it left */
            usbd_poll();
            tick(poll);
            if (millis() - t0 > IDLE_WAIT_MS) return " Busy, retry  ";
        }
    }
    msc_set_present(msc, 0);                            /* any late command fails with 02/3A/00 */
    if (host) {                                         /* soft unplug: the host drops its mount and cache */
        usbd_detach();
        for (uint32_t t0 = millis(); millis() - t0 < UNPLUG_MS; ) tick(poll);
    }
    int num = 0;
    cap_res_t r = capsave_write(rec, n, m, &num);
    msc_set_present(msc, 1);
    if (host) usbd_attach();                            /* re-plug: fresh enumeration and mount */
    if (r == CAP_FULL) return " Drive full   ";
    if (r != CAP_OK) return " Save failed  ";
    for (int i = 13; i >= 10; i--) { msg[i] = (char)('0' + num % 10); num /= 10; }
    return msg;
}
