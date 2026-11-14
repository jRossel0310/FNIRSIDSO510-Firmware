#include "test.h"
#include "usb_std.h"

static usb_dev_t D;
static usb_resp_t R;
static void setup(uint8_t type, uint8_t req, uint16_t val, uint16_t idx, uint16_t len) {
    uint8_t s[8] = {type, req, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)idx, (uint8_t)(idx >> 8),
                    (uint8_t)len, (uint8_t)(len >> 8)};
    usb_std_setup(&D, s, &R);
}

void test_usb_std(void) {
    usb_std_init(&D);
    setup(0x80, 6, 0x0100, 0, 64);                       /* device descriptor */
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 18);
    CHECK_EQ(R.data[0], 18); CHECK_EQ(R.data[1], 1); CHECK_EQ(R.data[7], 64);
    CHECK_EQ(R.data[8], 0x3C); CHECK_EQ(R.data[9], 0x2E); CHECK_EQ(R.data[10], 0x20); CHECK_EQ(R.data[11], 0x57);
    setup(0x80, 6, 0x0100, 0, 8);
    CHECK_EQ(R.len, 8);                                    /* truncated to wLength */
    setup(0x80, 6, 0x0200, 0, 9);                        /* config header only */
    CHECK_EQ(R.len, 9); CHECK_EQ(R.data[2], 32);
    setup(0x80, 6, 0x0200, 0, 255);
    CHECK_EQ(R.len, 32);
    CHECK_EQ(R.data[9 + 5], 0x08); CHECK_EQ(R.data[9 + 6], 0x06); CHECK_EQ(R.data[9 + 7], 0x50); /* MSC SCSI BOT */
    CHECK_EQ(R.data[18 + 2], 0x81); CHECK_EQ(R.data[18 + 3], 2); CHECK_EQ(R.data[18 + 4], 64);
    CHECK_EQ(R.data[25 + 2], 0x01);
    setup(0x80, 6, 0x0300, 0, 255);                      /* language ids */
    CHECK_EQ(R.len, 4); CHECK_EQ(R.data[2], 0x09); CHECK_EQ(R.data[3], 0x04);
    setup(0x80, 6, 0x0302, 0x0409, 255);                 /* product "DSO-510 Capture Drive" */
    CHECK_EQ(R.len, 2 + 2 * 21); CHECK_EQ(R.data[1], 3); CHECK_EQ(R.data[2], 'D'); CHECK_EQ(R.data[3], 0);
    setup(0x80, 6, 0x0303, 0x0409, 255);                 /* serial, 12 chars */
    CHECK_EQ(R.len, 2 + 2 * 12);
    setup(0x80, 6, 0x0304, 0x0409, 255);
    CHECK_EQ(R.act, USB_STALL);
    setup(0x80, 6, 0x0600, 0, 10);                       /* device qualifier: full-speed only */
    CHECK_EQ(R.act, USB_STALL);
    setup(0x00, 5, 7, 0, 0);                             /* SET_ADDRESS */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.set_address, 1); CHECK_EQ(R.new_address, 7);
    setup(0x00, 9, 1, 0, 0);                             /* SET_CONFIGURATION 1 */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(D.config, 1); CHECK_EQ(R.configured_changed, 1);
    setup(0x80, 8, 0, 0, 1);
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 1); CHECK_EQ(R.data[0], 1);
    setup(0x00, 9, 2, 0, 0);
    CHECK_EQ(R.act, USB_STALL);
    setup(0x80, 0, 0, 0, 2);                             /* GET_STATUS */
    CHECK_EQ(R.len, 2); CHECK_EQ(R.data[0], 0);
    setup(0x02, 1, 0, 0x81, 0);                          /* CLEAR_FEATURE ENDPOINT_HALT 0x81 */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.clear_halt_ep, 0x81);
    setup(0xA1, 0xFE, 0, 0, 1);                          /* GET_MAX_LUN */
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 1); CHECK_EQ(R.data[0], 0);
    setup(0x21, 0xFF, 0, 0, 0);                          /* BOT reset */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.bot_reset, 1);
    setup(0x01, 11, 0, 0, 0);                            /* SET_INTERFACE alt 0 */
    CHECK_EQ(R.act, USB_STATUS_IN);
    setup(0x40, 0x55, 0, 0, 0);                          /* vendor request */
    CHECK_EQ(R.act, USB_STALL);
    setup(0x00, 9, 0, 0, 0);                             /* deconfigure */
    CHECK_EQ(D.config, 0); CHECK_EQ(R.configured_changed, 1);
}
