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
    CHECK_EQ(R.data[4], 0xEF); CHECK_EQ(R.data[5], 0x02); CHECK_EQ(R.data[6], 0x01);   /* IAD composite */
    CHECK_EQ(R.data[8], 0x3C); CHECK_EQ(R.data[9], 0x2E); CHECK_EQ(R.data[10], 0x20); CHECK_EQ(R.data[11], 0x57);
    CHECK_EQ(R.data[12], 0x00); CHECK_EQ(R.data[13], 0x03);
    setup(0x80, 6, 0x0100, 0, 8);
    CHECK_EQ(R.len, 8);
    setup(0x80, 6, 0x0200, 0, 9);                        /* config header only */
    CHECK_EQ(R.len, 9); CHECK_EQ(R.data[2], 98); CHECK_EQ(R.data[3], 0); CHECK_EQ(R.data[4], 3);
    setup(0x80, 6, 0x0200, 0, 255);
    CHECK_EQ(R.len, 98);
    const uint8_t *c = R.data;
    /* MSC exactly as before */
    CHECK_EQ(c[9 + 2], 0); CHECK_EQ(c[9 + 5], 0x08); CHECK_EQ(c[9 + 6], 0x06); CHECK_EQ(c[9 + 7], 0x50);
    CHECK_EQ(c[18 + 2], 0x81); CHECK_EQ(c[18 + 3], 2); CHECK_EQ(c[18 + 4], 64);
    CHECK_EQ(c[25 + 2], 0x01);
    /* IAD + CDC */
    CHECK_EQ(c[32], 8); CHECK_EQ(c[33], 0x0B); CHECK_EQ(c[34], 1); CHECK_EQ(c[35], 2); CHECK_EQ(c[36], 0x02);
    CHECK_EQ(c[37], 0x02); CHECK_EQ(c[38], 0x00);              /* IAD function protocol: none */
    CHECK_EQ(c[40], 9); CHECK_EQ(c[41], 4); CHECK_EQ(c[42], 1); CHECK_EQ(c[45], 0x02); CHECK_EQ(c[46], 0x02);
    CHECK_EQ(c[47], 0x00);                     /* comm protocol 0, not AT: ModemManager leaves it alone */
    CHECK_EQ(c[49 + 1], 0x24); CHECK_EQ(c[49 + 2], 0x00);       /* header */
    CHECK_EQ(c[54 + 2], 0x01); CHECK_EQ(c[54 + 4], 2);          /* call management -> data iface 2 */
    CHECK_EQ(c[59 + 2], 0x02); CHECK_EQ(c[59 + 3], 0x02);       /* ACM: line coding + control line */
    CHECK_EQ(c[63 + 2], 0x06); CHECK_EQ(c[63 + 3], 1); CHECK_EQ(c[63 + 4], 2);   /* union */
    CHECK_EQ(c[68 + 2], 0x83); CHECK_EQ(c[68 + 3], 3);          /* notification EP */
    CHECK_EQ(c[75 + 2], 2); CHECK_EQ(c[75 + 5], 0x0A);          /* data iface */
    CHECK_EQ(c[84 + 2], 0x02); CHECK_EQ(c[84 + 3], 2); CHECK_EQ(c[84 + 4], 64);
    CHECK_EQ(c[91 + 2], 0x82); CHECK_EQ(c[91 + 3], 2); CHECK_EQ(c[91 + 4], 64);
    /* walk the descriptor chain: lengths add up to 98 */
    int off = 0, guard = 0;
    while (off < 98 && guard++ < 20) off += c[off];
    CHECK_EQ(off, 98);

    setup(0x80, 6, 0x0300, 0, 255);
    CHECK_EQ(R.len, 4); CHECK_EQ(R.data[2], 0x09); CHECK_EQ(R.data[3], 0x04);
    setup(0x80, 6, 0x0302, 0x0409, 255);
    CHECK_EQ(R.len, 2 + 2 * 21); CHECK_EQ(R.data[2], 'D');
    setup(0x80, 6, 0x0600, 0, 10);
    CHECK_EQ(R.act, USB_STALL);
    setup(0x00, 5, 7, 0, 0);
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.set_address, 1); CHECK_EQ(R.new_address, 7);
    setup(0x00, 9, 1, 0, 0);
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(D.config, 1); CHECK_EQ(R.configured_changed, 1);

    /* MSC class requests on interface 0 */
    setup(0xA1, 0xFE, 0, 0, 1);
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 1); CHECK_EQ(R.data[0], 0);
    setup(0x21, 0xFF, 0, 0, 0);
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.bot_reset, 1);
    setup(0x21, 0xFF, 0, 1, 0);                          /* BOT reset aimed at the CDC iface: stall */
    CHECK_EQ(R.act, USB_STALL); CHECK_EQ(R.bot_reset, 0);

    /* CDC class requests on interface 1 */
    setup(0xA1, 0x21, 0, 1, 7);                          /* GET_LINE_CODING default 115200 8N1 */
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 7);
    CHECK_EQ(R.data[0], 0x00); CHECK_EQ(R.data[1], 0xC2); CHECK_EQ(R.data[2], 0x01); CHECK_EQ(R.data[6], 8);
    setup(0x21, 0x20, 0, 1, 7);                          /* SET_LINE_CODING: data OUT stage */
    CHECK_EQ(R.act, USB_DATA_OUT); CHECK_EQ(R.len, 7);
    const uint8_t lc[7] = {0x00, 0x10, 0x0E, 0x00, 0, 0, 8};   /* 921600 */
    usb_std_data_out(&D, lc, 7);
    setup(0xA1, 0x21, 0, 1, 7);
    CHECK_EQ(R.data[1], 0x10); CHECK_EQ(R.data[2], 0x0E);
    CHECK_EQ(D.dtr, 0);
    setup(0x21, 0x22, 3, 1, 0);                          /* DTR | RTS */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.dtr_changed, 1); CHECK_EQ(D.dtr, 1);
    setup(0x21, 0x22, 0, 1, 0);
    CHECK_EQ(R.dtr_changed, 1); CHECK_EQ(D.dtr, 0);
    setup(0x21, 0x23, 0, 1, 0);                          /* SEND_BREAK */
    CHECK_EQ(R.act, USB_STATUS_IN);
    setup(0x21, 0x22, 1, 1, 0);
    CHECK_EQ(D.dtr, 1);
    setup(0x00, 9, 1, 0, 0);                             /* re-configure clears DTR */
    CHECK_EQ(D.dtr, 0);
    setup(0xA1, 0x21, 0, 2, 7);                          /* wrong interface: stall */
    CHECK_EQ(R.act, USB_STALL);

    setup(0x80, 8, 0, 0, 1);
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.data[0], 1);
    setup(0x80, 0, 0, 0, 2);
    CHECK_EQ(R.len, 2); CHECK_EQ(R.data[0], 0);
    setup(0x02, 1, 0, 0x81, 0);
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.clear_halt_ep, 0x81);
    setup(0x01, 11, 0, 0, 0);
    CHECK_EQ(R.act, USB_STATUS_IN);
    setup(0x40, 0x55, 0, 0, 0);
    CHECK_EQ(R.act, USB_STALL);
    setup(0x00, 9, 0, 0, 0);
    CHECK_EQ(D.config, 0); CHECK_EQ(R.configured_changed, 1);
}
