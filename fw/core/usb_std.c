#include "usb_std.h"

static const uint8_t dev_desc[18] = {
    18, 1, 0x00, 0x02, 0xEF, 0x02, 0x01, 64,   /* Misc / IAD */
    (uint8_t)USB_VID, (uint8_t)(USB_VID >> 8), (uint8_t)USB_PID, (uint8_t)(USB_PID >> 8),
    0x00, 0x03, 1, 2, 3, 1,
};

static const uint8_t cfg_desc[98] = {
    9, 2, 98, 0, 3, 1, 0, 0x80, 50,                  /* config: 3 interfaces, bus powered 100 mA */
    9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,              /* interface 0: MSC, SCSI transparent, BOT */
    7, 5, 0x81, 0x02, 64, 0, 0,                      /* EP 0x81 IN bulk 64 */
    7, 5, 0x01, 0x02, 64, 0, 0,                      /* EP 0x01 OUT bulk 64 */
    8, 0x0B, 1, 2, 0x02, 0x02, 0x00, 0,             /* IAD: CDC ACM, interfaces 1-2, no AT protocol */
    9, 4, 1, 0, 1, 0x02, 0x02, 0x00, 0,              /* interface 1: CDC comm, protocol 0: not a modem */
    5, 0x24, 0x00, 0x10, 0x01,                       /* CDC header */
    5, 0x24, 0x01, 0x00, 2,                          /* CDC call management: data iface 2 */
    4, 0x24, 0x02, 0x02,                             /* CDC ACM: line coding + control line */
    5, 0x24, 0x06, 1, 2,                             /* CDC union: master 1, slave 2 */
    7, 5, 0x83, 3, 8, 0, 255,                        /* EP 0x83 IN interrupt 8 */
    9, 4, 2, 0, 2, 0x0A, 0, 0, 0,                    /* interface 2: CDC data */
    7, 5, 0x02, 2, 64, 0, 0,                         /* EP 0x02 OUT bulk 64 */
    7, 5, 0x82, 2, 64, 0, 0,                         /* EP 0x82 IN bulk 64 */
};
static const uint8_t lc0[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};   /* 115200 8N1 */

static void make_str(uint8_t *out, const char *s) {
    int n = 0;
    while (s[n]) { out[2 + 2 * n] = (uint8_t)s[n]; out[3 + 2 * n] = 0; n++; }
    out[0] = (uint8_t)(2 + 2 * n); out[1] = 3;
}

void usb_std_init(usb_dev_t *d) {
    d->config = 0;
    d->dtr = 0;
    for (int i = 0; i < 7; i++) d->line_coding[i] = lc0[i];
    d->str[0][0] = 4; d->str[0][1] = 3; d->str[0][2] = 0x09; d->str[0][3] = 0x04;
    make_str(d->str[1], "DSO-510");
    make_str(d->str[2], "DSO-510 Capture Drive");
    make_str(d->str[3], "000000000001");
}

static void in(usb_resp_t *r, const uint8_t *p, uint16_t n, uint16_t wlen) {
    r->act = USB_DATA_IN; r->data = p; r->len = n < wlen ? n : wlen;
}

void usb_std_setup(usb_dev_t *d, const uint8_t s[8], usb_resp_t *r) {
    uint8_t type = s[0], req = s[1];
    uint16_t val = (uint16_t)(s[2] | s[3] << 8), idx = (uint16_t)(s[4] | s[5] << 8);
    uint16_t wlen = (uint16_t)(s[6] | s[7] << 8);
    r->act = USB_STALL; r->data = 0; r->len = 0;
    r->set_address = 0; r->new_address = 0; r->configured_changed = 0; r->clear_halt_ep = 0; r->bot_reset = 0;
    r->dtr_changed = 0;

    if ((type & 0x60) == 0x20) {                         /* class: route by interface */
        uint8_t iface = (uint8_t)idx;
        if (iface == 0) {                                /* MSC BOT */
            if (type == 0xA1 && req == 0xFE) { d->tmp[0] = 0; in(r, d->tmp, 1, wlen); }
            else if (type == 0x21 && req == 0xFF) { r->act = USB_STATUS_IN; r->bot_reset = 1; }
        } else if (iface == 1) {                         /* CDC ACM */
            if (type == 0x21 && req == 0x20) { r->act = USB_DATA_OUT; r->len = wlen < 7 ? wlen : 7; }
            else if (type == 0xA1 && req == 0x21) in(r, d->line_coding, 7, wlen);
            else if (type == 0x21 && req == 0x22) {
                d->dtr = (uint8_t)(val & 1u); r->dtr_changed = 1; r->act = USB_STATUS_IN;
            } else if (type == 0x21 && req == 0x23) r->act = USB_STATUS_IN;
        }
        return;
    }
    if (type & 0x60) return;                             /* vendor: stall */
    switch (req) {
    case 6:                                              /* GET_DESCRIPTOR */
        if (type != 0x80) return;
        switch (val >> 8) {
        case 1: in(r, dev_desc, sizeof dev_desc, wlen); break;
        case 2: in(r, cfg_desc, sizeof cfg_desc, wlen); break;
        case 3: if ((val & 0xFF) < 4) in(r, d->str[val & 0xFF], d->str[val & 0xFF][0], wlen); break;
        default: break;                                  /* qualifier etc.: stall */
        }
        break;
    case 5:                                              /* SET_ADDRESS */
        r->act = USB_STATUS_IN; r->set_address = 1; r->new_address = (uint8_t)(val & 0x7F);
        break;
    case 9:                                              /* SET_CONFIGURATION */
        if (val > 1) return;
        d->config = (uint8_t)val; d->dtr = 0; r->act = USB_STATUS_IN; r->configured_changed = 1;
        break;
    case 8:                                              /* GET_CONFIGURATION */
        d->tmp[0] = d->config; in(r, d->tmp, 1, wlen);
        break;
    case 0:                                              /* GET_STATUS */
        d->tmp[0] = 0; d->tmp[1] = 0; in(r, d->tmp, 2, wlen);
        break;
    case 1: case 3:                                      /* CLEAR_FEATURE / SET_FEATURE */
        r->act = USB_STATUS_IN;
        if (req == 1 && (type & 0x1F) == 2 && val == 0) r->clear_halt_ep = (uint8_t)idx;
        break;
    case 11:                                             /* SET_INTERFACE */
        if (val == 0) r->act = USB_STATUS_IN;
        break;
    case 10:                                             /* GET_INTERFACE */
        d->tmp[0] = 0; in(r, d->tmp, 1, wlen);
        break;
    default:
        break;
    }
}

void usb_std_data_out(usb_dev_t *d, const uint8_t *p, uint16_t n) {
    for (uint16_t i = 0; i < n && i < 7; i++) d->line_coding[i] = p[i];
}
