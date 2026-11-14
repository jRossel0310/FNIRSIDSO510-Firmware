#include "usb_std.h"

static const uint8_t dev_desc[18] = {
    18, 1, 0x00, 0x02, 0, 0, 0, 64,
    (uint8_t)USB_VID, (uint8_t)(USB_VID >> 8), (uint8_t)USB_PID, (uint8_t)(USB_PID >> 8),
    0x00, 0x02, 1, 2, 3, 1,
};

static const uint8_t cfg_desc[32] = {
    9, 2, 32, 0, 1, 1, 0, 0x80, 50,                 /* config: 1 interface, bus powered 100 mA */
    9, 4, 0, 0, 2, 0x08, 0x06, 0x50, 0,              /* interface: MSC, SCSI transparent, BOT */
    7, 5, 0x81, 0x02, 64, 0, 0,                      /* EP1 IN bulk 64 */
    7, 5, 0x01, 0x02, 64, 0, 0,                      /* EP1 OUT bulk 64 */
};

static void make_str(uint8_t *out, const char *s) {
    int n = 0;
    while (s[n]) { out[2 + 2 * n] = (uint8_t)s[n]; out[3 + 2 * n] = 0; n++; }
    out[0] = (uint8_t)(2 + 2 * n); out[1] = 3;
}

void usb_std_init(usb_dev_t *d) {
    d->config = 0;
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

    if ((type & 0x60) == 0x20) {                         /* class (MSC BOT) */
        if (type == 0xA1 && req == 0xFE) { d->tmp[0] = 0; in(r, d->tmp, 1, wlen); }
        else if (type == 0x21 && req == 0xFF) { r->act = USB_STATUS_IN; r->bot_reset = 1; }
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
        d->config = (uint8_t)val; r->act = USB_STATUS_IN; r->configured_changed = 1;
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
