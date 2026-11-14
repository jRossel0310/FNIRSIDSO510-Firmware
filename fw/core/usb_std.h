#pragma once
#include <stdint.h>

#define USB_VID 0x2E3C
#define USB_PID 0x5720

typedef enum { USB_STALL = 0, USB_DATA_IN, USB_STATUS_IN, USB_DATA_OUT } usb_act_t;

typedef struct {
    usb_act_t act;
    const uint8_t *data;       /* USB_DATA_IN: bytes to send */
    uint16_t len;              /* DATA_IN: limited to wLength; DATA_OUT: bytes to receive */
    uint8_t set_address, new_address;   /* apply the address after the status stage */
    uint8_t configured_changed;
    uint8_t clear_halt_ep;     /* 0 = none */
    uint8_t bot_reset;
    uint8_t dtr_changed;       /* CDC SET_CONTROL_LINE_STATE: d->dtr has the new value */
} usb_resp_t;

typedef struct {
    uint8_t config;
    uint8_t tmp[2];
    uint8_t str[4][2 + 2 * 24];   /* string descriptors 0..3, built by usb_std_init */
    uint8_t line_coding[7];       /* CDC: dwDTERate LE, bCharFormat, bParityType, bDataBits */
    uint8_t dtr;                  /* host has the serial port open */
} usb_dev_t;

/* Interfaces: 0 = MSC (EP 0x81/0x01), 1 = CDC comm (EP 0x83), 2 = CDC data (EP 0x82/0x02). */
void usb_std_init(usb_dev_t *d);
void usb_std_setup(usb_dev_t *d, const uint8_t setup[8], usb_resp_t *r);
/* Data stage of the last USB_DATA_OUT request (SET_LINE_CODING). */
void usb_std_data_out(usb_dev_t *d, const uint8_t *p, uint16_t n);
