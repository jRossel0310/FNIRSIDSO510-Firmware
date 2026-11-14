#pragma once
#include <stdint.h>
#include "msc.h"
#include "bytefifo.h"
/* USB FS composite device (MSC + CDC-ACM serial port): EP0 control + EP1 bulk (0x81 IN / 0x01 OUT)
   for mass storage + EP2 bulk / EP3 notify for the serial port. Interrupt driven
   (IRQ20): bus reset, EP0, EP1 and EP2 packets are handled in the ISR. Block I/O runs in main context:
   call usbd_poll() often (main loop and board_idle()). Re-entrant calls return. */
void usbd_init(msc_t *m);        /* clocks, peripheral reset, USB IRQ on, D+ pull-up on */
void usbd_poll(void);
void usbd_detach(void);          /* D+ pull-up off (host sees an unplug), USB IRQ off, not configured */
void usbd_attach(void);          /* D+ pull-up on again: the host re-enumerates */
int usbd_host(void);             /* configured by a host (also while the bus is suspended) */
int usbd_configured(void);       /* configured by a host and SOFs seen within the last second */
uint32_t usbd_last_sof_ms(void);

/* CDC-ACM serial port (interfaces 1-2, EP 0x82/0x02 bulk, 0x83 notifications). */
int usbd_cdc_open(void);                      /* configured, attached, and DTR set by the host */
bytefifo_t *usbd_cdc_txfifo(void);            /* main context only: append bytes to send */
uint32_t usbd_cdc_read(uint8_t *p, uint32_t max);   /* main context only: bytes received */
void usbd_cdc_kick(void);                     /* main context: start sending if the endpoint is idle */
