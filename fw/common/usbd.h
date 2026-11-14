#pragma once
#include <stdint.h>
#include "msc.h"
/* USB FS device: EP0 control + EP1 bulk (0x81 IN / 0x01 OUT) for mass storage. Interrupt driven
   (IRQ20): bus reset, EP0 and EP1 packets are handled in the ISR. Block I/O runs in main context:
   call usbd_poll() often (main loop and board_idle()). Re-entrant calls return. */
void usbd_init(msc_t *m);        /* clocks, peripheral reset, USB IRQ on, D+ pull-up on */
void usbd_poll(void);
void usbd_detach(void);          /* D+ pull-up off (host sees an unplug), USB IRQ off, not configured */
void usbd_attach(void);          /* D+ pull-up on again: the host re-enumerates */
int usbd_host(void);             /* configured by a host (also while the bus is suspended) */
int usbd_configured(void);       /* configured by a host and SOFs seen within the last second */
uint32_t usbd_last_sof_ms(void);
