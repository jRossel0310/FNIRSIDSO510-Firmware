#pragma once
#include <stdint.h>
#include "msc.h"
/* Polled USB FS device: EP0 control + EP1 bulk (0x81 IN / 0x01 OUT) for mass storage.
   No interrupts: call usbd_poll() often (main loop and board_idle()). Re-entrant calls return. */
void usbd_init(msc_t *m);        /* clocks, peripheral reset, D+ pull-up on */
void usbd_poll(void);
int usbd_configured(void);       /* configured by a host and SOFs seen within the last second */
uint32_t usbd_last_sof_ms(void);
