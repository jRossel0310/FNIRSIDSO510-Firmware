#include "usbd.h"
#include "usb_std.h"
#include "board.h"

#define USB_BASE   0x40005C00u
#define USB_EPR(n) REG32(USB_BASE + 4u * (n))
#define USB_CNTR   REG32(USB_BASE + 0x40)
#define USB_ISTR   REG32(USB_BASE + 0x44)
#define USB_DADDR  REG32(USB_BASE + 0x4C)
#define USB_BTABLE REG32(USB_BASE + 0x50)
#define USB_CFG    REG32(USB_BASE + 0x60)      /* AT32: bit 1 = 1 disables the D+ pull-up */
#define PMA(off)   (*(volatile uint32_t *)(0x40006000u + 2u * (off)))
#define ACC_CTRL1  REG32(0x40015804)
#define ACC_C1     REG32(0x4001580C)
#define ACC_C2     REG32(0x40015810)
#define ACC_C3     REG32(0x40015814)

#define ISTR_CTR   0x8000u
#define ISTR_SUSP  0x0800u
#define ISTR_WKUP  0x1000u
#define ISTR_RESET 0x0400u
#define ISTR_SOF   0x0200u
#define EP_CTR_RX  0x8000u
#define EP_SETUP   0x0800u
#define EP_CTR_TX  0x0080u
#define EP_MASK    0x8F8Fu                      /* non-toggle bits: CTR_RX SETUP TYPE KIND CTR_TX EA */
#define EP_CONTROL 0x0200u
#define ST_STALL 1u
#define ST_NAK   2u
#define ST_VALID 3u

/* PMA layout (byte offsets): BTABLE 0x00, EP0 RX 0x40, EP0 TX 0x80, EP1 RX 0xC0, EP1 TX 0x100 */
#define EP0_RX 0x40u
#define EP0_TX 0x80u
#define EP1_RX 0xC0u
#define EP1_TX 0x100u
#define RX64   0x8400u                          /* COUNT_RX: BL_SIZE=1, NUM_BLOCK=1 -> 64 bytes */

#define CNTR_IRQS  0xBE00u                         /* CTRM WKUPM SUSPM RESETM SOFM */
#define USB_IRQ_BIT (1u << 20)                      /* IRQ20: USBFS low priority, NVIC ISER0/ICER0 */

/* Everything below except `busy`, `lock_depth`, `irq_on` and `detached` is owned by the ISR; main
   context touches it only with the USB IRQ masked (usb_lock) or while the IRQ is off (detached). */
static msc_t *msc;
static usb_dev_t dev;
static uint8_t inited, busy, tx_busy, rx_held, rx_len, pending_addr, addr;
static volatile uint8_t suspended;
static uint8_t irq_on, detached, lock_depth;
static uint8_t rxbuf[64], txbuf[64];
static const uint8_t *ctl_ptr;
static uint16_t ctl_left;
static uint8_t ctl_zlp;
static volatile uint32_t last_sof;

/* Mask the USB IRQ (main context only; nests). The ISR never takes it. */
static void usb_lock(void) {
    NVIC_ICER(0) = USB_IRQ_BIT;
    __asm volatile ("dsb\n isb" ::: "memory");       /* no USB ISR runs after this point */
    lock_depth++;
}
static void usb_unlock(void) {
    __asm volatile ("" ::: "memory");
    if (--lock_depth == 0 && irq_on) NVIC_ISER(0) = USB_IRQ_BIT;
}

static void ep_set_tx(int n, uint32_t s) { uint32_t v = USB_EPR(n) & (EP_MASK | 0x0030u); v ^= s << 4; USB_EPR(n) = v | 0x8080u; }
static void ep_set_rx(int n, uint32_t s) { uint32_t v = USB_EPR(n) & (EP_MASK | 0x3000u); v ^= s << 12; USB_EPR(n) = v | 0x8080u; }
static void ep_clr_ctr_rx(int n) { USB_EPR(n) = (USB_EPR(n) & EP_MASK & ~EP_CTR_RX) | EP_CTR_TX; }
static void ep_clr_ctr_tx(int n) { USB_EPR(n) = (USB_EPR(n) & EP_MASK & ~EP_CTR_TX) | EP_CTR_RX; }
static void ep_clr_dtog(int n) { uint32_t v = USB_EPR(n); USB_EPR(n) = (v & EP_MASK) | (v & 0x4040u) | 0x8080u; }

static void ep_clr_dtog_bits(int n, uint32_t m) { uint32_t v = USB_EPR(n); USB_EPR(n) = (v & EP_MASK) | (v & m) | 0x8080u; }

static void pma_write(uint32_t off, const uint8_t *b, int n) {
    for (int i = 0; i < n; i += 2) PMA(off + (uint32_t)i) = b[i] | (uint32_t)(i + 1 < n ? b[i + 1] : 0) << 8;
}
static void pma_read(uint32_t off, uint8_t *b, int n) {
    for (int i = 0; i < n; i += 2) {
        uint32_t w = PMA(off + (uint32_t)i);
        b[i] = (uint8_t)w;
        if (i + 1 < n) b[i + 1] = (uint8_t)(w >> 8);
    }
}
static int rx_count(int ep) { return (int)(PMA((uint32_t)ep * 8u + 6u) & 0x3FFu); }

static void bus_reset(void) {
    USB_BTABLE = 0;
    PMA(0) = EP0_TX; PMA(2) = 0; PMA(4) = EP0_RX; PMA(6) = RX64;
    PMA(8) = EP1_TX; PMA(10) = 0; PMA(12) = EP1_RX; PMA(14) = RX64;
    USB_EPR(0) = EP_CONTROL | 0u;
    ep_clr_dtog(0); ep_set_rx(0, ST_VALID); ep_set_tx(0, ST_NAK);
    USB_EPR(1) = 0x0000u | 1u;                          /* bulk, address 1 */
    ep_clr_dtog(1); ep_set_rx(1, ST_VALID); ep_set_tx(1, ST_NAK);
    USB_DADDR = 0x80u;                                  /* enable, address 0 */
    usb_std_init(&dev);
    msc_reset(msc);
    tx_busy = rx_held = pending_addr = ctl_left = ctl_zlp = 0;
    suspended = 0;
}

/* EP1 packet movement: offer a held OUT packet to msc (stays NAKed while msc says -1), then start
   the next IN packet if the endpoint is free. ISR context, or main context with the IRQ masked. */
static void ep1_pump(void) {
    if (detached || !dev.config) return;
    if (rx_held && msc_rx(msc, rxbuf, rx_len, millis()) == 0) { rx_held = 0; ep_set_rx(1, ST_VALID); }
    if (!tx_busy) {
        int n = msc_tx(msc, txbuf);
        if (n >= 0) { pma_write(EP1_TX, txbuf, n); PMA(10) = (uint32_t)n; ep_set_tx(1, ST_VALID); tx_busy = 1; }
    }
}

static void ctl_send(void) {
    uint16_t n = ctl_left > 64 ? 64 : ctl_left;
    pma_write(EP0_TX, ctl_ptr, n);
    PMA(2) = n;
    ctl_ptr += n; ctl_left = (uint16_t)(ctl_left - n);
    if (n < 64) ctl_zlp = 0;                            /* a short packet ends the transfer */
    ep_set_tx(0, ST_VALID);
}

static void ep0_setup(void) {
    uint8_t s[8];
    usb_resp_t r;
    pma_read(EP0_RX, s, 8);
    ep_clr_ctr_rx(0);
    ctl_left = 0; ctl_zlp = 0;                          /* a new SETUP aborts any earlier IN transfer */
    usb_std_setup(&dev, s, &r);
    if (r.bot_reset) {
        msc_reset(msc);
        rx_held = 0; tx_busy = 0;
        ep_set_tx(1, ST_NAK); ep_set_rx(1, ST_VALID);
    }
    if (r.clear_halt_ep == 0x81) {                      /* host resets its IN toggle to DATA0 */
        ep_clr_dtog_bits(1, 0x0040u); ep_set_tx(1, ST_NAK); tx_busy = 0;
    } else if (r.clear_halt_ep == 0x01) {               /* host resets its OUT toggle to DATA0 */
        ep_clr_dtog_bits(1, 0x4000u); ep_set_rx(1, ST_VALID);
    }
    if (r.configured_changed) {
        ep_clr_dtog_bits(1, 0x4040u);
        ep_set_rx(1, ST_VALID); ep_set_tx(1, ST_NAK);
        tx_busy = rx_held = 0;
        msc_reset(msc);
    }
    if (r.set_address) { pending_addr = 1; addr = r.new_address; }
    switch (r.act) {
    case USB_DATA_IN: {
        uint16_t wlen = (uint16_t)(s[6] | s[7] << 8);
        ctl_ptr = r.data; ctl_left = r.len;
        ctl_zlp = (uint8_t)(r.len < wlen && (r.len % 64) == 0);   /* end with a ZLP */
        ctl_send();
        ep_set_rx(0, ST_VALID);                         /* status OUT */
        break;
    }
    case USB_STATUS_IN:
        PMA(2) = 0; ep_set_tx(0, ST_VALID); ep_set_rx(0, ST_VALID);
        break;
    default:
        ep_set_tx(0, ST_STALL); ep_set_rx(0, ST_VALID);  /* stall; next SETUP still accepted */
        break;
    }
}

static void service_ctr(void) {
    while (USB_ISTR & ISTR_CTR) {
        int ep = (int)(USB_ISTR & 0xFu);
        uint32_t v = USB_EPR(ep);
        if (ep == 0) {
            if (v & EP_CTR_TX) {
                ep_clr_ctr_tx(0);
                if (pending_addr) { USB_DADDR = 0x80u | addr; pending_addr = 0; }
                if (ctl_left || ctl_zlp) ctl_send();
            }
            if (v & EP_CTR_RX) {
                if (v & EP_SETUP) ep0_setup();
                else { ep_clr_ctr_rx(0); ep_set_rx(0, ST_VALID); }   /* status OUT */
            }
        } else if (ep == 1) {
            if (v & EP_CTR_RX) {                        /* hardware NAKs further OUT packets now */
                ep_clr_ctr_rx(1);
                rx_len = (uint8_t)(rx_count(1) > 64 ? 64 : rx_count(1));
                pma_read(EP1_RX, rxbuf, rx_len);
                rx_held = 1;
            }
            if (v & EP_CTR_TX) { ep_clr_ctr_tx(1); tx_busy = 0; }
            ep1_pump();
        } else {
            USB_EPR(ep) = 0;                            /* unexpected endpoint */
        }
    }
}

static void usb_service(void) {
    uint32_t is = USB_ISTR;
    if (is & ISTR_RESET) { USB_ISTR = (uint16_t)~ISTR_RESET; bus_reset(); }
    if (is & ISTR_SOF) { USB_ISTR = (uint16_t)~ISTR_SOF; last_sof = millis(); suspended = 0; }
    if (is & ISTR_SUSP) { USB_ISTR = (uint16_t)~ISTR_SUSP; suspended = 1; }
    if (is & ISTR_WKUP) { USB_ISTR = (uint16_t)~ISTR_WKUP; suspended = 0; }
    USB_ISTR = (uint16_t)~(0x6000u | 0x0100u);          /* clear PMAOVR, ERR, ESOF */
    service_ctr();
}

/* Reset, SOF/suspend, all of EP0 (SET_ADDRESS applied right after its status IN) and EP1 packets. */
void USB_LP_IRQHandler(void) { usb_service(); }

static void irq_start(void) {
    USB_ISTR = 0u;
    USB_CNTR = CNTR_IRQS;
    NVIC_ICPR(0) = USB_IRQ_BIT;
    irq_on = 1;
    if (!lock_depth) NVIC_ISER(0) = USB_IRQ_BIT;
}

static void irq_stop(void) {
    irq_on = 0;
    NVIC_ICER(0) = USB_IRQ_BIT;
    __asm volatile ("dsb\n isb" ::: "memory");
    USB_CNTR = 0u;
    NVIC_ICPR(0) = USB_IRQ_BIT;
}

void usbd_init(msc_t *m) {
    msc = m;
    msc_set_lock(m, usb_lock, usb_unlock);
    CRM_MISC1 |= 1u << 25;                              /* stock: HICK 48 MHz for USB */
    CRM_MISC3 |= 0x200u;
    CRM_MISC3 |= 0x100u;
    CRM_APB2EN |= 1u << 22;                             /* ACC: trim HICK to USB SOF */
    ACC_C1 = 0x1F2C; ACC_C2 = 8000; ACC_C3 = 0x1F54;
    ACC_CTRL1 |= 2u;
    ACC_CTRL1 |= 1u;
    CRM_APB1EN |= 1u << 23;                             /* USB */
    USB_CNTR = 1u;                                      /* FRES, power up (PDWN = 0) */
    for (uint32_t t0 = millis(); millis() - t0 < 2; ) WDT_CMD = 0xAAAA;
    USB_CNTR = 0u;
    USB_ISTR = 0u;
    bus_reset();
    inited = 1;
    irq_start();
    USB_CFG &= ~2u;                                     /* D+ pull-up on: host may enumerate */
}

/* Main context: block I/O, then re-offer what the finished I/O unblocked (nothing else would make
   the ISR run for it). Re-entrant calls (board_idle) return. */
void usbd_poll(void) {
    if (!inited || busy) return;
    busy = 1;
    msc_task(msc);                                      /* locks only around its state accesses */
    usb_lock();
    ep1_pump();
    usb_unlock();
    busy = 0;
}

void usbd_detach(void) {
    if (!inited || detached) return;
    usb_lock();
    USB_CFG |= 2u;                                      /* D+ pull-up off: the host sees an unplug */
    irq_stop();                                         /* no reset/SE0 storm, no EP1 service */
    detached = 1;
    ep_set_rx(1, ST_NAK); ep_set_tx(1, ST_NAK);
    usb_std_init(&dev);                                 /* not configured any more */
    msc_reset(msc);
    tx_busy = rx_held = pending_addr = ctl_left = ctl_zlp = 0;
    usb_unlock();
}

void usbd_attach(void) {
    if (!inited || !detached) return;
    usb_lock();
    bus_reset();                                        /* address 0, EP0 ready; the host resets us anyway */
    detached = 0;
    irq_start();                                        /* stays masked until usb_unlock */
    USB_CFG &= ~2u;                                     /* D+ pull-up on: the host re-enumerates */
    usb_unlock();
}

int usbd_host(void) {
    return inited && !detached && *(volatile uint8_t *)&dev.config != 0;
}

int usbd_configured(void) {
    return usbd_host() && !suspended && (millis() - last_sof) < 1000u;
}

uint32_t usbd_last_sof_ms(void) { return last_sof; }
