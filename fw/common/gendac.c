#include "gendac.h"
#include "board.h"

#define TMR6_BASE 0x40001000u
#define TMR_CTRL1(t) REG32((t) + 0x00)
#define TMR_CTRL2(t) REG32((t) + 0x04)
#define TMR_SWEVT(t) REG32((t) + 0x14)
#define TMR_DIV(t)   REG32((t) + 0x28)
#define TMR_PR(t)    REG32((t) + 0x2C)
#define DAC_CTRL     REG32(0x40007400)
#define DAC_D1DTH12R REG32(0x40007408)
#define DMA2_C3CTRL  REG32(0x40020430)
#define DMA2_C3DTCNT REG32(0x40020434)
#define DMA2_C3PADDR REG32(0x40020438)
#define DMA2_C3MADDR REG32(0x4002043C)
#define DMA2_SRC_SEL0 REG32(0x400204A0)
#define DMA2_SRC_SEL1 REG32(0x400204A4)
#define DMA_REQ_DAC1 0x05

/* DAC_CTRL channel 1 bits: EN 0, output buffer off 1, trigger enable 2, trigger select 5:3, wave 7:6, DMA 12 */
#define DAC1_BITS 0x0000FFFFu

static void stop(void) {
    TMR_CTRL1(TMR6_BASE) &= ~1u;                       /* no more updates */
    DMA2_C3CTRL &= ~1u;
    DAC_CTRL &= ~(0x1000u | 4u);                       /* DMA request off, trigger off: writes take effect at once */
}

void gendac_init(void) {
    CRM_APB1EN |= (1u << 29) | (1u << 4);              /* DAC, TMR6 */
    CRM_AHBEN |= 1u << 1;                              /* DMA2 */
    gpio_mode(GPIOA, 4, CFG_ANALOG);
    TMR_CTRL1(TMR6_BASE) = 0;
    TMR_CTRL2(TMR6_BASE) = (TMR_CTRL2(TMR6_BASE) & ~0x70u) | 0x20;   /* update event -> trigger output */
    DMA2_C3CTRL = 0;
    DMA2_SRC_SEL1 |= 1u << 24;                         /* flexible request mapping on */
    DMA2_SRC_SEL0 = (DMA2_SRC_SEL0 & 0xFF00FFFFu) | ((uint32_t)DMA_REQ_DAC1 << 16);
    DAC_CTRL = (DAC_CTRL & ~DAC1_BITS) | 2u;           /* channel 1: buffer off (as stock), TMR6 trigger select 000 */
    DAC_D1DTH12R = 0;
    DAC_CTRL |= 1u;                                    /* enable: output 0 V */
}

void gendac_load(const uint16_t *tab, uint16_t n, uint16_t psc, uint16_t arr) {
    stop();
    TMR_DIV(TMR6_BASE) = psc;
    TMR_PR(TMR6_BASE) = arr;
    DMA2_C3CTRL = 0;
    DMA2_C3PADDR = (uint32_t)&DAC_D1DTH12R;
    DMA2_C3MADDR = (uint32_t)tab;
    DMA2_C3DTCNT = n;
    DMA2_C3CTRL = 0x10 | 0x20 | 0x80 | 0x200 | 0x400;  /* mem -> periph, circular, mem increment, 16-bit both */
    DMA2_C3CTRL |= 1u;
    DAC_CTRL |= 4u;                                    /* trigger enable (TMR6) */
    DAC_CTRL |= 0x1000u;                               /* DMA request enable */
    TMR_SWEVT(TMR6_BASE) = 1;                          /* load prescaler/reload, first sample */
    TMR_CTRL1(TMR6_BASE) |= 1u;
}

void gendac_hold(void) { stop(); }

void gendac_off(void) {
    stop();
    DAC_D1DTH12R = 0;                                  /* trigger is off: 0 V at once */
}
