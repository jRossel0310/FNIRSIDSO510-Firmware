#include "adc.h"
#include "board.h"

#define TMR4_BASE 0x40000800u
#define TMR_CTRL1(t) REG32((t) + 0x00)
#define TMR_IDEN(t)  REG32((t) + 0x0C)
#define TMR_SWEVT(t) REG32((t) + 0x14)
#define TMR_CM2(t)   REG32((t) + 0x1C)
#define TMR_CCTRL(t) REG32((t) + 0x20)
#define TMR_DIV(t)   REG32((t) + 0x28)
#define TMR_PR(t)    REG32((t) + 0x2C)
#define TMR_C3DT(t)  REG32((t) + 0x3C)
#define TMR_BRK(t)   REG32((t) + 0x44)

#define DMA1_STS      REG32(0x40020000)
#define DMA1_CLR      REG32(0x40020004)
#define DMA1_C7CTRL   REG32(0x40020080)
#define DMA1_C7DTCNT  REG32(0x40020084)
#define DMA1_C7PADDR  REG32(0x40020088)
#define DMA1_C7MADDR  REG32(0x4002008C)
#define DMA1_SRC_SEL1 REG32(0x400200A4)
#define DMA_REQ_TMR4_OVF 0x4F   // stock value for the ch7 field

static volatile int busy;
static volatile uint32_t ring_done;

void adc_init(void) {
    CRM_AHBEN |= 1u << 0;            // DMA1
    CRM_APB1EN |= 1u << 2;           // TMR4
    for (int p = 0; p < 8; p++) gpio_mode(GPIOB, p, CFG_IN_FLOAT);
    // Stock: PB8 output, 1 us high pulse, low, then alternate function (TMR4 CH3).
    gpio_mode(GPIOB, 8, CFG_OUT_PP);
    GPIO_SCR(GPIOB) = 1u << 8;
    delay_ms(1);
    GPIO_CLR(GPIOB) = 1u << 8;
    gpio_mode(GPIOB, 8, CFG_AF_PP);

    TMR_CTRL1(TMR4_BASE) = 0;
    TMR_CM2(TMR4_BASE) = (TMR_CM2(TMR4_BASE) & ~0x70u) | 0x60;   // CH3 PWM mode 1
    TMR_CCTRL(TMR4_BASE) |= 0x300;                                // CH3 enable, active low (stock)
    TMR_IDEN(TMR4_BASE) |= 0x100;                                 // overflow DMA request
    TMR_BRK(TMR4_BASE) |= 0x8000;

    DMA1_C7CTRL = 0;
    DMA1_SRC_SEL1 |= 1u << 24;
    DMA1_SRC_SEL1 = (DMA1_SRC_SEL1 & 0xFF00FFFFu) | ((uint32_t)DMA_REQ_TMR4_OVF << 16);
    DMA1_C7PADDR = 0x40010C08;                                    // GPIOB->IDT
    busy = 0;
}

void adc_set_rate(uint16_t psc, uint16_t arr) {
    adc_abort();
    TMR_CTRL1(TMR4_BASE) &= ~1u;
    TMR_DIV(TMR4_BASE) = psc;
    TMR_PR(TMR4_BASE) = arr;
    TMR_C3DT(TMR4_BASE) = (arr + 1u) / 2u;
    TMR_SWEVT(TMR4_BASE) = 1;
    TMR_CTRL1(TMR4_BASE) |= 1;
    // Let >= 64 conversions flush the pipeline: 64 * (psc+1)(arr+1) / 240 MHz.
    uint32_t us = (uint32_t)((uint64_t)64u * (psc + 1u) * (arr + 1u) / 240u);
    delay_ms(us / 1000u + 1u);
}

void adc_start(uint8_t *buf, uint16_t n) {
    DMA1_C7CTRL = 0;
    DMA1_CLR = 0xFu << 24;
    DMA1_C7MADDR = (uint32_t)buf;
    DMA1_C7DTCNT = n;
    busy = 1;
    DMA1_C7CTRL = 0x3000 | 0x80 | 1;   // very high prio, 8-bit, mem increment, enable
}

int adc_done(void) {
    if (!busy) return 1;
    if (DMA1_STS & (1u << 25)) {       // ch7 transfer complete
        DMA1_C7CTRL = 0;
        DMA1_CLR = 0xFu << 24;
        busy = 0;
        return 1;
    }
    return 0;
}

void adc_abort(void) {
    DMA1_C7CTRL = 0;                   /* also clears HTIE/TCIE */
    NVIC_ICER(0) = 1u << 17;
    DMA1_CLR = 0xFu << 24;
    busy = 0;
}

#define DMA_CH7_HT (1u << 26)
#define DMA_CH7_TC (1u << 25)

void adc_ring_start(uint8_t *buf, uint16_t n) {
    DMA1_C7CTRL = 0;
    DMA1_CLR = 0xFu << 24;
    DMA1_C7MADDR = (uint32_t)buf;
    DMA1_C7DTCNT = n;
    busy = 0;                          /* not a one-shot capture: adc_done() stays 1 */
    ring_done = 0;
    NVIC_ICPR(0) = 1u << 17;
    NVIC_ISER(0) = 1u << 17;
    /* very high prio, 8-bit, mem increment, circular (0x20), HTIE (4), TCIE (2), enable */
    DMA1_C7CTRL = 0x3000 | 0x80 | 0x20 | 4 | 2 | 1;
}

uint32_t adc_ring_completed(void) { return ring_done; }

void DMA1_Channel7_IRQHandler(void) {
    uint32_t s = DMA1_STS & (DMA_CH7_HT | DMA_CH7_TC);
    if (s & DMA_CH7_HT) ring_done++;
    if (s & DMA_CH7_TC) ring_done++;
    DMA1_CLR = s;
}
