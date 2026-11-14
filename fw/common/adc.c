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

#define TMR1_BASE 0x40012C00u
#define TMR3_BASE 0x40000400u
#define TMR5_BASE 0x40000C00u
#define TMR8_BASE 0x40013400u
#define TMR_CTRL2(t)  REG32((t) + 0x04)
#define TMR_STCTRL(t) REG32((t) + 0x08)
#define TMR_CVAL(t)   REG32((t) + 0x24)
#define DMA1_SRC_SEL0 REG32(0x400200A0)
#define DMA2_CLR      REG32(0x40020404)
#define DMA2_SRC_SEL0 REG32(0x400204A0)
#define DMA2_SRC_SEL1 REG32(0x400204A4)
#define D_CTRL(d)  REG32((d) + 0x00)
#define D_CNT(d)   REG32((d) + 0x04)
#define D_PADDR(d) REG32((d) + 0x08)
#define D_MADDR(d) REG32((d) + 0x0C)

/* stream k: slave timer, its trigger-select bits (ITR to TMR4), DMA channel, start count */
static const struct { uint32_t tmr, ts, dma, cv; } ilv_s[4] = {
    {TMR3_BASE, 0x30, 0x40020030, 6},   /* DMA1 ch3 */
    {TMR5_BASE, 0x20, 0x4002041C, 5},   /* DMA2 ch2 */
    {TMR1_BASE, 0x30, 0x40020058, 4},   /* DMA1 ch5 */
    {TMR8_BASE, 0x20, 0x40020408, 3},   /* DMA2 ch1 */
};
static uint8_t ilv_mode;                 /* TMR4 is in the 48 MHz interleave configuration */
static uint8_t ilv_n;                    /* streams of the capture in progress / last capture */
static uint8_t ilv_busy;
static uint8_t ilv_hw[4];                /* logical stream -> hardware stream */
static const uint8_t *ilv_first[4];      /* logical stream -> first valid sample */

static void ilv_slaves_off(void) {
    TMR_CTRL2(TMR4_BASE) &= ~0x70u;      /* TRGO idle: the slaves cannot count */
    for (int k = 0; k < 4; k++) {
        TMR_IDEN(ilv_s[k].tmr) &= ~0x100u;
        TMR_CTRL1(ilv_s[k].tmr) &= ~1u;
        D_CTRL(ilv_s[k].dma) = 0;
    }
    DMA1_CLR = 0x000F0F00u;              /* ch3, ch5 flags */
    DMA2_CLR = 0x000000FFu;              /* ch1, ch2 flags */
    ilv_busy = 0;
}

void adc_ilv_enter(void) {
    adc_abort();
    CRM_AHBEN |= 3u;                                         /* DMA1, DMA2 */
    CRM_APB1EN |= (1u << 1) | (1u << 2) | (1u << 3);         /* TMR3, TMR4, TMR5 */
    CRM_APB2EN |= (1u << 11) | (1u << 13);                   /* TMR1, TMR8 */
    ilv_slaves_off();
    DMA2_SRC_SEL1 |= 1u << 24;                               /* flexible mapping */
    DMA2_SRC_SEL0 = (DMA2_SRC_SEL0 & 0xFFFF0000u) | 0x576Fu; /* ch1 = TMR8 overflow, ch2 = TMR5 overflow */
    DMA1_SRC_SEL1 |= 1u << 24;
    DMA1_SRC_SEL0 = (DMA1_SRC_SEL0 & 0xFF00FFFFu) | 0x470000u;   /* ch3 = TMR3 overflow */
    DMA1_SRC_SEL1 = (DMA1_SRC_SEL1 & 0xFFFFFF00u) | 0x37u;       /* ch5 = TMR1 overflow */
    if (!ilv_mode) {
        TMR_CTRL1(TMR4_BASE) &= ~1u;
        TMR_IDEN(TMR4_BASE) &= ~0x100u;                      /* no DMA request of its own */
        TMR_DIV(TMR4_BASE) = 0;
        TMR_PR(TMR4_BASE) = 4;                               /* 48 MHz */
        TMR_C3DT(TMR4_BASE) = 3;
        TMR_STCTRL(TMR4_BASE) |= 0x80u;                      /* master/slave sync, as stock */
        TMR_SWEVT(TMR4_BASE) = 1;
        TMR_CTRL1(TMR4_BASE) |= 1u;
        ilv_mode = 1;
        delay_ms(2);                                         /* > 64 conversions: pipeline flushed */
    }
}

void adc_ilv_start(uint8_t *raw, uint16_t per, int streams) {
    if (!ilv_mode) adc_ilv_enter();
    ilv_slaves_off();
    ilv_n = (uint8_t)(streams == 2 ? 2 : 4);
    uint32_t armed = (uint32_t)per + ADC_ILV_PAD;
    for (int j = 0; j < ilv_n; j++) {
        int k = ilv_n == 2 ? 2 * j : j;                      /* 24 MS/s: hardware streams 0 and 2 */
        uint32_t t = ilv_s[k].tmr, d = ilv_s[k].dma;
        uint8_t *base = raw + (uint32_t)j * armed;
        ilv_hw[j] = (uint8_t)k;
        TMR_CTRL1(t) = 0x80u;                                /* reload preload on, stopped */
        TMR_DIV(t) = 0;
        TMR_PR(t) = 7;                                       /* first period: 8 ticks */
        TMR_SWEVT(t) = 1;                                    /* load it */
        TMR_PR(t) = 3;                                       /* from the first overflow on: 4 ticks = 12 MHz */
        TMR_STCTRL(t) = (TMR_STCTRL(t) & ~0x77u) | 5u | ilv_s[k].ts;   /* gated by TMR4's TRGO */
        TMR_CVAL(t) = ilv_s[k].cv;
        D_CTRL(d) = 0;
        D_PADDR(d) = 0x40010C08;                             /* GPIOB->IDT */
        D_MADDR(d) = (uint32_t)base;
        D_CNT(d) = armed;
        D_CTRL(d) = 0x2000u | 0x80u | 1u;                    /* high priority, 8-bit, memory increment, enable */
        TMR_IDEN(t) |= 0x100u;                               /* overflow DMA request */
        TMR_CTRL1(t) |= 1u;                                  /* enabled, but frozen until TRGO runs */
    }
    for (volatile int w = 0; w < 200; w++) { }               /* let any stale request be served */
    for (int j = 0; j < ilv_n; j++) {
        uint32_t skip = armed - D_CNT(ilv_s[ilv_hw[j]].dma); /* 0 or 1 samples taken before the release */
        ilv_first[j] = raw + (uint32_t)j * armed + skip;
        if (skip > ADC_ILV_PAD) ilv_first[j] = raw + (uint32_t)j * armed + ADC_ILV_PAD;
    }
    ilv_busy = 1;
    TMR_CTRL2(TMR4_BASE) = (TMR_CTRL2(TMR4_BASE) & ~0x70u) | 0x20u;   /* release: every slave sees the same ticks */
}

int adc_ilv_done(void) {
    if (!ilv_busy) return 1;
    for (int j = 0; j < ilv_n; j++) if (D_CNT(ilv_s[ilv_hw[j]].dma) != 0u) return 0;
    ilv_slaves_off();
    return 1;
}

const uint8_t *adc_ilv_stream(int k) { return ilv_first[k]; }

void adc_set_rate(uint16_t psc, uint16_t arr) {
    adc_abort();
    if (ilv_mode) { ilv_slaves_off(); ilv_mode = 0; }
    TMR_CTRL1(TMR4_BASE) &= ~1u;
    TMR_DIV(TMR4_BASE) = psc;
    TMR_PR(TMR4_BASE) = arr;
    TMR_C3DT(TMR4_BASE) = (arr + 1u) / 2u;
    TMR_IDEN(TMR4_BASE) |= 0x100;
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
    if (ilv_busy) ilv_slaves_off();
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
