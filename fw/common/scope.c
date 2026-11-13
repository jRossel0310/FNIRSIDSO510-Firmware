#include "scope.h"
#include "board.h"

// ---- Timers ----
#define TMR4_BASE  0x40000800u
#define TMR14_BASE 0x40002000u
#define TMR_CTRL1(t) REG32((t) + 0x00)
#define TMR_CTRL2(t) REG32((t) + 0x04)
#define TMR_IDEN(t)  REG32((t) + 0x0C)
#define TMR_SWEVT(t) REG32((t) + 0x14)
#define TMR_CM1(t)   REG32((t) + 0x18)
#define TMR_CM2(t)   REG32((t) + 0x1C)
#define TMR_CCTRL(t) REG32((t) + 0x20)
#define TMR_CVAL(t)  REG32((t) + 0x24)
#define TMR_DIV(t)   REG32((t) + 0x28)
#define TMR_PR(t)    REG32((t) + 0x2C)
#define TMR_C1DT(t)  REG32((t) + 0x34)
#define TMR_C3DT(t)  REG32((t) + 0x3C)
#define TMR_BRK(t)   REG32((t) + 0x44)

// ---- DMA1 channel 7 + Artery flexible request mapping ----
#define DMA1_STS     REG32(0x40020000)
#define DMA1_CLR     REG32(0x40020004)
#define DMA1_C7CTRL  REG32(0x40020080)
#define DMA1_C7DTCNT REG32(0x40020084)
#define DMA1_C7PADDR REG32(0x40020088)
#define DMA1_C7MADDR REG32(0x4002008C)
#define DMA1_SRC_SEL1 REG32(0x400200A4)
#define DMA_REQ_TMR4_OVF 0x4F        // value stock firmware puts in the ch7 field

// ---- Front end ----
// range -> {PC4, PE6, PE5, PE4}; decoded from stock FUN_08008834.
static const uint8_t range_pins[FE_NUM_RANGES][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {1, 1, 1, 0}, {1, 0, 0, 1}, {1, 1, 0, 1},
    {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 1, 0, 1},
};

static void pin(uint32_t port, unsigned p, int v) {
    if (v) GPIO_SCR(port) = 1u << p; else GPIO_CLR(port) = 1u << p;
}

void fe_set_range(int r) {
    const uint8_t *t = range_pins[r];
    pin(GPIOC, 4, t[0]); pin(GPIOE, 6, t[1]); pin(GPIOE, 5, t[2]); pin(GPIOE, 4, t[3]);
}

void fe_set_coupling(int pc5) { pin(GPIOC, 5, pc5); }

void fe_set_offset(uint16_t pwm) { TMR_C1DT(TMR14_BASE) = pwm; }

void fe_init(void) {
    gpio_mode(GPIOC, 4, CFG_OUT_PP);
    gpio_mode(GPIOC, 5, CFG_OUT_PP);
    gpio_mode(GPIOE, 4, CFG_OUT_PP);
    gpio_mode(GPIOE, 5, CFG_OUT_PP);
    gpio_mode(GPIOE, 6, CFG_OUT_PP);

    // PA7 = TMR14 CH1 PWM (stock FUN_08019F38: ARR 0xFFE, PWM mode 2 + inverted polarity).
    CRM_APB1EN |= 1u << 8;
    gpio_mode(GPIOA, 7, CFG_AF_PP);
    TMR_PR(TMR14_BASE) = FE_OFFSET_MAX;
    TMR_DIV(TMR14_BASE) = 0;
    TMR_CM1(TMR14_BASE) = (TMR_CM1(TMR14_BASE) & ~0x70u) | 0x70;
    TMR_CCTRL(TMR14_BASE) |= 3;
    TMR_C1DT(TMR14_BASE) = FE_OFFSET_DEFAULT;
    TMR_BRK(TMR14_BASE) |= 0x8000;
    TMR_SWEVT(TMR14_BASE) = 1;
    TMR_CTRL1(TMR14_BASE) |= 1;
}

// ---- ADC ----
// Timer clock = 8 MHz while we run on HICK.
const samplerate_t sample_rates[] = {
    {0, 7, "1MS/s"},  {0, 15, "500kS/s"}, {0, 39, "200kS/s"}, {0, 79, "100kS/s"},
    {0, 159, "50kS/s"}, {7, 49, "20kS/s"}, {7, 99, "10kS/s"},  {7, 399, "2.5kS/s"},
};
const int num_sample_rates = sizeof sample_rates / sizeof sample_rates[0];

void adc_init(void) {
    CRM_AHBEN |= 1u << 0;            // DMA1
    CRM_APB1EN |= 1u << 2;           // TMR4
    for (int p = 0; p < 8; p++) gpio_mode(GPIOB, p, CFG_IN_FLOAT);   // ADC data
    // Stock: PB8 output, 1 us high pulse, low, then alternate function (TMR4 CH3).
    gpio_mode(GPIOB, 8, CFG_OUT_PP);
    GPIO_SCR(GPIOB) = 1u << 8;
    for (volatile int i = 0; i < 10; i++) { }
    GPIO_CLR(GPIOB) = 1u << 8;
    gpio_mode(GPIOB, 8, CFG_AF_PP);

    TMR_CTRL1(TMR4_BASE) = 0;
    TMR_CM2(TMR4_BASE) = (TMR_CM2(TMR4_BASE) & ~0x70u) | 0x60;   // CH3 PWM mode 1
    TMR_CCTRL(TMR4_BASE) |= 0x300;                                // CH3 enable, active low
    TMR_IDEN(TMR4_BASE) |= 0x100;                                 // overflow DMA request
    TMR_BRK(TMR4_BASE) |= 0x8000;
}

int adc_capture(uint8_t *buf, uint16_t n, const samplerate_t *sr) {
    TMR_CTRL1(TMR4_BASE) &= ~1u;
    TMR_DIV(TMR4_BASE) = sr->psc;
    TMR_PR(TMR4_BASE) = sr->arr;
    TMR_C3DT(TMR4_BASE) = sr->arr >> 1;
    TMR_CVAL(TMR4_BASE) = 0;

    DMA1_C7CTRL = 0;
    DMA1_CLR = 0xFu << 24;                        // clear ch7 flags
    DMA1_C7PADDR = 0x40010C08;                    // GPIOB->IDT, low byte = PB0..PB7
    DMA1_C7MADDR = (uint32_t)buf;
    DMA1_C7DTCNT = n;
    DMA1_SRC_SEL1 |= 1u << 24;                    // flexible mapping on
    DMA1_SRC_SEL1 = (DMA1_SRC_SEL1 & 0xFF00FFFFu) | ((uint32_t)DMA_REQ_TMR4_OVF << 16);
    DMA1_C7CTRL = 0x3000 | 0x80 | 1;              // very high prio, 8-bit, mem increment, enable

    TMR_SWEVT(TMR4_BASE) = 1;                     // load PSC/ARR
    DMA1_CLR = 0xFu << 24;
    TMR_CTRL1(TMR4_BASE) |= 1;

    uint32_t budget = 4000000;
    while (!(DMA1_STS & (1u << 25))) {            // ch7 transfer complete
        if (--budget == 0) { TMR_CTRL1(TMR4_BASE) &= ~1u; DMA1_C7CTRL = 0; return -1; }
    }
    TMR_CTRL1(TMR4_BASE) &= ~1u;
    DMA1_C7CTRL = 0;
    return 0;
}
