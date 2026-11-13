#include "frontend.h"
#include "board.h"

#define TMR14_BASE 0x40002000u
#define TMR_CTRL1(t) REG32((t) + 0x00)
#define TMR_SWEVT(t) REG32((t) + 0x14)
#define TMR_CM1(t)   REG32((t) + 0x18)
#define TMR_CCTRL(t) REG32((t) + 0x20)
#define TMR_DIV(t)   REG32((t) + 0x28)
#define TMR_PR(t)    REG32((t) + 0x2C)
#define TMR_C1DT(t)  REG32((t) + 0x34)
#define TMR_BRK(t)   REG32((t) + 0x44)

// range -> {PC4, PE6, PE5, PE4}
static const uint8_t range_pins[FE_NUM_RANGES][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {1, 1, 1, 0}, {1, 0, 0, 1}, {1, 1, 0, 1},
    {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 1, 0, 1},
};

static void pin(uint32_t port, unsigned p, int v) {
    if (v) GPIO_SCR(port) = 1u << p; else GPIO_CLR(port) = 1u << p;
}

void fe_init(void) {
    gpio_mode(GPIOC, 4, CFG_OUT_PP);
    gpio_mode(GPIOC, 5, CFG_OUT_PP);
    gpio_mode(GPIOE, 4, CFG_OUT_PP);
    gpio_mode(GPIOE, 5, CFG_OUT_PP);
    gpio_mode(GPIOE, 6, CFG_OUT_PP);
    CRM_APB1EN |= 1u << 8;                       // TMR14
    gpio_mode(GPIOA, 7, CFG_AF_PP);
    TMR_PR(TMR14_BASE) = FE_PWM_MAX;
    TMR_DIV(TMR14_BASE) = 0;
    TMR_CM1(TMR14_BASE) = (TMR_CM1(TMR14_BASE) & ~0x70u) | 0x70;   // PWM mode 2 (stock)
    TMR_CCTRL(TMR14_BASE) |= 3;                                     // enable, inverted (stock)
    TMR_C1DT(TMR14_BASE) = 0x834;
    TMR_BRK(TMR14_BASE) |= 0x8000;
    TMR_SWEVT(TMR14_BASE) = 1;
    TMR_CTRL1(TMR14_BASE) |= 1;
}

void fe_set_range(int r) {
    const uint8_t *t = range_pins[r];
    pin(GPIOC, 4, t[0]); pin(GPIOE, 6, t[1]); pin(GPIOE, 5, t[2]); pin(GPIOE, 4, t[3]);
}

void fe_set_coupling_dc(int dc) { pin(GPIOC, 5, dc ? FE_PC5_FOR_DC : !FE_PC5_FOR_DC); }

void fe_set_offset_pwm(uint16_t pwm) { TMR_C1DT(TMR14_BASE) = pwm > FE_PWM_MAX ? FE_PWM_MAX : pwm; }
