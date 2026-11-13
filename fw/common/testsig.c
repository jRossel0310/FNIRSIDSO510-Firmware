#include "testsig.h"
#include "board.h"
#include "sine100.h"

#define TMR6_BASE 0x40001000u
#define TMR9_BASE 0x40014C00u
#define TMR_CTRL1(t) REG32((t) + 0x00)
#define TMR_CTRL2(t) REG32((t) + 0x04)
#define TMR_SWEVT(t) REG32((t) + 0x14)
#define TMR_CM1(t)   REG32((t) + 0x18)
#define TMR_CCTRL(t) REG32((t) + 0x20)
#define TMR_DIV(t)   REG32((t) + 0x28)
#define TMR_PR(t)    REG32((t) + 0x2C)
#define TMR_C2DT(t)  REG32((t) + 0x38)
#define TMR_BRK(t)   REG32((t) + 0x44)
#define DAC_CTRL     REG32(0x40007400)
#define DAC_SWTRG    REG32(0x40007404)
#define DAC_D1DTH12R REG32(0x40007408)
#define DAC_D2DTH12R REG32(0x40007414)
#define DMA2_C3CTRL  REG32(0x40020430)
#define DMA2_C3DTCNT REG32(0x40020434)
#define DMA2_C3PADDR REG32(0x40020438)
#define DMA2_C3MADDR REG32(0x4002043C)
#define DMA2_SRC_SEL0 REG32(0x400204A0)
#define DMA2_SRC_SEL1 REG32(0x400204A4)
#define DMA_REQ_DAC1 0x05

void cal_out_init(void) {
    CRM_APB2EN |= 1u << 19;                 // TMR9
    gpio_mode(GPIOA, 3, CFG_AF_PP);
    TMR_DIV(TMR9_BASE) = 2399;              // 240 MHz / 2400 = 100 kHz
    TMR_PR(TMR9_BASE) = 99;                 // 1 kHz
    TMR_CM1(TMR9_BASE) = (TMR_CM1(TMR9_BASE) & ~0x7000u) | 0x7000;
    TMR_CCTRL(TMR9_BASE) |= 0x30;
    TMR_C2DT(TMR9_BASE) = 50;
    TMR_BRK(TMR9_BASE) |= 0x8000;
    TMR_SWEVT(TMR9_BASE) = 1;
    TMR_CTRL1(TMR9_BASE) |= 1;
}

void dac2_bias_init(void) {
    CRM_APB1EN |= 1u << 29;
    gpio_mode(GPIOA, 5, CFG_ANALOG);
    DAC_CTRL |= 0x380000;
    DAC_CTRL |= 0x40000;
    DAC_CTRL &= 0xFF3FFFFFu;
    DAC_CTRL |= 0x20000;
    DAC_CTRL &= 0xEFFFFFFFu;
    DAC_CTRL |= 0x10000;
    DAC_D2DTH12R = 0xF46;
    DAC_SWTRG |= 2;
}

void gen_sine_init(uint16_t arr) {
    CRM_APB1EN |= (1u << 29) | (1u << 4);   // DAC, TMR6
    CRM_AHBEN |= 1u << 1;                   // DMA2
    gpio_mode(GPIOA, 4, CFG_ANALOG);
    TMR_CTRL1(TMR6_BASE) = 0;
    TMR_DIV(TMR6_BASE) = 0;
    TMR_PR(TMR6_BASE) = arr;
    TMR_CTRL2(TMR6_BASE) = (TMR_CTRL2(TMR6_BASE) & ~0x70u) | 0x20;
    DAC_CTRL &= ~0x38u;
    DAC_CTRL |= 4;
    DAC_CTRL &= ~0xC0u;
    DAC_CTRL |= 2;
    DAC_CTRL |= 0x1000;
    DMA2_C3CTRL = 0;
    DMA2_C3PADDR = (uint32_t)&DAC_D1DTH12R;
    DMA2_C3MADDR = (uint32_t)sine100;
    DMA2_C3DTCNT = 100;
    DMA2_SRC_SEL1 |= 1u << 24;
    DMA2_SRC_SEL0 = (DMA2_SRC_SEL0 & 0xFF00FFFFu) | ((uint32_t)DMA_REQ_DAC1 << 16);
    DMA2_C3CTRL = 0x10 | 0x20 | 0x80 | 0x200 | 0x400;
    DMA2_C3CTRL |= 1;
    DAC_CTRL |= 1;
    TMR_SWEVT(TMR6_BASE) = 1;
    TMR_CTRL1(TMR6_BASE) |= 1;
}

void gen_set_arr(uint16_t arr) {
    TMR_PR(TMR6_BASE) = arr;
    TMR_SWEVT(TMR6_BASE) = 1;
}
