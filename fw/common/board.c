#include "board.h"

const key_def_t board_keys[NUM_KEYS] = {
    {GPIOD, 2, "PD2"}, {GPIOC, 9, "PC9"}, {GPIOC, 7, "PC7"}, {GPIOC, 8, "PC8"}, {GPIOC, 6, "PC6"},
    {GPIOD, 13, "PD13"}, {GPIOB, 15, "PB15"}, {GPIOB, 14, "PB14"}, {GPIOB, 13, "PB13"},
};

void gpio_mode(uint32_t port, unsigned pin, uint32_t cfg) {
    volatile uint32_t *r = pin < 8 ? &GPIO_CFGLR(port) : &GPIO_CFGHR(port);
    unsigned sh = (pin & 7) * 4;
    *r = (*r & ~(0xFu << sh)) | (cfg << sh);
}

uint32_t SystemCoreClock = 8000000;
static volatile uint32_t g_ms;

void SysTick_Handler(void) { g_ms++; }
uint32_t millis(void) { return g_ms; }

void delay_ms(uint32_t ms) {
    uint32_t t0 = g_ms;
    while (g_ms - t0 < ms) WDT_CMD = 0xAAAA;   // reload watchdog in case the bootloader started it
}

static void clock_init_240mhz(void) {
    // Back to HICK first (mirrors stock SystemInit), then PLL = HEXT 8 MHz / 2 * 60 = 240 MHz.
    CRM_CTRL |= 1;
    while (!(CRM_CTRL & 2)) { }
    CRM_CFG &= ~3u;
    while (CRM_CFG & 0xC) { }
    CRM_CTRL &= 0xFEF2FFFFu;
    CRM_CFG = 0;
    CRM_MISC1 = 0;
    CRM_CLKINT = 0x009F0000;

    CRM_CTRL |= 1u << 16;                                 // HEXTEN
    while (!(CRM_CTRL & (1u << 17))) { }                  // HEXTSTBL
    CRM_CFG |= 1u << 16;                                  // PLLRCS = HEXT
    CRM_CFG |= 1u << 17;                                  // HEXT / 2
    CRM_CFG = (CRM_CFG & ~(0xFu << 18)) | (0xBu << 18);   // PLLMULT[3:0]
    CRM_CFG |= 3u << 29;                                  // PLLMULT[5:4] -> x60
    CRM_CFG |= 1u << 31;                                  // PLL range > 72 MHz
    CRM_MISC3 &= ~0x3000u;
    CRM_CTRL |= 1u << 24;                                 // PLLEN
    while (!(CRM_CTRL & (1u << 25))) { }                  // PLLSTBL
    CRM_CFG &= ~0xF0u;                                    // AHB / 1
    CRM_CFG = (CRM_CFG & ~(7u << 11)) | (4u << 11);       // APB2 / 2 = 120 MHz
    CRM_CFG = (CRM_CFG & ~(7u << 8)) | (4u << 8);         // APB1 / 2 = 120 MHz
    CRM_MISC3 |= 0x30;                                    // auto step (as stock)
    CRM_CFG = (CRM_CFG & ~3u) | 2;                        // SCLK = PLL
    while ((CRM_CFG & 0xC) != 8) { }
    CRM_MISC3 &= ~0x30u;
    SystemCoreClock = 240000000;
}

void board_init(void) {
    __asm volatile ("cpsid i");
    SYST_CSR = 0;
    for (int i = 0; i < 8; i++) { NVIC_ICER(i) = 0xFFFFFFFFu; NVIC_ICPR(i) = 0xFFFFFFFFu; }
    REG32(0xE000ED04) = (1u << 27) | (1u << 25);          // ICSR: clear pending PendSV and SysTick
    SCB_VTOR = 0x08007000;
    clock_init_240mhz();
    SYST_RVR = SystemCoreClock / 1000 - 1;
    SYST_CVR = 0;
    SYST_CSR = 7;                                         // HCLK, interrupt, enable
    __asm volatile ("cpsie i");
    // IOMUX, GPIOA..E, ADC1; XMC.
    CRM_APB2EN |= (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5) | (1u << 6) | (1u << 9);
    CRM_AHBEN |= (1u << 8);
    GPIO_SCR(PWR_LATCH_PORT) = 1u << PWR_LATCH_PIN;
    gpio_mode(PWR_LATCH_PORT, PWR_LATCH_PIN, CFG_OUT_PP);
}

void keys_init(void) {
    for (int i = 0; i < NUM_KEYS; i++) {
        GPIO_SCR(board_keys[i].port) = 1u << board_keys[i].pin;   // ODT=1 -> pull-up
        gpio_mode(board_keys[i].port, board_keys[i].pin, CFG_IN_PULL);
    }
}

uint32_t keys_read(void) {
    uint32_t m = 0;
    for (int i = 0; i < NUM_KEYS; i++)
        if (!(GPIO_IDT(board_keys[i].port) & (1u << board_keys[i].pin))) m |= 1u << i;
    return m;
}

void backlight(int on) {
    if (on) GPIO_SCR(BL_PORT) = 1u << BL_PIN; else GPIO_CLR(BL_PORT) = 1u << BL_PIN;
    gpio_mode(BL_PORT, BL_PIN, CFG_OUT_PP);
}

void power_off(void) {
    GPIO_CLR(PWR_LATCH_PORT) = 1u << PWR_LATCH_PIN;
}

void batt_adc_init(void) {
    CRM_CFG = (CRM_CFG & ~((3u << 14) | (1u << 28))) | (2u << 14);   // ADCDIV = /6
    gpio_mode(GPIOC, 3, CFG_ANALOG);
    ADC1_SPT1 |= 7u << ((BATT_ADC_CH - 10) * 3);   // longest sample time (as stock)
    ADC1_OSQ3 = BATT_ADC_CH;                        // one conversion, channel 13
    ADC1_CTRL2 = (7u << 17) | (1u << 20) | 1u;      // software trigger, ADC on
    delay_ms(2);
    ADC1_CTRL2 |= 1u << 3;                          // reset calibration
    while (ADC1_CTRL2 & (1u << 3)) { }
    ADC1_CTRL2 |= 1u << 2;                          // calibrate
    while (ADC1_CTRL2 & (1u << 2)) { }
}

uint16_t batt_adc_read(void) {
    ADC1_STS = 0;
    ADC1_CTRL2 |= 1u << 22;                         // software start (as stock)
    for (uint32_t t = 0; !(ADC1_STS & 2u); t++) if (t > 100000) return 0xFFFF;
    return (uint16_t)ADC1_ODT;
}
