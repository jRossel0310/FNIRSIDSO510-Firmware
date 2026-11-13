// FNIRSI DSO-510 board definitions (AT32F403A/407). See NOTES.md for how each was derived.
#pragma once
#include <stdint.h>

#define REG32(a) (*(volatile uint32_t *)(a))
#define REG16(a) (*(volatile uint16_t *)(a))

// ---- CRM (clock & reset) ----
#define CRM_CTRL     REG32(0x40021000)
#define CRM_CFG      REG32(0x40021004)
#define CRM_CLKINT   REG32(0x40021008)
#define CRM_AHBEN    REG32(0x40021014)
#define CRM_APB2EN   REG32(0x40021018)
#define CRM_APB1EN   REG32(0x4002101C)
#define CRM_MISC1    REG32(0x40021030)

#define IOMUX_REMAP2 REG32(0x40010030)
#define WDT_CMD      REG32(0x40003000)
#define SCB_VTOR     REG32(0xE000ED08)

// ---- GPIO (STM32F1-style) ----
#define GPIOA 0x40010800u
#define GPIOB 0x40010C00u
#define GPIOC 0x40011000u
#define GPIOD 0x40011400u
#define GPIOE 0x40011800u
#define GPIO_CFGLR(p) REG32((p) + 0x00)
#define GPIO_CFGHR(p) REG32((p) + 0x04)
#define GPIO_IDT(p)   REG32((p) + 0x08)
#define GPIO_ODT(p)   REG32((p) + 0x0C)
#define GPIO_SCR(p)   REG32((p) + 0x10)
#define GPIO_CLR(p)   REG32((p) + 0x14)

#define CFG_ANALOG   0x0u
#define CFG_IN_FLOAT 0x4u
#define CFG_IN_PULL  0x8u   // pull direction from ODT bit (1 = up)
#define CFG_OUT_PP   0x3u
#define CFG_AF_PP    0xBu

// ---- ADC1 ----
#define ADC1_STS   REG32(0x40012400)
#define ADC1_CTRL2 REG32(0x40012408)
#define ADC1_SPT1  REG32(0x4001240C)
#define ADC1_OSQ3  REG32(0x40012434)
#define ADC1_ODT   REG32(0x4001244C)

// ---- Pin map ----
#define PWR_LATCH_PORT GPIOD   // PD3: power latch (confirmed: low = scope turns off)
#define PWR_LATCH_PIN  3
#define LCD_RST_PORT   GPIOD   // PD6
#define LCD_RST_PIN    6
#define BL_PORT        GPIOA   // PA2 (TMR9 CH1 PWM in stock fw)
#define BL_PIN         2
#define BATT_ADC_CH    13      // PC3, battery via 1/2 divider: V = raw * 3.3 / 4095 * 2

// Front-panel keys: active low, pull-up. Order = stock key-scan table @ 0x080349F8.
// short code = index + 1; long-press codes: K1=10 K2=11 K5=12 K6=13 K8=14.
// Physical mapping confirmed on hardware 2026-11-13.
#define NUM_KEYS 9
enum {
    KEY_POWER = 0,   // K1 PD2
    KEY_RUN,         // K2 PC9  Run/Stop
    KEY_UP,          // K3 PC7
    KEY_DOWN,        // K4 PC8
    KEY_MODE,        // K5 PC6
    KEY_AUTO,        // K6 PD13
    KEY_WHEEL_R,     // K7 PB15 thumb wheel pushed right
    KEY_WHEEL_PRESS, // K8 PB14 thumb wheel pressed in
    KEY_WHEEL_L,     // K9 PB13 thumb wheel pushed left
};
#define KEY_BIT(k) (1u << (k))
typedef struct { uint32_t port; uint8_t pin; char name[5]; } key_def_t;
extern const key_def_t board_keys[NUM_KEYS];

void board_init(void);           // clocks (8 MHz HICK), power latch, VTOR, interrupts off
void gpio_mode(uint32_t port, unsigned pin, uint32_t cfg);
void delay_ms(uint32_t ms);
void keys_init(void);
uint32_t keys_read(void);        // bit i set = key i pressed
void backlight(int on);
void power_off(void);            // releases the power latch
void batt_adc_init(void);
uint16_t batt_adc_read(void);    // raw 12-bit
