#include "spiflash.h"
#include "board.h"

#define SPI3_CTRL1 REG32(0x40003C00)
#define SPI3_STS   REG32(0x40003C08)
#define SPI3_DT    REG32(0x40003C0C)
#define IOMUX_REMAP5 REG32(0x40010028)
#define CS_PIN 12

static int spi_err;

static void cs(int level) { if (level) GPIO_SCR(GPIOD) = 1u << CS_PIN; else GPIO_CLR(GPIOD) = 1u << CS_PIN; }

static uint8_t xfer(uint8_t b) {
    uint32_t n = 100000;
    while (!(SPI3_STS & 2u)) if (!--n) { spi_err = 1; return 0; }   /* TDBE */
    SPI3_DT = b;
    n = 100000;
    while (!(SPI3_STS & 1u)) if (!--n) { spi_err = 1; return 0; }   /* RDBF */
    return (uint8_t)SPI3_DT;
}

static void set_cfg(uint32_t port, int pin, uint32_t cfg) {
    volatile uint32_t *r = pin < 8 ? &GPIO_CFGLR(port) : &GPIO_CFGHR(port);
    int s = (pin & 7) * 4;
    *r = (*r & ~(0xFu << s)) | (cfg << s);
}

void spiflash_init(void) {
    CRM_APB2EN |= (1u << 0) | (1u << 4) | (1u << 5);   /* IOMUX, GPIOC, GPIOD (as stock) */
    CRM_APB1EN |= 1u << 15;                            /* SPI3 */
    IOMUX_REMAP5 = (IOMUX_REMAP5 & ~(0xFu << 24)) | (3u << 24);   /* SPI3 -> PC10/11/12 (stock value) */
    cs(1);
    set_cfg(GPIOD, CS_PIN, CFG_OUT_PP);
    set_cfg(GPIOC, 10, CFG_AF_PP);
    set_cfg(GPIOC, 11, CFG_IN_FLOAT);
    set_cfg(GPIOC, 12, CFG_AF_PP);
    /* master, mode 0, fPCLK1/8 = 15 MHz, software CS (SWCSEN + SWCSIL), 8-bit, MSB first */
    SPI3_CTRL1 = (1u << 2) | (2u << 3) | (1u << 8) | (1u << 9);
    SPI3_CTRL1 |= 1u << 6;
    cs(0); xfer(0xAB); cs(1);                          /* release from deep power-down */
    delay_ms(1);
}

uint32_t spiflash_jedec_id(void) {
    spi_err = 0;
    cs(0);
    xfer(0x9F);
    uint32_t id = (uint32_t)xfer(0) << 16;
    id |= (uint32_t)xfer(0) << 8;
    id |= xfer(0);
    cs(1);
    return spi_err ? 0xFFFFFFFFu : id;
}

int spiflash_read(uint32_t addr, uint8_t *buf, uint32_t n) {
    spi_err = 0;
    cs(0);
    xfer(0x03); xfer((uint8_t)(addr >> 16)); xfer((uint8_t)(addr >> 8)); xfer((uint8_t)addr);
    for (uint32_t i = 0; i < n && !spi_err; i++) buf[i] = xfer(0);
    cs(1);
    return spi_err ? -1 : 0;
}
