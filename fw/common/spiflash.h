#pragma once
#include <stdint.h>
/* External SPI NOR flash on SPI3 (remapped: SCK PC10, MISO PC11, MOSI PC12), CS = PD12.
   Read-only for now: only JEDEC ID (0x9F), release-power-down (0xAB) and READ (0x03). */
void spiflash_init(void);
uint32_t spiflash_jedec_id(void);              /* manufacturer << 16 | type << 8 | capacity */
/* Read n bytes from a 24-bit address. Returns 0, or -1 on SPI timeout. */
int spiflash_read(uint32_t addr, uint8_t *buf, uint32_t n);
