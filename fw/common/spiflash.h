#pragma once
#include <stdint.h>
/* External SPI NOR flash (Puya 8 MiB, JEDEC 0x856017) on SPI3 (remapped: SCK PC10, MISO PC11,
   MOSI PC12), CS = PD12. Erase/program are refused outside the user volume (flashmap.h).
   Never calls delay_ms() after init (delay_ms runs the USB idle hook). */
#define SPIFLASH_PAGE 256u

void spiflash_init(void);
uint32_t spiflash_jedec_id(void);              /* manufacturer << 16 | type << 8 | capacity */
/* Read n bytes from a 24-bit address. 0 ok, -1 SPI timeout. */
int spiflash_read(uint32_t addr, uint8_t *buf, uint32_t n);
/* Erase the 4 KiB sector at addr (must be 4096-aligned). 0 ok, -1 timeout, -2 refused. */
int spiflash_erase_sector(uint32_t addr);
/* Program n (1..256) bytes inside one 256-byte page. 0 ok, -1 timeout, -2 refused. */
int spiflash_program(uint32_t addr, const uint8_t *buf, uint32_t n);
/* Erase + program + read-back verify one 4 KiB sector (addr 4096-aligned).
   0 ok, -1 timeout, -2 refused, -3 verify mismatch. */
int spiflash_write_sector(uint32_t addr, const uint8_t *buf);
