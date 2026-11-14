#include "fake_spiflash.h"
#include "flashmap.h"
#include <string.h>

#define FAKE_SIZE (8u * 1024 * 1024)
static uint8_t mem[FAKE_SIZE];
static int refused, fail;
static uint32_t erases[FAKE_SIZE / FLASH_SECTOR], erase_total;

uint8_t *fake_flash_mem(void) { return mem; }
void fake_flash_reset(uint8_t fill) { memset(mem, fill, sizeof mem); refused = 0; fail = 0; }
int fake_flash_refused(void) { return refused; }
void fake_flash_fail_writes(int on) { fail = on; }
void fake_flash_erase_reset(void) { memset(erases, 0, sizeof erases); erase_total = 0; }
uint32_t fake_flash_erases(uint32_t addr) { return addr < FAKE_SIZE ? erases[addr / FLASH_SECTOR] : 0; }
uint32_t fake_flash_erase_total(void) { return erase_total; }

void spiflash_init(void) { }
uint32_t spiflash_jedec_id(void) { return 0x856017u; }

int spiflash_read(uint32_t addr, uint8_t *buf, uint32_t n) {
    if (addr >= FAKE_SIZE || n > FAKE_SIZE - addr) return -1;
    memcpy(buf, mem + addr, n);
    return 0;
}

int spiflash_erase_sector(uint32_t addr) {
    if ((addr & (FLASH_SECTOR - 1)) || !flashmap_writable(addr, FLASH_SECTOR)) { refused++; return -2; }
    if (fail) return -1;
    erases[addr / FLASH_SECTOR]++; erase_total++;
    memset(mem + addr, 0xFF, FLASH_SECTOR);
    return 0;
}

int spiflash_program(uint32_t addr, const uint8_t *buf, uint32_t n) {
    if (n == 0 || n > SPIFLASH_PAGE || (addr & (SPIFLASH_PAGE - 1)) + n > SPIFLASH_PAGE ||
        !flashmap_writable(addr, n)) { refused++; return -2; }
    for (uint32_t i = 0; i < n; i++) mem[addr + i] &= buf[i];
    return 0;
}

int spiflash_write_sector(uint32_t addr, const uint8_t *buf) {
    int r = spiflash_erase_sector(addr);
    if (r) return r;
    for (uint32_t o = 0; o < FLASH_SECTOR; o += SPIFLASH_PAGE)
        if ((r = spiflash_program(addr + o, buf + o, SPIFLASH_PAGE))) return r;
    return memcmp(mem + addr, buf, FLASH_SECTOR) ? -3 : 0;
}
