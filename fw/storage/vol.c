#include "vol.h"
#include "flashmap.h"
#include "spiflash.h"

int vol_read(uint32_t lba, uint8_t *buf) {
    if (lba >= USER_VOL_SECTORS) return -2;
    return spiflash_read(USER_VOL_BASE + lba * FLASH_SECTOR, buf, FLASH_SECTOR);
}

int vol_write(uint32_t lba, const uint8_t *buf) {
    if (lba >= USER_VOL_SECTORS) return -2;
    return spiflash_write_sector(USER_VOL_BASE + lba * FLASH_SECTOR, buf);
}
