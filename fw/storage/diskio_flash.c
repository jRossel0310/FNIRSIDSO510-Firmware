/* FatFs media access for volume 0 = the user volume (4096-byte sectors). */
#include "ff.h"
#include "diskio.h"
#include "vol.h"
#include "flashmap.h"

DSTATUS disk_initialize(BYTE pdrv) { return pdrv ? STA_NOINIT : 0; }
DSTATUS disk_status(BYTE pdrv) { return pdrv ? STA_NOINIT : 0; }

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv) return RES_PARERR;
    for (UINT i = 0; i < count; i++)
        if (vol_read((uint32_t)sector + i, buff + i * FLASH_SECTOR)) return RES_ERROR;
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv) return RES_PARERR;
    for (UINT i = 0; i < count; i++)
        if (vol_write((uint32_t)sector + i, buff + i * FLASH_SECTOR)) return RES_ERROR;
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv) return RES_PARERR;
    switch (cmd) {
    case CTRL_SYNC: return RES_OK;
    case GET_SECTOR_COUNT: *(LBA_t *)buff = USER_VOL_SECTORS; return RES_OK;
    case GET_BLOCK_SIZE: *(DWORD *)buff = 1; return RES_OK;
    default: return RES_PARERR;
    }
}
