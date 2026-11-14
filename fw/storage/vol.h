#pragma once
#include <stdint.h>
/* The user volume as 4096-byte blocks 0..USER_VOL_SECTORS-1 at flash offset USER_VOL_BASE.
   Shared by FatFs (diskio_flash.c) and the USB drive. 0 ok, <0 error. */
int vol_read(uint32_t lba, uint8_t *buf);
int vol_write(uint32_t lba, const uint8_t *buf);
