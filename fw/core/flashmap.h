#pragma once
#include <stdint.h>
/* External SPI NOR layout (NOTES.md § SPI NOR flash). */
#define FLASH_SECTOR     4096u
#define USER_VOL_BASE    0x000000u
#define USER_VOL_SECTORS 1280u
#define USER_VOL_BYTES   (USER_VOL_SECTORS * FLASH_SECTOR)   /* 0x500000 */
#define SYS_VOL_BASE     0x500000u                           /* stock system volume: never written */

/* 1 if [addr, addr + len) is non-empty and lies entirely inside the user volume, else 0. */
int flashmap_writable(uint32_t addr, uint32_t len);
