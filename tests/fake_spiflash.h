#pragma once
#include <stdint.h>
#include "spiflash.h"
/* Host test model of the 8 MiB NOR: erase sets 0xFF, program ANDs bits (NOR semantics),
   the same flashmap guard as the driver; refused calls are counted. */
uint8_t *fake_flash_mem(void);
void fake_flash_reset(uint8_t fill);
int fake_flash_refused(void);
void fake_flash_fail_writes(int on);       /* 1: every erase returns -1 (I/O error injection) */
