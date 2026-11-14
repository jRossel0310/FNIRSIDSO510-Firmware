#include "flashmap.h"

int flashmap_writable(uint32_t addr, uint32_t len) {
    if (len == 0) return 0;
    if (addr >= USER_VOL_BYTES) return 0;
    if (len > USER_VOL_BYTES - addr) return 0;    /* also rejects wrap-around */
    return 1;
}
