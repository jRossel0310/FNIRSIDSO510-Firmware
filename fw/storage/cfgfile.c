#include "cfgfile.h"
#include "fsobj.h"

#define fs g_fs
#define fil g_fil

int cfgfile_load(uint8_t *buf, uint32_t cap) {
    int res = -1;
    UINT br;
    if (f_mount(&fs, "", 1) != FR_OK) return -1;
    if (f_open(&fil, CFG_PATH, FA_READ) == FR_OK) {
        int ok = f_read(&fil, buf, cap, &br) == FR_OK;
        if (ok) res = (int)f_size(&fil);
        f_close(&fil);
    }
    f_mount(0, "", 0);
    return res;
}

int cfgfile_save(const uint8_t *buf, uint32_t len) {
    int res = -1;
    UINT bw = 0;
    if (f_mount(&fs, "", 1) != FR_OK) return -1;
    if (f_open(&fil, CFG_PATH, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
        int ok = f_write(&fil, buf, len, &bw) == FR_OK && bw == len;
        if (f_close(&fil) == FR_OK && ok) res = 0;
        else f_unlink(CFG_PATH);
    }
    f_mount(0, "", 0);
    return res;
}
