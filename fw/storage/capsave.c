#include "capsave.h"
#include "fsobj.h"

FATFS g_fs;
FIL g_fil;
#define fs g_fs
#define fil g_fil
static DIR dir;
static FILINFO fno;
#define CHUNK 4096                                      /* = sector size: whole-sector f_writes bypass the window */
static char buf[CHUNK + CSV_ROW_MAX];

static int highest_number(void) {
    int hi = 0;
    if (f_opendir(&dir, CAP_DIR) != FR_OK) return 0;
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        int k = capname_number(fno.fname);
        if (k > hi) hi = k;
    }
    f_closedir(&dir);
    return hi;
}

/* Write len bytes; CAP_FULL if the volume ran out of space. */
static cap_res_t put(const char *p, UINT len) {
    UINT bw;
    if (f_write(&fil, p, len, &bw) != FR_OK) return CAP_ERR;
    return bw == len ? CAP_OK : CAP_FULL;
}

cap_res_t capsave_write(const uint8_t *rec, int n, const csv_meta_t *m, int *num_out) {
    char path[5 + 13] = CAP_DIR "/";
    cap_res_t res = CAP_ERR;
    *num_out = 0;
    if (f_mount(&fs, "", 1) != FR_OK) return CAP_ERR;
    FRESULT fr = f_mkdir(CAP_DIR);
    if (fr != FR_OK && fr != FR_EXIST) goto out;
    int num = capname_next(highest_number());
    if (num < 0) { res = CAP_FULL; goto out; }
    *num_out = num;
    capname_make(path + sizeof CAP_DIR, num);           /* after "/CSV/" */
    if (f_open(&fil, path, FA_WRITE | FA_CREATE_NEW) != FR_OK) goto out;
    int k = csv_header(buf, CHUNK, m);
    res = k < 0 ? CAP_ERR : CAP_OK;
    for (int i = 0; i < n && res == CAP_OK; i++) {
        k += csv_row(buf + k, m, i, rec[i]);             /* k < CHUNK before, so k < CHUNK + CSV_ROW_MAX after */
        if (k >= CHUNK) {                                /* exactly CHUNK bytes out, carry the overflow over */
            res = put(buf, CHUNK);
            k -= CHUNK;
            for (int j = 0; j < k; j++) buf[j] = buf[CHUNK + j];
        }
    }
    if (res == CAP_OK && k) res = put(buf, (UINT)k);
    if (f_close(&fil) != FR_OK && res == CAP_OK) res = CAP_ERR;
    if (res != CAP_OK) f_unlink(path);                   /* never leave a partial file */
out:
    f_mount(0, "", 0);
    return res;
}
