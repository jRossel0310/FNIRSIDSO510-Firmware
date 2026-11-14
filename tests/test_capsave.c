#include "test.h"
#include "fake_spiflash.h"
#include "flashmap.h"
#include "capsave.h"
#include "ff.h"
#include <stdlib.h>

static BYTE work[FF_MAX_SS];
static uint8_t rec[200000];

static void format(void) {
    fake_flash_reset(0xFF);
    memset(fake_flash_mem() + SYS_VOL_BASE, 0xA5, 0x300000);   /* system volume sentinel */
    MKFS_PARM opt = {FM_FAT, 2, 0, 512, 4096};
    FATFS fs;
    CHECK_EQ(f_mkfs("", &opt, work, sizeof work), FR_OK);
    CHECK_EQ(f_mount(&fs, "", 1), FR_OK);
    UINT bw; FIL f;
    CHECK_EQ(f_mkdir("/LOGO"), FR_OK);                          /* like the stock drive */
    CHECK_EQ(f_open(&f, "/KEEP.TXT", FA_WRITE | FA_CREATE_NEW), FR_OK);
    CHECK_EQ(f_write(&f, "stock", 5, &bw), FR_OK);
    f_close(&f);
    f_mount(0, "", 0);
}

static char *read_file(const char *path, UINT *len) {
    static char buf[300000];
    FATFS fs; FIL f;
    *len = 0;
    if (f_mount(&fs, "", 1) != FR_OK) return 0;
    if (f_open(&f, path, FA_READ) != FR_OK) { f_mount(0, "", 0); return 0; }
    f_read(&f, buf, sizeof buf - 1, len);
    buf[*len] = 0;
    f_close(&f);
    f_mount(0, "", 0);
    return buf;
}

void test_capsave(void) {
    csv_meta_t m;
    memset(&m, 0, sizeof m);
    m.rate_hz = 1000000; m.vdiv_mv = 1000; m.coupling_dc = 1; m.zero_code = 128;
    m.trig_level_code = 153; m.trig_index = 2; m.offset_pwm = 2033;
    for (int i = 0; i < 4800; i++) rec[i] = (uint8_t)(i * 13);

    format();
    int num = 0;
    CHECK_EQ(capsave_write(rec, 4800, &m, &num), CAP_OK);
    CHECK_EQ(num, 1);
    CHECK_EQ(capsave_write(rec, 5, &m, &num), CAP_OK);
    CHECK_EQ(num, 2);

    /* expected text of the 5-sample file */
    static char exp[2000];
    int k = csv_header(exp, sizeof exp, &m);
    for (int i = 0; i < 5; i++) k += csv_row(exp + k, &m, i, rec[i]);
    exp[k] = 0;
    UINT len;
    char *got = read_file("/CSV/CAP0002.CSV", &len);
    CHECK(got != 0);
    if (got) CHECK_STR(got, exp);
    got = read_file("/CSV/CAP0001.CSV", &len);
    CHECK(got != 0 && len > 4800 * 20);                         /* ~25 bytes per row */
    static char big[200000];                                    /* 4096-byte chunks with carry-over: exact text */
    k = csv_header(big, sizeof big, &m);
    for (int i = 0; i < 4800; i++) k += csv_row(big + k, &m, i, rec[i]);
    big[k] = 0;
    if (got) { CHECK_EQ(len, k); CHECK(strcmp(got, big) == 0); }
    got = read_file("/KEEP.TXT", &len);
    CHECK(got != 0 && len == 5);                                /* other files untouched */

    /* numbering continues after the highest existing number */
    FATFS fs; FIL f;
    f_mount(&fs, "", 1);
    f_open(&f, "/CSV/CAP0041.CSV", FA_WRITE | FA_CREATE_NEW); f_close(&f);
    f_mount(0, "", 0);
    CHECK_EQ(capsave_write(rec, 5, &m, &num), CAP_OK);
    CHECK_EQ(num, 42);

    /* drive full: partial file is removed, CAP_FULL returned */
    for (int i = 0; i < (int)sizeof rec; i++) rec[i] = (uint8_t)i;
    int full = 0;
    for (int t = 0; t < 10 && !full; t++) full = capsave_write(rec, sizeof rec, &m, &num) == CAP_FULL;
    CHECK(full);
    f_mount(&fs, "", 1);
    char path[24] = "/CSV/";
    capname_make(path + 5, num);
    FILINFO fi;
    CHECK(f_stat(path, &fi) == FR_NO_FILE);                     /* partial file deleted */
    f_mount(0, "", 0);

    CHECK_EQ(fake_flash_refused(), 0);                          /* no write ever hit the guard */

    /* flash wear of one 4800-sample save: each FAT sector <= 3 erases, <= 45 erases in total */
    format();
    for (int i = 0; i < 4800; i++) rec[i] = (uint8_t)(i * 13);
    fake_flash_erase_reset();
    CHECK_EQ(capsave_write(rec, 4800, &m, &num), CAP_OK);
    f_mount(&fs, "", 1);
    uint32_t fat_max = 0, all_max = 0;
    for (uint32_t s = 0; s < (uint32_t)fs.fsize * fs.n_fats; s++) {
        uint32_t e = fake_flash_erases(USER_VOL_BASE + (uint32_t)(fs.fatbase + s) * FLASH_SECTOR);
        if (e > fat_max) fat_max = e;
    }
    uint32_t all_at = 0;
    for (uint32_t a = USER_VOL_BASE; a < USER_VOL_BASE + USER_VOL_BYTES; a += FLASH_SECTOR)
        if (fake_flash_erases(a) > all_max) { all_max = fake_flash_erases(a); all_at = a / FLASH_SECTOR; }
    f_mount(0, "", 0);
    printf("capsave wear: %u erases total, FAT sector max %u, any sector max %u (lba %u; fat %u+%u*%u, dirbase %u, database %u)\n",
           (unsigned)fake_flash_erase_total(), (unsigned)fat_max, (unsigned)all_max, (unsigned)all_at,
           (unsigned)fs.fatbase, (unsigned)fs.n_fats, (unsigned)fs.fsize, (unsigned)fs.dirbase, (unsigned)fs.database);
    CHECK(fat_max >= 1 && fat_max <= 3);
    CHECK(all_max <= 4);                                        /* the new /CSV directory sector: mkdir, create, close */
    CHECK(fake_flash_erase_total() <= 45);

    /* I/O error */
    format();
    fake_flash_fail_writes(1);
    CHECK_EQ(capsave_write(rec, 5, &m, &num), CAP_ERR);
    fake_flash_fail_writes(0);

    /* the system volume was never written */
    const uint8_t *sys = fake_flash_mem() + SYS_VOL_BASE;
    int intact = 1;
    for (uint32_t i = 0; i < 0x300000; i++) if (sys[i] != 0xA5) { intact = 0; break; }
    CHECK(intact);
    CHECK_EQ(fake_flash_refused(), 0);
}
