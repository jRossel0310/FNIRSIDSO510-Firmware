#include "test.h"
#include "fake_spiflash.h"
#include "flashmap.h"
#include "cfgfile.h"
#include "capsave.h"
#include "ff.h"

static BYTE work[FF_MAX_SS];

static void format(void) {
    fake_flash_reset(0xFF);
    MKFS_PARM opt = {FM_FAT, 2, 0, 512, 4096};
    CHECK_EQ(f_mkfs("", &opt, work, sizeof work), FR_OK);
}

void test_cfgfile(void) {
    uint8_t a[156], b[156], got[200];
    for (int i = 0; i < 156; i++) { a[i] = (uint8_t)(i * 7 + 1); b[i] = (uint8_t)(255 - i); }

    format();
    CHECK_EQ(cfgfile_load(got, sizeof got), -1);                /* fresh volume: no file */
    CHECK_EQ(cfgfile_save(a, 156), 0);
    memset(got, 0, sizeof got);
    CHECK_EQ(cfgfile_load(got, sizeof got), 156);
    CHECK(memcmp(got, a, 156) == 0);

    CHECK_EQ(cfgfile_save(b, 156), 0);                          /* overwrite, not append */
    CHECK_EQ(cfgfile_load(got, sizeof got), 156);
    CHECK(memcmp(got, b, 156) == 0);

    memset(got, 0xEE, sizeof got);                              /* small buffer: size reported, cap respected */
    CHECK_EQ(cfgfile_load(got, 10), 156);
    CHECK(memcmp(got, b, 10) == 0);
    CHECK_EQ(got[10], 0xEE);

    /* in-place rewrite: far fewer erases than the creation */
    format();
    fake_flash_erase_reset();
    CHECK_EQ(cfgfile_save(a, 156), 0);
    uint32_t e_create = fake_flash_erase_total();
    fake_flash_erase_reset();
    CHECK_EQ(cfgfile_save(b, 156), 0);
    uint32_t e_over = fake_flash_erase_total();
    printf("cfgfile erases: create %u, overwrite %u\n", (unsigned)e_create, (unsigned)e_over);
    CHECK(e_over <= 3);
    CHECK(e_over < e_create);
    CHECK_EQ(cfgfile_load(got, sizeof got), 156);
    CHECK(memcmp(got, b, 156) == 0);

    /* shorter blob over a longer file: file is cut; real size is reported */
    CHECK_EQ(cfgfile_save(a, 100), 0);
    memset(got, 0, sizeof got);
    CHECK_EQ(cfgfile_load(got, sizeof got), 100);
    CHECK(memcmp(got, a, 100) == 0);
    CHECK_EQ(cfgfile_save(b, 156), 0);                          /* and growing again */
    CHECK_EQ(cfgfile_load(got, sizeof got), 156);
    CHECK_EQ(cfgfile_save(a, 100), 0);

    static uint8_t rec[100];                                    /* both users of the volume, one after the other */
    csv_meta_t m;
    memset(&m, 0, sizeof m);
    m.rate_hz = 1000000; m.vdiv_mv = 1000; m.coupling_dc = 1; m.zero_code = 128;
    int num = 0;
    CHECK_EQ(capsave_write(rec, 100, &m, &num), CAP_OK);
    CHECK_EQ(cfgfile_load(got, sizeof got), 100);
    CHECK(memcmp(got, a, 100) == 0);
}
