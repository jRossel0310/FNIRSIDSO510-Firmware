#include "test.h"
#include "flashmap.h"
#include "fake_spiflash.h"

void test_flashmap(void) {
    CHECK_EQ(USER_VOL_BYTES, 0x500000u);
    CHECK_EQ(SYS_VOL_BASE, 0x500000u);
    CHECK_EQ(flashmap_writable(0, 4096), 1);
    CHECK_EQ(flashmap_writable(0x4FF000, 4096), 1);          /* last user sector */
    CHECK_EQ(flashmap_writable(0x4FFFFF, 1), 1);
    CHECK_EQ(flashmap_writable(0x4FF000, 4097), 0);          /* crosses into system volume */
    CHECK_EQ(flashmap_writable(0x500000, 1), 0);             /* system volume */
    CHECK_EQ(flashmap_writable(0x7FF000, 4096), 0);
    CHECK_EQ(flashmap_writable(0, 0), 0);                    /* empty range */
    CHECK_EQ(flashmap_writable(0, USER_VOL_BYTES), 1);
    CHECK_EQ(flashmap_writable(0, USER_VOL_BYTES + 1), 0);
    CHECK_EQ(flashmap_writable(0xFFFFFFFFu, 2), 0);          /* wraps */
    CHECK_EQ(flashmap_writable(0x100, 0xFFFFFF00u), 0);      /* wraps */
}

void test_fake_flash(void) {
    static uint8_t s[4096];
    fake_flash_reset(0xA5);
    memset(s, 0x3C, sizeof s);
    CHECK_EQ(spiflash_write_sector(0x4FF000, s), 0);
    CHECK_EQ(fake_flash_mem()[0x4FF000], 0x3C);
    CHECK_EQ(spiflash_write_sector(0x500000, s), -2);         /* system volume refused */
    CHECK_EQ(spiflash_erase_sector(0x500000), -2);
    CHECK_EQ(spiflash_program(0x500000, s, 16), -2);
    CHECK_EQ(spiflash_program(0x0000F0, s, 32), -2);           /* crosses a page */
    CHECK_EQ(fake_flash_mem()[0x500000], 0xA5);                /* untouched */
    CHECK_EQ(fake_flash_refused(), 4);
}
