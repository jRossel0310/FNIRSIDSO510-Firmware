#include "test.h"
#include "flashmap.h"

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
