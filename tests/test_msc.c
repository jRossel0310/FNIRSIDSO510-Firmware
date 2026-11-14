#include "test.h"
#include "msc.h"

#define NBLK 8
static uint8_t disk[NBLK][MSC_BLOCK];
static int fail_write, fail_read_lba = -1;
static msc_t M;
/* "ISR" probe run from inside the block I/O, i.e. while msc_task's read/write is in flight */
static int lock_depth, lock_max, io_lock_depth = -1, isr_rx = 99, isr_tx = 99, isr_reset;
static void isr_probe(void) {
    uint8_t pkt[64] = {0x55, 0x53, 0x42, 0x43};
    io_lock_depth = lock_depth;
    if (isr_reset) {                                     /* BOT reset, then the host's next CBW (TUR) */
        msc_reset(&M);
        pkt[8] = 0; pkt[12] = 0; pkt[14] = 6;
    }
    isr_rx = msc_rx(&M, pkt, 31, 0);
    isr_tx = msc_tx(&M, pkt);
}
static int probe_io;
static int rd(uint32_t lba, uint8_t *b) {
    if (probe_io) isr_probe();
    memcpy(b, disk[lba], MSC_BLOCK);
    return (int)lba == fail_read_lba ? -1 : 0;
}
static int wr(uint32_t lba, const uint8_t *b) {
    if (probe_io) isr_probe();
    if (fail_write) return -1;
    memcpy(disk[lba], b, MSC_BLOCK); return 0;
}
static const msc_disk_t D = {rd, wr, NBLK};
static void lk(void) { lock_depth++; if (lock_depth > lock_max) lock_max = lock_depth; }
static void ul(void) { lock_depth--; }
static uint8_t in[70000];
static int in_len, csw_status;
static uint32_t csw_residue, csw_tag;

static void cbw(uint32_t tag, uint32_t hi, int dir_in, const uint8_t *cb, int cblen) {
    uint8_t p[31] = {0x55, 0x53, 0x42, 0x43};
    p[4] = (uint8_t)tag; p[5] = (uint8_t)(tag >> 8); p[6] = (uint8_t)(tag >> 16); p[7] = (uint8_t)(tag >> 24);
    p[8] = (uint8_t)hi; p[9] = (uint8_t)(hi >> 8); p[10] = (uint8_t)(hi >> 16); p[11] = (uint8_t)(hi >> 24);
    p[12] = dir_in ? 0x80 : 0; p[13] = 0; p[14] = (uint8_t)cblen;
    memcpy(p + 15, cb, (size_t)cblen);
    CHECK_EQ(msc_rx(&M, p, 31, 0), 0);
}

/* Pump IN packets (running msc_task) until the CSW arrives; data bytes go to in[]. */
static void run_in(void) {
    uint8_t pkt[64];
    in_len = 0; csw_status = -1;
    for (int guard = 0; guard < 5000; guard++) {
        msc_task(&M);
        int n = msc_tx(&M, pkt);
        if (n < 0) continue;
        if (n == 13 && pkt[0] == 0x55 && pkt[1] == 0x53 && pkt[2] == 0x42 && pkt[3] == 0x53) {
            csw_tag = pkt[4] | pkt[5] << 8 | pkt[6] << 16 | (uint32_t)pkt[7] << 24;
            csw_residue = pkt[8] | pkt[9] << 8 | pkt[10] << 16 | (uint32_t)pkt[11] << 24;
            csw_status = pkt[12];
            return;
        }
        memcpy(in + in_len, pkt, (size_t)n); in_len += n;
    }
}

/* Send hi bytes of data-out from src in 64-byte packets, running msc_task when held off. */
static void run_out(const uint8_t *src, int hi, uint32_t now) {
    for (int o = 0; o < hi; ) {
        int n = hi - o < 64 ? hi - o : 64;
        if (msc_rx(&M, src + o, n, now) == 0) o += n; else msc_task(&M);
    }
}

static void sense(int *key, int *asc, int *ascq) {
    uint8_t cb[6] = {0x03, 0, 0, 0, 18, 0};
    cbw(9, 18, 1, cb, 6); run_in();
    *key = in[2] & 0x0F; *asc = in[12]; *ascq = in[13];
}

void test_msc(void) {
    int k, a, q;
    uint8_t pkt0[64];
    msc_init(&M, &D);
    /* TEST UNIT READY */
    uint8_t tur[6] = {0};
    cbw(0x11223344, 0, 0, tur, 6); run_in();
    CHECK_EQ(csw_status, 0); CHECK_EQ(csw_tag, 0x11223344); CHECK_EQ(csw_residue, 0); CHECK_EQ(in_len, 0);
    /* INQUIRY */
    uint8_t inq[6] = {0x12, 0, 0, 0, 36, 0};
    cbw(1, 36, 1, inq, 6); run_in();
    CHECK_EQ(in_len, 36); CHECK_EQ(in[0], 0); CHECK_EQ(in[1], 0x80); CHECK_EQ(csw_status, 0);
    CHECK(memcmp(in + 8, "DSO-510 ", 8) == 0);
    /* READ CAPACITY(10) */
    uint8_t rc[10] = {0x25};
    cbw(2, 8, 1, rc, 10); run_in();
    CHECK_EQ(in_len, 8);
    CHECK_EQ(in[3], NBLK - 1); CHECK_EQ(in[6], 0x10); CHECK_EQ(in[7], 0x00);   /* 4096 */
    /* READ FORMAT CAPACITIES */
    uint8_t rfc[10] = {0x23, 0, 0, 0, 0, 0, 0, 0, 252, 0};
    cbw(3, 252, 1, rfc, 10); run_in();
    CHECK_EQ(in_len, 12); CHECK_EQ(in[3], 8); CHECK_EQ(in[7], NBLK); CHECK_EQ(in[8], 2);
    CHECK_EQ(csw_residue, 240);
    /* MODE SENSE(6): not write-protected */
    uint8_t ms[6] = {0x1A, 0, 0x3F, 0, 192, 0};
    cbw(4, 192, 1, ms, 6); run_in();
    CHECK_EQ(in_len, 4); CHECK_EQ(in[2], 0); CHECK_EQ(csw_residue, 188);
    /* WRITE(10) lba 2, 2 blocks, then READ(10) back */
    static uint8_t pat[2 * MSC_BLOCK];
    for (int i = 0; i < (int)sizeof pat; i++) pat[i] = (uint8_t)(i * 31 + 7);
    uint8_t w10[10] = {0x2A, 0, 0, 0, 0, 2, 0, 0, 2, 0};
    cbw(5, sizeof pat, 0, w10, 10);
    CHECK_EQ(msc_rx(&M, pat, 64, 1000), 0);
    run_out(pat + 64, (int)sizeof pat - 64, 1000);
    run_in();
    CHECK_EQ(csw_status, 0); CHECK_EQ(csw_residue, 0);
    CHECK(memcmp(disk[2], pat, sizeof pat) == 0);
    CHECK_EQ(msc_busy(&M, 1500), 1);                      /* wrote within the last second */
    CHECK_EQ(msc_busy(&M, 2001), 0);
    uint8_t r10[10] = {0x28, 0, 0, 0, 0, 2, 0, 0, 2, 0};
    cbw(6, sizeof pat, 1, r10, 10); run_in();
    CHECK_EQ(in_len, (int)sizeof pat); CHECK(memcmp(in, pat, sizeof pat) == 0); CHECK_EQ(csw_status, 0);
    /* READ past the end: zero fill, failed CSW, sense 05/21/00 */
    uint8_t rbad[10] = {0x28, 0, 0, 0, 0, NBLK - 1, 0, 0, 2, 0};
    cbw(7, 2 * MSC_BLOCK, 1, rbad, 10); run_in();
    CHECK_EQ(in_len, 2 * MSC_BLOCK); CHECK_EQ(in[0], 0); CHECK_EQ(csw_status, 1);
    CHECK_EQ(csw_residue, 2 * MSC_BLOCK);
    sense(&k, &a, &q); CHECK_EQ(k, 5); CHECK_EQ(a, 0x21);
    /* unknown opcode with data-in */
    uint8_t unk[12] = {0xA0};
    cbw(8, 16, 1, unk, 12); run_in();
    CHECK_EQ(in_len, 16); CHECK_EQ(csw_status, 1);
    sense(&k, &a, &q); CHECK_EQ(k, 5); CHECK_EQ(a, 0x20);
    /* medium removed: everything medium-related fails with 02/3A/00 */
    msc_set_present(&M, 0);
    cbw(10, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 1);
    sense(&k, &a, &q); CHECK_EQ(k, 2); CHECK_EQ(a, 0x3A); CHECK_EQ(q, 0);
    uint8_t r1[10] = {0x28, 0, 0, 0, 0, 0, 0, 0, 1, 0};
    cbw(11, MSC_BLOCK, 1, r1, 10); run_in();
    CHECK_EQ(in_len, MSC_BLOCK); CHECK_EQ(csw_status, 1);
    cbw(12, 8, 1, rc, 10); run_in(); CHECK_EQ(csw_status, 1);
    /* write while removed: data discarded, disk unchanged */
    uint8_t w1[10] = {0x2A, 0, 0, 0, 0, 0, 0, 0, 1, 0};
    uint8_t before = disk[0][0];
    static uint8_t junk[MSC_BLOCK];
    memset(junk, 0x77, sizeof junk);
    cbw(13, MSC_BLOCK, 0, w1, 10); run_out(junk, MSC_BLOCK, 3000); run_in();
    CHECK_EQ(csw_status, 1); CHECK_EQ(disk[0][0], before);
    /* re-inserted: one UNIT ATTENTION 06/28/00, then ready */
    msc_set_present(&M, 1);
    cbw(14, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 1);
    sense(&k, &a, &q); CHECK_EQ(k, 6); CHECK_EQ(a, 0x28);
    cbw(15, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    /* write error: 03/0C/00 */
    fail_write = 1;
    cbw(16, MSC_BLOCK, 0, w1, 10); run_out(junk, MSC_BLOCK, 4000); run_in();
    CHECK_EQ(csw_status, 1);
    sense(&k, &a, &q); CHECK_EQ(k, 3); CHECK_EQ(a, 0x0C);
    fail_write = 0;
    /* held off while a block write is pending (before msc_task runs) */
    cbw(17, MSC_BLOCK, 0, w1, 10);
    for (int o = 0; o < (int)MSC_BLOCK; o += 64) CHECK_EQ(msc_rx(&M, junk + o, 64, 5000), 0);
    CHECK_EQ(msc_rx(&M, junk, 31, 5000), -1);             /* next CBW must wait */
    CHECK_EQ(msc_busy(&M, 9000), 1);                      /* command in progress */
    run_in(); CHECK_EQ(csw_status, 0);
    /* PREVENT/ALLOW, START STOP, VERIFY pass */
    uint8_t pa[6] = {0x1E, 0, 0, 0, 1, 0}, ss[6] = {0x1B, 0, 0, 0, 2, 0}, vf[10] = {0x2F};
    cbw(18, 0, 0, pa, 6); run_in(); CHECK_EQ(csw_status, 0);
    cbw(19, 0, 0, ss, 6); run_in(); CHECK_EQ(csw_status, 0);
    cbw(20, 0, 0, vf, 10); run_in(); CHECK_EQ(csw_status, 0);
    /* BOT Hi/direction mismatches: exactly Hi bytes move in the host's direction, then CSW */
    static uint8_t keep[2 * MSC_BLOCK];
    memcpy(keep, disk[0], sizeof keep);
    cbw(30, 2 * MSC_BLOCK, 1, r1, 10); run_in();                       /* (a) READ n=1, Hi=2 blocks */
    CHECK_EQ(in_len, 2 * MSC_BLOCK); CHECK_EQ(csw_status, 1); CHECK_EQ(csw_residue, 2 * MSC_BLOCK);
    cbw(31, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    uint8_t w2[10] = {0x2A, 0, 0, 0, 0, 0, 0, 0, 2, 0};
    cbw(32, MSC_BLOCK, 0, w2, 10); run_out(junk, MSC_BLOCK, 6000); run_in();   /* (b) WRITE n=2, Hi=1 block */
    CHECK_EQ(csw_status, 1); CHECK(memcmp(disk[0], keep, sizeof keep) == 0);
    cbw(33, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    cbw(34, MSC_BLOCK, 0, r1, 10); run_out(junk, MSC_BLOCK, 6000); run_in();   /* (c) READ with OUT direction */
    CHECK_EQ(in_len, 0); CHECK_EQ(csw_status, 1);
    cbw(35, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    cbw(36, 64, 1, tur, 6); run_in();                                  /* (d) TUR with Hi=64 IN */
    CHECK_EQ(in_len, 64); CHECK_EQ(in[0], 0); CHECK_EQ(in[63], 0); CHECK_EQ(csw_status, 0); CHECK_EQ(csw_residue, 64);
    cbw(37, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    cbw(38, 36, 0, inq, 6); run_out(junk, 36, 6000); run_in();         /* (e) INQUIRY with OUT direction */
    CHECK_EQ(in_len, 0); CHECK_EQ(csw_status, 1);
    cbw(39, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    cbw(40, 0, 1, r1, 10); run_in(); CHECK_EQ(csw_status, 1);          /* READ n=1 with Hi=0 */
    cbw(41, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    /* INQUIRY EVPD unsupported */
    uint8_t evpd[6] = {0x12, 1, 0x80, 0, 36, 0};
    cbw(42, 36, 1, evpd, 6); run_in();
    CHECK_EQ(in_len, 36); CHECK_EQ(csw_status, 1);
    sense(&k, &a, &q); CHECK_EQ(k, 5); CHECK_EQ(a, 0x24);
    /* SYNCHRONIZE CACHE(10) passes */
    uint8_t sc[10] = {0x35};
    cbw(43, 0, 0, sc, 10); run_in(); CHECK_EQ(csw_status, 0);
    /* medium removed: format capacities report no media; pending write is not performed */
    memcpy(keep, disk[4], sizeof keep);
    uint8_t w2b[10] = {0x2A, 0, 0, 0, 0, 4, 0, 0, 2, 0};
    cbw(44, 2 * MSC_BLOCK, 0, w2b, 10);
    run_out(pat, MSC_BLOCK, 7000); msc_task(&M);                       /* block 0 written */
    CHECK(memcmp(disk[4], pat, MSC_BLOCK) == 0);
    msc_set_present(&M, 0);
    run_out(pat + MSC_BLOCK, MSC_BLOCK, 7000); run_in();
    CHECK_EQ(csw_status, 1); CHECK(memcmp(disk[5], keep + MSC_BLOCK, MSC_BLOCK) == 0);
    sense(&k, &a, &q); CHECK_EQ(k, 2); CHECK_EQ(a, 0x3A);
    cbw(45, 252, 1, rfc, 10); run_in();
    CHECK_EQ(in_len, 12); CHECK_EQ(in[8], 3);
    msc_set_present(&M, 1);
    cbw(46, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 1);          /* unit attention */
    cbw(47, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    /* M2: removal while a WRITE is still receiving data fails it at once (02/3A/00), rest discarded */
    memcpy(keep, disk[2], sizeof keep);
    uint8_t w2c[10] = {0x2A, 0, 0, 0, 0, 2, 0, 0, 2, 0};
    cbw(48, 2 * MSC_BLOCK, 0, w2c, 10);
    run_out(pat, MSC_BLOCK / 2, 8000);                                   /* half a block, nothing written */
    msc_set_present(&M, 0);
    CHECK_EQ(msc_busy(&M, 8000), 1);                                     /* still owes the host a CSW */
    msc_set_present(&M, 1);                                              /* back before the rest arrives */
    run_out(pat + MSC_BLOCK / 2, 3 * MSC_BLOCK / 2, 8000); run_in();      /* the remaining 1.5 blocks */
    CHECK_EQ(csw_status, 1); CHECK_EQ(csw_tag, 48); CHECK_EQ(csw_residue, 2 * MSC_BLOCK);
    CHECK(memcmp(disk[2], keep, sizeof keep) == 0);                      /* the stale write never lands */
    sense(&k, &a, &q); CHECK_EQ(k, 2); CHECK_EQ(a, 0x3A); CHECK_EQ(q, 0);
    cbw(49, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 1);            /* unit attention */
    cbw(50, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    /* M2: a full block received but not yet written is dropped too */
    cbw(51, MSC_BLOCK, 0, w1, 10);
    for (int o = 0; o < (int)MSC_BLOCK; o += 64) CHECK_EQ(msc_rx(&M, junk + o, 64, 8100), 0);
    before = disk[0][0];
    msc_set_present(&M, 0); msc_set_present(&M, 1); run_in();
    CHECK_EQ(csw_status, 1); CHECK_EQ(disk[0][0], before);
    cbw(52, 0, 0, tur, 6); run_in(); cbw(53, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0);
    /* M5: read error on the 2nd of 3 blocks: block 1 sent, rest zero-filled, residue = Hi - sent */
    uint8_t r3[10] = {0x28, 0, 0, 0, 0, 2, 0, 0, 3, 0};
    fail_read_lba = 3;
    cbw(54, 3 * MSC_BLOCK, 1, r3, 10); run_in();
    fail_read_lba = -1;
    CHECK_EQ(in_len, 3 * MSC_BLOCK); CHECK(memcmp(in, disk[2], MSC_BLOCK) == 0);
    CHECK_EQ(in[MSC_BLOCK], 0); CHECK_EQ(in[3 * MSC_BLOCK - 1], 0);
    CHECK_EQ(csw_status, 1); CHECK_EQ(csw_residue, 2 * MSC_BLOCK);
    sense(&k, &a, &q); CHECK_EQ(k, 3); CHECK_EQ(a, 0x11);
    fail_read_lba = 2;                                                   /* first block fails: residue = Hi */
    cbw(55, 3 * MSC_BLOCK, 1, r3, 10); run_in();
    fail_read_lba = -1;
    CHECK_EQ(in_len, 3 * MSC_BLOCK); CHECK_EQ(csw_status, 1); CHECK_EQ(csw_residue, 3 * MSC_BLOCK);
    sense(&k, &a, &q);

    /* Concurrency: the lock is taken around state access but never held during the block I/O,
       and an ISR running during the I/O is refused (packet stays held), so buf is never shared. */
    msc_set_lock(&M, lk, ul);
    probe_io = 1;
    cbw(60, 2 * MSC_BLOCK, 1, r10, 10); run_in();                        /* READ lba 2, 2 blocks */
    CHECK_EQ(in_len, 2 * MSC_BLOCK); CHECK(memcmp(in, disk[2], 2 * MSC_BLOCK) == 0); CHECK_EQ(csw_status, 0);
    CHECK_EQ(io_lock_depth, 0); CHECK_EQ(isr_rx, -1); CHECK_EQ(isr_tx, -1);
    CHECK_EQ(lock_depth, 0); CHECK_EQ(lock_max, 1);
    isr_rx = isr_tx = 99;
    cbw(61, 2 * MSC_BLOCK, 0, w10, 10); run_out(pat, 2 * MSC_BLOCK, 9000); run_in();   /* WRITE */
    CHECK_EQ(csw_status, 0); CHECK(memcmp(disk[2], pat, 2 * MSC_BLOCK) == 0);
    CHECK_EQ(io_lock_depth, 0); CHECK_EQ(isr_rx, -1); CHECK_EQ(isr_tx, -1); CHECK_EQ(lock_depth, 0);
    /* io stays set until the I/O has finished: a packet offered meanwhile is held, then accepted */
    cbw(62, MSC_BLOCK, 0, w1, 10);
    for (int o = 0; o < (int)MSC_BLOCK; o += 64) CHECK_EQ(msc_rx(&M, junk + o, 64, 9100), 0);
    CHECK_EQ(M.io, 2);
    msc_task(&M);                                                        /* probe saw io busy -> -1 */
    CHECK_EQ(isr_rx, -1); CHECK_EQ(M.io, 0); CHECK_EQ(M.inflight, 0);
    run_in(); CHECK_EQ(csw_status, 0);
    /* bus/BOT reset from the ISR during the I/O: the result is dropped, the ISR stays locked out
       until the I/O ends, and the next command runs normally */
    isr_reset = 1; isr_rx = isr_tx = 99;
    cbw(63, 2 * MSC_BLOCK, 1, r10, 10);
    msc_task(&M);
    CHECK_EQ(isr_rx, -1); CHECK_EQ(isr_tx, -1);                          /* CBW after the reset held */
    CHECK_EQ(M.inflight, 0); CHECK_EQ(M.io, 0); CHECK_EQ(msc_busy(&M, 99999), 0);   /* back to IDLE */
    CHECK_EQ(msc_tx(&M, pkt0), -1);                                      /* no stale data-in */
    cbw(67, MSC_BLOCK, 0, w1, 10);                                       /* same for a 1-block WRITE */
    for (int o = 0; o < (int)MSC_BLOCK; o += 64) CHECK_EQ(msc_rx(&M, junk + o, 64, 9200), 0);
    msc_task(&M);
    CHECK_EQ(msc_tx(&M, pkt0), -1);                                      /* no CSW for the reset command */
    isr_reset = 0; probe_io = 0;
    cbw(64, 0, 0, tur, 6); run_in(); CHECK_EQ(csw_status, 0); CHECK_EQ(csw_tag, 64);
    msc_set_present(&M, 0); msc_set_present(&M, 1);
    CHECK_EQ(lock_depth, 0);
    cbw(65, 0, 0, tur, 6); run_in(); cbw(66, 0, 0, tur, 6); run_in();
    msc_set_lock(&M, 0, 0);

    /* garbage instead of a CBW is ignored */
    uint8_t junk31[31] = {1, 2, 3};
    CHECK_EQ(msc_rx(&M, junk31, 31, 0), 0);
    uint8_t pkt[64];
    CHECK_EQ(msc_tx(&M, pkt), -1);
}
