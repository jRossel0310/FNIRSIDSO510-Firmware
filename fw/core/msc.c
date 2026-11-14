#include "msc.h"

enum { S_IDLE, S_IN, S_OUT, S_ZERO_IN, S_DISCARD, S_READ, S_WRITE, S_STATUS };

static uint32_t be32(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static void put_be32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static uint32_t le32(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

static void set_sense(msc_t *m, uint8_t k, uint8_t a, uint8_t q) { m->key = k; m->asc = a; m->ascq = q; }

void msc_reset(msc_t *m) { m->state = S_IDLE; m->io = 0; }

void msc_init(msc_t *m, const msc_disk_t *d) {
    m->disk = d; m->present = 1; m->attention = 0; m->wrote = 0;
    set_sense(m, 0, 0, 0);
    msc_reset(m);
}

void msc_set_present(msc_t *m, int present) {
    if (present && !m->present) m->attention = 1;
    m->present = (uint8_t)(present != 0);
}

int msc_busy(const msc_t *m, uint32_t now_ms) {
    if (m->state != S_IDLE) return 1;
    return m->wrote && (now_ms - m->last_write_ms) < 1000u;
}

/* Fail the command: fill/discard the data phase, CSW status 1, residue = Hi. */
static void fail(msc_t *m, int dir_in, uint8_t k, uint8_t a, uint8_t q) {
    set_sense(m, k, a, q);
    m->status = 1; m->residue = m->hi; m->left = m->hi;
    m->state = m->hi == 0 ? S_STATUS : dir_in ? S_ZERO_IN : S_DISCARD;
}

/* Medium usable? Otherwise fail with the right sense (consumes a pending UNIT ATTENTION). */
static int ready(msc_t *m, int dir_in) {
    if (!m->present) { fail(m, dir_in, 2, 0x3A, 0); return 0; }
    if (m->attention) { m->attention = 0; fail(m, dir_in, 6, 0x28, 0); return 0; }
    return 1;
}

/* Small data-in response already in buf[0..n): send min(n, Hi). */
static void respond(msc_t *m, uint32_t n, int dir_in) {
    if (m->hi > 0 && !dir_in) { fail(m, dir_in, 5, 0x24, 0); return; }   /* host expects data-out */
    if (n > m->hi) n = m->hi;
    m->len = n; m->pos = 0; m->status = 0; m->residue = m->hi - n;
    m->state = n ? S_IN : S_STATUS;
}

/* No data of our own: move Hi bytes in the host's direction (fill/discard), CSW good, residue = Hi. */
static void pass(msc_t *m, int dir_in) {
    m->status = 0; m->residue = m->hi; m->left = m->hi;
    m->state = m->hi == 0 ? S_STATUS : dir_in ? S_ZERO_IN : S_DISCARD;
}

static void command(msc_t *m, const uint8_t *cb, int dir_in) {
    uint8_t *b = m->buf;
    for (int i = 0; i < 36; i++) b[i] = 0;
    switch (cb[0]) {
    case 0x00:                                           /* TEST UNIT READY */
        if (ready(m, dir_in)) pass(m, dir_in);
        break;
    case 0x03:                                           /* REQUEST SENSE */
        b[0] = 0x70; b[2] = m->key; b[7] = 10; b[12] = m->asc; b[13] = m->ascq;
        set_sense(m, 0, 0, 0);
        respond(m, 18, dir_in);
        break;
    case 0x12: {                                         /* INQUIRY */
        static const char id[] = "DSO-510 Capture Drive   0.2 ";   /* vendor 8, product 16, rev 4 */
        if (cb[1] & 1) { fail(m, dir_in, 5, 0x24, 0); break; }   /* EVPD not supported */
        b[1] = 0x80; b[2] = 0x02; b[3] = 0x02; b[4] = 31;
        for (int i = 0; i < 28; i++) b[8 + i] = (uint8_t)id[i];
        respond(m, 36, dir_in);
        break;
    }
    case 0x1A:                                           /* MODE SENSE(6) */
        b[0] = 3;
        respond(m, 4, dir_in);
        break;
    case 0x5A:                                           /* MODE SENSE(10) */
        b[1] = 6;
        respond(m, 8, dir_in);
        break;
    case 0x1E: case 0x1B: case 0x2F: case 0x35:          /* PREVENT/ALLOW, START STOP, VERIFY, SYNC CACHE */
        pass(m, dir_in);
        break;
    case 0x23:                                           /* READ FORMAT CAPACITIES */
        b[3] = 8; put_be32(b + 4, m->disk->blocks); b[8] = m->present ? 2 : 3; b[10] = 0x10;   /* 0x001000 = 4096 */
        respond(m, 12, dir_in);
        break;
    case 0x25:                                           /* READ CAPACITY(10) */
        if (!ready(m, dir_in)) break;
        put_be32(b, m->disk->blocks - 1); put_be32(b + 4, MSC_BLOCK);
        respond(m, 8, dir_in);
        break;
    case 0x28: case 0x2A: {                              /* READ(10) / WRITE(10) */
        int rd = cb[0] == 0x28;
        if (!ready(m, dir_in)) break;
        uint32_t lba = be32(cb + 2), n = (uint32_t)cb[7] << 8 | cb[8];
        if (dir_in != rd || m->hi != n * MSC_BLOCK) {      /* BOT cases 2-13: Hi/direction mismatch */
            if (n == 0 && m->hi == 0) { pass(m, dir_in); break; }
            fail(m, dir_in, 5, 0x24, 0); break;
        }
        if (lba >= m->disk->blocks || n > m->disk->blocks - lba) { fail(m, dir_in, 5, 0x21, 0); break; }
        if (n == 0) { pass(m, dir_in); break; }
        m->lba = lba; m->nblk = n; m->status = 0;
        m->residue = 0;
        m->pos = 0;
        if (rd) { m->state = S_READ; m->io = 1; } else { m->state = S_WRITE; }
        break;
    }
    default:
        fail(m, dir_in, 5, 0x20, 0);                     /* invalid command opcode */
    }
}

int msc_rx(msc_t *m, const uint8_t *pkt, int len, uint32_t now_ms) {
    switch (m->state) {
    case S_IDLE:
        if (len != 31 || le32(pkt) != 0x43425355u || (pkt[14] & 0x1F) == 0 || (pkt[14] & 0x1F) > 16) return 0;
        for (int i = 0; i < 4; i++) m->tag[i] = pkt[4 + i];
        m->hi = le32(pkt + 8);
        command(m, pkt + 15, (pkt[12] & 0x80) != 0);
        return 0;
    case S_DISCARD:
        m->left = (uint32_t)len >= m->left ? 0 : m->left - (uint32_t)len;
        if (!m->left) m->state = S_STATUS;
        return 0;
    case S_WRITE:
        if (m->io) return -1;                            /* previous block still being written */
        for (int i = 0; i < len && m->pos < MSC_BLOCK; i++) m->buf[m->pos++] = pkt[i];
        m->wrote = 1; m->last_write_ms = now_ms;
        if (m->pos == MSC_BLOCK) m->io = 2;
        return 0;
    default:
        return -1;
    }
}

void msc_task(msc_t *m) {
    if (m->io == 1) {
        m->io = 0;
        if (m->disk->read(m->lba, m->buf)) {             /* unrecovered read error: zero-fill rest */
            set_sense(m, 3, 0x11, 0);
            m->status = 1; m->left = m->nblk * MSC_BLOCK; m->residue = m->hi; m->state = S_ZERO_IN;
            return;
        }
        m->len = MSC_BLOCK; m->pos = 0;
    } else if (m->io == 2) {
        m->io = 0;
        if (!m->present) {                               /* medium removed while writing */
            set_sense(m, 2, 0x3A, 0);
            m->status = 1; m->residue = m->hi; m->left = (m->nblk - 1) * MSC_BLOCK;
            m->state = m->left ? S_DISCARD : S_STATUS;
            return;
        }
        if (m->disk->write(m->lba, m->buf)) {            /* write error: discard the rest */
            set_sense(m, 3, 0x0C, 0);
            m->status = 1; m->residue = m->hi; m->left = (m->nblk - 1) * MSC_BLOCK;
            m->state = m->left ? S_DISCARD : S_STATUS;
            return;
        }
        m->lba++; m->pos = 0;
        if (--m->nblk == 0) m->state = S_STATUS;
    }
}

int msc_tx(msc_t *m, uint8_t *pkt) {
    uint32_t n;
    switch (m->state) {
    case S_IN:
        n = m->len - m->pos > MSC_PKT ? MSC_PKT : m->len - m->pos;
        for (uint32_t i = 0; i < n; i++) pkt[i] = m->buf[m->pos + i];
        m->pos += n;
        if (m->pos == m->len) m->state = S_STATUS;
        return (int)n;
    case S_READ:
        if (m->io || m->pos >= MSC_BLOCK) return -1;     /* waiting for msc_task */
        for (uint32_t i = 0; i < MSC_PKT; i++) pkt[i] = m->buf[m->pos + i];
        m->pos += MSC_PKT;
        if (m->pos == MSC_BLOCK) {
            m->lba++;
            if (--m->nblk == 0) m->state = S_STATUS; else m->io = 1;
        }
        return MSC_PKT;
    case S_ZERO_IN:
        n = m->left > MSC_PKT ? MSC_PKT : m->left;
        for (uint32_t i = 0; i < n; i++) pkt[i] = 0;
        m->left -= n;
        if (!m->left) m->state = S_STATUS;
        return (int)n;
    case S_STATUS:
        pkt[0] = 0x55; pkt[1] = 0x53; pkt[2] = 0x42; pkt[3] = 0x53;
        for (int i = 0; i < 4; i++) pkt[4 + i] = m->tag[i];
        pkt[8] = (uint8_t)m->residue; pkt[9] = (uint8_t)(m->residue >> 8);
        pkt[10] = (uint8_t)(m->residue >> 16); pkt[11] = (uint8_t)(m->residue >> 24);
        pkt[12] = m->status;
        m->state = S_IDLE;
        return 13;
    default:
        return -1;
    }
}
