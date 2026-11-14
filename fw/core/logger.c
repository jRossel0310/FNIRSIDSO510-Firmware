#include "logger.h"

void logger_init(logger_t *l) { l->index = 0; l->pending_drop = 0; l->sent = 0; l->dropped = 0; }

void logger_lost(logger_t *l, uint32_t n) { l->index += n; l->pending_drop += n; l->dropped += n; }

void logger_push(logger_t *l, bytefifo_t *f, const uint8_t *s, uint32_t n) {
    while (n) {
        uint32_t c = n > SP_LOG_MAX ? SP_LOG_MAX : n;
        uint32_t need = 4 + c + SP_OVERHEAD + (l->pending_drop ? 8 + SP_OVERHEAD : 0);
        if (bf_space(f) < need) {
            logger_lost(l, c);                        /* this chunk is lost too */
        } else {
            uint8_t h[8];
            if (l->pending_drop) {
                sp_u32x2(h, l->index, l->pending_drop);
                sp_put(f, SP_DROP, h, 8, 0, 0);
                l->pending_drop = 0;
            }
            sp_u32x2(h, l->index, 0);
            sp_put(f, SP_LOG, h, 4, s, c);
            l->index += c; l->sent += c;
        }
        s += c; n -= c;
    }
}

ring_pick_t logger_pick(int flags, uint32_t remaining, uint32_t n) {
    ring_pick_t r = {-1, -1, 0};
    int ht = flags & 1, tc = (flags >> 1) & 1;
    int in_first = remaining > n / 2;                  /* DMA now writing the first half */
    if (ht && tc) {                                    /* late: one half is being overwritten */
        r.first = in_first ? 1 : 0;
        r.lost = n / 2;
    } else if (ht) r.first = 0;
    else if (tc) r.first = 1;
    return r;
}
