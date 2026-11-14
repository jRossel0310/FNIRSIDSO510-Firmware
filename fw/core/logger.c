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

uint32_t logger_push_fit(logger_t *l, bytefifo_t *f, const uint8_t *s, uint32_t n) {
    uint32_t done = 0;
    while (done < n) {
        uint32_t c = n - done > SP_LOG_MAX ? SP_LOG_MAX : n - done;
        uint32_t need = 4 + c + SP_OVERHEAD + (l->pending_drop ? 8 + SP_OVERHEAD : 0);
        if (bf_space(f) < need) break;               /* the rest stays with the caller */
        uint8_t h[8];
        if (l->pending_drop) {
            sp_u32x2(h, l->index, l->pending_drop);
            sp_put(f, SP_DROP, h, 8, 0, 0);
            l->pending_drop = 0;
        }
        sp_u32x2(h, l->index, 0);
        sp_put(f, SP_LOG, h, 4, s + done, c);
        l->index += c; l->sent += c;
        done += c;
    }
    return done;
}

int logger_put_drop(logger_t *l, bytefifo_t *f) {
    if (!l->pending_drop) return 0;
    uint8_t h[8];
    sp_u32x2(h, l->index, l->pending_drop);
    if (sp_put(f, SP_DROP, h, 8, 0, 0) < 0) return -1;
    l->pending_drop = 0;
    return 0;
}

ring_pick_t logger_pick(uint32_t completed, uint32_t *consumed, uint32_t n) {
    ring_pick_t r = {-1, -1, 0};
    uint32_t pending = completed - *consumed;          /* wrap-safe */
    if (pending == 0) return r;
    r.first = (int)((completed - 1) % 2);              /* newest completed half */
    r.lost = (pending - 1) * (n / 2);                  /* all older completions were overwritten */
    *consumed = completed;
    return r;
}
