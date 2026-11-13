#include "keys.h"
#include <string.h>

void keys_reset(keys_state_t *s, uint32_t repeat_mask) {
    memset(s, 0, sizeof *s);
    s->repeat_mask = repeat_mask;
}

int keys_update(keys_state_t *s, uint32_t raw, uint32_t now, kev_t *out, int max) {
    int n = 0;
    for (int k = 0; k < NUM_KEYS; k++) {
        uint32_t bit = 1u << k;
        int rep = (s->repeat_mask & bit) != 0;
        if ((raw ^ s->raw_prev) & bit) s->raw_since[k] = now;
        int r = (raw & bit) != 0, d = (s->state & bit) != 0;
        if (r != d && now - s->raw_since[k] >= KEY_DEBOUNCE_MS) {
            s->state ^= bit;
            if (r) {
                s->down_at[k] = now;
                s->long_done &= ~bit;
                if (rep && n < max) { out[n].type = KEV_SHORT; out[n].key = (uint8_t)k; n++; }
                s->next_rep[k] = now + KEY_REPEAT_DELAY_MS;
            } else if (!rep && !(s->long_done & bit) && n < max) {
                out[n].type = KEV_SHORT; out[n].key = (uint8_t)k; n++;
            }
        }
        if (s->state & bit) {
            if (rep) {
                if ((int32_t)(now - s->next_rep[k]) > KEY_REPEAT_MS) s->next_rep[k] = now;  /* no catch-up burst */
                if ((int32_t)(now - s->next_rep[k]) >= 0 && n < max) {
                    out[n].type = KEV_REPEAT; out[n].key = (uint8_t)k; n++;
                    s->next_rep[k] += KEY_REPEAT_MS;
                }
            } else if (!(s->long_done & bit) && now - s->down_at[k] >= KEY_LONG_MS && n < max) {
                out[n].type = KEV_LONG; out[n].key = (uint8_t)k; n++;
                s->long_done |= bit;
            }
        }
    }
    s->raw_prev = raw;
    return n;
}
