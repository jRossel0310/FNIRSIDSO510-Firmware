#include "test.h"
#include "keys.h"

static int feed(keys_state_t *s, uint32_t raw, uint32_t t, kev_t *ev) { return keys_update(s, raw, t, ev, 8); }

void test_keys(void) {
    keys_state_t s; kev_t ev[8]; int n;
    keys_reset(&s, KEY_BIT(KEY_UP));

    /* short press of RUN: event on release, after debounce */
    CHECK_EQ(feed(&s, KEY_BIT(KEY_RUN), 0, ev), 0);
    CHECK_EQ(feed(&s, KEY_BIT(KEY_RUN), 25, ev), 0);       /* pressed (debounced), no event yet */
    CHECK_EQ(feed(&s, 0, 100, ev), 0);
    n = feed(&s, 0, 125, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_SHORT); CHECK_EQ(ev[0].key, KEY_RUN);

    /* bounce shorter than debounce is ignored */
    CHECK_EQ(feed(&s, KEY_BIT(KEY_RUN), 200, ev), 0);
    CHECK_EQ(feed(&s, 0, 205, ev), 0);
    CHECK_EQ(feed(&s, 0, 300, ev), 0);

    /* long press of POWER: LONG at 700 ms held, nothing on release */
    feed(&s, KEY_BIT(KEY_POWER), 1000, ev);
    feed(&s, KEY_BIT(KEY_POWER), 1025, ev);                /* debounced press at 1025 */
    CHECK_EQ(feed(&s, KEY_BIT(KEY_POWER), 1700, ev), 0);
    n = feed(&s, KEY_BIT(KEY_POWER), 1725, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_LONG); CHECK_EQ(ev[0].key, KEY_POWER);
    CHECK_EQ(feed(&s, KEY_BIT(KEY_POWER), 1900, ev), 0);   /* fires once */
    feed(&s, 0, 2000, ev);
    CHECK_EQ(feed(&s, 0, 2030, ev), 0);                    /* no SHORT after LONG */

    /* repeat key: SHORT on press, REPEAT after 400 ms then every 80 ms */
    feed(&s, KEY_BIT(KEY_UP), 3000, ev);
    n = feed(&s, KEY_BIT(KEY_UP), 3025, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_SHORT); CHECK_EQ(ev[0].key, KEY_UP);
    CHECK_EQ(feed(&s, KEY_BIT(KEY_UP), 3400, ev), 0);
    n = feed(&s, KEY_BIT(KEY_UP), 3425, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_REPEAT);
    n = feed(&s, KEY_BIT(KEY_UP), 3505, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_REPEAT);
    feed(&s, 0, 3600, ev);
    CHECK_EQ(feed(&s, 0, 3625, ev), 0);                    /* no event on release */

    /* held repeat key after a long stall (blocking auto-set): one REPEAT, not a burst */
    {
        keys_state_t r; kev_t e[8];
        keys_reset(&r, KEY_BIT(KEY_UP));
        feed(&r, KEY_BIT(KEY_UP), 0, e); feed(&r, KEY_BIT(KEY_UP), 25, e);
        int total = 0;
        total += feed(&r, KEY_BIT(KEY_UP), 20000, e);
        for (int i = 0; i < 4; i++) total += feed(&r, KEY_BIT(KEY_UP), 20001 + i, e);
        CHECK_EQ(total, 1);
        CHECK_EQ(feed(&r, KEY_BIT(KEY_UP), 20000 + 80, e), 1);   /* normal cadence resumes */
    }
}
