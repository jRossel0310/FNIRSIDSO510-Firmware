#include "test.h"
#include "render.h"
#include <stdint.h>

void test_render(void) {
    uint8_t ramp[10];
    for (int i = 0; i < 10; i++) ramp[i] = (uint8_t)(i * 10);
    span_t s[8];

    /* 2 samples per column */
    render_spans(ramp, 10, 0, 512, s, 3);
    CHECK_EQ(s[0].lo, 0);  CHECK_EQ(s[0].hi, 10);
    CHECK_EQ(s[1].lo, 10); CHECK_EQ(s[1].hi, 30);   /* includes previous sample 10 */
    CHECK_EQ(s[2].lo, 30); CHECK_EQ(s[2].hi, 50);

    /* start offset and running past the end */
    render_spans(ramp, 10, 6, 512, s, 3);
    CHECK_EQ(s[0].lo, 50); CHECK_EQ(s[0].hi, 70);   /* samples 6,7 + previous 5 */
    CHECK_EQ(s[1].lo, 70); CHECK_EQ(s[1].hi, 90);
    CHECK_EQ(s[2].lo, -1);                          /* beyond n */

    /* half a sample per column: interpolate */
    render_spans(ramp, 10, 0, 128, s, 4);
    CHECK_EQ(s[0].lo, 0);  CHECK_EQ(s[0].hi, 0);
    CHECK_EQ(s[1].lo, 0);  CHECK_EQ(s[1].hi, 5);
    CHECK_EQ(s[2].lo, 5);  CHECK_EQ(s[2].hi, 10);
    CHECK_EQ(s[3].lo, 10); CHECK_EQ(s[3].hi, 15);

    /* min/max with a spike inside a 4-sample column */
    uint8_t sp[8] = {100, 100, 250, 100, 100, 100, 100, 100};
    render_spans(sp, 8, 0, 1024, s, 2);
    CHECK_EQ(s[0].lo, 100); CHECK_EQ(s[0].hi, 250);
    CHECK_EQ(s[1].lo, 100); CHECK_EQ(s[1].hi, 100);
}
