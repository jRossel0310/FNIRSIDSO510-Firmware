#include "test.h"
#include "trigger.h"
#include <stdint.h>

void test_trigger(void) {
    uint8_t sq[400];
    for (int i = 0; i < 400; i++) sq[i] = (i % 100) < 50 ? 200 : 50;   /* high first */
    CHECK_EQ(trigger_find(sq, 0, 400, 125, 10, EDGE_RISING), 100);
    CHECK_EQ(trigger_find(sq, 0, 400, 125, 10, EDGE_FALLING), 50);
    CHECK_EQ(trigger_find(sq, 101, 400, 125, 10, EDGE_RISING), 200);
    CHECK_EQ(trigger_find(sq, 0, 100, 125, 10, EDGE_RISING), -1);       /* window too short */

    /* noise near the level on a falling slope must not fire a rising trigger */
    uint8_t ns[8] = {200, 150, 128, 124, 127, 120, 60, 130};
    CHECK_EQ(trigger_find(ns, 0, 8, 125, 10, EDGE_RISING), 7);

    uint8_t flat[50];
    memset(flat, 128, sizeof flat);
    CHECK_EQ(trigger_find(flat, 0, 50, 128, 4, EDGE_RISING), -1);
    CHECK_EQ(trigger_hysteresis(flat, 50), 4);
    CHECK_EQ(trigger_hysteresis(sq, 400), 18);   /* (200-50)/8 */
}
