#pragma once
#include <stdint.h>

typedef enum { EDGE_RISING = 0, EDGE_FALLING = 1 } edge_t;

/* Hysteresis used for triggering: max(4, peak-to-peak / 8) in ADC codes. */
int trigger_hysteresis(const uint8_t *buf, int n);
/* First index in [from, to) where the signal reaches `level` in the edge direction, after having
 * been at least `hyst` codes on the other side. Unarmed at `from`. Returns -1 if none. */
int trigger_find(const uint8_t *buf, int from, int to, int level, int hyst, edge_t edge);
