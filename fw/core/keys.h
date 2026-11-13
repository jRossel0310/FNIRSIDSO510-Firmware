#pragma once
#include <stdint.h>

/* Logical keys; order matches board_keys[] (stock key-scan table @ 0x080349F8). */
enum {
    KEY_POWER = 0,   /* K1 PD2  */
    KEY_RUN,         /* K2 PC9  */
    KEY_UP,          /* K3 PC7  */
    KEY_DOWN,        /* K4 PC8  */
    KEY_MODE,        /* K5 PC6  */
    KEY_AUTO,        /* K6 PD13 */
    KEY_WHEEL_R,     /* K7 PB15 */
    KEY_WHEEL_PRESS, /* K8 PB14 */
    KEY_WHEEL_L,     /* K9 PB13 */
    NUM_KEYS
};
#define KEY_BIT(k) (1u << (k))

#define KEY_DEBOUNCE_MS     20
#define KEY_LONG_MS         700
#define KEY_REPEAT_DELAY_MS 400
#define KEY_REPEAT_MS       80

enum { KEV_SHORT = 1, KEV_LONG, KEV_REPEAT };
typedef struct { uint8_t type, key; } kev_t;

typedef struct {
    uint32_t repeat_mask, state, raw_prev, long_done;
    uint32_t raw_since[NUM_KEYS], down_at[NUM_KEYS], next_rep[NUM_KEYS];
} keys_state_t;

void keys_reset(keys_state_t *s, uint32_t repeat_mask);
/* Feed the raw pressed-key bitmask at time now_ms; writes up to max events, returns count. */
int keys_update(keys_state_t *s, uint32_t raw, uint32_t now_ms, kev_t *out, int max);
