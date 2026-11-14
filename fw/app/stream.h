#pragma once
#include <stdint.h>
#include "csv.h"

typedef enum { STREAM_IDLE = 0, STREAM_FRAMES, STREAM_LOG } stream_mode_t;

/* What the scope app does when the host starts/stops the logger. log_start returns 0, or -1 to refuse. */
typedef struct {
    int (*log_start)(uint32_t rate_hz);
    void (*log_stop)(void);
} stream_hooks_t;

void stream_init(const stream_hooks_t *h);
/* Main loop: port open/close (INFO on open; logger stops on close) and host commands. */
void stream_poll(void);
stream_mode_t stream_mode(void);
/* Frames mode: queue this capture as a FRAME, or skip it if the previous one is still being sent. */
void stream_frame(const uint8_t *rec, int n, const csv_meta_t *m);
/* Logger: settings packet (at start and whenever V/div or offset change). */
void stream_logmeta(uint32_t rate_hz, uint16_t vdiv_mv, uint8_t probe_x10, uint8_t coupling_dc,
                    int16_t zero_code, uint16_t offset_pwm);
/* Logger: queue as many whole LOG chunks of s[0..n) as fit now (after any waiting reply and LOGMETA).
   Returns the samples taken; the caller keeps the rest for the next pass or reports it lost. */
uint32_t stream_log_fit(const uint8_t *s, uint32_t n);
void stream_log_lost(uint32_t n);
/* Leave logger mode from the scope side (Run key): hook log_stop, mode idle, INFO to the host. */
void stream_stop_log(void);
void stream_log_stats(uint32_t *sent, uint32_t *dropped);
