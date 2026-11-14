#include "stream.h"
#include "stream_proto.h"
#include "logger.h"
#include "usbd.h"

static const stream_hooks_t *hk;
static stream_mode_t mode;
static uint8_t was_open;
static sp_line_t line;
static logger_t lg;
static uint8_t meta_buf[SP_LOGMETA_LEN];
static uint8_t meta_pending;

static int port_open(void) { return usbd_cdc_open() && usbd_configured(); }   /* a pulled cable only suspends */

static const char *mode_name(void) { return mode == STREAM_FRAMES ? "frames" : mode == STREAM_LOG ? "log" : "idle"; }

/* Control replies (INFO/OK/ERR) never get lost to a full TX FIFO: they wait here and are retried at
   the start of stream_poll and before every bulk push (FRAME, LOGMETA, LOG). Bulk data is not queued
   while a reply is waiting, so it can never take the room a reply needs. Commands are only read while
   three slots are free (INFO? queues INFO + OK), so one is always left for the INFO stream_stop_log
   sends on its own. */
#define REPLY_N   4
#define REPLY_MAX 160
typedef struct { uint8_t type; uint8_t len; uint8_t d[REPLY_MAX]; } reply_t;
static reply_t replies[REPLY_N];
static uint8_t rq_head, rq_n;

static int replies_flush(void) {                  /* returns 1 when none is left waiting */
    while (rq_n && port_open()) {
        reply_t *r = &replies[rq_head];
        if (sp_put(usbd_cdc_txfifo(), r->type, r->d, r->len, 0, 0) < 0) break;
        rq_head = (uint8_t)((rq_head + 1) % REPLY_N); rq_n--;
    }
    if (rq_n == 0) return 1;
    usbd_cdc_kick();
    return 0;
}

static void reply(uint8_t type, const uint8_t *p, uint32_t n) {
    if (n > REPLY_MAX || rq_n == REPLY_N) return;  /* cannot happen: callers check room and length */
    reply_t *r = &replies[(rq_head + rq_n) % REPLY_N];
    r->type = type; r->len = (uint8_t)n;
    for (uint32_t i = 0; i < n; i++) r->d[i] = p[i];
    rq_n++;
    replies_flush();
    usbd_cdc_kick();
}

static void send_text(uint8_t type, const char *t) {
    uint32_t n = 0;
    while (t[n]) n++;
    reply(type, (const uint8_t *)t, n);
}

static void send_info(void) {
    char t[128];
    uint32_t n = sp_info(t, sizeof t, mode_name());
    if (n) reply(SP_INFO, (const uint8_t *)t, n);
}

static void leave_log(void) { if (mode == STREAM_LOG) hk->log_stop(); }

void stream_init(const stream_hooks_t *h) { hk = h; mode = STREAM_IDLE; was_open = 0; line.n = 0; line.overflow = 0; rq_n = 0; }

stream_mode_t stream_mode(void) { return mode; }

static void command(sp_cmd_t c, uint32_t arg) {
    switch (c) {
    case SP_CMD_INFO: send_info(); send_text(SP_OK, "INFO?"); break;
    case SP_CMD_FRAMES: leave_log(); mode = STREAM_FRAMES; send_text(SP_OK, "FRAMES"); break;
    case SP_CMD_STOP: leave_log(); mode = STREAM_IDLE; send_text(SP_OK, "STOP"); break;
    case SP_CMD_LOG:
        if (!sp_rate_ok(arg)) { send_text(SP_ERR, "bad rate"); break; }
        leave_log();
        mode = STREAM_IDLE;
        logger_init(&lg);
        meta_pending = 0;
        if (hk->log_start(arg) < 0) { send_text(SP_ERR, "unsupported rate"); break; }
        mode = STREAM_LOG;
        send_text(SP_OK, line.buf);                  /* echo "LOG <rate>" */
        break;
    case SP_CMD_BAD: send_text(SP_ERR, "unknown command"); break;
    default: break;
    }
}

void stream_poll(void) {
    int open = port_open();
    if (!open && was_open) { leave_log(); mode = STREAM_IDLE; line.n = 0; line.overflow = 0; rq_n = 0; }
    if (open && !was_open) { rq_n = 0; send_info(); }   /* a new session starts with INFO */
    was_open = (uint8_t)open;
    if (!open) return;
    replies_flush();
    uint8_t c;
    for (int k = 0; k < 64 && REPLY_N - rq_n >= 3 && usbd_cdc_read(&c, 1) == 1; k++) {
        if (!sp_line_feed(&line, c)) continue;
        uint32_t arg;
        command(sp_parse(line.buf, &arg), arg);
    }
}

void stream_frame(const uint8_t *rec, int n, const csv_meta_t *m) {
    if (mode != STREAM_FRAMES || !port_open() || n <= 0 || n > 65535 - (int)SP_FRAME_META - 2) return;
    uint8_t h[SP_FRAME_META + 2];
    uint32_t hl = sp_frame_meta(h, m, (uint16_t)n);
    if (!replies_flush()) return;                               /* a reply is waiting: skip this one */
    if (sp_put(usbd_cdc_txfifo(), SP_FRAME, h, hl, rec, (uint32_t)n) == 0) usbd_cdc_kick();   /* else: skipped */
}

/* LOGMETA goes out after the DROP for every sample lost before it, so a reader sees the gap first and
   the new settings start exactly at the next LOG. */
static void try_send_meta(void) {
    if (!meta_pending || !replies_flush()) return;
    bytefifo_t *f = usbd_cdc_txfifo();
    if (logger_put_drop(&lg, f) == 0 && sp_put(f, SP_LOGMETA, meta_buf, SP_LOGMETA_LEN, 0, 0) == 0) meta_pending = 0;
    usbd_cdc_kick();
}

void stream_logmeta(uint32_t rate_hz, uint16_t vdiv_mv, uint8_t probe_x10, uint8_t coupling_dc,
                    int16_t zero_code, uint16_t offset_pwm) {
    sp_logmeta(meta_buf, rate_hz, vdiv_mv, probe_x10, coupling_dc, zero_code, offset_pwm);
    meta_pending = 1;
    try_send_meta();
}

uint32_t stream_log_fit(const uint8_t *s, uint32_t n) {
    if (mode != STREAM_LOG) return n;                  /* not logging: nothing to keep */
    if (!port_open() || !replies_flush()) return 0;     /* replies first */
    try_send_meta();
    if (meta_pending) return 0;                         /* never ahead of its LOGMETA */
    uint32_t k = logger_push_fit(&lg, usbd_cdc_txfifo(), s, n);
    usbd_cdc_kick();
    return k;
}

void stream_log_lost(uint32_t n) { if (mode == STREAM_LOG) logger_lost(&lg, n); }

void stream_stop_log(void) {
    if (mode != STREAM_LOG) return;
    hk->log_stop();
    mode = STREAM_IDLE;
    if (usbd_cdc_open()) send_info();
}

void stream_log_stats(uint32_t *sent, uint32_t *dropped) { *sent = lg.sent; *dropped = lg.dropped; }
