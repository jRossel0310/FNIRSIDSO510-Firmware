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

static void send_text(uint8_t type, const char *t) {
    uint32_t n = 0;
    while (t[n]) n++;
    sp_put(usbd_cdc_txfifo(), type, (const uint8_t *)t, n, 0, 0);
    usbd_cdc_kick();
}

static void send_info(void) {
    char t[128];
    uint32_t n = sp_info(t, sizeof t, mode_name());
    if (n) { sp_put(usbd_cdc_txfifo(), SP_INFO, (const uint8_t *)t, n, 0, 0); usbd_cdc_kick(); }
}

static void leave_log(void) { if (mode == STREAM_LOG) hk->log_stop(); }

void stream_init(const stream_hooks_t *h) { hk = h; mode = STREAM_IDLE; was_open = 0; line.n = 0; line.overflow = 0; }

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
    if (open && !was_open) send_info();
    if (!open && was_open) { leave_log(); mode = STREAM_IDLE; line.n = 0; line.overflow = 0; }
    was_open = (uint8_t)open;
    if (!open) return;
    uint8_t in[32];
    uint32_t n;
    while ((n = usbd_cdc_read(in, sizeof in)) > 0) {
        for (uint32_t i = 0; i < n; i++) {
            if (!sp_line_feed(&line, in[i])) continue;
            uint32_t arg;
            command(sp_parse(line.buf, &arg), arg);
        }
    }
}

void stream_frame(const uint8_t *rec, int n, const csv_meta_t *m) {
    if (mode != STREAM_FRAMES || !port_open() || n <= 0 || n > 65535 - (int)SP_FRAME_META - 2) return;
    uint8_t h[SP_FRAME_META + 2];
    uint32_t hl = sp_frame_meta(h, m, (uint16_t)n);
    if (sp_put(usbd_cdc_txfifo(), SP_FRAME, h, hl, rec, (uint32_t)n) == 0) usbd_cdc_kick();   /* else: skipped */
}

static void try_send_meta(void) {
    if (!meta_pending) return;
    if (sp_put(usbd_cdc_txfifo(), SP_LOGMETA, meta_buf, SP_LOGMETA_LEN, 0, 0) == 0) meta_pending = 0;
    usbd_cdc_kick();
}

void stream_logmeta(uint32_t rate_hz, uint16_t vdiv_mv, uint8_t probe_x10, uint8_t coupling_dc,
                    int16_t zero_code, uint16_t offset_pwm) {
    sp_logmeta(meta_buf, rate_hz, vdiv_mv, probe_x10, coupling_dc, zero_code, offset_pwm);
    meta_pending = 1;
    try_send_meta();
}

void stream_log(const uint8_t *s, uint32_t n) {
    if (mode != STREAM_LOG) return;
    try_send_meta();
    if (meta_pending) { logger_lost(&lg, n); return; }   /* never label samples with a stale scale */
    logger_push(&lg, usbd_cdc_txfifo(), s, n);
    usbd_cdc_kick();
}

void stream_log_lost(uint32_t n) { if (mode == STREAM_LOG) logger_lost(&lg, n); }

void stream_stop_log(void) {
    if (mode != STREAM_LOG) return;
    hk->log_stop();
    mode = STREAM_IDLE;
    if (usbd_cdc_open()) send_info();
}

void stream_log_stats(uint32_t *sent, uint32_t *dropped) { *sent = lg.sent; *dropped = lg.dropped; }
