#pragma once
#include <stdint.h>
#include "bytefifo.h"
#include "csv.h"

/* Streaming protocol (spec § Sub-project 4): A5 5A | type | len u16 LE | payload | crc16 LE. */
enum { SP_INFO = 1, SP_FRAME = 2, SP_LOGMETA = 3, SP_LOG = 4, SP_DROP = 5, SP_OK = 6, SP_ERR = 7 };
#define SP_OVERHEAD    7u       /* sync 2 + type 1 + len 2 + crc 2 */
#define SP_FRAME_META  21u      /* FRAME meta bytes before the u16 sample count */
#define SP_LOGMETA_LEN 12u
#define SP_LOG_MAX     1024u    /* samples per LOG packet, at most */
#define SP_LINE_MAX    32
#define SP_NUM_RATES   9

extern const uint32_t sp_rates[SP_NUM_RATES];

/* CRC-16/CCITT-FALSE of p[0..n), continuing from crc (start with 0xFFFF). */
uint16_t sp_crc16(uint16_t crc, const uint8_t *p, uint32_t n);
/* Append one packet whose payload is a[0..alen) then b[0..blen) (either may be empty), all or nothing.
   Returns 0, or -1 if the payload exceeds 65535 bytes or f lacks alen + blen + SP_OVERHEAD bytes. */
int sp_put(bytefifo_t *f, uint8_t type, const uint8_t *a, uint32_t alen, const uint8_t *b, uint32_t blen);
/* FRAME meta + u16 sample count into out (SP_FRAME_META + 2 bytes); returns that length. */
uint32_t sp_frame_meta(uint8_t *out, const csv_meta_t *m, uint16_t n);
/* LOGMETA payload into out (SP_LOGMETA_LEN bytes); returns that length. */
uint32_t sp_logmeta(uint8_t *out, uint32_t rate_hz, uint16_t vdiv_mv, uint8_t probe_x10,
                    uint8_t coupling_dc, int16_t zero_code, uint16_t offset_pwm);
/* Two u32 LE into out (8 bytes; LOG header uses the first 4, DROP all 8). Returns 8. */
uint32_t sp_u32x2(uint8_t *out, uint32_t a, uint32_t b);
/* INFO text "fw=<FW_VERSION>\nmode=<mode>\nrates=<list>\n" into out; returns its length, 0 if cap too small. */
uint32_t sp_info(char *out, uint32_t cap, const char *mode);
int sp_rate_ok(uint32_t hz);

typedef enum { SP_CMD_NONE = 0, SP_CMD_INFO, SP_CMD_FRAMES, SP_CMD_LOG, SP_CMD_STOP, SP_CMD_BAD } sp_cmd_t;
typedef struct { char buf[SP_LINE_MAX + 1]; uint8_t n, overflow; } sp_line_t;   /* zero-initialise */
/* Feed one received byte. Returns 1 when a line is complete: l->buf holds it NUL-terminated without
   CR/LF; a line longer than SP_LINE_MAX comes back as "?" (which sp_parse reports as BAD). */
int sp_line_feed(sp_line_t *l, uint8_t c);
/* INFO? / FRAMES / STOP / LOG <decimal> (arg) / "" -> NONE / anything else -> BAD. */
sp_cmd_t sp_parse(const char *line, uint32_t *arg);
