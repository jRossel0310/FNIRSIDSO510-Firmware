# Live USB Streaming + Python Plotter (Sub-project 4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The scope becomes a composite USB device (existing drive + a CDC-ACM serial port). Over the serial port it streams either every capture ("frames" mode) or a continuous gap-accounted sample stream ("logger" mode, 1 k–500 kS/s). A Python tool plots and records it live.

**Architecture:** Pure logic, host-tested, lives in `fw/core/`:
- a lock-free byte FIFO
- the packet encoder, CRC and command parser
- logger chunking and drop accounting
- the DMA ring-half picker
- composite USB descriptors and the CDC class requests

The interrupt-driven USB driver gains the CDC endpoints and the two FIFOs. The ADC driver gains a circular DMA mode. A new `fw/app/stream.c` owns the streaming state machine. On the PC, `tools/dsoproto.py` is the protocol, volts and CSV library (stdlib only, unittest), `tools/scopeclient.py` is the serial reader with auto-reconnect, and `tools/scope_live.py` is the pyqtgraph GUI. Shared golden byte vectors in `tests/vectors/` tie the firmware encoder to the Python decoder.

**Tech Stack:** arm-none-eabi-gcc (Cortex-M4F), host `cc` tests, Python 3 (stdlib for library and tests; pyserial, pyqtgraph and PyQt5 only for the live tool).

**Spec:** `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` § "Sub-project 4: live USB streaming + Python plotter". It reuses the § "CSV format" and the sub-project 2 USB driver. Hardware facts are in `NOTES.md`.

## Global Constraints

- Never write or erase internal flash below 0x08007000. Never touch the factory page 0x08006000. SPI flash erase/program only inside `[0, 0x500000)`.
- Every app implements power-off: hold Power ~1 s, which calls `usbd_detach()` and then `power_off()`.
- `fw/core/*` includes no hardware header and compiles with host `cc -std=c99 -Wall -Wextra -Werror`.
- Firmware builds with 0 warnings: `make -C fw clean everything 2>&1 | grep -ciE "warning|error"` prints `0`. Host tests: `make -C tests` prints `… 0 failed`.
- USB: VID:PID `2e3c:5720` unchanged. The MSC interface (0, EP 0x81/0x01) must keep working exactly as before. CDC-ACM uses interfaces 1 and 2: EP 0x83 interrupt IN, and EP 0x82/0x02 bulk 64.
- Packet: `A5 5A | type | len u16 LE | payload | crc16 LE`, CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over type, len and payload. Types: 1 INFO, 2 FRAME, 3 LOGMETA, 4 LOG, 5 DROP, 6 OK, 7 ERR. All multi-byte fields are little-endian.
- Host commands: `INFO?`, `FRAMES`, `LOG <rate_hz>`, `STOP`, each terminated by `\n` and at most 32 characters.
- Logger rates (S/s): `1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000`.
- Streaming never blocks or slows capture. A frame that doesn't fit is skipped, and logger data that doesn't fit is counted and reported with DROP.
- `FW_VERSION` becomes `"0.3.0"`.
- The Python library and tests use the stdlib only. The tests run with `python3 -m unittest discover -s tools/tests -v`.
- Flashing: put the scope in update mode (hold the update button, press Power), then run `tools/flash.sh <bin>`.
- Commit after every task with targeted `git add` (never `-A`, never `docs/` photos or `.superpowers/`). Every commit ends with the line `Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp`.

## File Structure

| File | Responsibility |
|---|---|
| `fw/core/bytefifo.[ch]` | SPSC byte FIFO (one side may be an ISR) |
| `fw/core/units.[ch]` | + `units_rate_for_hz()` exact timer setting for a sample rate |
| `fw/core/stream_proto.[ch]` | CRC, packet put into FIFO, payload builders, command line parser, rate table |
| `fw/core/logger.[ch]` | LOG chunking + DROP accounting; DMA ring half picker |
| `fw/core/usb_std.[ch]` | composite descriptors (IAD), MSC/CDC class request routing, EP0 OUT data |
| `fw/core/csv.h` | `FW_VERSION` "0.3.0" |
| `fw/common/usbd.[ch]` | + EP2/EP3, EP0 OUT data stage, CDC FIFOs, DTR, CDC API |
| `fw/common/adc.[ch]` | + circular DMA ring mode |
| `fw/app/stream.[ch]` | streaming state machine: commands, port open/close, frames, logger output |
| `fw/app/ui.[ch]` | + `ui_draw_usb()`, `ui_draw_logger()` |
| `fw/app/main.c` | wire stream into the scope loop; logger mode loop |
| `fw/apps/cdctest/main.c` | on-device serial echo + throughput bench |
| `tests/test_bytefifo.c`, `test_stream_proto.c`, `test_logger.c`, `test_usb_std.c` | host tests |
| `tests/vectors/*` | golden packets + frame CSV (committed, shared with Python) |
| `tools/dsoproto.py` | decoder, packet types, volts, CSV writers |
| `tools/scopeclient.py` | serial reader thread, reconnect, commands |
| `tools/scope_live.py` | pyqtgraph GUI |
| `tools/requirements.txt`, `tools/README.md` | deps + usage |
| `tools/tests/test_dsoproto.py`, `tools/tests/test_scopeclient.py` | Python tests |

---

### Task 1: `bytefifo` and `units_rate_for_hz`

**Files:**
- Create: `fw/core/bytefifo.h`, `fw/core/bytefifo.c`, `tests/test_bytefifo.c`
- Modify: `fw/core/units.h`, `fw/core/units.c`, `tests/test_units.c`, `tests/main.c`

**Interfaces:**
- Produces: `bytefifo_t`, `bf_init`, `bf_used`, `bf_space`, `bf_write`, `bf_read` and `bf_clear`; also `rate_cfg_t units_rate_for_hz(uint32_t hz)`.

- [ ] **Step 1: Write the failing tests**

`tests/test_bytefifo.c`:
```c
#include "test.h"
#include "bytefifo.h"

void test_bytefifo(void) {
    static uint8_t mem[8];
    bytefifo_t f;
    uint8_t out[16];
    bf_init(&f, mem, sizeof mem);
    CHECK_EQ(bf_used(&f), 0); CHECK_EQ(bf_space(&f), 8);
    const uint8_t a[5] = {1, 2, 3, 4, 5};
    CHECK_EQ(bf_write(&f, a, 5), 5);
    CHECK_EQ(bf_used(&f), 5); CHECK_EQ(bf_space(&f), 3);
    CHECK_EQ(bf_read(&f, out, 3), 3);
    CHECK_EQ(out[0], 1); CHECK_EQ(out[2], 3);
    const uint8_t b[6] = {6, 7, 8, 9, 10, 11};
    CHECK_EQ(bf_write(&f, b, 6), 6);                  /* wraps around the end */
    CHECK_EQ(bf_used(&f), 8); CHECK_EQ(bf_space(&f), 0);
    CHECK_EQ(bf_write(&f, a, 1), 0);                  /* full: nothing written */
    CHECK_EQ(bf_read(&f, out, 16), 8);                /* only what is there */
    CHECK_EQ(out[0], 4); CHECK_EQ(out[1], 5); CHECK_EQ(out[2], 6); CHECK_EQ(out[7], 11);
    CHECK_EQ(bf_read(&f, out, 4), 0);
    CHECK_EQ(bf_write(&f, b, 6), 6);
    CHECK_EQ(bf_write(&f, a, 5), 2);                  /* partial: only what fits */
    bf_clear(&f);
    CHECK_EQ(bf_used(&f), 0); CHECK_EQ(bf_space(&f), 8);
    /* free-running counters wrap past 2^32 */
    f.head = f.tail = 0xFFFFFFFCu;
    CHECK_EQ(bf_write(&f, b, 6), 6);
    CHECK_EQ(bf_used(&f), 6);
    CHECK_EQ(bf_read(&f, out, 6), 6);
    CHECK_EQ(out[0], 6); CHECK_EQ(out[5], 11);
    CHECK_EQ(bf_used(&f), 0);
}
```

Append to `tests/test_units.c` inside `test_units()` (before its closing brace):
```c
    /* exact timer settings for logger rates: 240 MHz / ((psc+1)(arr+1)) */
    rate_cfg_t q = units_rate_for_hz(500000);
    CHECK_EQ(q.rate_hz, 500000); CHECK_EQ(q.psc, 0); CHECK_EQ(q.arr, 479);
    q = units_rate_for_hz(10000);
    CHECK_EQ(q.rate_hz, 10000); CHECK_EQ(q.psc, 0); CHECK_EQ(q.arr, 23999);
    q = units_rate_for_hz(2000);                      /* 120000 ticks > 65536: psc 1 */
    CHECK_EQ(q.rate_hz, 2000); CHECK_EQ(q.psc, 1); CHECK_EQ(q.arr, 59999);
    q = units_rate_for_hz(1000);
    CHECK_EQ(q.rate_hz, 1000); CHECK_EQ(q.psc, 3); CHECK_EQ(q.arr, 59999);
    q = units_rate_for_hz(7);                          /* 240e6 not divisible by 7 */
    CHECK_EQ(q.rate_hz, 0);
    q = units_rate_for_hz(0);
    CHECK_EQ(q.rate_hz, 0);
```

Register `test_bytefifo` in `tests/main.c` (declaration + call).

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile error, `bytefifo.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/bytefifo.h`:
```c
#pragma once
#include <stdint.h>
/* Single-producer / single-consumer byte FIFO; one side may run in an interrupt. The producer only
   writes `head`, the consumer only writes `tail` (free-running counters, size a power of two). */
typedef struct {
    uint8_t *buf;
    uint32_t mask;              /* size - 1 */
    volatile uint32_t head;     /* total bytes ever written */
    volatile uint32_t tail;     /* total bytes ever read */
} bytefifo_t;

void bf_init(bytefifo_t *f, uint8_t *buf, uint32_t size);    /* size must be a power of two */
uint32_t bf_used(const bytefifo_t *f);
uint32_t bf_space(const bytefifo_t *f);
/* Producer: append up to n bytes (only what fits). Returns bytes written. */
uint32_t bf_write(bytefifo_t *f, const uint8_t *p, uint32_t n);
/* Consumer: remove up to max bytes. Returns bytes read. */
uint32_t bf_read(bytefifo_t *f, uint8_t *p, uint32_t max);
/* Consumer: discard everything currently queued. */
void bf_clear(bytefifo_t *f);
```

`fw/core/bytefifo.c`:
```c
#include "bytefifo.h"

#define BF_BARRIER() __asm__ __volatile__("" ::: "memory")   /* data before the index update */

void bf_init(bytefifo_t *f, uint8_t *buf, uint32_t size) {
    f->buf = buf; f->mask = size - 1u; f->head = 0; f->tail = 0;
}

uint32_t bf_used(const bytefifo_t *f) { return f->head - f->tail; }
uint32_t bf_space(const bytefifo_t *f) { return f->mask + 1u - (f->head - f->tail); }

uint32_t bf_write(bytefifo_t *f, const uint8_t *p, uint32_t n) {
    uint32_t s = bf_space(f), h = f->head;
    if (n > s) n = s;
    for (uint32_t i = 0; i < n; i++) f->buf[(h + i) & f->mask] = p[i];
    BF_BARRIER();
    f->head = h + n;
    return n;
}

uint32_t bf_read(bytefifo_t *f, uint8_t *p, uint32_t max) {
    uint32_t u = bf_used(f), t = f->tail;
    if (max > u) max = u;
    for (uint32_t i = 0; i < max; i++) p[i] = f->buf[(t + i) & f->mask];
    BF_BARRIER();
    f->tail = t + max;
    return max;
}

void bf_clear(bytefifo_t *f) { f->tail = f->head; }
```

In `fw/core/units.h`, after the `units_rate_for_tdiv` declaration, add:
```c
/* Exact timer setting for a sample rate: TIMER_CLK_HZ / ((psc+1)(arr+1)) == hz.
   rate_hz = 0 if hz is 0 or no exact setting exists. */
rate_cfg_t units_rate_for_hz(uint32_t hz);
```
In `fw/core/units.c` append:
```c
rate_cfg_t units_rate_for_hz(uint32_t hz) {
    rate_cfg_t r = {0, 0, 0};
    if (hz == 0 || TIMER_CLK_HZ % hz) return r;
    uint32_t div = TIMER_CLK_HZ / hz;
    for (uint32_t p = 1; p <= 65536u; p++) {
        if (div % p == 0 && div / p <= 65536u) {
            r.psc = (uint16_t)(p - 1u); r.arr = (uint16_t)(div / p - 1u); r.rate_hz = hz;
            return r;
        }
    }
    return r;
}
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `make -C tests` and `make -C fw everything 2>&1 | grep -ciE "warning|error"`
Expected: `… 0 failed` and `0`.

- [ ] **Step 5: Commit**

```bash
git add fw/core/bytefifo.h fw/core/bytefifo.c fw/core/units.h fw/core/units.c tests/test_bytefifo.c tests/test_units.c tests/main.c
git commit -m "core: SPSC byte FIFO; exact timer setting for a sample rate

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 2: `stream_proto` (packets, CRC, commands) + golden vectors

**Files:**
- Create: `fw/core/stream_proto.h`, `fw/core/stream_proto.c`, `tests/test_stream_proto.c`, `tests/vectors/*.bin`, `tests/vectors/frame.csv`
- Modify: `fw/core/csv.h` (FW_VERSION), `tests/test_csv.c` (version literal), `tests/Makefile` (`vectors` target), `tests/main.c`

**Interfaces:**
- Consumes: `bytefifo_t`, `bf_*` (Task 1); `csv_meta_t`, `csv_header` and `csv_row` (sub-project 2).
- Produces: the `SP_*` constants, `sp_crc16`, `sp_put`, `sp_frame_meta`, `sp_logmeta`, `sp_u32x2`, `sp_line_t`, `sp_line_feed`, `sp_cmd_t`, `sp_parse`, `sp_rates[]`, `sp_rate_ok`, `sp_info`, and the vector files.

LOGMETA layout. This is an addition to the spec. Ruling: it carries `offset_pwm` so the PC CSV's `cal=` line matches the scope's. The fields are `rate_hz u32, vdiv_mv u16, probe_x10 u8, coupling_dc u8, zero_code i16, offset_pwm u16`, 12 bytes. Update the spec's protocol table row for LOGMETA in this task.

FRAME payload: 21 bytes of meta (`rate_hz u32, vdiv_mv u16, coupling_dc u8, probe_x10 u8, trig_mode u8, edge u8, triggered u8, zero_code i16, trig_level_code i16, trig_index i32, offset_pwm u16`), then `n u16`, then the n samples.

- [ ] **Step 1: Bump the version**

In `fw/core/csv.h` set `#define FW_VERSION "0.3.0"`. Run `grep -rnF "0.2.0" tests fw/core` and change every test literal `0.2.0` to `0.3.0`, for example the header expectation in `tests/test_csv.c`.

- [ ] **Step 2: Write the failing test**

`tests/test_stream_proto.c`:
```c
#include "test.h"
#include "stream_proto.h"
#include <stdlib.h>

static uint8_t mem[8192];
static bytefifo_t F;
static uint8_t out[8192];

/* Canonical inputs, also hard-coded in tools/tests/test_dsoproto.py. */
static const uint8_t FRAME_S[8] = {46, 71, 96, 121, 146, 121, 96, 71};
static csv_meta_t frame_meta(void) {
    csv_meta_t m;
    memset(&m, 0, sizeof m);
    m.rate_hz = 600000; m.vdiv_mv = 500; m.coupling_dc = 1; m.probe_x10 = 0; m.trig_mode = 0;
    m.edge = 0; m.triggered = 1; m.zero_code = 46; m.trig_level_code = 75; m.trig_index = 3;
    m.offset_pwm = 1630;
    return m;
}

static uint32_t drain(void) { return bf_read(&F, out, sizeof out); }

/* Compare out[0..n) with tests/vectors/<name>; with WRITE_VECTORS=1 in the environment, write it. */
static void vector(const char *name, const uint8_t *p, uint32_t n) {
    char path[96];
    snprintf(path, sizeof path, "vectors/%s", name);
    if (getenv("WRITE_VECTORS")) {
        FILE *f = fopen(path, "wb");
        CHECK(f != 0);
        if (f) { fwrite(p, 1, n, f); fclose(f); }
        return;
    }
    static uint8_t ref[16384];
    FILE *f = fopen(path, "rb");
    CHECK(f != 0);
    if (!f) return;
    size_t m = fread(ref, 1, sizeof ref, f);
    fclose(f);
    CHECK_EQ(m, n);
    CHECK(m == n && memcmp(ref, p, n) == 0);
}

static uint32_t put_info(void) {
    char t[160];
    uint32_t n = sp_info(t, sizeof t, "idle");
    CHECK_EQ(sp_put(&F, SP_INFO, (const uint8_t *)t, n, 0, 0), 0);
    return drain();
}
static uint32_t put_frame(void) {
    uint8_t h[SP_FRAME_META + 2];
    csv_meta_t m = frame_meta();
    uint32_t hl = sp_frame_meta(h, &m, 8);
    CHECK_EQ(hl, SP_FRAME_META + 2);
    CHECK_EQ(sp_put(&F, SP_FRAME, h, hl, FRAME_S, 8), 0);
    return drain();
}
static uint32_t put_logmeta(void) {
    uint8_t h[SP_LOGMETA_LEN];
    uint32_t hl = sp_logmeta(h, 100000, 1000, 0, 1, 128, 2033);
    CHECK_EQ(hl, SP_LOGMETA_LEN);
    CHECK_EQ(sp_put(&F, SP_LOGMETA, h, hl, 0, 0), 0);
    return drain();
}
static uint32_t put_log(uint8_t flip) {
    uint8_t h[8], s[16];
    sp_u32x2(h, 1000, 0);
    for (int i = 0; i < 16; i++) s[i] = (uint8_t)(120 + i);
    CHECK_EQ(sp_put(&F, SP_LOG, h, 4, s, 16), 0);
    uint32_t n = drain();
    out[9] ^= flip;                                    /* corrupt one payload byte if asked */
    return n;
}
static uint32_t put_drop(void) {
    uint8_t h[8];
    sp_u32x2(h, 2048, 512);
    CHECK_EQ(sp_put(&F, SP_DROP, h, 8, 0, 0), 0);
    return drain();
}
static uint32_t put_text(uint8_t type, const char *t) {
    CHECK_EQ(sp_put(&F, type, (const uint8_t *)t, (uint32_t)strlen(t), 0, 0), 0);
    return drain();
}

void test_stream_proto(void) {
    bf_init(&F, mem, sizeof mem);
    /* CRC-16/CCITT-FALSE check value */
    CHECK_EQ(sp_crc16(0xFFFF, (const uint8_t *)"123456789", 9), 0x29B1);

    /* exact framing of a tiny packet: A5 5A 06 02 00 'O' 'K' crc_lo crc_hi */
    uint32_t n = put_text(SP_OK, "OK");
    CHECK_EQ(n, 9);
    CHECK_EQ(out[0], 0xA5); CHECK_EQ(out[1], 0x5A); CHECK_EQ(out[2], SP_OK);
    CHECK_EQ(out[3], 2); CHECK_EQ(out[4], 0); CHECK_EQ(out[5], 'O'); CHECK_EQ(out[6], 'K');
    uint16_t c = sp_crc16(0xFFFF, out + 2, 5);
    CHECK_EQ(out[7], c & 0xFF); CHECK_EQ(out[8], c >> 8);

    /* all or nothing */
    static uint8_t small_mem[16];
    bytefifo_t s;
    bf_init(&s, small_mem, sizeof small_mem);
    uint8_t big[10] = {0};
    CHECK_EQ(sp_put(&s, SP_LOG, big, 10, 0, 0), -1);   /* needs 17 > 16 */
    CHECK_EQ(bf_used(&s), 0);
    CHECK_EQ(sp_put(&s, SP_LOG, big, 9, 0, 0), 0);     /* exactly 16 */
    CHECK_EQ(bf_used(&s), 16);

    /* golden vectors */
    vector("info.bin", out, put_info());
    vector("frame.bin", out, put_frame());
    vector("logmeta.bin", out, put_logmeta());
    vector("log.bin", out, put_log(0));
    vector("drop.bin", out, put_drop());
    vector("ok.bin", out, put_text(SP_OK, "LOG 100000"));
    vector("err.bin", out, put_text(SP_ERR, "bad rate"));
    /* a stream: junk, then every packet, with one corrupted LOG before the good one */
    static uint8_t st[4096];
    uint32_t k = 0;
    st[k++] = 0x00; st[k++] = 0xA5; st[k++] = 0x13;
    n = put_info(); memcpy(st + k, out, n); k += n;
    n = put_frame(); memcpy(st + k, out, n); k += n;
    n = put_log(0x40); memcpy(st + k, out, n); k += n;
    n = put_log(0); memcpy(st + k, out, n); k += n;
    n = put_drop(); memcpy(st + k, out, n); k += n;
    n = put_text(SP_OK, "LOG 100000"); memcpy(st + k, out, n); k += n;
    n = put_text(SP_ERR, "bad rate"); memcpy(st + k, out, n); k += n;
    vector("stream.bin", st, k);
    /* the firmware's CSV of the frame, for the Python CSV writer test */
    static char csvt[2048];
    csv_meta_t m = frame_meta();
    int cl = csv_header(csvt, sizeof csvt, &m);
    CHECK(cl > 0);
    for (int i = 0; i < 8; i++) cl += csv_row(csvt + cl, &m, i, FRAME_S[i]);
    vector("frame.csv", (const uint8_t *)csvt, (uint32_t)cl);

    /* info text */
    char t[160];
    sp_info(t, sizeof t, "log");
    CHECK(strstr(t, "fw=" FW_VERSION "\n") == t);
    CHECK(strstr(t, "mode=log\n") != 0);
    CHECK(strstr(t, "rates=1000,2000,5000,10000,20000,50000,100000,200000,500000\n") != 0);
    CHECK_EQ(sp_info(t, 10, "idle"), 0);                   /* too small */

    /* command lines */
    sp_line_t l;
    memset(&l, 0, sizeof l);
    const char *in = "INFO?\r\nLOG 200000\nSTOP\nFRAMES\nfoo\n\nLOG x\nLOG\n";
    sp_cmd_t got[10]; uint32_t args[10]; int ng = 0;
    for (const char *p = in; *p; p++)
        if (sp_line_feed(&l, (uint8_t)*p) && ng < 10) { got[ng] = sp_parse(l.buf, &args[ng]); ng++; }
    CHECK_EQ(ng, 8);
    CHECK_EQ(got[0], SP_CMD_INFO);
    CHECK_EQ(got[1], SP_CMD_LOG); CHECK_EQ(args[1], 200000);
    CHECK_EQ(got[2], SP_CMD_STOP);
    CHECK_EQ(got[3], SP_CMD_FRAMES);
    CHECK_EQ(got[4], SP_CMD_BAD);
    CHECK_EQ(got[5], SP_CMD_NONE);                          /* empty line ignored */
    CHECK_EQ(got[6], SP_CMD_BAD);
    CHECK_EQ(got[7], SP_CMD_BAD);
    /* an over-long line is reported once as BAD, then parsing recovers */
    memset(&l, 0, sizeof l);
    int lines = 0; sp_cmd_t last = SP_CMD_NONE; uint32_t a;
    for (int i = 0; i < 50; i++) if (sp_line_feed(&l, 'A')) lines++;
    if (sp_line_feed(&l, '\n')) { lines++; last = sp_parse(l.buf, &a); }
    CHECK_EQ(lines, 1); CHECK_EQ(last, SP_CMD_BAD);
    const char *ok = "STOP\n";
    for (const char *p = ok; *p; p++) if (sp_line_feed(&l, (uint8_t)*p)) last = sp_parse(l.buf, &a);
    CHECK_EQ(last, SP_CMD_STOP);

    /* rates */
    CHECK_EQ(sp_rate_ok(100000), 1); CHECK_EQ(sp_rate_ok(500000), 1); CHECK_EQ(sp_rate_ok(1000), 1);
    CHECK_EQ(sp_rate_ok(3000), 0); CHECK_EQ(sp_rate_ok(0), 0); CHECK_EQ(sp_rate_ok(1000000), 0);
}
```
Register `test_stream_proto` in `tests/main.c`.

Add to `tests/Makefile`:
```make
vectors: build/tests
	mkdir -p vectors
	WRITE_VECTORS=1 ./build/tests
```
Add `vectors` to `.PHONY`.

- [ ] **Step 3: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile error, `stream_proto.h: No such file or directory`.

- [ ] **Step 4: Implement**

`fw/core/stream_proto.h`:
```c
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
```

`fw/core/stream_proto.c`:
```c
#include "stream_proto.h"

const uint32_t sp_rates[SP_NUM_RATES] = {1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000};

uint16_t sp_crc16(uint16_t crc, const uint8_t *p, uint32_t n) {
    while (n--) {
        crc ^= (uint16_t)(*p++ << 8);
        for (int i = 0; i < 8; i++) crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

int sp_put(bytefifo_t *f, uint8_t type, const uint8_t *a, uint32_t alen, const uint8_t *b, uint32_t blen) {
    uint32_t len = alen + blen;
    if (len > 65535u || bf_space(f) < len + SP_OVERHEAD) return -1;
    uint8_t h[5] = {0xA5, 0x5A, type, (uint8_t)len, (uint8_t)(len >> 8)};
    uint16_t crc = sp_crc16(0xFFFF, h + 2, 3);
    crc = sp_crc16(crc, a, alen);
    crc = sp_crc16(crc, b, blen);
    uint8_t t[2] = {(uint8_t)crc, (uint8_t)(crc >> 8)};
    bf_write(f, h, 5);
    bf_write(f, a, alen);
    bf_write(f, b, blen);
    bf_write(f, t, 2);
    return 0;
}

static uint8_t *le16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); return p + 2; }
static uint8_t *le32(uint8_t *p, uint32_t v) { p = le16(p, v & 0xFFFFu); return le16(p, v >> 16); }

uint32_t sp_frame_meta(uint8_t *out, const csv_meta_t *m, uint16_t n) {
    uint8_t *p = le32(out, m->rate_hz);
    p = le16(p, m->vdiv_mv);
    *p++ = m->coupling_dc; *p++ = m->probe_x10; *p++ = m->trig_mode; *p++ = m->edge; *p++ = m->triggered;
    p = le16(p, (uint16_t)m->zero_code);
    p = le16(p, (uint16_t)m->trig_level_code);
    p = le32(p, (uint32_t)m->trig_index);
    p = le16(p, m->offset_pwm);
    p = le16(p, n);
    return (uint32_t)(p - out);
}

uint32_t sp_logmeta(uint8_t *out, uint32_t rate_hz, uint16_t vdiv_mv, uint8_t probe_x10,
                    uint8_t coupling_dc, int16_t zero_code, uint16_t offset_pwm) {
    uint8_t *p = le32(out, rate_hz);
    p = le16(p, vdiv_mv);
    *p++ = probe_x10; *p++ = coupling_dc;
    p = le16(p, (uint16_t)zero_code);
    p = le16(p, offset_pwm);
    return (uint32_t)(p - out);
}

uint32_t sp_u32x2(uint8_t *out, uint32_t a, uint32_t b) { le32(le32(out, a), b); return 8; }

static uint32_t put_s(char *out, uint32_t cap, uint32_t k, const char *s) {
    while (*s) { if (k + 1 >= cap) return cap; out[k++] = *s++; }
    return k;
}
static uint32_t put_u(char *out, uint32_t cap, uint32_t k, uint32_t v) {
    char t[12]; int n = 0;
    do { t[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    while (n--) { if (k + 1 >= cap) return cap; out[k++] = t[n]; }
    return k;
}

uint32_t sp_info(char *out, uint32_t cap, const char *mode) {
    uint32_t k = put_s(out, cap, 0, "fw=" FW_VERSION "\nmode=");
    k = put_s(out, cap, k, mode);
    k = put_s(out, cap, k, "\nrates=");
    for (int i = 0; i < SP_NUM_RATES && k < cap; i++) {
        if (i) k = put_s(out, cap, k, ",");
        k = put_u(out, cap, k, sp_rates[i]);
    }
    k = put_s(out, cap, k, "\n");
    if (k >= cap) return 0;
    out[k] = 0;
    return k;
}

int sp_rate_ok(uint32_t hz) {
    for (int i = 0; i < SP_NUM_RATES; i++) if (sp_rates[i] == hz) return 1;
    return 0;
}

int sp_line_feed(sp_line_t *l, uint8_t c) {
    if (c == '\r') return 0;
    if (c == '\n') {
        if (l->overflow) { l->buf[0] = '?'; l->buf[1] = 0; }
        else l->buf[l->n] = 0;
        l->n = 0; l->overflow = 0;
        return 1;
    }
    if (l->n < SP_LINE_MAX) l->buf[l->n++] = (char)c; else l->overflow = 1;
    return 0;
}

static int eq(const char *a, const char *b) { while (*a && *a == *b) { a++; b++; } return *a == *b; }

sp_cmd_t sp_parse(const char *s, uint32_t *arg) {
    *arg = 0;
    if (!s[0]) return SP_CMD_NONE;
    if (eq(s, "INFO?")) return SP_CMD_INFO;
    if (eq(s, "FRAMES")) return SP_CMD_FRAMES;
    if (eq(s, "STOP")) return SP_CMD_STOP;
    if (s[0] == 'L' && s[1] == 'O' && s[2] == 'G' && s[3] == ' ' && s[4]) {
        uint32_t v = 0;
        for (const char *p = s + 4; *p; p++) {
            if (*p < '0' || *p > '9' || v > 100000000u) return SP_CMD_BAD;
            v = v * 10 + (uint32_t)(*p - '0');
        }
        *arg = v;
        return SP_CMD_LOG;
    }
    return SP_CMD_BAD;
}
```

- [ ] **Step 5: Generate the vectors, then run the tests**

Run: `make -C tests vectors && make -C tests`
Expected: `vectors` writes `tests/vectors/{info,frame,logmeta,log,drop,ok,err,stream}.bin` and `frame.csv`. The plain run then prints `… 0 failed`, with the vectors compared rather than written. Inspect `xxd tests/vectors/ok.bin`; it must start `a5 5a 06 0a 00 4c 4f 47`.

- [ ] **Step 6: Update the spec's LOGMETA row**

In `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` § Sub-project 4 Protocol table, change the LOGMETA payload to `rate_hz u32, vdiv_mv u16, probe_x10 u8, coupling_dc u8, zero_code i16, offset_pwm u16`.

- [ ] **Step 7: Firmware build**

Run: `make -C fw everything 2>&1 | grep -ciE "warning|error"`
Expected: `0`.

- [ ] **Step 8: Commit**

```bash
git add fw/core/stream_proto.h fw/core/stream_proto.c fw/core/csv.h tests/test_stream_proto.c tests/test_csv.c tests/Makefile tests/main.c tests/vectors docs/superpowers/specs/2026-11-13-dso510-firmware-design.md
git commit -m "core: streaming packet encoder, CRC-16, command parser; golden vectors; FW 0.3.0

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 3: `logger` — LOG chunking, DROP accounting, ring-half picker

**Files:**
- Create: `fw/core/logger.h`, `fw/core/logger.c`, `tests/test_logger.c`
- Modify: `tests/main.c`

**Interfaces:**
- Consumes: `bytefifo_t`, `bf_*` (Task 1); `sp_put`, `sp_u32x2`, `SP_LOG`, `SP_DROP`, `SP_LOG_MAX`, `SP_OVERHEAD` (Task 2).
- Produces: `logger_t`, `logger_init`, `logger_push`, `logger_lost`, `ring_pick_t`, `logger_pick`.

Semantics:
- `index` counts every sample since logger start, including lost ones.
- A LOG packet's `first_index` is the index of its first sample.
- `DROP(first_index, count)` means "count samples were lost immediately before sample first_index". It is emitted right before the next LOG.
- If a DROP+LOG pair doesn't fit in the FIFO, that chunk is lost too, and its samples are added to the pending drop.
- `logger_pick` decides which DMA ring halves are complete from the half-transfer (HT) and transfer-complete (TC) flags, plus the DMA's remaining count.

- [ ] **Step 1: Write the failing test**

`tests/test_logger.c`:
```c
#include "test.h"
#include "logger.h"

static uint8_t mem[4096];
static bytefifo_t F;

typedef struct { int type; uint32_t a, b; uint32_t n; uint8_t first; } pk_t;

/* Pop one packet from F (assumes well-formed). Returns 0 if the FIFO is empty. */
static int pop(pk_t *p) {
    uint8_t h[5];
    if (bf_used(&F) < 5) return 0;
    bf_read(&F, h, 5);
    uint32_t len = (uint32_t)(h[3] | h[4] << 8);
    static uint8_t pl[2048];
    bf_read(&F, pl, len + 2);
    p->type = h[2];
    p->a = pl[0] | pl[1] << 8 | pl[2] << 16 | (uint32_t)pl[3] << 24;
    p->b = len >= 8 ? (pl[4] | pl[5] << 8 | pl[6] << 16 | (uint32_t)pl[7] << 24) : 0;
    p->n = p->type == SP_LOG ? len - 4 : 0;
    p->first = p->type == SP_LOG ? pl[4] : 0;
    return 1;
}

void test_logger(void) {
    static uint8_t s[3000];
    for (int i = 0; i < 3000; i++) s[i] = (uint8_t)i;
    logger_t L;
    pk_t p;
    bf_init(&F, mem, sizeof mem);
    logger_init(&L);

    logger_push(&L, &F, s, 100);                         /* one LOG, index 0 */
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 0); CHECK_EQ(p.n, 100); CHECK_EQ(p.first, 0);
    CHECK(!pop(&p));
    CHECK_EQ(L.index, 100); CHECK_EQ(L.sent, 100); CHECK_EQ(L.dropped, 0);

    logger_push(&L, &F, s, 2500);                         /* chunks of <= SP_LOG_MAX */
    CHECK(pop(&p)); CHECK_EQ(p.a, 100); CHECK_EQ(p.n, 1024);
    CHECK(pop(&p)); CHECK_EQ(p.a, 1124); CHECK_EQ(p.n, 1024);
    CHECK(pop(&p)); CHECK_EQ(p.a, 2148); CHECK_EQ(p.n, 452);
    CHECK(!pop(&p));
    CHECK_EQ(L.index, 2600);

    /* FIFO nearly full: the chunk is lost and counted */
    uint8_t fill[4000] = {0};
    bf_write(&F, fill, 4000);                             /* 96 bytes left */
    logger_push(&L, &F, s, 200);
    CHECK_EQ(bf_used(&F), 4000);
    CHECK_EQ(L.index, 2800); CHECK_EQ(L.dropped, 200); CHECK_EQ(L.pending_drop, 200);
    uint8_t junk[4000];
    bf_read(&F, junk, 4000);
    /* next push: DROP(first_index = 2800, count = 200) then LOG at 2800 */
    logger_push(&L, &F, s, 50);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, 2800); CHECK_EQ(p.b, 200);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 2800); CHECK_EQ(p.n, 50);
    CHECK_EQ(L.pending_drop, 0);

    /* samples lost in the DMA ring */
    logger_lost(&L, 4096);
    CHECK_EQ(L.index, 2850 + 4096); CHECK_EQ(L.dropped, 200 + 4096);
    logger_push(&L, &F, s, 10);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_DROP); CHECK_EQ(p.a, 6946); CHECK_EQ(p.b, 4096);
    CHECK(pop(&p)); CHECK_EQ(p.type, SP_LOG); CHECK_EQ(p.a, 6946); CHECK_EQ(p.n, 10);

    /* ring half picker: n = 1000, halves of 500. flags bit0 = HT, bit1 = TC */
    ring_pick_t r = logger_pick(0, 800, 1000);
    CHECK_EQ(r.first, -1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0);
    r = logger_pick(1, 400, 1000);                         /* HT only: first half done */
    CHECK_EQ(r.first, 0); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0);
    r = logger_pick(2, 900, 1000);                         /* TC only: second half done */
    CHECK_EQ(r.first, 1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 0);
    r = logger_pick(3, 900, 1000);                         /* both, DMA in first half: first is being overwritten */
    CHECK_EQ(r.first, 1); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 500);
    r = logger_pick(3, 300, 1000);                         /* both, DMA in second half: second is being overwritten */
    CHECK_EQ(r.first, 0); CHECK_EQ(r.second, -1); CHECK_EQ(r.lost, 500);
}
```
In this test, `remaining` is the DMA's count of transfers still to go. A remaining count greater than n/2 means the DMA is writing the first half. Register `test_logger` in `tests/main.c`.

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile error, `logger.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/logger.h`:
```c
#pragma once
#include <stdint.h>
#include "bytefifo.h"
#include "stream_proto.h"

/* Continuous logger output: LOG packets of <= SP_LOG_MAX samples; samples that can't be queued are
   counted and reported as DROP(first_index, count) right before the next LOG. */
typedef struct {
    uint32_t index;          /* running sample index (includes lost samples) */
    uint32_t pending_drop;   /* lost samples not yet reported */
    uint32_t sent, dropped;  /* totals */
} logger_t;

void logger_init(logger_t *l);
/* Queue n consecutive samples (the next ones after l->index). */
void logger_push(logger_t *l, bytefifo_t *f, const uint8_t *s, uint32_t n);
/* n samples were lost before reaching the logger (DMA ring overrun). */
void logger_lost(logger_t *l, uint32_t n);

/* Which DMA ring halves to take, oldest first: 0 = first half, 1 = second half, -1 = none.
   lost = samples overwritten before they could be taken. flags: bit0 HT, bit1 TC (as read and
   cleared); remaining = the DMA's transfers still to go in this pass; n = ring length (even). */
typedef struct { int first, second; uint32_t lost; } ring_pick_t;
ring_pick_t logger_pick(int flags, uint32_t remaining, uint32_t n);
```

`fw/core/logger.c`:
```c
#include "logger.h"

void logger_init(logger_t *l) { l->index = 0; l->pending_drop = 0; l->sent = 0; l->dropped = 0; }

void logger_lost(logger_t *l, uint32_t n) { l->index += n; l->pending_drop += n; l->dropped += n; }

void logger_push(logger_t *l, bytefifo_t *f, const uint8_t *s, uint32_t n) {
    while (n) {
        uint32_t c = n > SP_LOG_MAX ? SP_LOG_MAX : n;
        uint32_t need = 4 + c + SP_OVERHEAD + (l->pending_drop ? 8 + SP_OVERHEAD : 0);
        if (bf_space(f) < need) {
            logger_lost(l, c);                        /* this chunk is lost too */
        } else {
            uint8_t h[8];
            if (l->pending_drop) {
                sp_u32x2(h, l->index, l->pending_drop);
                sp_put(f, SP_DROP, h, 8, 0, 0);
                l->pending_drop = 0;
            }
            sp_u32x2(h, l->index, 0);
            sp_put(f, SP_LOG, h, 4, s, c);
            l->index += c; l->sent += c;
        }
        s += c; n -= c;
    }
}

ring_pick_t logger_pick(int flags, uint32_t remaining, uint32_t n) {
    ring_pick_t r = {-1, -1, 0};
    int ht = flags & 1, tc = (flags >> 1) & 1;
    int in_first = remaining > n / 2;                  /* DMA now writing the first half */
    if (ht && tc) {                                    /* late: one half is being overwritten */
        r.first = in_first ? 1 : 0;
        r.lost = n / 2;
    } else if (ht) r.first = 0;
    else if (tc) r.first = 1;
    return r;
}
```
`second` is reserved for an order where both halves are intact. That can't happen with one flag pair, so it stays -1. Keep the field: the test checks it, and Task 8 iterates over both fields.

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `make -C tests` and `make -C fw everything 2>&1 | grep -ciE "warning|error"`
Expected: `… 0 failed` and `0`.

- [ ] **Step 5: Commit**

```bash
git add fw/core/logger.h fw/core/logger.c tests/test_logger.c tests/main.c
git commit -m "core: logger chunking + DROP accounting, DMA ring half picker

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 4: `usb_std` — composite descriptors + CDC class requests

**Files:**
- Modify: `fw/core/usb_std.h`, `fw/core/usb_std.c`, `tests/test_usb_std.c`

**Interfaces:**
- Produces:
  - `USB_DATA_OUT` (a new `usb_act_t` value). The driver must receive `r.len` bytes on EP0 OUT, hand them to `usb_std_data_out()`, then send a zero-length status IN.
  - New `usb_resp_t.dtr_changed`.
  - New `usb_dev_t` fields `line_coding[7]` and `dtr`.
  - `void usb_std_data_out(usb_dev_t *d, const uint8_t *p, uint16_t n)`.
  - Interface numbers: MSC = 0, CDC comm = 1, CDC data = 2.

Configuration descriptor, 98 bytes in this exact order:
1. config: `9,2,98,0,3,1,0,0x80,50`
2. MSC interface 0: `9,4,0,0,2,0x08,0x06,0x50,0`
3. EP 0x81: `7,5,0x81,2,64,0,0`
4. EP 0x01: `7,5,0x01,2,64,0,0`
5. IAD: `8,0x0B,1,2,0x02,0x02,0x01,0`
6. comm interface 1: `9,4,1,0,1,0x02,0x02,0x01,0`
7. header: `5,0x24,0x00,0x10,0x01`
8. call management: `5,0x24,0x01,0x00,2`
9. ACM: `4,0x24,0x02,0x02`
10. union: `5,0x24,0x06,1,2`
11. EP 0x83 interrupt: `7,5,0x83,3,8,0,255`
12. data interface 2: `9,4,2,0,2,0x0A,0,0,0`
13. EP 0x02: `7,5,0x02,2,64,0,0`
14. EP 0x82: `7,5,0x82,2,64,0,0`

Device descriptor class bytes become `0xEF,0x02,0x01` (Miscellaneous / IAD), and bcdDevice becomes `0x0300`.

Class requests are routed by interface (`wIndex` low byte):
- Interface 0:
  - `0xA1 0xFE`: GET_MAX_LUN, returns 1 byte 0.
  - `0x21 0xFF`: BOT reset.
- Interface 1:
  - `0x21 0x20` SET_LINE_CODING: `USB_DATA_OUT`, len = min(wLength, 7).
  - `0xA1 0x21` GET_LINE_CODING: returns the 7 bytes.
  - `0x21 0x22` SET_CONTROL_LINE_STATE: `dtr = wValue & 1`, `dtr_changed = 1`, status IN.
  - `0x21 0x23` SEND_BREAK: status IN.
- Anything else: stall.

The default line coding is 115200 8N1 (`00 C2 01 00 00 00 08`). SET_CONFIGURATION and `usb_std_init` both clear `dtr`.

- [ ] **Step 1: Update the tests first**

Replace `tests/test_usb_std.c` with:
```c
#include "test.h"
#include "usb_std.h"

static usb_dev_t D;
static usb_resp_t R;
static void setup(uint8_t type, uint8_t req, uint16_t val, uint16_t idx, uint16_t len) {
    uint8_t s[8] = {type, req, (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)idx, (uint8_t)(idx >> 8),
                    (uint8_t)len, (uint8_t)(len >> 8)};
    usb_std_setup(&D, s, &R);
}

void test_usb_std(void) {
    usb_std_init(&D);
    setup(0x80, 6, 0x0100, 0, 64);                       /* device descriptor */
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 18);
    CHECK_EQ(R.data[0], 18); CHECK_EQ(R.data[1], 1); CHECK_EQ(R.data[7], 64);
    CHECK_EQ(R.data[4], 0xEF); CHECK_EQ(R.data[5], 0x02); CHECK_EQ(R.data[6], 0x01);   /* IAD composite */
    CHECK_EQ(R.data[8], 0x3C); CHECK_EQ(R.data[9], 0x2E); CHECK_EQ(R.data[10], 0x20); CHECK_EQ(R.data[11], 0x57);
    CHECK_EQ(R.data[12], 0x00); CHECK_EQ(R.data[13], 0x03);
    setup(0x80, 6, 0x0100, 0, 8);
    CHECK_EQ(R.len, 8);
    setup(0x80, 6, 0x0200, 0, 9);                        /* config header only */
    CHECK_EQ(R.len, 9); CHECK_EQ(R.data[2], 98); CHECK_EQ(R.data[3], 0); CHECK_EQ(R.data[4], 3);
    setup(0x80, 6, 0x0200, 0, 255);
    CHECK_EQ(R.len, 98);
    const uint8_t *c = R.data;
    /* MSC exactly as before */
    CHECK_EQ(c[9 + 2], 0); CHECK_EQ(c[9 + 5], 0x08); CHECK_EQ(c[9 + 6], 0x06); CHECK_EQ(c[9 + 7], 0x50);
    CHECK_EQ(c[18 + 2], 0x81); CHECK_EQ(c[18 + 3], 2); CHECK_EQ(c[18 + 4], 64);
    CHECK_EQ(c[25 + 2], 0x01);
    /* IAD + CDC */
    CHECK_EQ(c[32], 8); CHECK_EQ(c[33], 0x0B); CHECK_EQ(c[34], 1); CHECK_EQ(c[35], 2); CHECK_EQ(c[36], 0x02);
    CHECK_EQ(c[40], 9); CHECK_EQ(c[41], 4); CHECK_EQ(c[42], 1); CHECK_EQ(c[45], 0x02); CHECK_EQ(c[46], 0x02);
    CHECK_EQ(c[49 + 1], 0x24); CHECK_EQ(c[49 + 2], 0x00);       /* header */
    CHECK_EQ(c[54 + 2], 0x01); CHECK_EQ(c[54 + 4], 2);          /* call management -> data iface 2 */
    CHECK_EQ(c[59 + 2], 0x02); CHECK_EQ(c[59 + 3], 0x02);       /* ACM: line coding + control line */
    CHECK_EQ(c[63 + 2], 0x06); CHECK_EQ(c[63 + 3], 1); CHECK_EQ(c[63 + 4], 2);   /* union */
    CHECK_EQ(c[68 + 2], 0x83); CHECK_EQ(c[68 + 3], 3);          /* notification EP */
    CHECK_EQ(c[75 + 2], 2); CHECK_EQ(c[75 + 5], 0x0A);          /* data iface */
    CHECK_EQ(c[84 + 2], 0x02); CHECK_EQ(c[84 + 3], 2); CHECK_EQ(c[84 + 4], 64);
    CHECK_EQ(c[91 + 2], 0x82); CHECK_EQ(c[91 + 3], 2); CHECK_EQ(c[91 + 4], 64);
    /* walk the descriptor chain: lengths add up to 98 */
    int off = 0, guard = 0;
    while (off < 98 && guard++ < 20) off += c[off];
    CHECK_EQ(off, 98);

    setup(0x80, 6, 0x0300, 0, 255);
    CHECK_EQ(R.len, 4); CHECK_EQ(R.data[2], 0x09); CHECK_EQ(R.data[3], 0x04);
    setup(0x80, 6, 0x0302, 0x0409, 255);
    CHECK_EQ(R.len, 2 + 2 * 21); CHECK_EQ(R.data[2], 'D');
    setup(0x80, 6, 0x0600, 0, 10);
    CHECK_EQ(R.act, USB_STALL);
    setup(0x00, 5, 7, 0, 0);
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.set_address, 1); CHECK_EQ(R.new_address, 7);
    setup(0x00, 9, 1, 0, 0);
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(D.config, 1); CHECK_EQ(R.configured_changed, 1);

    /* MSC class requests on interface 0 */
    setup(0xA1, 0xFE, 0, 0, 1);
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 1); CHECK_EQ(R.data[0], 0);
    setup(0x21, 0xFF, 0, 0, 0);
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.bot_reset, 1);
    setup(0x21, 0xFF, 0, 1, 0);                          /* BOT reset aimed at the CDC iface: stall */
    CHECK_EQ(R.act, USB_STALL); CHECK_EQ(R.bot_reset, 0);

    /* CDC class requests on interface 1 */
    setup(0xA1, 0x21, 0, 1, 7);                          /* GET_LINE_CODING default 115200 8N1 */
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.len, 7);
    CHECK_EQ(R.data[0], 0x00); CHECK_EQ(R.data[1], 0xC2); CHECK_EQ(R.data[2], 0x01); CHECK_EQ(R.data[6], 8);
    setup(0x21, 0x20, 0, 1, 7);                          /* SET_LINE_CODING: data OUT stage */
    CHECK_EQ(R.act, USB_DATA_OUT); CHECK_EQ(R.len, 7);
    const uint8_t lc[7] = {0x00, 0x10, 0x0E, 0x00, 0, 0, 8};   /* 921600 */
    usb_std_data_out(&D, lc, 7);
    setup(0xA1, 0x21, 0, 1, 7);
    CHECK_EQ(R.data[1], 0x10); CHECK_EQ(R.data[2], 0x0E);
    CHECK_EQ(D.dtr, 0);
    setup(0x21, 0x22, 3, 1, 0);                          /* DTR | RTS */
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.dtr_changed, 1); CHECK_EQ(D.dtr, 1);
    setup(0x21, 0x22, 0, 1, 0);
    CHECK_EQ(R.dtr_changed, 1); CHECK_EQ(D.dtr, 0);
    setup(0x21, 0x23, 0, 1, 0);                          /* SEND_BREAK */
    CHECK_EQ(R.act, USB_STATUS_IN);
    setup(0x21, 0x22, 1, 1, 0);
    CHECK_EQ(D.dtr, 1);
    setup(0x00, 9, 1, 0, 0);                             /* re-configure clears DTR */
    CHECK_EQ(D.dtr, 0);
    setup(0xA1, 0x21, 0, 2, 7);                          /* wrong interface: stall */
    CHECK_EQ(R.act, USB_STALL);

    setup(0x80, 8, 0, 0, 1);
    CHECK_EQ(R.act, USB_DATA_IN); CHECK_EQ(R.data[0], 1);
    setup(0x80, 0, 0, 0, 2);
    CHECK_EQ(R.len, 2); CHECK_EQ(R.data[0], 0);
    setup(0x02, 1, 0, 0x81, 0);
    CHECK_EQ(R.act, USB_STATUS_IN); CHECK_EQ(R.clear_halt_ep, 0x81);
    setup(0x01, 11, 0, 0, 0);
    CHECK_EQ(R.act, USB_STATUS_IN);
    setup(0x40, 0x55, 0, 0, 0);
    CHECK_EQ(R.act, USB_STALL);
    setup(0x00, 9, 0, 0, 0);
    CHECK_EQ(D.config, 0); CHECK_EQ(R.configured_changed, 1);
}
```
Offsets: MSC interface at 9, EP 0x81 at 18, EP 0x01 at 25, IAD at 32, comm interface at 40, header at 49, call management at 54, ACM at 59, union at 63, EP 0x83 at 68, data interface at 75, EP 0x02 at 84, EP 0x82 at 91, total 98.

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile errors (no `USB_DATA_OUT`, no `dtr`), or failed checks.

- [ ] **Step 3: Implement**

`fw/core/usb_std.h`: replace the type definitions with:
```c
typedef enum { USB_STALL = 0, USB_DATA_IN, USB_STATUS_IN, USB_DATA_OUT } usb_act_t;

typedef struct {
    usb_act_t act;
    const uint8_t *data;       /* USB_DATA_IN: bytes to send */
    uint16_t len;              /* DATA_IN: limited to wLength; DATA_OUT: bytes to receive */
    uint8_t set_address, new_address;   /* apply the address after the status stage */
    uint8_t configured_changed;
    uint8_t clear_halt_ep;     /* 0 = none */
    uint8_t bot_reset;
    uint8_t dtr_changed;       /* CDC SET_CONTROL_LINE_STATE: d->dtr has the new value */
} usb_resp_t;

typedef struct {
    uint8_t config;
    uint8_t tmp[2];
    uint8_t str[4][2 + 2 * 24];   /* string descriptors 0..3, built by usb_std_init */
    uint8_t line_coding[7];       /* CDC: dwDTERate LE, bCharFormat, bParityType, bDataBits */
    uint8_t dtr;                  /* host has the serial port open */
} usb_dev_t;

/* Interfaces: 0 = MSC (EP 0x81/0x01), 1 = CDC comm (EP 0x83), 2 = CDC data (EP 0x82/0x02). */
void usb_std_init(usb_dev_t *d);
void usb_std_setup(usb_dev_t *d, const uint8_t setup[8], usb_resp_t *r);
/* Data stage of the last USB_DATA_OUT request (SET_LINE_CODING). */
void usb_std_data_out(usb_dev_t *d, const uint8_t *p, uint16_t n);
```

`fw/core/usb_std.c`:
- Change the device descriptor class bytes to `0xEF, 0x02, 0x01` and bcdDevice to `0x00, 0x03`.
- Replace `cfg_desc` with the 98-byte array listed above (`static const uint8_t cfg_desc[98] = { ... };`, one comment per descriptor).
- In `usb_std_init`, add `static const uint8_t lc0[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};`, copy it to `d->line_coding`, and set `d->dtr = 0;`.
- In `usb_std_setup`, also clear `r->dtr_changed = 0;` with the other fields, and replace the class block with:
```c
    if ((type & 0x60) == 0x20) {                         /* class: route by interface */
        uint8_t iface = (uint8_t)idx;
        if (iface == 0) {                                /* MSC BOT */
            if (type == 0xA1 && req == 0xFE) { d->tmp[0] = 0; in(r, d->tmp, 1, wlen); }
            else if (type == 0x21 && req == 0xFF) { r->act = USB_STATUS_IN; r->bot_reset = 1; }
        } else if (iface == 1) {                         /* CDC ACM */
            if (type == 0x21 && req == 0x20) { r->act = USB_DATA_OUT; r->len = wlen < 7 ? wlen : 7; }
            else if (type == 0xA1 && req == 0x21) in(r, d->line_coding, 7, wlen);
            else if (type == 0x21 && req == 0x22) {
                d->dtr = (uint8_t)(val & 1u); r->dtr_changed = 1; r->act = USB_STATUS_IN;
            } else if (type == 0x21 && req == 0x23) r->act = USB_STATUS_IN;
        }
        return;
    }
```
- In SET_CONFIGURATION, also set `d->dtr = 0;`.
- Append:
```c
void usb_std_data_out(usb_dev_t *d, const uint8_t *p, uint16_t n) {
    for (uint16_t i = 0; i < n && i < 7; i++) d->line_coding[i] = p[i];
}
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `make -C tests` and `make -C fw everything 2>&1 | grep -ciE "warning|error"`
Expected: `… 0 failed` and `0`. The firmware still builds: usbd treats `USB_DATA_OUT` as `default` (stall) until Task 5.

- [ ] **Step 5: Commit**

```bash
git add fw/core/usb_std.h fw/core/usb_std.c tests/test_usb_std.c
git commit -m "usb_std: composite MSC + CDC-ACM descriptors (IAD), CDC class requests, EP0 OUT data

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 5: `usbd` CDC endpoints + `cdctest` app

**Files:**
- Modify: `fw/common/usbd.h`, `fw/common/usbd.c`
- Create: `fw/apps/cdctest/main.c`

**Interfaces:**
- Consumes: `bytefifo_t`, `bf_*` (Task 1); `sp_put`, `sp_line_t`, `sp_line_feed`, `SP_OK` (Task 2, cdctest only); `USB_DATA_OUT`, `usb_std_data_out`, `dev.dtr`, `r.dtr_changed` (Task 4).
- Produces:
  - `int usbd_cdc_open(void)`: configured, not detached, and DTR set.
  - `bytefifo_t *usbd_cdc_txfifo(void)`: main-context producer.
  - `uint32_t usbd_cdc_read(uint8_t *p, uint32_t max)`: main-context consumer.
  - `void usbd_cdc_kick(void)`: start an IN transfer if idle.

PMA layout (byte offsets): BTABLE 0x00 (EP0..EP3 entries at 0, 8, 16, 24), EP0 RX 0x40, EP0 TX 0x80, EP1 RX 0xC0, EP1 TX 0x100, EP2 RX 0x140, EP2 TX 0x180, EP3 TX 0x1C0 (8 bytes). The total is 0x1C8, within the 512-byte packet memory.

Concurrency:
- The TX FIFO (8192 B) is produced by main and consumed by the ISR in `ep2_pump`.
- The RX FIFO (256 B) is produced by the ISR and consumed by main.
- When the port closes (DTR cleared), on reset or configuration change, and on detach, the ISR/locked code clears the TX FIFO with `bf_clear`. That is a consumer op, so it is safe there.
- The RX FIFO is never cleared from the ISR, because its tail belongs to main. Stale command bytes are harmless: `stream_poll` resets its line parser when the port closes.
- An OUT packet that doesn't fit in the RX FIFO is held (NAKed) and retried in `ep2_pump`.

- [ ] **Step 1: Header**

In `fw/common/usbd.h`, add `#include "bytefifo.h"`, update the top comment to say the device is composite (MSC + CDC-ACM serial port), and append:
```c
/* CDC-ACM serial port (interfaces 1-2, EP 0x82/0x02 bulk, 0x83 notifications). */
int usbd_cdc_open(void);                      /* configured, attached, and DTR set by the host */
bytefifo_t *usbd_cdc_txfifo(void);            /* main context only: append bytes to send */
uint32_t usbd_cdc_read(uint8_t *p, uint32_t max);   /* main context only: bytes received */
void usbd_cdc_kick(void);                     /* main context: start sending if the endpoint is idle */
```

- [ ] **Step 2: Driver changes in `fw/common/usbd.c`**

1. Includes and definitions. Add:
   - `#include "bytefifo.h"`
   - `#define EP2_RX 0x140u`, `#define EP2_TX 0x180u`, `#define EP3_TX 0x1C0u`, `#define EP_INTERRUPT 0x0600u`
   - `#define CDC_TX_SIZE 8192u`, `#define CDC_RX_SIZE 256u`
   - The globals:
```c
static uint8_t cdc_tx_mem[CDC_TX_SIZE], cdc_rx_mem[CDC_RX_SIZE];
static bytefifo_t cdc_tx, cdc_rx;
static uint8_t tx2_busy, rx2_held, rx2_len, cdc_dtr;
static uint8_t rx2buf[64], tx2buf[64];
static uint8_t ctl_out_left;                  /* EP0 OUT data stage bytes still expected */
```
   Also extend the ownership comment: the CDC state is ISR-owned like EP1, except the FIFO producer/consumer split.
2. `cdc_reset_state()`, a new static function called from `bus_reset`, from `configured_changed`, and inside `usbd_detach`'s locked section:
```c
static void cdc_reset_state(void) {
    tx2_busy = rx2_held = 0; cdc_dtr = 0; ctl_out_left = 0;
    bf_clear(&cdc_tx);                 /* consumer op: the ISR side owns cdc_tx.tail */
}
```
3. `bus_reset()`:
   - Extend the BTABLE: `PMA(16) = EP2_TX; PMA(18) = 0; PMA(20) = EP2_RX; PMA(22) = RX64; PMA(24) = EP3_TX; PMA(26) = 0; PMA(28) = 0; PMA(30) = 0;`
   - After the EP1 lines, add:
```c
    USB_EPR(2) = 0x0000u | 2u;                          /* bulk, address 2 (CDC data) */
    ep_clr_dtog(2); ep_set_rx(2, ST_VALID); ep_set_tx(2, ST_NAK);
    USB_EPR(3) = EP_INTERRUPT | 3u;                     /* interrupt IN, address 3 (CDC notify, unused) */
    ep_clr_dtog(3); ep_set_tx(3, ST_NAK);
```
   - Call `cdc_reset_state();` at the end.
4. `ep2_pump()`, a new static function, ISR context or main context with the lock held:
```c
static void ep2_pump(void) {
    if (detached || !dev.config) return;
    if (rx2_held && bf_space(&cdc_rx) >= rx2_len) {
        bf_write(&cdc_rx, rx2buf, rx2_len); rx2_held = 0; ep_set_rx(2, ST_VALID);
    }
    if (!tx2_busy && cdc_dtr && bf_used(&cdc_tx)) {
        uint32_t n = bf_read(&cdc_tx, tx2buf, 64);
        pma_write(EP2_TX, tx2buf, (int)n); PMA(18) = n; ep_set_tx(2, ST_VALID); tx2_busy = 1;
    }
}
```
5. `ep0_setup()`:
   - After the `configured_changed` block, add `if (r.configured_changed) { ep_clr_dtog_bits(2, 0x4040u); ep_set_rx(2, ST_VALID); ep_set_tx(2, ST_NAK); cdc_reset_state(); }`.
   - Add `if (r.dtr_changed) { cdc_dtr = dev.dtr; if (!cdc_dtr) { bf_clear(&cdc_tx); } }`.
   - Add the case:
```c
    case USB_DATA_OUT:
        ctl_out_left = (uint8_t)r.len;
        if (ctl_out_left == 0) { PMA(2) = 0; ep_set_tx(0, ST_VALID); }   /* no data: straight to status */
        ep_set_rx(0, ST_VALID);
        break;
```
6. `service_ctr()`:
   - The EP0 non-SETUP `CTR_RX` branch becomes:
```c
                else {
                    ep_clr_ctr_rx(0);
                    if (ctl_out_left) {                  /* SET_LINE_CODING data stage */
                        uint8_t b[64]; int n = rx_count(0); if (n > 64) n = 64;
                        pma_read(EP0_RX, b, n);
                        usb_std_data_out(&dev, b, (uint16_t)n);
                        ctl_out_left = (uint8_t)(n >= ctl_out_left ? 0 : ctl_out_left - n);
                        if (!ctl_out_left) { PMA(2) = 0; ep_set_tx(0, ST_VALID); }   /* status IN */
                    }
                    ep_set_rx(0, ST_VALID);
                }
```
   - Add a branch for EP2:
```c
        } else if (ep == 2) {
            if (v & EP_CTR_RX) {                         /* hardware NAKs further OUT packets now */
                ep_clr_ctr_rx(2);
                int n = rx_count(2); if (n > 64) n = 64;
                pma_read(EP2_RX, rx2buf, n);
                rx2_len = (uint8_t)n; rx2_held = 1;
            }
            if (v & EP_CTR_TX) { ep_clr_ctr_tx(2); tx2_busy = 0; }
            ep2_pump();
```
   - Keep the existing `else { USB_EPR(ep) = 0; }` for any other endpoint, but make EP3 explicit: `else if (ep == 3) { ep_clr_ctr_tx(3); ep_clr_ctr_rx(3); }`. The CDC notify endpoint never sends, so this is defensive.
7. `usbd_init()`: before `bus_reset()`, call `bf_init(&cdc_tx, cdc_tx_mem, CDC_TX_SIZE); bf_init(&cdc_rx, cdc_rx_mem, CDC_RX_SIZE);`.
8. `usbd_poll()`: inside the locked section, after `ep1_pump();`, add `ep2_pump();`.
9. `usbd_detach()`: inside the locked section, also call `ep_set_rx(2, ST_NAK); ep_set_tx(2, ST_NAK); cdc_reset_state();`.
10. Public functions:
```c
int usbd_cdc_open(void) { return usbd_host() && *(volatile uint8_t *)&cdc_dtr; }
bytefifo_t *usbd_cdc_txfifo(void) { return &cdc_tx; }
uint32_t usbd_cdc_read(uint8_t *p, uint32_t max) {
    uint32_t n = bf_read(&cdc_rx, p, max);
    if (n) usbd_cdc_kick();                          /* a held OUT packet may fit now */
    return n;
}
void usbd_cdc_kick(void) {
    if (!inited) return;
    usb_lock(); ep2_pump(); usb_unlock();
}
```

- [ ] **Step 3: The `cdctest` app**

`fw/apps/cdctest/main.c`:
```c
// cdctest: the composite device. Serial port: replies "OK <line>" to every line; "BENCH" floods
// 5 s of LOG packets for a throughput measurement. Drive works as usual. Hold POWER = off.
#include "board.h"
#include "lcd.h"
#include "spiflash.h"
#include "flashmap.h"
#include "vol.h"
#include "msc.h"
#include "usbd.h"
#include "stream_proto.h"

static msc_t msc;
static const msc_disk_t disk = {vol_read, vol_write, USER_VOL_SECTORS};
void board_idle(void) { usbd_poll(); }

int main(void) {
    board_init(); keys_init(); lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK); backlight(1);
    lcd_text(0, 0, " cdctest: drive + serial port", C_CYAN, C_BLACK);
    spiflash_init();
    msc_init(&msc, &disk);
    usbd_init(&msc);
    sp_line_t line = {{0}, 0, 0};
    uint32_t next = 0, bench_until = 0, bench_bytes = 0, bench_t0 = 0, seq = 0, lines = 0;
    static uint8_t blk[1024];
    char b[12];
    for (;;) {
        usbd_poll();
        uint8_t in[32];
        uint32_t n = usbd_cdc_read(in, sizeof in);
        for (uint32_t i = 0; i < n; i++) {
            if (!sp_line_feed(&line, in[i])) continue;
            lines++;
            if (line.buf[0] == 'B' && line.buf[1] == 'E' && line.buf[2] == 'N' && line.buf[3] == 'C' && line.buf[4] == 'H') {
                bench_t0 = millis(); bench_until = bench_t0 + 5000; bench_bytes = 0;
            }
            uint32_t len = 0; while (line.buf[len]) len++;
            sp_put(usbd_cdc_txfifo(), SP_OK, (const uint8_t *)line.buf, len, 0, 0);
            usbd_cdc_kick();
        }
        if (bench_until && usbd_cdc_open()) {
            if ((int32_t)(millis() - bench_until) >= 0) bench_until = 0;
            else {
                uint8_t h[8];
                for (int i = 0; i < 1024; i++) blk[i] = (uint8_t)(seq + (uint32_t)i);
                sp_u32x2(h, seq, 0);
                if (sp_put(usbd_cdc_txfifo(), SP_LOG, h, 4, blk, 1024) == 0) { seq += 1024; bench_bytes += 1024; }
                usbd_cdc_kick();
            }
        }
        uint32_t now = millis();
        if ((int32_t)(now - next) >= 0) {
            next = now + 250;
            lcd_text(0, 2, usbd_host() ? " host: CONFIGURED " : " host: none       ", usbd_host() ? C_GREEN : C_GRAY, C_BLACK);
            lcd_text(0, 3, usbd_cdc_open() ? " serial: OPEN  " : " serial: closed", usbd_cdc_open() ? C_GREEN : C_GRAY, C_BLACK);
            int c = lcd_text(0, 4, " lines ", C_GRAY, C_BLACK); lcd_text(c, 4, fmt_x(b, lines, 6), C_WHITE, C_BLACK);
            c = lcd_text(0, 5, " bench bytes ", C_GRAY, C_BLACK); lcd_text(c, 5, fmt_x(b, bench_bytes, 8), C_WHITE, C_BLACK);
            uint32_t ms = (bench_until ? now : (bench_t0 ? bench_t0 + 5000 : now)) - bench_t0;
            c = lcd_text(0, 6, " bench KB/s  ", C_GRAY, C_BLACK);
            lcd_text(c, 6, fmt_x(b, ms ? bench_bytes / ms : 0, 6), C_WHITE, C_BLACK);
            lcd_text(0, 14, " Hold POWER = off", C_GRAY, C_BLACK);
        }
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER)) {
                usbd_poll(); WDT_CMD = 0xAAAA;
                if (millis() - t0 > 1000) { backlight(0); usbd_detach(); power_off(); for (;;) WDT_CMD = 0xAAAA; }
            }
        }
        WDT_CMD = 0xAAAA;
    }
}
```
"KB/s" is bytes per millisecond, about the same as kB/s. It is shown in hex, like the other counters.

- [ ] **Step 4: Build**

Run: `make -C fw clean everything 2>&1 | grep -ciE "warning|error"` and `make -C tests`
Expected: `0`, and `… 0 failed`.

- [ ] **Step 5: Commit**

```bash
git add fw/common/usbd.h fw/common/usbd.c fw/apps/cdctest/main.c
git commit -m "usbd: composite MSC + CDC-ACM (EP2 bulk, EP3 notify, EP0 OUT data, DTR, FIFOs); cdctest app

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

- [ ] **Step 6 (controller with the user, on device):**
  1. Flash `cdctest` and plug in USB.
  2. Run `lsblk` and `ls /dev/ttyACM*`, and check the kernel log: there should be one `2e3c:5720` device with both `usb-storage` and `cdc_acm`.
  3. Echo test: `stty -F /dev/ttyACM0 raw -echo; (cat /dev/ttyACM0 > /tmp/claude-…/scratchpad/acm.bin &) ; printf 'HELLO\n' > /dev/ttyACM0; sleep 1; xxd …/acm.bin` should show an OK packet with `HELLO`.
  4. Throughput: `printf 'BENCH\n' > /dev/ttyACM0`, capture for 6 s, and divide the file size by 5 s. Record the result in the ledger. The target is ≥ 300 kB/s, which is enough for 200 kS/s with headroom.
  5. Mount the drive and list it while the port is open.

---

### Task 6: ADC circular ring mode

**Files:**
- Modify: `fw/common/adc.h`, `fw/common/adc.c`

**Interfaces:**
- Produces: `void adc_ring_start(uint8_t *buf, uint16_t n)` and `int adc_ring_poll(uint32_t *remaining)`. The return value has bit0 = HT and bit1 = TC; both flags are cleared, and `*remaining` is the DMA's count still to go in this pass. Stop with `adc_abort()`.

- [ ] **Step 1: Implement**

In `fw/common/adc.h`, append:
```c
// Continuous capture into a circular buffer of n bytes (n even): DMA1 ch7 circular mode. The timer
// rate is whatever adc_set_rate() last set. Stop with adc_abort().
void adc_ring_start(uint8_t *buf, uint16_t n);
// Half-transfer / transfer-complete flags since the last call (bit0 HT, bit1 TC; cleared on read)
// and the DMA's transfers still to go in the current pass (feed both to logger_pick()).
int adc_ring_poll(uint32_t *remaining);
```
In `fw/common/adc.c`, add:
```c
#define DMA_CH7_HT (1u << 26)
#define DMA_CH7_TC (1u << 25)

void adc_ring_start(uint8_t *buf, uint16_t n) {
    DMA1_C7CTRL = 0;
    DMA1_CLR = 0xFu << 24;
    DMA1_C7MADDR = (uint32_t)buf;
    DMA1_C7DTCNT = n;
    busy = 0;                          /* not a one-shot capture: adc_done() stays 1 */
    DMA1_C7CTRL = 0x3000 | 0x80 | 0x20 | 1;   /* very high prio, 8-bit, mem increment, circular, enable */
}

int adc_ring_poll(uint32_t *remaining) {
    uint32_t s = DMA1_STS & (DMA_CH7_HT | DMA_CH7_TC);
    *remaining = DMA1_C7DTCNT;
    if (s) DMA1_CLR = s;               /* clear only what we saw */
    return ((s & DMA_CH7_HT) ? 1 : 0) | ((s & DMA_CH7_TC) ? 2 : 0);
}
```
The existing `adc_abort()` already disables the channel and clears its flags, so it also stops the ring.

- [ ] **Step 2: Build**

Run: `make -C fw everything 2>&1 | grep -ciE "warning|error"`
Expected: `0`. On-device verification happens in Task 10 through logger mode.

- [ ] **Step 3: Commit**

```bash
git add fw/common/adc.h fw/common/adc.c
git commit -m "adc: circular DMA ring mode with HT/TC polling for continuous logging

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 7: Python protocol library + tests

**Files:**
- Create: `tools/dsoproto.py`, `tools/tests/test_dsoproto.py`

**Interfaces:**
- Consumes: `tests/vectors/*.bin` and `tests/vectors/frame.csv` (Task 2).
- Produces:
  - `dsoproto.Decoder` with `.feed(bytes) -> list` and `.bad_crc`.
  - Packet classes `Info(fields: dict)`, `Frame(meta: Meta, samples: bytes)`, `LogMeta(meta: Meta)`, `Log(first_index, samples)`, `Drop(first_index, count)`, `Ok(text)`, `Err(text)`.
  - `Meta` with `.uv_per_code()` and `.volts(code)`.
  - `frame_csv_text(meta, samples, fw)`.
  - `LogCsvWriter(fileobj, fw)` with `.meta(Meta)`, `.log(Log)`, `.drop(Drop)`.

- [ ] **Step 1: Write the failing tests**

`tools/tests/test_dsoproto.py`:
```python
import io
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import dsoproto as dp  # noqa: E402

VEC = os.path.join(HERE, "..", "..", "tests", "vectors")


def vec(name):
    with open(os.path.join(VEC, name), "rb") as f:
        return f.read()


class TestCrc(unittest.TestCase):
    def test_check_value(self):
        self.assertEqual(dp.crc16(b"123456789"), 0x29B1)


class TestVectors(unittest.TestCase):
    def one(self, name):
        d = dp.Decoder()
        pk = d.feed(vec(name))
        self.assertEqual(len(pk), 1)
        self.assertEqual(d.bad_crc, 0)
        return pk[0]

    def test_info(self):
        p = self.one("info.bin")
        self.assertIsInstance(p, dp.Info)
        self.assertEqual(p.fields["fw"], "0.3.0")
        self.assertEqual(p.fields["mode"], "idle")
        self.assertEqual(p.fields["rates"], "1000,2000,5000,10000,20000,50000,100000,200000,500000")

    def test_frame(self):
        p = self.one("frame.bin")
        self.assertIsInstance(p, dp.Frame)
        m = p.meta
        self.assertEqual((m.rate_hz, m.vdiv_mv, m.coupling_dc, m.probe_x10), (600000, 500, 1, 0))
        self.assertEqual((m.trig_mode, m.edge, m.triggered), (0, 0, 1))
        self.assertEqual((m.zero_code, m.trig_level_code, m.trig_index, m.offset_pwm), (46, 75, 3, 1630))
        self.assertEqual(list(p.samples), [46, 71, 96, 121, 146, 121, 96, 71])
        self.assertEqual(m.uv_per_code(), 20000)
        self.assertAlmostEqual(m.volts(146), 2.0)

    def test_logmeta(self):
        p = self.one("logmeta.bin")
        self.assertIsInstance(p, dp.LogMeta)
        m = p.meta
        self.assertEqual((m.rate_hz, m.vdiv_mv, m.probe_x10, m.coupling_dc, m.zero_code, m.offset_pwm),
                         (100000, 1000, 0, 1, 128, 2033))

    def test_log_drop_ok_err(self):
        p = self.one("log.bin")
        self.assertIsInstance(p, dp.Log)
        self.assertEqual(p.first_index, 1000)
        self.assertEqual(list(p.samples), list(range(120, 136)))
        p = self.one("drop.bin")
        self.assertEqual((p.first_index, p.count), (2048, 512))
        self.assertEqual(self.one("ok.bin").text, "LOG 100000")
        self.assertEqual(self.one("err.bin").text, "bad rate")


class TestStream(unittest.TestCase):
    def expect(self, pk):
        kinds = [type(p).__name__ for p in pk]
        self.assertEqual(kinds, ["Info", "Frame", "Log", "Drop", "Ok", "Err"])

    def test_resync_and_bad_crc(self):
        d = dp.Decoder()
        pk = d.feed(vec("stream.bin"))
        self.expect(pk)
        self.assertEqual(d.bad_crc, 1)              # the corrupted LOG was dropped

    def test_byte_by_byte(self):
        d = dp.Decoder()
        pk = []
        for b in vec("stream.bin"):
            pk += d.feed(bytes([b]))
        self.expect(pk)


class TestCsv(unittest.TestCase):
    def test_frame_csv_matches_firmware(self):
        p = dp.Decoder().feed(vec("frame.bin"))[0]
        text = dp.frame_csv_text(p.meta, p.samples, "0.3.0")
        self.assertEqual(text, vec("frame.csv").decode("ascii"))

    def test_log_csv(self):
        m = dp.Meta(rate_hz=1000, vdiv_mv=1000, coupling_dc=1, probe_x10=0, zero_code=128, offset_pwm=2033)
        out = io.StringIO()
        w = dp.LogCsvWriter(out, "0.3.0")
        w.meta(m)
        w.log(dp.Log(0, bytes([128, 153])))
        w.drop(dp.Drop(5, 3))
        w.log(dp.Log(5, bytes([103])))
        self.assertEqual(out.getvalue(),
                         "# DSO-510 custom firmware 0.3.0 (PC log)\n"
                         "# sample_rate_hz=1000\n"
                         "# volts_per_div=1.0000\n"
                         "# coupling=DC\n"
                         "# probe=1\n"
                         "# trigger=none\n"
                         "# cal=offset_pwm=2033,volts_per_code=0.040000\n"
                         "time_s,volts,adc_code\n"
                         "0.000000000,0.0000,128\n"
                         "0.001000000,1.0000,153\n"
                         "# gap=3 samples at 2\n"
                         "0.005000000,-1.0000,103\n")


if __name__ == "__main__":
    unittest.main()
```
In `test_log_csv`, the gap line says `at 2`. The gap begins right after the last written sample, index 2, and lasts 3 samples, so the next sample has index 5. Its time is `index / rate` = 0.005 s.

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `python3 -m unittest discover -s tools/tests -v`
Expected: `ModuleNotFoundError: No module named 'dsoproto'`.

- [ ] **Step 3: Implement `tools/dsoproto.py`**

```python
"""DSO-510 live streaming protocol (spec: sub-project 4).

Packets: A5 5A | type u8 | len u16 LE | payload | crc16 LE (CRC-16/CCITT-FALSE over type, len, payload).
Stdlib only, so it can be reused in any script (e.g. plotting a recorded CSV with matplotlib).
"""
import struct
from dataclasses import dataclass, field

INFO, FRAME, LOGMETA, LOG, DROP, OK, ERR = range(1, 8)
SYNC = b"\xa5\x5a"


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass
class Meta:
    rate_hz: int
    vdiv_mv: int
    coupling_dc: int
    probe_x10: int
    zero_code: int
    offset_pwm: int
    trig_mode: int = 0
    edge: int = 0
    triggered: int = 0
    trig_level_code: int = 0
    trig_index: int = 0

    def uv_per_code(self):
        return self.vdiv_mv * 40 * (10 if self.probe_x10 else 1)

    def volts(self, code):
        return (code - self.zero_code) * self.uv_per_code() / 1e6


@dataclass
class Info:
    fields: dict = field(default_factory=dict)


@dataclass
class Frame:
    meta: Meta
    samples: bytes


@dataclass
class LogMeta:
    meta: Meta


@dataclass
class Log:
    first_index: int
    samples: bytes


@dataclass
class Drop:
    first_index: int
    count: int


@dataclass
class Ok:
    text: str


@dataclass
class Err:
    text: str


def parse(ptype, p):
    if ptype == INFO:
        kv = {}
        for line in p.decode("ascii", "replace").splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                kv[k] = v
        return Info(kv)
    if ptype == FRAME:
        (rate, vdiv, dc, x10, mode, edge, trig, zero, level, tidx, pwm) = struct.unpack_from("<IHBBBBBhhiH", p, 0)
        (n,) = struct.unpack_from("<H", p, 21)
        m = Meta(rate, vdiv, dc, x10, zero, pwm, mode, edge, trig, level, tidx)
        return Frame(m, bytes(p[23:23 + n]))
    if ptype == LOGMETA:
        (rate, vdiv, x10, dc, zero, pwm) = struct.unpack_from("<IHBBhH", p, 0)
        return LogMeta(Meta(rate, vdiv, dc, x10, zero, pwm))
    if ptype == LOG:
        return Log(struct.unpack_from("<I", p, 0)[0], bytes(p[4:]))
    if ptype == DROP:
        a, b = struct.unpack_from("<II", p, 0)
        return Drop(a, b)
    if ptype == OK:
        return Ok(p.decode("ascii", "replace"))
    if ptype == ERR:
        return Err(p.decode("ascii", "replace"))
    return None


class Decoder:
    """Feed raw serial bytes; returns complete, CRC-checked packets. Resynchronises after junk."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                keep = 1 if self.buf[-1:] == b"\xa5" else 0
                del self.buf[:len(self.buf) - keep]
                return out
            if i:
                del self.buf[:i]
            if len(self.buf) < 5:
                return out
            ptype = self.buf[2]
            n = self.buf[3] | self.buf[4] << 8
            if len(self.buf) < 5 + n + 2:
                return out
            body = bytes(self.buf[2:5 + n])
            crc = self.buf[5 + n] | self.buf[6 + n] << 8
            if crc16(body) != crc:
                self.bad_crc += 1
                del self.buf[:1]                    # skip this sync, look for the next one
                continue
            pk = parse(ptype, body[3:])
            del self.buf[:7 + n]
            if pk is not None:
                out.append(pk)


def _rdiv(n, d):
    return (n + d // 2) // d if n >= 0 else -((-n + d // 2) // d)


def _fixed(v, dec):
    sign = "-" if v < 0 else ""
    digits = str(abs(v)).rjust(dec + 1, "0")
    return sign + (digits[:-dec] + "." + digits[-dec:] if dec else digits)


def _row(t_ns, code, m):
    return "%s,%s,%d\n" % (_fixed(t_ns, 9), _fixed(_rdiv((code - m.zero_code) * m.uv_per_code(), 100), 4), code)


def frame_csv_text(m, samples, fw):
    """Exactly the scope's own CSV (fw/core/csv.c) for one frame."""
    modes = ["Auto", "Normal", "Single"]
    uv = m.uv_per_code()
    lines = [
        "# DSO-510 custom firmware %s\n" % fw,
        "# sample_rate_hz=%d\n" % m.rate_hz,
        "# volts_per_div=%s\n" % _fixed(m.vdiv_mv * 10 * (10 if m.probe_x10 else 1), 4),
        "# coupling=%s\n" % ("DC" if m.coupling_dc else "AC"),
        "# probe=%s\n" % ("10" if m.probe_x10 else "1"),
        "# trigger=%s,%s,%s\n" % (modes[m.trig_mode] if m.trig_mode < 3 else "Auto",
                                  "falling" if m.edge else "rising",
                                  _fixed(_rdiv((m.trig_level_code - m.zero_code) * uv, 100), 4)),
        "# trigger_index=%d\n" % m.trig_index,
        "# triggered=%d\n" % (1 if m.triggered else 0),
        "# cal=offset_pwm=%d,volts_per_code=%s\n" % (m.offset_pwm, _fixed(uv, 6)),
        "time_s,volts,adc_code\n",
    ]
    for i, c in enumerate(samples):
        lines.append(_row(_rdiv((i - m.trig_index) * 10**9, m.rate_hz), c, m))
    return "".join(lines)


class LogCsvWriter:
    """Streams a logger session to CSV: time 0 = first sample, '# gap=' lines at drops."""

    def __init__(self, f, fw):
        self.f = f
        self.fw = fw
        self.m = None
        self.next_index = 0

    def meta(self, m):
        uv = m.uv_per_code()
        if self.m is None:
            self.f.write("# DSO-510 custom firmware %s (PC log)\n" % self.fw)
            self.f.write("# sample_rate_hz=%d\n" % m.rate_hz)
            self.f.write("# volts_per_div=%s\n" % _fixed(m.vdiv_mv * 10 * (10 if m.probe_x10 else 1), 4))
            self.f.write("# coupling=%s\n" % ("DC" if m.coupling_dc else "AC"))
            self.f.write("# probe=%s\n" % ("10" if m.probe_x10 else "1"))
            self.f.write("# trigger=none\n")
            self.f.write("# cal=offset_pwm=%d,volts_per_code=%s\n" % (m.offset_pwm, _fixed(uv, 6)))
            self.f.write("time_s,volts,adc_code\n")
        else:
            self.f.write("# settings changed at %d: volts_per_div=%s,offset_pwm=%d,volts_per_code=%s\n" % (
                self.next_index, _fixed(m.vdiv_mv * 10 * (10 if m.probe_x10 else 1), 4), m.offset_pwm, _fixed(uv, 6)))
        self.m = m

    def drop(self, d):
        self.f.write("# gap=%d samples at %d\n" % (d.count, d.first_index - d.count))

    def log(self, lg):
        if self.m is None:
            return
        rate = self.m.rate_hz
        parts = []
        for k, c in enumerate(lg.samples):
            idx = lg.first_index + k
            parts.append(_row(_rdiv(idx * 10**9, rate), c, self.m))
        self.f.write("".join(parts))
        self.next_index = lg.first_index + len(lg.samples)
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `python3 -m unittest discover -s tools/tests -v`
Expected: all tests `ok`. If `test_frame_csv_matches_firmware` fails, diff the output against `tests/vectors/frame.csv` and fix `frame_csv_text`. Never edit the vector.

- [ ] **Step 5: Commit**

```bash
git add tools/dsoproto.py tools/tests/test_dsoproto.py
git commit -m "tools: dsoproto (decoder, volts, CSV writers matching the firmware) + unittest on golden vectors

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 8: Firmware streaming state machine + logger mode in the scope app

**Files:**
- Create: `fw/app/stream.h`, `fw/app/stream.c`
- Modify: `fw/app/ui.h`, `fw/app/ui.c`, `fw/app/main.c`

**Interfaces:**
- Consumes:
  - `usbd_cdc_open`, `usbd_cdc_txfifo`, `usbd_cdc_read` and `usbd_cdc_kick` (Task 5)
  - `sp_*` (Task 2)
  - `logger_t`, `logger_init`, `logger_push`, `logger_lost` and `logger_pick` (Task 3)
  - `adc_ring_start` and `adc_ring_poll` (Task 6)
  - `units_rate_for_hz` (Task 1)
  - from main.c (sub-projects 1–2): `csv_meta_t`, `shown`, `shown_n`, `shown_meta`, `process_capture`, `apply_fe`, `apply_time`, `redraw_all` and `power_down`
- Produces: `stream_mode_t`, `stream_hooks_t`, `stream_init`, `stream_poll`, `stream_mode`, `stream_frame`, `stream_logmeta`, `stream_log`, `stream_log_lost`, `stream_stop_log`, `stream_log_stats`, `ui_draw_usb` and `ui_draw_logger`.

- [ ] **Step 1: `fw/app/stream.h`**

```c
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
void stream_log(const uint8_t *s, uint32_t n);
void stream_log_lost(uint32_t n);
/* Leave logger mode from the scope side (Run key): hook log_stop, mode idle, INFO to the host. */
void stream_stop_log(void);
void stream_log_stats(uint32_t *sent, uint32_t *dropped);
```

- [ ] **Step 2: `fw/app/stream.c`**

```c
#include "stream.h"
#include "stream_proto.h"
#include "logger.h"
#include "usbd.h"

static const stream_hooks_t *hk;
static stream_mode_t mode;
static uint8_t was_open;
static sp_line_t line;
static logger_t lg;

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
        if (hk->log_start(arg) < 0) { send_text(SP_ERR, "busy"); break; }
        mode = STREAM_LOG;
        send_text(SP_OK, line.buf);                  /* echo "LOG <rate>" */
        break;
    case SP_CMD_BAD: send_text(SP_ERR, "unknown command"); break;
    default: break;
    }
}

void stream_poll(void) {
    int open = usbd_cdc_open();
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
    if (mode != STREAM_FRAMES || !usbd_cdc_open() || n <= 0 || n > 65535 - (int)SP_FRAME_META - 2) return;
    uint8_t h[SP_FRAME_META + 2];
    uint32_t hl = sp_frame_meta(h, m, (uint16_t)n);
    if (sp_put(usbd_cdc_txfifo(), SP_FRAME, h, hl, rec, (uint32_t)n) == 0) usbd_cdc_kick();   /* else: skipped */
}

void stream_logmeta(uint32_t rate_hz, uint16_t vdiv_mv, uint8_t probe_x10, uint8_t coupling_dc,
                    int16_t zero_code, uint16_t offset_pwm) {
    uint8_t p[SP_LOGMETA_LEN];
    uint32_t k = sp_logmeta(p, rate_hz, vdiv_mv, probe_x10, coupling_dc, zero_code, offset_pwm);
    sp_put(usbd_cdc_txfifo(), SP_LOGMETA, p, k, 0, 0);   /* small; if it can't fit, the host keeps the old meta */
    usbd_cdc_kick();
}

void stream_log(const uint8_t *s, uint32_t n) {
    if (mode != STREAM_LOG) return;
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
```

- [ ] **Step 3: UI additions**

In `fw/app/ui.h`, add:
```c
void ui_draw_usb(int on);                  /* top bar cols 29..31: "USB" while frames stream */
/* Logger status screen (replaces the plot while logging). */
void ui_draw_logger(uint32_t rate_hz, uint32_t sent, uint32_t dropped, float vmin, float vmax, float vmean,
                    float vdiv_v);
```
In `fw/app/ui.c`, add:
```c
void ui_draw_usb(int on) { lcd_text(29, 0, on ? "USB" : "   ", C_BLACK, on ? C_CYAN : C_BLACK); }

static void kv_line(int row, const char *k, const char *v, uint16_t col) {
    int c = lcd_text(2, row, k, C_GRAY, C_BLACK);
    c = lcd_text(c, row, v, col, C_BLACK);
    lcd_fill(c * 8, row * 16, LCD_W - c * 8, 16, C_BLACK);   /* clear the rest of the row */
}

void ui_draw_logger(uint32_t rate_hz, uint32_t sent, uint32_t dropped, float vmin, float vmax, float vmean,
                    float vdiv_v) {
    char b[16];
    lcd_text(2, 2, "LOGGING -> USB      Run = stop", C_CYAN, C_BLACK);
    kv_line(4, "rate      ", fmt_si(b, (float)rate_hz, "S/s", 0), C_WHITE);
    kv_line(5, "V/div     ", fmt_si(b, vdiv_v, "V", 1), C_YELLOW);
    kv_line(7, "sent      ", fmt_u(b, sent, 1), C_WHITE);
    kv_line(8, "dropped   ", fmt_u(b, dropped, 1), dropped ? C_RED : C_GREEN);
    kv_line(10, "min       ", fmt_si(b, vmin, "V", 0), C_WHITE);
    kv_line(11, "max       ", fmt_si(b, vmax, "V", 0), C_WHITE);
    kv_line(12, "mean      ", fmt_si(b, vmean, "V", 0), C_WHITE);
}
```
`fmt_si(char *buf, float v, const char *unit, int digits)` and `fmt_u(char *buf, uint32_t v, int width)` already exist in `fmt.h`/`lcd.h` and are used elsewhere in ui.c. Check their exact signatures with `grep -n "fmt_si\|fmt_u" fw/core/fmt.h fw/common/lcd.h`, and adapt the calls if a parameter differs. `fmt_si` must handle negative volts. If it doesn't, print the sign separately.

- [ ] **Step 4: Wire into `fw/app/main.c`**

1. Add `#include "stream.h"`, `#include "logger.h"` and `#include "units.h"` (units.h is already included).
2. Add the logger globals:
```c
static uint8_t ring[8192];
static uint32_t ring_n;                 /* even, <= sizeof ring: halves of ~100 ms */
static uint32_t log_rate, log_next_ui;
static int log_vmin = 255, log_vmax, log_cnt; static uint32_t log_sum;
static uint8_t usb_marker;
```
3. Add the hook functions. They come after `apply_time`, `redraw_all` and `power_down` are declared; add a forward declaration of `power_down` if needed.
```c
static void log_send_meta(void) {
    stream_logmeta(log_rate, vdiv_mv[st.vdiv], st.probe_x10, st.coupling_dc,
                   (int16_t)(ADC_CENTER + st.offset), fe_pwm);
}

static int hook_log_start(uint32_t rate) {
    rate_cfg_t q = units_rate_for_hz(rate);
    if (!q.rate_hz) return -1;
    adc_abort(); armed = 0;
    st.menu_open = 0;
    rc = q;                              /* apply_fe uses rc.rate_hz for the cal set */
    adc_set_rate(q.psc, q.arr);
    apply_fe();
    log_rate = rate;
    ring_n = rate / 5;                   /* two halves of ~100 ms */
    if (ring_n > sizeof ring) ring_n = sizeof ring;
    if (ring_n < 256) ring_n = 256;
    ring_n &= ~1u;
    log_vmin = 255; log_vmax = 0; log_sum = 0; log_cnt = 0;
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    log_next_ui = millis();
    log_send_meta();
    adc_ring_start(ring, (uint16_t)ring_n);
    return 0;
}

static void hook_log_stop(void) {
    adc_abort(); armed = 0;
    apply_time();                         /* back to the scope's own timebase */
    have_trace = 0;
    redraw_all();
}

static const stream_hooks_t hooks = {hook_log_start, hook_log_stop};

/* Logger mode, one main-loop step: move completed ring halves to USB, keys, status screen. */
static void logger_step(void) {
    uint32_t rem;
    int fl = adc_ring_poll(&rem);
    ring_pick_t pk = logger_pick(fl, rem, ring_n);
    if (pk.lost) stream_log_lost(pk.lost);
    int halves[2] = {pk.first, pk.second};
    for (int h = 0; h < 2; h++) {
        if (halves[h] < 0) continue;
        const uint8_t *p = ring + (uint32_t)halves[h] * (ring_n / 2);
        stream_log(p, ring_n / 2);
        for (uint32_t i = 0; i < ring_n / 2; i++) {
            if (p[i] < log_vmin) log_vmin = p[i];
            if (p[i] > log_vmax) log_vmax = p[i];
            log_sum += p[i];
        }
        log_cnt += (int)(ring_n / 2);
    }
    kev_t ev[8];
    int ne = keys_update(&ks, keys_read(), millis(), ev, 8);
    for (int i = 0; i < ne; i++) {
        int tap = ev[i].type == KEV_SHORT || ev[i].type == KEV_REPEAT;
        if (ev[i].key == KEY_POWER && ev[i].type == KEV_LONG) power_down();
        else if (ev[i].key == KEY_RUN && ev[i].type == KEV_SHORT) { stream_stop_log(); return; }
        else if (tap && (ev[i].key == KEY_UP || ev[i].key == KEY_DOWN)) {
            int nv = st.vdiv + (ev[i].key == KEY_UP ? -1 : 1);       /* Up = more sensitive */
            if (nv >= 0 && nv < NUM_VDIV) { st.vdiv = (int8_t)nv; apply_fe(); log_send_meta(); }
        }
    }
    uint32_t now = millis();
    if ((int32_t)(now - log_next_ui) >= 0) {
        log_next_ui = now + 250;
        uint32_t sent, dropped;
        stream_log_stats(&sent, &dropped);
        float vpc = units_volts_per_code(st.vdiv, st.probe_x10);
        int zero = ADC_CENTER + st.offset;
        float mean = log_cnt ? (float)log_sum / (float)log_cnt : (float)zero;
        ui_draw_logger(log_rate, sent, dropped,
                       log_cnt ? (float)(log_vmin - zero) * vpc : 0.0f,
                       log_cnt ? (float)(log_vmax - zero) * vpc : 0.0f,
                       (mean - (float)zero) * vpc,
                       (float)vdiv_mv[st.vdiv] * 1e-3f * (st.probe_x10 ? 10.0f : 1.0f));
        log_vmin = 255; log_vmax = 0; log_sum = 0; log_cnt = 0;
    }
}
```
4. In `main()`, after `usbd_init(&msc);`, add `stream_init(&hooks);`.
5. At the top of the main loop, after `usbd_poll();`, add:
```c
        stream_poll();
        if (stream_mode() == STREAM_LOG) { logger_step(); WDT_CMD = 0xAAAA; continue; }
        int usb_on = stream_mode() == STREAM_FRAMES;
        if (usb_on != usb_marker) { usb_marker = (uint8_t)usb_on; ui_draw_usb(usb_on); }
```
6. In `process_capture()`, after `ui_draw_meas(&st, &last_meas);`, add `stream_frame(shown, shown_n, &shown_meta);`.
7. In `draw_bars()`, after `ui_draw_bars(...)`, add `if (usb_marker) ui_draw_usb(1);`. Move the `usb_marker` declaration above `draw_bars`.
8. In `power_down()`, call `adc_abort();` first, so the ring DMA is stopped before power-off.

- [ ] **Step 5: Build and test**

Run: `make -C fw clean everything 2>&1 | grep -ciE "warning|error"`, `make -C tests` and `python3 -m unittest discover -s tools/tests`
Expected: `0`, `… 0 failed`, and `OK`. Also check RAM: `arm-none-eabi-size fw/build/app.elf`. bss must stay below 64 KiB, leaving at least 32 KiB for the stack and margin within the 96 KiB RAM.

- [ ] **Step 6: Commit**

```bash
git add fw/app/stream.h fw/app/stream.c fw/app/ui.h fw/app/ui.c fw/app/main.c
git commit -m "app: live streaming (frames + logger mode) over the CDC serial port

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 9: PC client + live GUI

**Files:**
- Create: `tools/scopeclient.py`, `tools/scope_live.py`, `tools/requirements.txt`, `tools/README.md`, `tools/tests/test_scopeclient.py`

**Interfaces:**
- Consumes: `dsoproto` (Task 7).
- Produces: `scopeclient.find_port()` and `scopeclient.ScopeClient(port=None, opener=None)`. The client has `.start(mode, rate)`, `.stop()`, `.packets(timeout)` and `.close()`, plus `.connected` and `.mode` attributes. The GUI is `python3 tools/scope_live.py [--port P]`.

`ScopeClient` runs a reader thread. It opens the port (pyserial by default, or the injected `opener` in tests), decodes packets, and queues them. On error or EOF it marks itself disconnected, retries the open every 0.5 s, and on reconnect re-sends the last start command (`FRAMES` or `LOG <rate>`).

- [ ] **Step 1: Write the failing test**

`tools/tests/test_scopeclient.py`:
```python
import os
import queue
import sys
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import scopeclient  # noqa: E402

VEC = os.path.join(HERE, "..", "..", "tests", "vectors")


class FakePort:
    """Serves bytes from a script; records writes; can 'unplug'."""

    def __init__(self, chunks, fail_after=None):
        self.chunks = list(chunks)
        self.written = b""
        self.fail_after = fail_after
        self.reads = 0
        self.closed = False

    def read(self, n):
        self.reads += 1
        if self.fail_after is not None and self.reads > self.fail_after:
            raise OSError("unplugged")
        if self.chunks:
            return self.chunks.pop(0)
        time.sleep(0.01)
        return b""

    def write(self, b):
        self.written += b

    def close(self):
        self.closed = True


class TestClient(unittest.TestCase):
    def test_decodes_and_sends_commands(self):
        with open(os.path.join(VEC, "stream.bin"), "rb") as f:
            data = f.read()
        port = FakePort([data[:50], data[50:]])
        c = scopeclient.ScopeClient(port="fake", opener=lambda p: port)
        c.start("frames")
        got = []
        deadline = time.time() + 2
        while len(got) < 6 and time.time() < deadline:
            got += c.packets(timeout=0.1)
        c.close()
        self.assertEqual([type(p).__name__ for p in got], ["Info", "Frame", "Log", "Drop", "Ok", "Err"])
        self.assertIn(b"FRAMES\n", port.written)

    def test_reconnects_and_resumes(self):
        ports = [FakePort([], fail_after=2), FakePort([])]
        opened = []

        def opener(p):
            if not ports:
                raise OSError("no device")
            opened.append(ports[0])
            return ports.pop(0)

        c = scopeclient.ScopeClient(port="fake", opener=opener, retry_s=0.05)
        c.start("log", 100000)
        deadline = time.time() + 3
        while len(opened) < 2 and time.time() < deadline:
            time.sleep(0.05)
        time.sleep(0.2)
        c.close()
        self.assertEqual(len(opened), 2)
        self.assertIn(b"LOG 100000\n", opened[1].written)   # resumed after the reconnect


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `python3 -m unittest discover -s tools/tests -v`
Expected: `ModuleNotFoundError: No module named 'scopeclient'`.

- [ ] **Step 3: Implement `tools/scopeclient.py`**

```python
"""Serial connection to the DSO-510: reader thread, decoding, auto-reconnect, commands."""
import queue
import threading
import time

import dsoproto

VID, PID = 0x2E3C, 0x5720


def find_port():
    """First CDC-ACM port of the scope (pyserial's list_ports), or None."""
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == VID and p.pid == PID:
            return p.device
    return None


def _serial_opener(port):
    import serial
    return serial.Serial(port, 115200, timeout=0.05)


class ScopeClient:
    def __init__(self, port=None, opener=None, retry_s=0.5):
        self.port_name = port
        self.opener = opener or _serial_opener
        self.retry_s = retry_s
        self.q = queue.Queue()
        self.connected = False
        self.mode = None            # None, "frames" or "log"
        self.rate = None
        self._port = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _command(self):
        if self.mode == "frames":
            return b"FRAMES\n"
        if self.mode == "log":
            return b"LOG %d\n" % self.rate
        return None

    def _send(self, data):
        with self._lock:
            if self._port is not None:
                try:
                    self._port.write(data)
                except OSError:
                    pass

    def start(self, mode, rate=None):
        self.mode, self.rate = mode, rate
        cmd = self._command()
        if cmd:
            self._send(cmd)

    def stop(self):
        self.mode = None
        self._send(b"STOP\n")

    def packets(self, timeout=0.05):
        out = []
        try:
            out.append(self.q.get(timeout=timeout))
            while True:
                out.append(self.q.get_nowait())
        except queue.Empty:
            pass
        return out

    def close(self):
        self._stop.set()
        self._t.join(timeout=2)
        with self._lock:
            if self._port is not None:
                try:
                    self._port.close()
                except OSError:
                    pass
                self._port = None

    def _run(self):
        while not self._stop.is_set():
            try:
                name = self.port_name or find_port()
                if not name:
                    raise OSError("scope not found")
                p = self.opener(name)
            except Exception:                       # noqa: BLE001 - any open failure means retry
                time.sleep(self.retry_s)
                continue
            dec = dsoproto.Decoder()
            with self._lock:
                self._port = p
            self.connected = True
            cmd = self._command()
            if cmd:
                self._send(cmd)                     # resume the previous mode
            try:
                while not self._stop.is_set():
                    data = p.read(4096)
                    if data:
                        for pk in dec.feed(data):
                            self.q.put(pk)
            except Exception:                       # noqa: BLE001 - unplug / soft-disconnect
                pass
            self.connected = False
            with self._lock:
                self._port = None
            try:
                p.close()
            except Exception:                       # noqa: BLE001
                pass
            if not self._stop.is_set():
                time.sleep(self.retry_s)
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `python3 -m unittest discover -s tools/tests -v`
Expected: all `ok`. These tests need no pyserial: the opener is injected.

- [ ] **Step 5: `tools/scope_live.py` (GUI)**

```python
#!/usr/bin/env python3
"""Live plot of the DSO-510 stream (frames or logger mode), with CSV recording.

pip install -r tools/requirements.txt ; python3 tools/scope_live.py [--port /dev/ttyACM0]
"""
import argparse
import collections
import os
import sys
import time

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import dsoproto as dp  # noqa: E402
from scopeclient import ScopeClient  # noqa: E402

RATES = [1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000]


class Window(QtWidgets.QWidget):
    def __init__(self, client):
        super().__init__()
        self.c = client
        self.fw = "?"
        self.logmeta = None
        self.log_t = collections.deque()
        self.log_v = collections.deque()
        self.frame_csv_dir = None
        self.frame_count = 0
        self.log_csv = None
        self.log_writer = None
        self.stats = {"frames": 0, "samples": 0, "drops": 0, "t0": time.time()}

        self.setWindowTitle("DSO-510 live")
        self.plot = pg.PlotWidget()
        self.plot.setLabel("bottom", "time", units="s")
        self.plot.setLabel("left", "voltage", units="V")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.curve = self.plot.plot(pen=pg.mkPen("y", width=1), connect="finite")

        self.mode = QtWidgets.QComboBox(); self.mode.addItems(["Frames", "Logger"])
        self.rate = QtWidgets.QComboBox(); self.rate.addItems([str(r) for r in RATES]); self.rate.setCurrentText("10000")
        self.window_s = QtWidgets.QDoubleSpinBox(); self.window_s.setRange(0.1, 600); self.window_s.setValue(5); self.window_s.setSuffix(" s")
        self.btn = QtWidgets.QPushButton("Start")
        self.rec = QtWidgets.QCheckBox("Record CSV")
        self.status = QtWidgets.QLabel("connecting…")

        bar = QtWidgets.QHBoxLayout()
        for w in (QtWidgets.QLabel("Mode"), self.mode, QtWidgets.QLabel("Logger rate"), self.rate,
                  QtWidgets.QLabel("Window"), self.window_s, self.btn, self.rec):
            bar.addWidget(w)
        bar.addStretch(1)
        lay = QtWidgets.QVBoxLayout(self)
        lay.addLayout(bar)
        lay.addWidget(self.plot, 1)
        lay.addWidget(self.status)

        self.btn.clicked.connect(self.toggle)
        self.rec.toggled.connect(self.toggle_record)
        self.running = False
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(30)

    # --- controls -------------------------------------------------------------------------------
    def toggle(self):
        if self.running:
            self.c.stop(); self.running = False; self.btn.setText("Start")
            return
        self.log_t.clear(); self.log_v.clear(); self.logmeta = None
        if self.mode.currentText() == "Frames":
            self.c.start("frames")
        else:
            self.c.start("log", int(self.rate.currentText()))
        self.stats.update(frames=0, samples=0, drops=0, t0=time.time())
        self.running = True; self.btn.setText("Stop")

    def toggle_record(self, on):
        if not on:
            if self.log_csv:
                self.log_csv.close(); self.log_csv = None; self.log_writer = None
            self.frame_csv_dir = None
            return
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Record CSV files into")
        if not d:
            self.rec.setChecked(False); return
        stamp = time.strftime("%Y%m%d-%H%M%S")
        if self.mode.currentText() == "Frames":
            self.frame_csv_dir = os.path.join(d, "frames-" + stamp); os.makedirs(self.frame_csv_dir, exist_ok=True)
            self.frame_count = 0
        else:
            self.log_csv = open(os.path.join(d, "log-%s.csv" % stamp), "w", newline="")
            self.log_writer = dp.LogCsvWriter(self.log_csv, self.fw)
            if self.logmeta:
                self.log_writer.meta(self.logmeta)

    # --- data -------------------------------------------------------------------------------------
    def on_packet(self, p):
        if isinstance(p, dp.Info):
            self.fw = p.fields.get("fw", self.fw)
        elif isinstance(p, dp.Frame):
            self.stats["frames"] += 1
            m = p.meta
            n = len(p.samples)
            t = (np.arange(n) - m.trig_index) / m.rate_hz
            v = (np.frombuffer(p.samples, dtype=np.uint8).astype(np.int32) - m.zero_code) * (m.uv_per_code() / 1e6)
            self.curve.setData(t, v)
            if self.frame_csv_dir:
                self.frame_count += 1
                with open(os.path.join(self.frame_csv_dir, "frame_%04d.csv" % self.frame_count), "w", newline="") as f:
                    f.write(dp.frame_csv_text(m, p.samples, self.fw))
        elif isinstance(p, dp.LogMeta):
            self.logmeta = p.meta
            if self.log_writer:
                self.log_writer.meta(p.meta)
        elif isinstance(p, dp.Log) and self.logmeta:
            m = self.logmeta
            n = len(p.samples)
            self.stats["samples"] += n
            t = (p.first_index + np.arange(n)) / m.rate_hz
            v = (np.frombuffer(p.samples, dtype=np.uint8).astype(np.int32) - m.zero_code) * (m.uv_per_code() / 1e6)
            self.log_t.extend(t.tolist()); self.log_v.extend(v.tolist())
            if self.log_writer:
                self.log_writer.log(p)
        elif isinstance(p, dp.Drop):
            self.stats["drops"] += p.count
            if self.logmeta:                          # break the line at the gap
                self.log_t.append(p.first_index / self.logmeta.rate_hz); self.log_v.append(float("nan"))
            if self.log_writer:
                self.log_writer.drop(p)
        elif isinstance(p, dp.Err):
            self.status.setText("scope: " + p.text)

    def tick(self):
        for p in self.c.packets(timeout=0):
            self.on_packet(p)
        if self.mode.currentText() == "Logger" and self.log_t:
            horizon = self.log_t[-1] - self.window_s.value()
            while self.log_t and self.log_t[0] < horizon:
                self.log_t.popleft(); self.log_v.popleft()
            step = max(1, len(self.log_t) // 20000)     # keep redraws fast
            t = np.fromiter(self.log_t, float)[::step]; v = np.fromiter(self.log_v, float)[::step]
            self.curve.setData(t, v)
        dt = max(1e-3, time.time() - self.stats["t0"])
        conn = "connected" if self.c.connected else "reconnecting…"
        if self.mode.currentText() == "Frames":
            self.status.setText("%s · %.1f frames/s · fw %s" % (conn, self.stats["frames"] / dt, self.fw))
        else:
            self.status.setText("%s · %d samples (%.0f S/s) · dropped %d · fw %s" % (
                conn, self.stats["samples"], self.stats["samples"] / dt, self.stats["drops"], self.fw))

    def closeEvent(self, e):
        if self.log_csv:
            self.log_csv.close()
        self.c.stop(); self.c.close()
        e.accept()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="serial port (default: find the scope by USB ID)")
    a = ap.parse_args()
    app = pg.mkQApp("DSO-510 live")
    w = Window(ScopeClient(port=a.port))
    w.resize(1100, 600); w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
```

`tools/requirements.txt`:
```
pyserial>=3.5
pyqtgraph>=0.13
PyQt5>=5.15
numpy>=1.24
```

`tools/README.md` must cover, in about 30 lines:
- installing into a venv (`python3 -m venv .venv && .venv/bin/pip install -r tools/requirements.txt`)
- running `scope_live.py`
- the serial-port permission on Linux (the `dialout` group, or `uucp` on Arch)
- what each mode shows
- recording behaviour
- that `dsoproto.py` is a stdlib-only library for your own scripts, with a 5-line example that reads a recorded CSV and plots it with matplotlib
- the test commands

- [ ] **Step 6: Syntax-check the GUI without its dependencies**

Run: `python3 -m py_compile tools/scope_live.py tools/scopeclient.py tools/dsoproto.py && python3 -m unittest discover -s tools/tests -v`
Expected: no output from `py_compile`, and all tests `ok`.

- [ ] **Step 7: Commit**

```bash
git add tools/scopeclient.py tools/scope_live.py tools/requirements.txt tools/README.md tools/tests/test_scopeclient.py
git commit -m "tools: scopeclient (reader thread, auto-reconnect) + scope_live pyqtgraph GUI + README

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 10: On-device verification (controller with the user)

**Files:**
- Modify: `NOTES.md`, `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` (status line)

- [ ] **Step 1: Set up the PC.** Create a venv: `python3 -m venv ~/.venvs/dso && ~/.venvs/dso/bin/pip install -r tools/requirements.txt`. Ask the user before installing. Check that the user can open `/dev/ttyACM0`.
- [ ] **Step 2: Enumeration.** Flash `APP_DSO-510_app.bin`. The kernel log should show both `usb-storage` and `cdc_acm`. The drive mounts, and `/dev/ttyACM0` exists.
- [ ] **Step 3: Frames.** Connect the generator sine, then press Auto. Run `scope_live.py` and choose Frames. The plot matches the scope at ≥ 10 frames/s, and the top bar shows `USB`. Change the timebase and V/div on the scope; the plot follows. Record 5 frames, and check that one CSV equals the scope's own save of the same signal in format (header and columns).
- [ ] **Step 4: Logger.** Run at 10 kS/s for 2 minutes, then at 200 kS/s for 2 minutes: dropped stays 0. Run 500 kS/s and record the dropped rate. Press Up/Down on the scope while logging; the volts stay right because LOGMETA is re-sent. Run stops the logger, and the scope screen returns.
- [ ] **Step 5: Robustness.**
  - Unplug and replug during frames: the GUI reconnects and resumes.
  - Do a CSV save on the scope while frames are streaming. It soft-disconnects, then the stream resumes.
  - Copy a file from the drive while logging at 100 kS/s, and note any drops.
  - Close the GUI while logging: the scope returns to scope mode.
- [ ] **Step 6: Record and commit.** Write the results (throughput, drop rates, frames/s) into `NOTES.md`, and mark sub-project 4 done in the spec status line. Then:
```bash
git add NOTES.md docs/superpowers/specs/2026-11-13-dso510-firmware-design.md
git commit -m "Notes: sub-project 4 verified on hardware; spec status

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```
