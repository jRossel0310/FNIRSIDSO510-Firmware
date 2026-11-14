# Interleaved Capture (Sub-project 1b) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Real-time 24 and 48 MS/s capture in the scope app, with the 50, 100 and 200 ns/div timebases.

**Architecture:** Pure, host-tested maths in `fw/core` (rate table with a stream count, weaving of the per-stream buffers into one record, calibration set choice). A driver extension in `fw/common/adc.[ch]` runs the four-timer, four-DMA-channel capture exactly as measured with the `ilvtest` bench. The scope app captures the raw streams into a separate buffer and weaves them into the normal record, so trigger, measurements, CSV and streaming need no change.

**Tech Stack:** arm-none-eabi-gcc (Cortex-M4F, no libm), host `cc` tests.

**Spec:** `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` § "Sub-project 1b: interleaved capture (24 / 48 MS/s)". Measurements: `NOTES.md` § "Interleaved capture experiments". Reference implementation of the register sequence: `fw/apps/ilvtest/main.c` (function `ilv_capture`, the `keep_clock` path).

## Global Constraints

- Never write or erase internal flash below 0x08007000. Never touch 0x08006000. SPI flash writes only inside `[0, 0x500000)`.
- Every app implements power-off (unchanged here).
- `fw/core/*` includes no hardware header, compiles with host `cc -std=c99 -Wall -Wextra -Werror`, uses no libm.
- Firmware builds with 0 warnings: `make -C fw clean everything 2>&1 | grep -ciE "warning|error"` prints `0`. Host tests: `make -C tests` ends `… 0 failed`. Python: `python3 -m unittest discover -s tools/tests` ends `OK`.
- bss of `fw/build/app.elf` stays below 73728 bytes (72 KiB).
- Timebases: 19 entries, 50 ns … 50 ms per division, 1-2-5.
- Rates: 10 µs/div and slower unchanged (single stream, ≤ 12 MS/s). 5 µs/div = 24 MS/s (2 streams). 2 µs/div and faster = 48 MS/s (4 streams).
- Stream k of an interleaved capture: 0 = TMR3 → DMA1 ch3 (request 0x47), 1 = TMR5 → DMA2 ch2 (0x57), 2 = TMR1 → DMA1 ch5 (0x37), 3 = TMR8 → DMA2 ch1 (0x6F). Record sample j = stream j mod 4, index j / 4. 24 MS/s uses streams 0 and 2.
- The ADC clock (TMR4 CH3 on PB8) never stops. In interleaved mode it runs at 48 MHz (reload 4, compare 3).
- Slave timers: gated mode (SMS 5), reload preload on, first reload 7, preloaded reload 3, start counts 6, 5, 4, 3 for streams 0..3.
- DMA2 channel 3 (generator, flex request 0x05 in byte 2 of DMA2 `SRC_SEL0`) and DMA1 channel 7 (single-stream capture, byte 2 of DMA1 `SRC_SEL1`) mappings must be preserved by every mapping write.
- The generator, CSV save, USB drive, frame streaming and the logger must keep working unchanged.
- Subagents never flash and never open serial or USB devices.
- Commit after every task with targeted `git add` (never `-A`, never `.superpowers/`). Every commit message ends with the line `Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp`.

---

### Task 1: Core — rate table with streams, weaving, calibration set

**Files:**
- Modify: `fw/core/units.h`, `fw/core/units.c`, `fw/core/cal.c`, `tests/test_units.c`, `tests/test_cal.c`, `tests/main.c`
- Create: `fw/core/ilv.h`, `fw/core/ilv.c`, `tests/test_ilv.c`
- Modify as needed: any other host test that hard-codes a timebase index or `NUM_TDIV` (search `tests/` for `tdiv`).

**Interfaces:**
- Produces:
  - `NUM_TDIV` 19; `tdiv_ns[]` starts 50, 100, 200, 500, …; `TDIV_10US` 7; `TDIV_1MS` 13; new `TDIV_2US` 5.
  - `MAX_RATE_HZ` stays 12000000 (single-stream limit); new `ILV_RATE_HZ` 48000000.
  - `rate_cfg_t` gains `uint8_t streams;` — 1 = single stream (psc/arr are the sample timer setting), 2 or 4 = interleaved (psc 0, arr 4: the 48 MHz ADC clock).
  - `void ilv_weave(uint8_t *dst, const uint8_t *const *src, int streams, uint32_t per);`
  - `cal_set_for_rate` returns `CAL_SET_FAST` above 12 MS/s.

- [ ] **Step 1: Write the failing tests**

In `tests/test_units.c`, replace the first check (`units_rate_for_tdiv(500)` "capped at 12 MS/s") and add, keeping every other existing expectation:
```c
    r = units_rate_for_tdiv(50);           /* 48 MS/s, 4 streams */
    CHECK_EQ(r.streams, 4); CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 4); CHECK_EQ(r.rate_hz, 48000000);
    r = units_rate_for_tdiv(500);
    CHECK_EQ(r.streams, 4); CHECK_EQ(r.rate_hz, 48000000);
    r = units_rate_for_tdiv(2000);         /* 96 samples/div */
    CHECK_EQ(r.streams, 4); CHECK_EQ(r.rate_hz, 48000000);
    r = units_rate_for_tdiv(5000);         /* 120 samples/div at 24 MS/s, 2 streams */
    CHECK_EQ(r.streams, 2); CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 4); CHECK_EQ(r.rate_hz, 24000000);
    r = units_rate_for_tdiv(10000);        /* single stream from here on */
    CHECK_EQ(r.streams, 1); CHECK_EQ(r.arr, 19); CHECK_EQ(r.rate_hz, 12000000);
    r = units_rate_for_tdiv(50000000);
    CHECK_EQ(r.streams, 1);
    CHECK_EQ(NUM_TDIV, 19); CHECK_EQ(tdiv_ns[0], 50); CHECK_EQ(tdiv_ns[2], 200); CHECK_EQ(tdiv_ns[3], 500);
    CHECK_EQ(tdiv_ns[TDIV_2US], 2000); CHECK_EQ(tdiv_ns[NUM_TDIV - 1], 50000000);
    for (int i = 0; i < NUM_TDIV; i++) {   /* every record splits evenly over the streams */
        rate_cfg_t c = units_rate_for_tdiv(tdiv_ns[i]);
        uint32_t n = 4u * units_samples_per_screen(c.rate_hz, tdiv_ns[i]);
        if (n == 0 || n % c.streams) { CHECK(0); break; }
    }
    CHECK_EQ(units_samples_per_screen(48000000, 50), 24);
    CHECK_EQ(units_rate_for_hz(500000).streams, 1);
```
The existing line `CHECK_EQ(tdiv_ns[TDIV_10US], 10000); CHECK_EQ(tdiv_ns[TDIV_1MS], 1000000);` must keep passing.

In `tests/test_cal.c`, next to the existing `cal_set_for_rate` checks, add:
```c
    CHECK_EQ(cal_set_for_rate(12000000), CAL_SET_MID);
    CHECK_EQ(cal_set_for_rate(24000000), CAL_SET_FAST);
    CHECK_EQ(cal_set_for_rate(48000000), CAL_SET_FAST);
```

`tests/test_ilv.c` (register `test_ilv` in `tests/main.c`):
```c
#include "test.h"
#include "ilv.h"

void test_ilv(void) {
    uint8_t s0[4] = {0, 4, 8, 12}, s1[4] = {1, 5, 9, 13}, s2[4] = {2, 6, 10, 14}, s3[5] = {99, 3, 7, 11, 15};
    uint8_t out[17];
    memset(out, 0xEE, sizeof out);
    const uint8_t *four[4] = {s0, s1, s2, s3 + 1};      /* stream 3 skips its stale first sample */
    ilv_weave(out, four, 4, 4);
    for (int i = 0; i < 16; i++) if (out[i] != i) { CHECK_EQ(out[i], i); break; }
    CHECK_EQ(out[16], 0xEE);                             /* nothing written past streams * per */
    memset(out, 0xEE, sizeof out);
    const uint8_t *two[2] = {s0, s2};
    ilv_weave(out, two, 2, 4);
    CHECK_EQ(out[0], 0); CHECK_EQ(out[1], 2); CHECK_EQ(out[2], 4); CHECK_EQ(out[7], 14); CHECK_EQ(out[8], 0xEE);
    memset(out, 0xEE, sizeof out);
    ilv_weave(out, four, 4, 0);
    CHECK_EQ(out[0], 0xEE);
}
```

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile errors (`streams`, `ilv.h`, `TDIV_2US`).

- [ ] **Step 3: Implement**

`fw/core/units.h`: `NUM_TDIV 19`, `TDIV_2US 5`, `TDIV_10US 7`, `TDIV_1MS 13`, add `#define ILV_RATE_HZ 48000000u`, add `uint8_t streams;` to `rate_cfg_t`, and replace the comment on `units_rate_for_tdiv` with:
```c
/* Sample rate for a timebase: 120 samples/div where possible. Up to 12 MS/s a single stream
   (streams = 1, psc/arr = sample timer). Above that the capture is interleaved with the ADC clocked
   at 48 MHz (psc 0, arr 4): streams = 2 gives 24 MS/s, streams = 4 gives 48 MS/s. */
```

`fw/core/units.c`: prepend `50, 100, 200,` to `tdiv_ns`, and:
```c
rate_cfg_t units_rate_for_tdiv(uint32_t tdiv) {
    /* timer ticks per sample = 240e6 * tdiv[s] / 120 = tdiv_ns / 500 */
    uint32_t div = tdiv / 500;
    uint32_t min_div = TIMER_CLK_HZ / MAX_RATE_HZ;
    rate_cfg_t r;
    if (div < min_div) {                                  /* faster than one stream can go */
        r.psc = 0; r.arr = (uint16_t)(TIMER_CLK_HZ / ILV_RATE_HZ - 1u);
        r.streams = div >= min_div / 2 ? 2 : 4;
        r.rate_hz = ILV_RATE_HZ / 4u * r.streams;
        return r;
    }
    r.streams = 1;
    if (div <= 65536) { r.psc = 0; r.arr = (uint16_t)(div - 1); }
    else { r.psc = (uint16_t)(div / 1000 - 1); r.arr = 999; }
    r.rate_hz = TIMER_CLK_HZ / div;
    return r;
}
```
In `units_rate_for_hz`, initialise `streams` to 1 in every returned value (the zero result too).

`fw/core/cal.c`:
```c
int cal_set_for_rate(uint32_t rate_hz) {
    return rate_hz > 12000000u ? CAL_SET_FAST : rate_hz > 2400000u ? CAL_SET_MID : CAL_SET_SLOW;
}
```

`fw/core/ilv.h`:
```c
#pragma once
#include <stdint.h>
/* Interleaved capture: weave `streams` equally long sample streams into one record.
   dst[i * streams + k] = src[k][i] for i < per. src[k] already points at stream k's first valid sample. */
void ilv_weave(uint8_t *dst, const uint8_t *const *src, int streams, uint32_t per);
```
`fw/core/ilv.c`:
```c
#include "ilv.h"

void ilv_weave(uint8_t *dst, const uint8_t *const *src, int streams, uint32_t per) {
    for (int k = 0; k < streams; k++) {
        const uint8_t *s = src[k];
        uint8_t *d = dst + k;
        for (uint32_t i = 0; i < per; i++, d += streams) *d = s[i];
    }
}
```

- [ ] **Step 4: Fix other users of timebase indices**

Search `fw/` and `tests/` for `tdiv` and `NUM_TDIV`. Anything that uses a literal index into `tdiv_ns` (tests of control, autoset, render, csv; `state_defaults` uses `TDIV_1MS` and needs no change) must now mean the same timebase as before. Update literals in tests to keep each test's intent, and list every changed line in your report. `fw/core/autoset.c` picks from the whole table and needs no change; add one check to `tests/test_autoset.c` only if an existing expectation there encodes an index.

- [ ] **Step 5: Run tests and build**

Run: `make -C tests` and `make -C fw clean everything 2>&1 | grep -ciE "warning|error"`
Expected: `… 0 failed` and `0`. (The firmware still captures single-stream at every timebase after this task; at 5 µs/div and faster the app would program TMR4 for 48 MHz as a sample clock, which Task 2 replaces. Do not flash this intermediate state.)

- [ ] **Step 6: Commit**

```bash
git add fw/core/units.h fw/core/units.c fw/core/cal.c fw/core/ilv.h fw/core/ilv.c tests/test_units.c tests/test_cal.c tests/test_ilv.c tests/main.c
# plus any other test file changed in Step 4, named explicitly
git commit -m "core: 19 timebases, rate table with interleave streams, ilv_weave, fast cal set

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 2: Interleaved capture driver and app wiring

**Files:**
- Modify: `fw/common/adc.h`, `fw/common/adc.c`, `fw/app/main.c`

**Interfaces:**
- Consumes: `rate_cfg_t.streams`, `ilv_weave`, `CAL_SET_FAST`, `TDIV_2US` (Task 1).
- Produces (in `adc.h`):
```c
// Interleaved capture (24 / 48 MS/s): ADC clocked at 48 MHz, 2 or 4 DMA streams at 12 MS/s each.
#define ADC_ILV_PAD 1u                     // extra sample armed per stream (a stale request may eat one)
void adc_ilv_enter(void);                  // ADC clock to 48 MHz, slaves frozen; waits for the pipeline flush
// Capture `per` samples on each of `streams` (2 or 4) streams. raw must hold streams * (per + ADC_ILV_PAD) bytes.
void adc_ilv_start(uint8_t *raw, uint16_t per, int streams);
int adc_ilv_done(void);                    // 1 once every stream is complete (or none armed)
const uint8_t *adc_ilv_stream(int k);      // first valid sample of logical stream k of the last capture
```

- [ ] **Step 1: Driver**

Read `fw/apps/ilvtest/main.c` (`ilv_capture`, `slaves_off`, `tmr4_clock`) first: it is the sequence proven on hardware. The driver below is that sequence with reload preload and stale-sample accounting added (both verified on hardware by register pokes, see `NOTES.md`).

Add to `fw/common/adc.c` (keep everything that is there):
```c
#define TMR1_BASE 0x40012C00u
#define TMR3_BASE 0x40000400u
#define TMR5_BASE 0x40000C00u
#define TMR8_BASE 0x40013400u
#define TMR_CTRL2(t)  REG32((t) + 0x04)
#define TMR_STCTRL(t) REG32((t) + 0x08)
#define TMR_CVAL(t)   REG32((t) + 0x24)
#define DMA1_SRC_SEL0 REG32(0x400200A0)
#define DMA2_CLR      REG32(0x40020404)
#define DMA2_SRC_SEL0 REG32(0x400204A0)
#define DMA2_SRC_SEL1 REG32(0x400204A4)
#define D_CTRL(d)  REG32((d) + 0x00)
#define D_CNT(d)   REG32((d) + 0x04)
#define D_PADDR(d) REG32((d) + 0x08)
#define D_MADDR(d) REG32((d) + 0x0C)

/* stream k: slave timer, its trigger-select bits (ITR to TMR4), DMA channel, start count */
static const struct { uint32_t tmr, ts, dma, cv; } ilv_s[4] = {
    {TMR3_BASE, 0x30, 0x40020030, 6},   /* DMA1 ch3 */
    {TMR5_BASE, 0x20, 0x4002041C, 5},   /* DMA2 ch2 */
    {TMR1_BASE, 0x30, 0x40020058, 4},   /* DMA1 ch5 */
    {TMR8_BASE, 0x20, 0x40020408, 3},   /* DMA2 ch1 */
};
static uint8_t ilv_mode;                 /* TMR4 is in the 48 MHz interleave configuration */
static uint8_t ilv_n;                    /* streams of the capture in progress / last capture */
static uint8_t ilv_busy;
static uint8_t ilv_hw[4];                /* logical stream -> hardware stream */
static const uint8_t *ilv_first[4];      /* logical stream -> first valid sample */

static void ilv_slaves_off(void) {
    TMR_CTRL2(TMR4_BASE) &= ~0x70u;      /* TRGO idle: the slaves cannot count */
    for (int k = 0; k < 4; k++) {
        TMR_IDEN(ilv_s[k].tmr) &= ~0x100u;
        TMR_CTRL1(ilv_s[k].tmr) &= ~1u;
        D_CTRL(ilv_s[k].dma) = 0;
    }
    DMA1_CLR = 0x000F0F00u;              /* ch3, ch5 flags */
    DMA2_CLR = 0x000000FFu;              /* ch1, ch2 flags */
    ilv_busy = 0;
}

void adc_ilv_enter(void) {
    adc_abort();
    CRM_AHBEN |= 3u;                                         /* DMA1, DMA2 */
    CRM_APB1EN |= (1u << 1) | (1u << 2) | (1u << 3);         /* TMR3, TMR4, TMR5 */
    CRM_APB2EN |= (1u << 11) | (1u << 13);                   /* TMR1, TMR8 */
    ilv_slaves_off();
    DMA2_SRC_SEL1 |= 1u << 24;                               /* flexible mapping */
    DMA2_SRC_SEL0 = (DMA2_SRC_SEL0 & 0xFFFF0000u) | 0x576Fu; /* ch1 = TMR8 overflow, ch2 = TMR5 overflow */
    DMA1_SRC_SEL1 |= 1u << 24;
    DMA1_SRC_SEL0 = (DMA1_SRC_SEL0 & 0xFF00FFFFu) | 0x470000u;   /* ch3 = TMR3 overflow */
    DMA1_SRC_SEL1 = (DMA1_SRC_SEL1 & 0xFFFFFF00u) | 0x37u;       /* ch5 = TMR1 overflow */
    if (!ilv_mode) {
        TMR_CTRL1(TMR4_BASE) &= ~1u;
        TMR_IDEN(TMR4_BASE) &= ~0x100u;                      /* no DMA request of its own */
        TMR_DIV(TMR4_BASE) = 0;
        TMR_PR(TMR4_BASE) = 4;                               /* 48 MHz */
        TMR_C3DT(TMR4_BASE) = 3;
        TMR_STCTRL(TMR4_BASE) |= 0x80u;                      /* master/slave sync, as stock */
        TMR_SWEVT(TMR4_BASE) = 1;
        TMR_CTRL1(TMR4_BASE) |= 1u;
        ilv_mode = 1;
        delay_ms(2);                                         /* > 64 conversions: pipeline flushed */
    }
}

void adc_ilv_start(uint8_t *raw, uint16_t per, int streams) {
    if (!ilv_mode) adc_ilv_enter();
    ilv_slaves_off();
    ilv_n = (uint8_t)(streams == 2 ? 2 : 4);
    uint32_t armed = (uint32_t)per + ADC_ILV_PAD;
    for (int j = 0; j < ilv_n; j++) {
        int k = ilv_n == 2 ? 2 * j : j;                      /* 24 MS/s: hardware streams 0 and 2 */
        uint32_t t = ilv_s[k].tmr, d = ilv_s[k].dma;
        uint8_t *base = raw + (uint32_t)j * armed;
        ilv_hw[j] = (uint8_t)k;
        TMR_CTRL1(t) = 0x80u;                                /* reload preload on, stopped */
        TMR_DIV(t) = 0;
        TMR_PR(t) = 7;                                       /* first period: 8 ticks */
        TMR_SWEVT(t) = 1;                                    /* load it */
        TMR_PR(t) = 3;                                       /* from the first overflow on: 4 ticks = 12 MHz */
        TMR_STCTRL(t) = (TMR_STCTRL(t) & ~0x77u) | 5u | ilv_s[k].ts;   /* gated by TMR4's TRGO */
        TMR_CVAL(t) = ilv_s[k].cv;
        D_CTRL(d) = 0;
        D_PADDR(d) = 0x40010C08;                             /* GPIOB->IDT */
        D_MADDR(d) = (uint32_t)base;
        D_CNT(d) = armed;
        D_CTRL(d) = 0x2000u | 0x80u | 1u;                    /* high priority, 8-bit, memory increment, enable */
        TMR_IDEN(t) |= 0x100u;                               /* overflow DMA request */
        TMR_CTRL1(t) |= 1u;                                  /* enabled, but frozen until TRGO runs */
    }
    for (volatile int w = 0; w < 200; w++) { }               /* let any stale request be served */
    for (int j = 0; j < ilv_n; j++) {
        uint32_t skip = armed - D_CNT(ilv_s[ilv_hw[j]].dma); /* 0 or 1 samples taken before the release */
        ilv_first[j] = raw + (uint32_t)j * armed + skip;
        if (skip > ADC_ILV_PAD) ilv_first[j] = raw + (uint32_t)j * armed + ADC_ILV_PAD;
    }
    ilv_busy = 1;
    TMR_CTRL2(TMR4_BASE) = (TMR_CTRL2(TMR4_BASE) & ~0x70u) | 0x20u;   /* release: every slave sees the same ticks */
}

int adc_ilv_done(void) {
    if (!ilv_busy) return 1;
    for (int j = 0; j < ilv_n; j++) if (D_CNT(ilv_s[ilv_hw[j]].dma) != 0u) return 0;
    ilv_slaves_off();
    return 1;
}

const uint8_t *adc_ilv_stream(int k) { return ilv_first[k]; }
```
Every stream is armed for `per + 1` transfers, so when its count reaches 0 it holds at least `per` valid samples after its skip (skip 0: one spare at the end; skip 1: none spare). All streams started from the same release tick, so sample i after the skip is time-aligned across streams.

Also change the existing functions:
- `adc_set_rate`: before touching TMR4, leave interleave mode: `if (ilv_mode) { ilv_slaves_off(); ilv_mode = 0; }`, and after setting the reload make sure the single-stream DMA request is on again: `TMR_IDEN(TMR4_BASE) |= 0x100;`. (`adc_init` sets it once; interleave mode clears it.)
- `adc_abort`: add `if (ilv_busy) ilv_slaves_off();` at its start.
- `DMA1_SRC_SEL0` is new in this file; `DMA1_SRC_SEL1`, `DMA1_CLR`, `TMR_*` macros exist already. Reuse the existing ones; do not redefine.

- [ ] **Step 2: App wiring in `fw/app/main.c`**

Read the whole file first.
1. Include `ilv.h`. Add the raw buffer next to `recbuf`:
```c
static uint8_t ilv_raw[4 * (REC_MAX / 4 + ADC_ILV_PAD)];   /* per-stream buffers of an interleaved capture */
```
2. `set_time`: replace `adc_set_rate(rc.psc, rc.arr);` by
```c
    if (rc.streams > 1) adc_ilv_enter(); else adc_set_rate(rc.psc, rc.arr);
```
   `rec_n` stays `4 * sps` capped at `REC_MAX`; Task 1's test guarantees it divides by `rc.streams`.
3. Add two helpers above `capture_blocking` and use them everywhere a capture is armed or polled:
```c
/* Arm one capture of rec_n samples into rec. */
static void cap_start(void) {
    if (rc.streams > 1) adc_ilv_start(ilv_raw, (uint16_t)(rec_n / rc.streams), rc.streams);
    else adc_start(rec, (uint16_t)rec_n);
}

/* 1 once the armed capture is complete and rec holds the record. */
static int cap_done(void) {
    if (rc.streams == 1) return adc_done();
    if (!adc_ilv_done()) return 0;
    const uint8_t *src[4];
    for (int k = 0; k < rc.streams; k++) src[k] = adc_ilv_stream(k);
    ilv_weave(rec, src, rc.streams, (uint32_t)(rec_n / rc.streams));
    return 1;
}
```
   - `capture_blocking`: `adc_start(rec, …)` → `cap_start()`, `adc_done()` → `cap_done()`.
   - Main loop: `adc_start(rec, (uint16_t)rec_n); armed = 1;` → `cap_start(); armed = 1;` and `else if (adc_done())` → `else if (cap_done())`.
   - `cap_done()` weaves once per capture because `adc_ilv_done()` returns 1 only… **careful:** after completion `adc_ilv_done()` keeps returning 1 (`ilv_busy` is 0). Both callers stop polling after the first 1 (`armed = 0` / loop exit), so the weave runs once. Keep it that way: do not call `cap_done()` when nothing is armed.
   - Search the file for every other `adc_start(`, `adc_done(` and `adc_set_rate(` (auto-set, auto-zero, logger hooks). Scope-capture uses go through `cap_start`/`cap_done`/`set_time`. The logger (`hook_log_start`, ring capture) is single-stream: it calls `adc_set_rate`, which leaves interleave mode; make sure that when the logger stops, the scope's rate is restored through `set_time`/`apply_time` (it should already call it; verify and say what you found).
4. Auto-zero: `do_autozero` has `plan[2] = {{CAL_SET_SLOW, TDIV_1MS}, {CAL_SET_MID, TDIV_10US}}`. Make it three entries by adding `{ CAL_SET_FAST, TDIV_2US }`, and update the loop bound and anything sized by 2 (search for `plan[`).
5. Nothing else changes: `process_capture`, trigger, measurements, CSV and streaming read `rec`/`rc.rate_hz` as before. `capture_timeout_ms` works unchanged.

- [ ] **Step 3: Build and test**

Run: `make -C fw clean everything 2>&1 | grep -ciE "warning|error"`, `make -C tests`, `python3 -m unittest discover -s tools/tests`, `arm-none-eabi-size fw/build/app.elf`
Expected: `0`; `… 0 failed`; `OK`; bss below 73728 (report the line).

- [ ] **Step 4: Commit**

```bash
git add fw/common/adc.h fw/common/adc.c fw/app/main.c
git commit -m "adc: interleaved 24/48 MS/s capture (4 gated slave timers, 4 DMA streams); app weaves the streams into the record

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 3: On-device verification (controller with the user)

**Files:**
- Modify: `NOTES.md`, spec status line

- [ ] **Step 1:** Flash `APP_DSO-510_app.bin`. With the probe on the generator output: 1 kHz default sine, Auto → normal trace (regression check).
- [ ] **Step 2:** Controller, over USB frames: generator 50 kHz; user steps the timebase to 5 µs, 2 µs and 1 µs/div; frames report 24 / 48 / 48 MS/s; the record has no period-4 comb (per-phase residual < 0.3 code) and no period-2 comb at 24 MS/s.
- [ ] **Step 3:** User: 500, 200, 100, 50 ns/div draw a stable trace; back to 1 ms/div works; frequency reading right at 5 µs/div with a 50 kHz signal; CSV save at 2 µs/div (`sample_rate_hz=48000000`); auto-zero (long Auto with the input shorted) completes; logger start/stop from the PC returns to a working scope at a fast timebase.
- [ ] **Step 4:** Record results in `NOTES.md`, mark sub-project 1b done in the spec status line, commit.
