# Core Scope (Sub-project 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the test apps with a real oscilloscope firmware for the FNIRSI DSO-510: 240 MHz clock, timebases 500 ns/div–50 ms/div (up to 12 MS/s), 10 mV–10 V/div with factory offset calibration, Auto/Normal/Single software trigger, stock-like screen and controls, auto-set, auto-zero, and a visible crash screen.

**Architecture:** Hardware-free logic lives in `fw/core/` and is unit-tested on the PC (`tests/`, host gcc). Drivers live in `fw/common/`. The firmware app in `fw/app/` wires keys → `control_handle()` (state changes) → drivers, and capture → trigger → render → LCD. Everything is bare-metal C on the AT32F403A, no vendor SDK, linked at 0x08007000 behind FNIRSI's IAP bootloader.

**Tech Stack:** `arm-none-eabi-gcc` 14 (Cortex-M4F, hard float), GNU make, host `cc` for unit tests, Python 3 only for existing tools.

**Spec:** `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` (sub-project 1). Hardware facts: `NOTES.md`.

## Global Constraints

- App is linked at **0x08007000**; never write flash below 0x08007000. **Never erase or write the factory page 0x08006000.**
- Every app must implement power-off (release PD3), because the power button cannot cut power by itself.
- `fw/core/*` must not include any hardware header (`board.h`, `lcd.h`, `adc.h`, …) and must compile with host gcc `-std=c99 -Wall -Wextra -Werror`.
- System clock 240 MHz (HEXT 8 MHz /2 ×60); APB1 = APB2 = 120 MHz; timer clocks 240 MHz.
- ADC: 3PA1030 top 8 bits on PB0–PB7, clock = TMR4 CH3 on PB8, capture = DMA1 ch7 (flex request 0x4F). ADC clock must keep running between captures.
- Gain: 25 ADC codes per division; 0 V at code 128 + offset; offset −100…+100.
- V/div table (mV): `10,20,50,100,200,500,1000,2000,5000,10000`; range index r drives stock pin state r.
- Timebase table (ns/div): `500,1000,2000,5000,10000,20000,50000,100000,200000,500000,1000000,2000000,5000000,10000000,20000000,50000000`.
- Flashing: put scope in update mode (hold update button, press Power), then `tools/flash.sh <bin>`.
- Commit after every task. Commit message trailer: `Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp`.

## File Structure

| File | Responsibility |
|---|---|
| `tests/Makefile`, `tests/test.h`, `tests/main.c` | Host unit-test harness (`make -C tests`) |
| `tests/test_*.c` | One test file per core module |
| `fw/core/fmt.[ch]` | Number formatting with SI prefixes (`1.00kHz`, `500ns`) |
| `fw/core/units.[ch]` | V/div + timebase tables, sample-rate selection, code↔volts, battery % |
| `fw/core/trigger.[ch]` | Edge trigger search with hysteresis |
| `fw/core/render.[ch]` | Record → per-column min/max spans (decimate or interpolate) |
| `fw/core/keys.[ch]` | Logical key ids; debounce, short/long/repeat events |
| `fw/core/measure.[ch]` | Frequency, duty, min/max |
| `fw/core/cal.[ch]` | Factory calibration page parser, offset PWM, auto-zero search |
| `fw/core/control.[ch]` | Scope state + key event → state change mapping |
| `fw/core/autoset.[ch]` | Pick V/div and timebase from a measured signal |
| `fw/common/startup.c` | Vector table (weak handlers), FPU enable, libc bits |
| `fw/common/board.[ch]` | 240 MHz clocks, SysTick ms timer, power latch, keys, backlight, battery |
| `fw/common/adc.[ch]` | Free-running ADC clock + non-blocking DMA capture |
| `fw/common/frontend.[ch]` | Range pins, coupling, offset PWM |
| `fw/common/testsig.[ch]` | Cal-pad 1 kHz square, DAC2 bias, generator sine (test aid until sub-project 3) |
| `fw/common/fault.c` | HardFault screen |
| `fw/common/lcd.[ch]` | Existing ST7789 driver (unchanged) |
| `fw/app/ui.[ch]` | All drawing for the scope screen |
| `fw/app/main.c` | Init, main loop, acquisition, auto-set/auto-zero sequences |
| `fw/apps/caldump/main.c` | Shows the factory calibration page |
| `fw/apps/faulttest/main.c` | Triggers a fault to test the crash screen |
| `fw/apps/scopetest/main.c` | Updated to new drivers (kept as hardware test) |
| `tools/flash.sh` | Wait for IAP drive, copy a .bin |
| Removed: `fw/common/scope.[ch]` (split into adc/frontend/testsig) | |

---

### Task 1: Host test harness + `fmt` + `units`

**Files:**
- Create: `tests/Makefile`, `tests/test.h`, `tests/main.c`, `tests/test_fmt.c`, `tests/test_units.c`
- Create: `fw/core/fmt.h`, `fw/core/fmt.c`, `fw/core/units.h`, `fw/core/units.c`
- Modify: `.gitignore` (add `tests/build/`)

**Interfaces:**
- Produces:
  - `char *fmt_si(char *buf, float v, const char *unit, int trim);` — 3 significant digits, SI prefix n/u/m/k/M; `trim` removes trailing zeros. `buf` ≥ 16 bytes.
  - `units.h`: `NUM_VDIV`(10), `NUM_TDIV`(16), `TDIV_10US`(4), `TDIV_1MS`(10), `CODES_PER_DIV`(25), `ADC_CENTER`(128), `TIMER_CLK_HZ`, `MAX_RATE_HZ`, `const uint16_t vdiv_mv[]`, `const uint32_t tdiv_ns[]`, `typedef struct { uint16_t psc, arr; uint32_t rate_hz; } rate_cfg_t;`, `rate_cfg_t units_rate_for_tdiv(uint32_t tdiv_ns);`, `uint32_t units_samples_per_screen(uint32_t rate_hz, uint32_t tdiv_ns);`, `float units_volts_per_code(int vdiv_idx, int probe_x10);`, `float units_code_to_volts(int code, int vdiv_idx, int offset, int probe_x10);`, `int units_batt_pct(float vbat);`
  - Test harness: `CHECK(c)`, `CHECK_EQ(a,b)`, `CHECK_NEAR(a,b,eps)`, `CHECK_STR(a,b)`; each test file exports `void test_<module>(void);` registered in `tests/main.c`.

- [ ] **Step 1: Create the harness**

`tests/test.h`:
```c
#pragma once
#include <math.h>
#include <stdio.h>
#include <string.h>

extern int t_fail, t_run;
#define CHECK(c) do { t_run++; if (!(c)) { t_fail++; \
    printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define CHECK_EQ(a, b) do { long long _a = (long long)(a), _b = (long long)(b); t_run++; \
    if (_a != _b) { t_fail++; printf("%s:%d: %s == %lld, expected %lld\n", \
    __FILE__, __LINE__, #a, _a, _b); } } while (0)
#define CHECK_NEAR(a, b, eps) do { double _a = (a), _b = (b); t_run++; \
    if (fabs(_a - _b) > (eps)) { t_fail++; printf("%s:%d: %s == %g, expected %g\n", \
    __FILE__, __LINE__, #a, _a, _b); } } while (0)
#define CHECK_STR(a, b) do { const char *_a = (a), *_b = (b); t_run++; \
    if (strcmp(_a, _b)) { t_fail++; printf("%s:%d: %s == \"%s\", expected \"%s\"\n", \
    __FILE__, __LINE__, #a, _a, _b); } } while (0)
```

`tests/main.c`:
```c
#include "test.h"
int t_fail, t_run;

void test_fmt(void);
void test_units(void);

int main(void) {
    test_fmt();
    test_units();
    printf("%d checks, %d failed\n", t_run, t_fail);
    return t_fail != 0;
}
```

`tests/Makefile`:
```make
CC     ?= cc
CFLAGS  = -std=c99 -Wall -Wextra -Werror -O1 -g -I../fw/core
SRCS    = $(wildcard *.c) $(wildcard ../fw/core/*.c)
HDRS    = $(wildcard *.h) $(wildcard ../fw/core/*.h)

run: build/tests
	./build/tests

build/tests: $(SRCS) $(HDRS)
	mkdir -p build
	$(CC) $(CFLAGS) $(SRCS) -lm -o $@

clean:
	rm -rf build

.PHONY: run clean
```

Append `tests/build/` to `.gitignore`.

- [ ] **Step 2: Write the failing tests**

`tests/test_fmt.c`:
```c
#include "test.h"
#include "fmt.h"

void test_fmt(void) {
    char b[16];
    CHECK_STR(fmt_si(b, 1000.0f, "Hz", 0), "1.00kHz");
    CHECK_STR(fmt_si(b, 3.3f, "V", 0), "3.30V");
    CHECK_STR(fmt_si(b, -0.025f, "V", 0), "-25.0mV");
    CHECK_STR(fmt_si(b, 0.0f, "V", 0), "0.00V");
    CHECK_STR(fmt_si(b, 999.6f, "Hz", 0), "1.00kHz");
    CHECK_STR(fmt_si(b, 123456.0f, "Hz", 0), "123kHz");
    CHECK_STR(fmt_si(b, 500e-9f, "s", 1), "500ns");
    CHECK_STR(fmt_si(b, 1e-6f, "s", 1), "1us");
    CHECK_STR(fmt_si(b, 2.5e-3f, "s", 1), "2.5ms");
    CHECK_STR(fmt_si(b, 10.0f, "V", 1), "10V");
    CHECK_STR(fmt_si(b, 0.01f, "V", 1), "10mV");
    CHECK_STR(fmt_si(b, 100.0f, "V", 1), "100V");
}
```

`tests/test_units.c`:
```c
#include "test.h"
#include "units.h"

void test_units(void) {
    rate_cfg_t r;
    r = units_rate_for_tdiv(500);          /* capped at 12 MS/s */
    CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 19); CHECK_EQ(r.rate_hz, 12000000);
    r = units_rate_for_tdiv(10000);        /* 120 samples/div at 12 MS/s */
    CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 19); CHECK_EQ(r.rate_hz, 12000000);
    r = units_rate_for_tdiv(20000);
    CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 39); CHECK_EQ(r.rate_hz, 6000000);
    r = units_rate_for_tdiv(20000000);
    CHECK_EQ(r.psc, 0); CHECK_EQ(r.arr, 39999); CHECK_EQ(r.rate_hz, 6000);
    r = units_rate_for_tdiv(50000000);
    CHECK_EQ(r.psc, 99); CHECK_EQ(r.arr, 999); CHECK_EQ(r.rate_hz, 2400);

    CHECK_EQ(units_samples_per_screen(12000000, 500), 60);
    CHECK_EQ(units_samples_per_screen(12000000, 10000), 1200);
    CHECK_EQ(units_samples_per_screen(2400, 50000000), 1200);

    CHECK_EQ(vdiv_mv[0], 10); CHECK_EQ(vdiv_mv[NUM_VDIV - 1], 10000);
    CHECK_EQ(tdiv_ns[TDIV_10US], 10000); CHECK_EQ(tdiv_ns[TDIV_1MS], 1000000);

    CHECK_NEAR(units_code_to_volts(153, 6, 0, 0), 1.0, 1e-6);      /* +1 div at 1 V/div */
    CHECK_NEAR(units_code_to_volts(153, 6, 0, 1), 10.0, 1e-5);     /* probe x10 */
    CHECK_NEAR(units_code_to_volts(153, 6, 25, 0), 0.0, 1e-6);     /* offset moves 0 V */
    CHECK_NEAR(units_code_to_volts(103, 0, 0, 0), -0.010, 1e-7);   /* -1 div at 10 mV/div */
    CHECK_NEAR(units_volts_per_code(6, 0), 0.04, 1e-7);

    CHECK_EQ(units_batt_pct(4.10f), 100);
    CHECK_EQ(units_batt_pct(3.95f), 80);
    CHECK_EQ(units_batt_pct(3.80f), 50);
    CHECK_EQ(units_batt_pct(3.50f), 10);
}
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `make -C tests`
Expected: compile error, `fmt.h: No such file or directory`.

- [ ] **Step 4: Implement `fmt`**

`fw/core/fmt.h`:
```c
#pragma once
/* Format v with 3 significant digits and an SI prefix (n u m k M), then unit.
 * trim != 0 drops trailing zeros ("1.00us" -> "1us"). buf must hold >= 16 chars. */
char *fmt_si(char *buf, float v, const char *unit, int trim);
```

`fw/core/fmt.c`:
```c
#include "fmt.h"
#include <stdint.h>

char *fmt_si(char *buf, float v, const char *unit, int trim) {
    static const char pfx[] = {'n', 'u', 'm', 0, 'k', 'M'};
    char *p = buf;
    if (v < 0) { *p++ = '-'; v = -v; }
    int e = 3;
    if (v != 0.0f) {
        while (v >= 999.5f && e < 5) { v /= 1000.0f; e++; }
        while (v < 0.9995f && e > 0) { v *= 1000.0f; e--; }
    }
    int dec = v >= 99.95f ? 0 : v >= 9.995f ? 1 : 2;
    uint32_t mul = dec == 0 ? 1 : dec == 1 ? 10 : 100;
    uint32_t scaled = (uint32_t)(v * (float)mul + 0.5f);
    uint32_t ip = scaled / mul, fp = scaled % mul;
    char t[12]; int n = 0;
    do { t[n++] = (char)('0' + ip % 10); ip /= 10; } while (ip);
    while (n) *p++ = t[--n];
    if (dec) {
        *p++ = '.';
        if (dec == 2) { *p++ = (char)('0' + fp / 10); *p++ = (char)('0' + fp % 10); }
        else *p++ = (char)('0' + fp);
        if (trim) {
            while (p[-1] == '0') p--;
            if (p[-1] == '.') p--;
        }
    }
    if (pfx[e]) *p++ = pfx[e];
    while (*unit) *p++ = *unit++;
    *p = 0;
    return buf;
}
```

- [ ] **Step 5: Implement `units`**

`fw/core/units.h`:
```c
#pragma once
#include <stdint.h>

#define NUM_VDIV      10
#define NUM_TDIV      16
#define TDIV_10US     4
#define TDIV_1MS      10
#define CODES_PER_DIV 25
#define ADC_CENTER    128
#define TIMER_CLK_HZ  240000000u
#define MAX_RATE_HZ   12000000u
#define SAMPLES_PER_DIV 120

extern const uint16_t vdiv_mv[NUM_VDIV];
extern const uint32_t tdiv_ns[NUM_TDIV];

typedef struct { uint16_t psc, arr; uint32_t rate_hz; } rate_cfg_t;

/* Sample rate for a timebase: 120 samples/div, capped at 12 MS/s. */
rate_cfg_t units_rate_for_tdiv(uint32_t tdiv_ns);
/* Samples covering the 10-division screen. */
uint32_t units_samples_per_screen(uint32_t rate_hz, uint32_t tdiv_ns);
float units_volts_per_code(int vdiv_idx, int probe_x10);
/* 0 V is at code ADC_CENTER + offset (offset in codes, -100..100). */
float units_code_to_volts(int code, int vdiv_idx, int offset, int probe_x10);
/* Stock FNIRSI battery thresholds. */
int units_batt_pct(float vbat);
```

`fw/core/units.c`:
```c
#include "units.h"

const uint16_t vdiv_mv[NUM_VDIV] = {10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000};
const uint32_t tdiv_ns[NUM_TDIV] = {
    500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000,
    1000000, 2000000, 5000000, 10000000, 20000000, 50000000,
};

rate_cfg_t units_rate_for_tdiv(uint32_t tdiv) {
    /* timer ticks per sample = 240e6 * tdiv[s] / 120 = tdiv_ns / 500 */
    uint32_t div = tdiv / 500;
    uint32_t min_div = TIMER_CLK_HZ / MAX_RATE_HZ;
    if (div < min_div) div = min_div;
    rate_cfg_t r;
    if (div <= 65536) { r.psc = 0; r.arr = (uint16_t)(div - 1); }
    else { r.psc = (uint16_t)(div / 1000 - 1); r.arr = 999; }
    r.rate_hz = TIMER_CLK_HZ / div;
    return r;
}

uint32_t units_samples_per_screen(uint32_t rate_hz, uint32_t tdiv) {
    return (uint32_t)((uint64_t)rate_hz * tdiv * 10u / 1000000000u);
}

float units_volts_per_code(int vdiv_idx, int probe_x10) {
    return (float)vdiv_mv[vdiv_idx] / (CODES_PER_DIV * 1000.0f) * (probe_x10 ? 10.0f : 1.0f);
}

float units_code_to_volts(int code, int vdiv_idx, int offset, int probe_x10) {
    return (float)(code - (ADC_CENTER + offset)) * units_volts_per_code(vdiv_idx, probe_x10);
}

int units_batt_pct(float v) {
    static const float th[] = {4.06f, 3.98f, 3.92f, 3.87f, 3.82f, 3.79f, 3.77f, 3.74f, 3.68f};
    for (int i = 0; i < 9; i++) if (v >= th[i]) return 100 - 10 * i;
    return 10;
}
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `make -C tests`
Expected: last line `N checks, 0 failed`, exit status 0.

- [ ] **Step 7: Commit**

```bash
git add tests fw/core .gitignore
git commit -m "core: host test harness, fmt_si, units (timebase/V-div tables, rates, volts)"
```

---

### Task 2: `trigger`

**Files:**
- Create: `fw/core/trigger.h`, `fw/core/trigger.c`, `tests/test_trigger.c`
- Modify: `tests/main.c` (register `test_trigger`)

**Interfaces:**
- Produces:
  - `typedef enum { EDGE_RISING = 0, EDGE_FALLING = 1 } edge_t;`
  - `int trigger_hysteresis(const uint8_t *buf, int n);` → `max(4, (max-min)/8)`
  - `int trigger_find(const uint8_t *buf, int from, int to, int level, int hyst, edge_t edge);` → index of the first sample in `[from,to)` that reaches `level` after the signal was at least `hyst` on the other side (state starts unarmed at `from`), or −1.

- [ ] **Step 1: Write the failing test**

`tests/test_trigger.c`:
```c
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
```

Add `void test_trigger(void);` and a `test_trigger();` call to `tests/main.c`.

- [ ] **Step 2: Run to verify it fails**

Run: `make -C tests`
Expected: compile error `trigger.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/trigger.h`:
```c
#pragma once
#include <stdint.h>

typedef enum { EDGE_RISING = 0, EDGE_FALLING = 1 } edge_t;

/* Hysteresis used for triggering: max(4, peak-to-peak / 8) in ADC codes. */
int trigger_hysteresis(const uint8_t *buf, int n);
/* First index in [from, to) where the signal reaches `level` in the edge direction, after having
 * been at least `hyst` codes on the other side. Unarmed at `from`. Returns -1 if none. */
int trigger_find(const uint8_t *buf, int from, int to, int level, int hyst, edge_t edge);
```

`fw/core/trigger.c`:
```c
#include "trigger.h"

int trigger_hysteresis(const uint8_t *buf, int n) {
    int mn = 255, mx = 0;
    for (int i = 0; i < n; i++) { if (buf[i] < mn) mn = buf[i]; if (buf[i] > mx) mx = buf[i]; }
    int h = (mx - mn) / 8;
    return h < 4 ? 4 : h;
}

int trigger_find(const uint8_t *buf, int from, int to, int level, int hyst, edge_t edge) {
    int armed = 0;
    for (int i = from < 0 ? 0 : from; i < to; i++) {
        int v = buf[i];
        if (edge == EDGE_RISING) {
            if (v <= level - hyst) armed = 1;
            else if (armed && v >= level) return i;
        } else {
            if (v >= level + hyst) armed = 1;
            else if (armed && v <= level) return i;
        }
    }
    return -1;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `make -C tests`
Expected: `0 failed`.

- [ ] **Step 5: Commit**

```bash
git add fw/core/trigger.* tests/test_trigger.c tests/main.c
git commit -m "core: edge trigger search with hysteresis"
```

---

### Task 3: `render` (record → column spans)

**Files:**
- Create: `fw/core/render.h`, `fw/core/render.c`, `tests/test_render.c`
- Modify: `tests/main.c` (register `test_render`)

**Interfaces:**
- Produces:
  - `typedef struct { int16_t lo, hi; } span_t;` (ADC codes; `lo == -1` = empty column)
  - `void render_spans(const uint8_t *buf, int n, int start, uint32_t spc_q8, span_t *out, int ncols);` — `spc_q8` = samples per column × 256. ≥ 1 sample/column: min/max of the column's samples plus the previous sample (so adjacent columns connect). < 1 sample/column: linear interpolation, span joins with the previous column's value.

- [ ] **Step 1: Write the failing test**

`tests/test_render.c`:
```c
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
```

Register `test_render` in `tests/main.c`.

- [ ] **Step 2: Run to verify it fails**

Run: `make -C tests`
Expected: compile error `render.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/render.h`:
```c
#pragma once
#include <stdint.h>

typedef struct { int16_t lo, hi; } span_t;   /* ADC codes; lo == -1 means empty */

/* Map buf[start ...] onto ncols columns; spc_q8 = samples per column * 256. */
void render_spans(const uint8_t *buf, int n, int start, uint32_t spc_q8, span_t *out, int ncols);
```

`fw/core/render.c`:
```c
#include "render.h"

void render_spans(const uint8_t *buf, int n, int start, uint32_t spc_q8, span_t *out, int ncols) {
    int have_prev = 0, prev = 0;
    for (int x = 0; x < ncols; x++) {
        span_t s = {-1, -1};
        if (spc_q8 >= 256) {
            int a = start + (int)(((uint32_t)x * spc_q8) >> 8);
            int b = start + (int)(((uint32_t)(x + 1) * spc_q8) >> 8);
            if (b <= a) b = a + 1;
            if (a >= 0 && a < n) {
                if (b > n) b = n;
                int lo = buf[a], hi = buf[a];
                for (int i = a + 1; i < b; i++) { if (buf[i] < lo) lo = buf[i]; if (buf[i] > hi) hi = buf[i]; }
                if (a > 0) { int p = buf[a - 1]; if (p < lo) lo = p; if (p > hi) hi = p; }
                s.lo = (int16_t)lo; s.hi = (int16_t)hi;
            }
        } else {
            int32_t pos = start * 256 + (int32_t)((uint32_t)x * spc_q8);
            int i = pos >> 8, f = pos & 255;
            if (i >= 0 && i < n) {
                int v = (i + 1 < n) ? (buf[i] * (256 - f) + buf[i + 1] * f + 128) >> 8 : buf[i];
                int lo = v, hi = v;
                if (have_prev) { if (prev < lo) lo = prev; if (prev > hi) hi = prev; }
                s.lo = (int16_t)lo; s.hi = (int16_t)hi;
                prev = v; have_prev = 1;
            }
        }
        out[x] = s;
    }
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `make -C tests`
Expected: `0 failed`.

- [ ] **Step 5: Commit**

```bash
git add fw/core/render.* tests/test_render.c tests/main.c
git commit -m "core: render record to per-column min/max spans"
```

---

### Task 4: `keys` (logical keys, debounce, short/long/repeat)

**Files:**
- Create: `fw/core/keys.h`, `fw/core/keys.c`, `tests/test_keys.c`
- Modify: `fw/common/board.h` (remove the key enum, `NUM_KEYS`, `KEY_BIT`; add `#include "keys.h"`), `tests/main.c`

**Interfaces:**
- Produces:
  - Key ids `KEY_POWER=0, KEY_RUN, KEY_UP, KEY_DOWN, KEY_MODE, KEY_AUTO, KEY_WHEEL_R, KEY_WHEEL_PRESS, KEY_WHEEL_L` (same order as `board_keys[]`), `NUM_KEYS` (9), `KEY_BIT(k)`.
  - `KEY_DEBOUNCE_MS` 20, `KEY_LONG_MS` 700, `KEY_REPEAT_DELAY_MS` 400, `KEY_REPEAT_MS` 80.
  - `enum { KEV_SHORT = 1, KEV_LONG, KEV_REPEAT };` `typedef struct { uint8_t type, key; } kev_t;`
  - `keys_state_t`, `void keys_reset(keys_state_t *s, uint32_t repeat_mask);`, `int keys_update(keys_state_t *s, uint32_t raw, uint32_t now_ms, kev_t *out, int max);`
  - Semantics: repeat keys emit SHORT on (debounced) press, then REPEAT every 80 ms after 400 ms. Other keys emit LONG once at 700 ms held, else SHORT on release.

- [ ] **Step 1: Write the failing test**

`tests/test_keys.c`:
```c
#include "test.h"
#include "keys.h"

static int feed(keys_state_t *s, uint32_t raw, uint32_t t, kev_t *ev) { return keys_update(s, raw, t, ev, 8); }

void test_keys(void) {
    keys_state_t s; kev_t ev[8]; int n;
    keys_reset(&s, KEY_BIT(KEY_UP));

    /* short press of RUN: event on release, after debounce */
    CHECK_EQ(feed(&s, KEY_BIT(KEY_RUN), 0, ev), 0);
    CHECK_EQ(feed(&s, KEY_BIT(KEY_RUN), 25, ev), 0);       /* pressed (debounced), no event yet */
    CHECK_EQ(feed(&s, 0, 100, ev), 0);
    n = feed(&s, 0, 125, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_SHORT); CHECK_EQ(ev[0].key, KEY_RUN);

    /* bounce shorter than debounce is ignored */
    CHECK_EQ(feed(&s, KEY_BIT(KEY_RUN), 200, ev), 0);
    CHECK_EQ(feed(&s, 0, 205, ev), 0);
    CHECK_EQ(feed(&s, 0, 300, ev), 0);

    /* long press of POWER: LONG at 700 ms held, nothing on release */
    feed(&s, KEY_BIT(KEY_POWER), 1000, ev);
    feed(&s, KEY_BIT(KEY_POWER), 1025, ev);                /* debounced press at 1025 */
    CHECK_EQ(feed(&s, KEY_BIT(KEY_POWER), 1700, ev), 0);
    n = feed(&s, KEY_BIT(KEY_POWER), 1725, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_LONG); CHECK_EQ(ev[0].key, KEY_POWER);
    CHECK_EQ(feed(&s, KEY_BIT(KEY_POWER), 1900, ev), 0);   /* fires once */
    feed(&s, 0, 2000, ev);
    CHECK_EQ(feed(&s, 0, 2030, ev), 0);                    /* no SHORT after LONG */

    /* repeat key: SHORT on press, REPEAT after 400 ms then every 80 ms */
    feed(&s, KEY_BIT(KEY_UP), 3000, ev);
    n = feed(&s, KEY_BIT(KEY_UP), 3025, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_SHORT); CHECK_EQ(ev[0].key, KEY_UP);
    CHECK_EQ(feed(&s, KEY_BIT(KEY_UP), 3400, ev), 0);
    n = feed(&s, KEY_BIT(KEY_UP), 3425, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_REPEAT);
    n = feed(&s, KEY_BIT(KEY_UP), 3505, ev);
    CHECK_EQ(n, 1); CHECK_EQ(ev[0].type, KEV_REPEAT);
    feed(&s, 0, 3600, ev);
    CHECK_EQ(feed(&s, 0, 3625, ev), 0);                    /* no event on release */
}
```

Register `test_keys` in `tests/main.c`.

- [ ] **Step 2: Run to verify it fails**

Run: `make -C tests`
Expected: compile error `keys.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/keys.h`:
```c
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
```

`fw/core/keys.c`:
```c
#include "keys.h"
#include <string.h>

void keys_reset(keys_state_t *s, uint32_t repeat_mask) {
    memset(s, 0, sizeof *s);
    s->repeat_mask = repeat_mask;
}

int keys_update(keys_state_t *s, uint32_t raw, uint32_t now, kev_t *out, int max) {
    int n = 0;
    for (int k = 0; k < NUM_KEYS; k++) {
        uint32_t bit = 1u << k;
        int rep = (s->repeat_mask & bit) != 0;
        if ((raw ^ s->raw_prev) & bit) s->raw_since[k] = now;
        int r = (raw & bit) != 0, d = (s->state & bit) != 0;
        if (r != d && now - s->raw_since[k] >= KEY_DEBOUNCE_MS) {
            s->state ^= bit;
            if (r) {
                s->down_at[k] = now;
                s->long_done &= ~bit;
                if (rep && n < max) { out[n].type = KEV_SHORT; out[n].key = (uint8_t)k; n++; }
                s->next_rep[k] = now + KEY_REPEAT_DELAY_MS;
            } else if (!rep && !(s->long_done & bit) && n < max) {
                out[n].type = KEV_SHORT; out[n].key = (uint8_t)k; n++;
            }
        }
        if (s->state & bit) {
            if (rep) {
                if ((int32_t)(now - s->next_rep[k]) >= 0 && n < max) {
                    out[n].type = KEV_REPEAT; out[n].key = (uint8_t)k; n++;
                    s->next_rep[k] += KEY_REPEAT_MS;
                }
            } else if (!(s->long_done & bit) && now - s->down_at[k] >= KEY_LONG_MS && n < max) {
                out[n].type = KEV_LONG; out[n].key = (uint8_t)k; n++;
                s->long_done |= bit;
            }
        }
    }
    s->raw_prev = raw;
    return n;
}
```

In `fw/common/board.h`, delete the block from `#define NUM_KEYS 9` through `#define KEY_BIT(k) (1u << (k))` (the enum and its comments) and add `#include "keys.h"` after `#include <stdint.h>`. Keep `key_def_t` and `extern const key_def_t board_keys[NUM_KEYS];`.

In `fw/Makefile`, change `-Icommon` to `-Icommon -Icore`, add `$(wildcard core/*.c)` to the non-hello `SRCS` line so apps link the core, and link libgcc (the core uses float math, soft-float until Task 8):
```make
SRCS = apps/$(APP)/main.c $(wildcard common/*.c) $(wildcard core/*.c)
```
and in the `$(ELF)` rule change `$(SRCS) -o $@` to `$(SRCS) -lgcc -o $@`.

- [ ] **Step 4: Run tests and rebuild device apps**

Run: `make -C tests && make -C fw APP=keytest && make -C fw APP=scopetest`
Expected: `0 failed`; both firmware builds finish with no warnings.

- [ ] **Step 5: Commit**

```bash
git add fw/core/keys.* tests/test_keys.c tests/main.c fw/common/board.h fw/Makefile
git commit -m "core: logical key ids + debounced short/long/repeat key events"
```

---

### Task 5: `measure`

**Files:**
- Create: `fw/core/measure.h`, `fw/core/measure.c`, `tests/test_measure.c`
- Modify: `tests/main.c`

**Interfaces:**
- Consumes: `trigger_find`, `edge_t` (Task 2).
- Produces: `typedef struct { int valid; float freq_hz, duty_pct; uint8_t vmin, vmax; } meas_t;` and `void measure(const uint8_t *buf, int n, uint32_t rate_hz, meas_t *m);` — `vmin/vmax` always set; `valid` = at least one full period found and peak-to-peak ≥ 8 codes.

- [ ] **Step 1: Write the failing test**

`tests/test_measure.c`:
```c
#include "test.h"
#include "measure.h"
#include <stdint.h>

void test_measure(void) {
    static uint8_t b[1000];
    meas_t m;
    for (int i = 0; i < 1000; i++) b[i] = (i % 100) < 30 ? 200 : 50;    /* 1 kHz @ 100 kS/s, 30 % */
    measure(b, 1000, 100000, &m);
    CHECK(m.valid);
    CHECK_NEAR(m.freq_hz, 1000.0, 0.01);
    CHECK_NEAR(m.duty_pct, 30.0, 0.01);
    CHECK_EQ(m.vmin, 50); CHECK_EQ(m.vmax, 200);

    for (int i = 0; i < 1000; i++) b[i] = (uint8_t)(128 + 90 * sin(2 * M_PI * i / 77.0));
    measure(b, 1000, 1000000, &m);
    CHECK(m.valid);
    CHECK_NEAR(m.freq_hz, 1000000.0 / 77.0, 1000000.0 / 77.0 * 0.01);
    CHECK_NEAR(m.duty_pct, 50.0, 2.0);

    memset(b, 128, sizeof b);
    measure(b, 1000, 100000, &m);
    CHECK(!m.valid);
    CHECK_EQ(m.vmin, 128); CHECK_EQ(m.vmax, 128);
}
```

`M_PI` needs `_DEFAULT_SOURCE` under `-std=c99`; add `-D_DEFAULT_SOURCE` to `CFLAGS` in `tests/Makefile`. Register `test_measure` in `tests/main.c`.

- [ ] **Step 2: Run to verify it fails**

Run: `make -C tests`
Expected: compile error `measure.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/measure.h`:
```c
#pragma once
#include <stdint.h>

typedef struct { int valid; float freq_hz, duty_pct; uint8_t vmin, vmax; } meas_t;

/* Frequency and duty from rising crossings of the mid level (with hysteresis). */
void measure(const uint8_t *buf, int n, uint32_t rate_hz, meas_t *m);
```

`fw/core/measure.c`:
```c
#include "measure.h"
#include "trigger.h"

void measure(const uint8_t *buf, int n, uint32_t rate_hz, meas_t *m) {
    int mn = 255, mx = 0;
    for (int i = 0; i < n; i++) { if (buf[i] < mn) mn = buf[i]; if (buf[i] > mx) mx = buf[i]; }
    m->vmin = (uint8_t)mn; m->vmax = (uint8_t)mx;
    m->valid = 0; m->freq_hz = 0; m->duty_pct = 0;
    if (n <= 0 || mx - mn < 8) return;
    int level = (mn + mx) / 2, hyst = (mx - mn) / 8;
    if (hyst < 4) hyst = 4;
    int first = trigger_find(buf, 0, n, level, hyst, EDGE_RISING);
    if (first < 0) return;
    int last = first, periods = 0;
    for (;;) {
        int j = trigger_find(buf, last + 1, n, level, hyst, EDGE_RISING);
        if (j < 0) break;
        last = j; periods++;
    }
    if (periods == 0) return;
    int high = 0;
    for (int i = first; i < last; i++) if (buf[i] >= level) high++;
    m->freq_hz = (float)periods * (float)rate_hz / (float)(last - first);
    m->duty_pct = 100.0f * (float)high / (float)(last - first);
    m->valid = 1;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `make -C tests`
Expected: `0 failed`.

- [ ] **Step 5: Commit**

```bash
git add fw/core/measure.* tests/test_measure.c tests/main.c tests/Makefile
git commit -m "core: frequency/duty/min/max measurement"
```

---

### Task 6: `cal` (factory calibration parser, offset PWM, auto-zero search)

**Files:**
- Create: `fw/core/cal.h`, `fw/core/cal.c`, `tests/test_cal.c`
- Modify: `tests/main.c`

**Interfaces:**
- Produces:
  - `CAL_PAGE_ADDR` 0x08006000, `CAL_PAGE_WORDS` 0x27, `CAL_SET_SLOW` 0 (≤ 2.4 MS/s), `CAL_SET_MID` 1 (> 2.4 MS/s single stream), `CAL_SET_FAST` 2 (interleaved, 1b), `CAL_NSETS` 3, `CAL_NRANGES` 10, `CAL_PWM_MAX` 0xFFE.
  - `typedef struct { uint16_t lo[3][10], hi[3][10]; uint8_t valid; } cal_t;`
  - `int cal_parse(const uint32_t *page, cal_t *c);` (1 = valid factory data; else defaults loaded, `valid = 0`)
  - `void cal_defaults(cal_t *c);`
  - `int cal_set_for_rate(uint32_t rate_hz);`
  - `uint16_t cal_offset_pwm(const cal_t *c, int set, int range, int offset);` (offset −100…100)
  - `typedef int (*cal_mean_fn)(uint16_t pwm, void *ctx);` `uint16_t cal_search_pwm(cal_mean_fn f, void *ctx, int target, uint16_t a, uint16_t b);`
  - `void cal_shift_zero(cal_t *c, int set, int range, uint16_t pwm_at_zero);`

Layout (from stock `FUN_0801B218`): `page[0] & 0xFF` is 0x55 (saved) or 0xAA (defaults). The bytes of `page[8]..page[0x25]` are RAM 0x20000202…0x20000279: six tables of 10 little-endian u16. Byte offsets in that area: SLOW lo 0x00 / hi 0x3C, MID lo 0x14 / hi 0x50, FAST lo 0x28 / hi 0x64.

- [ ] **Step 1: Write the failing test**

`tests/test_cal.c`:
```c
#include "test.h"
#include "cal.h"
#include <stdint.h>

static void put16(uint32_t *page, int byte_off, uint16_t v) {
    uint8_t *b = (uint8_t *)&page[8];
    b[byte_off] = (uint8_t)v; b[byte_off + 1] = (uint8_t)(v >> 8);
}

static int lin_up(uint16_t pwm, void *ctx) { (void)ctx; return 128 + ((int)pwm - 2000) / 10; }
static int lin_down(uint16_t pwm, void *ctx) { (void)ctx; return 128 - ((int)pwm - 2500) / 10; }

void test_cal(void) {
    uint32_t page[CAL_PAGE_WORDS];
    memset(page, 0, sizeof page);
    page[0] = 0x00000555;                       /* low byte 0x55 */
    static const int lo_off[3] = {0x00, 0x14, 0x28}, hi_off[3] = {0x3C, 0x50, 0x64};
    for (int s = 0; s < 3; s++)
        for (int r = 0; r < 10; r++) {
            put16(page, lo_off[s] + 2 * r, (uint16_t)(1000 + 10 * s + r));
            put16(page, hi_off[s] + 2 * r, (uint16_t)(3000 + 10 * s + r));
        }
    cal_t c;
    CHECK_EQ(cal_parse(page, &c), 1);
    CHECK_EQ(c.valid, 1);
    CHECK_EQ(c.lo[CAL_SET_SLOW][0], 1000);
    CHECK_EQ(c.hi[CAL_SET_MID][9], 3019);
    CHECK_EQ(c.lo[CAL_SET_FAST][5], 1025);
    CHECK_EQ(cal_offset_pwm(&c, CAL_SET_SLOW, 0, -100), 1000);
    CHECK_EQ(cal_offset_pwm(&c, CAL_SET_SLOW, 0, 100), 3000);
    CHECK_EQ(cal_offset_pwm(&c, CAL_SET_SLOW, 0, 0), 2000);

    page[0] = 0x000005AA;                       /* defaults marker is also accepted */
    CHECK_EQ(cal_parse(page, &c), 1);
    put16(page, 0x3C, 0xFFFF);                  /* erased value -> invalid */
    CHECK_EQ(cal_parse(page, &c), 0);
    CHECK_EQ(c.valid, 0);
    CHECK_EQ(c.lo[0][0], 0x400); CHECK_EQ(c.hi[0][0], 0xC00);   /* defaults */
    page[0] = 0;
    put16(page, 0x3C, 3000);
    CHECK_EQ(cal_parse(page, &c), 0);           /* bad marker */

    CHECK_EQ(cal_set_for_rate(2400000), CAL_SET_SLOW);
    CHECK_EQ(cal_set_for_rate(6000000), CAL_SET_MID);

    uint16_t p = cal_search_pwm(lin_up, 0, 128, 1000, 3000);
    CHECK(p >= 1990 && p <= 2010);   /* integer model: mean is 128 for pwm 1991..2009 */
    p = cal_search_pwm(lin_down, 0, 128, 1000, 3000);
    CHECK(p >= 2490 && p <= 2510);

    cal_defaults(&c);
    cal_shift_zero(&c, CAL_SET_SLOW, 3, 0x900);     /* mid was 0x800 -> shift +0x100 */
    CHECK_EQ(c.lo[CAL_SET_SLOW][3], 0x500);
    CHECK_EQ(c.hi[CAL_SET_SLOW][3], 0xD00);
    CHECK_EQ(cal_offset_pwm(&c, CAL_SET_SLOW, 3, 0), 0x900);
}
```

Register `test_cal` in `tests/main.c`.

- [ ] **Step 2: Run to verify it fails**

Run: `make -C tests`
Expected: compile error `cal.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/cal.h`:
```c
#pragma once
#include <stdint.h>

#define CAL_PAGE_ADDR  0x08006000u   /* stock settings+calibration page: READ ONLY for us */
#define CAL_PAGE_WORDS 0x27
#define CAL_SET_SLOW   0             /* <= 2.4 MS/s  (stock timebase index >= 10) */
#define CAL_SET_MID    1             /* > 2.4 MS/s single stream (stock index 9) */
#define CAL_SET_FAST   2             /* 4-way interleaved (stock index < 9), sub-project 1b */
#define CAL_NSETS      3
#define CAL_NRANGES    10
#define CAL_PWM_MAX    0xFFE

typedef struct {
    uint16_t lo[CAL_NSETS][CAL_NRANGES];   /* offset PWM at offset -100 */
    uint16_t hi[CAL_NSETS][CAL_NRANGES];   /* offset PWM at offset +100 */
    uint8_t valid;                         /* 1 = factory data, 0 = defaults */
} cal_t;

int cal_parse(const uint32_t *page, cal_t *c);
void cal_defaults(cal_t *c);
int cal_set_for_rate(uint32_t rate_hz);
uint16_t cal_offset_pwm(const cal_t *c, int set, int range, int offset);

/* Auto-zero helper: returns the PWM in [a, b] whose measured mean code is closest to target.
 * mean_at must set the PWM, wait for settling, capture and return the mean ADC code. */
typedef int (*cal_mean_fn)(uint16_t pwm, void *ctx);
uint16_t cal_search_pwm(cal_mean_fn mean_at, void *ctx, int target, uint16_t a, uint16_t b);
/* Move a range's lo/hi so that offset 0 maps to pwm_at_zero (span unchanged). */
void cal_shift_zero(cal_t *c, int set, int range, uint16_t pwm_at_zero);
```

`fw/core/cal.c`:
```c
#include "cal.h"

static const uint8_t lo_off[CAL_NSETS] = {0x00, 0x14, 0x28};
static const uint8_t hi_off[CAL_NSETS] = {0x3C, 0x50, 0x64};

void cal_defaults(cal_t *c) {
    for (int s = 0; s < CAL_NSETS; s++)
        for (int r = 0; r < CAL_NRANGES; r++) { c->lo[s][r] = 0x400; c->hi[s][r] = 0xC00; }
    c->valid = 0;
}

int cal_parse(const uint32_t *page, cal_t *c) {
    uint32_t mark = page[0] & 0xFF;
    const uint8_t *b = (const uint8_t *)&page[8];
    int ok = (mark == 0x55 || mark == 0xAA);
    for (int s = 0; s < CAL_NSETS && ok; s++)
        for (int r = 0; r < CAL_NRANGES && ok; r++) {
            uint16_t lo = (uint16_t)(b[lo_off[s] + 2 * r] | b[lo_off[s] + 2 * r + 1] << 8);
            uint16_t hi = (uint16_t)(b[hi_off[s] + 2 * r] | b[hi_off[s] + 2 * r + 1] << 8);
            if (lo > CAL_PWM_MAX || hi > CAL_PWM_MAX || lo == hi) ok = 0;
            c->lo[s][r] = lo; c->hi[s][r] = hi;
        }
    if (!ok) { cal_defaults(c); return 0; }
    c->valid = 1;
    return 1;
}

int cal_set_for_rate(uint32_t rate_hz) { return rate_hz > 2400000u ? CAL_SET_MID : CAL_SET_SLOW; }

uint16_t cal_offset_pwm(const cal_t *c, int set, int range, int offset) {
    int lo = c->lo[set][range], hi = c->hi[set][range];
    int v = lo + (hi - lo) * (offset + 100) / 200;
    if (v < 0) v = 0;
    if (v > CAL_PWM_MAX) v = CAL_PWM_MAX;
    return (uint16_t)v;
}

uint16_t cal_search_pwm(cal_mean_fn mean_at, void *ctx, int target, uint16_t a, uint16_t b) {
    int lo = a < b ? a : b, hi = a < b ? b : a;
    int up = mean_at((uint16_t)hi, ctx) > mean_at((uint16_t)lo, ctx);
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        int m = mean_at((uint16_t)mid, ctx);
        if ((m < target) == up) lo = mid; else hi = mid;
    }
    int dl = mean_at((uint16_t)lo, ctx) - target, dh = mean_at((uint16_t)hi, ctx) - target;
    if (dl < 0) dl = -dl;
    if (dh < 0) dh = -dh;
    return (uint16_t)(dl <= dh ? lo : hi);
}

void cal_shift_zero(cal_t *c, int set, int range, uint16_t pwm_at_zero) {
    int d = (int)pwm_at_zero - (int)cal_offset_pwm(c, set, range, 0);
    int lo = c->lo[set][range] + d, hi = c->hi[set][range] + d;
    if (lo < 0) lo = 0;
    if (hi < 0) hi = 0;
    if (lo > CAL_PWM_MAX) lo = CAL_PWM_MAX;
    if (hi > CAL_PWM_MAX) hi = CAL_PWM_MAX;
    c->lo[set][range] = (uint16_t)lo; c->hi[set][range] = (uint16_t)hi;
}
```

- [ ] **Step 4: Run to verify it passes**

Run: `make -C tests`
Expected: `0 failed`.

- [ ] **Step 5: Commit**

```bash
git add fw/core/cal.* tests/test_cal.c tests/main.c
git commit -m "core: factory calibration parser, offset PWM, auto-zero search"
```

---

### Task 7: `control` (scope state + key → state) and `autoset`

**Files:**
- Create: `fw/core/control.h`, `fw/core/control.c`, `fw/core/autoset.h`, `fw/core/autoset.c`, `tests/test_control.c`, `tests/test_autoset.c`
- Modify: `tests/main.c`

**Interfaces:**
- Consumes: `kev_t`, key ids (Task 4), `NUM_TDIV`, `NUM_VDIV`, `vdiv_mv`, `tdiv_ns`, `TDIV_1MS` (Task 1), `edge_t` (Task 2).
- Produces:
  - `scope_state_t` fields: `int8_t tdiv, vdiv; int16_t offset, trig_level, trig_pos; uint8_t trig_mode, edge, coupling_dc, probe_x10, run, ctl, menu_open, menu_item;`
  - `TRIG_AUTO, TRIG_NORMAL, TRIG_SINGLE`; `CTL_TIME_VOLT, CTL_POS_OFFSET, CTL_POS_LEVEL`; `PLOT_COLS` 300; `MENU_ITEMS` 4 (0 coupling, 1 probe, 2 trigger mode, 3 edge).
  - Change flags: `CHG_TIME 0x001, CHG_FE 0x002, CHG_TRIG 0x004, CHG_BARS 0x008, CHG_SCREEN 0x010, CHG_RUN 0x020, CHG_MENU 0x040, ACT_AUTOSET 0x100, ACT_AUTOZERO 0x200, ACT_POWEROFF 0x400, ACT_SAVE 0x800`.
  - `void state_defaults(scope_state_t *s);` `uint32_t control_handle(scope_state_t *s, kev_t ev);`
  - `int autoset_vdiv(int cur, int vmin, int vmax);` `int autoset_tdiv(float freq_hz);` (−1 if freq ≤ 0)

- [ ] **Step 1: Write the failing tests**

`tests/test_control.c`:
```c
#include "test.h"
#include "control.h"

static kev_t K(int type, int key) { kev_t e; e.type = (uint8_t)type; e.key = (uint8_t)key; return e; }

void test_control(void) {
    scope_state_t s;
    state_defaults(&s);
    CHECK_EQ(s.tdiv, TDIV_1MS); CHECK_EQ(s.vdiv, 6); CHECK_EQ(s.trig_pos, 150);
    CHECK_EQ(s.trig_mode, TRIG_AUTO); CHECK_EQ(s.run, 1); CHECK_EQ(s.ctl, CTL_TIME_VOLT);

    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_TIME | CHG_BARS);
    CHECK_EQ(s.tdiv, TDIV_1MS + 1);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), CHG_FE | CHG_BARS);
    CHECK_EQ(s.vdiv, 5);                                   /* up = more sensitive */
    s.vdiv = 0;
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), 0); /* clamped */

    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_BARS);
    CHECK_EQ(s.ctl, CTL_POS_OFFSET);
    CHECK_EQ(control_handle(&s, K(KEV_REPEAT, KEY_UP)), CHG_FE | CHG_TRIG);
    CHECK_EQ(s.offset, 5);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L)), CHG_TRIG);
    CHECK_EQ(s.trig_pos, 140);
    control_handle(&s, K(KEV_SHORT, KEY_MODE));
    CHECK_EQ(s.ctl, CTL_POS_LEVEL);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.trig_level, 126);
    control_handle(&s, K(KEV_SHORT, KEY_MODE));
    CHECK_EQ(s.ctl, CTL_TIME_VOLT);

    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 1);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_MENU | CHG_FE | CHG_BARS);
    CHECK_EQ(s.coupling_dc, 0);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.menu_item, 2);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.trig_mode, TRIG_NORMAL);
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    CHECK_EQ(s.menu_item, 3);                              /* wraps */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 0);

    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_RUN)), CHG_RUN | CHG_BARS);
    CHECK_EQ(s.run, 0);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_RUN)), ACT_SAVE);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_AUTO)), ACT_AUTOSET);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_AUTO)), ACT_AUTOZERO);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_POWER)), 0);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_POWER)), ACT_POWEROFF);
}
```

`tests/test_autoset.c`:
```c
#include "test.h"
#include "autoset.h"
#include "units.h"

void test_autoset(void) {
    /* 1 V/div (idx 6), 2 codes pk-pk = 80 mV -> needs 20 mV/div (80 mV <= 6 div * 20 mV) */
    CHECK_EQ(autoset_vdiv(6, 127, 129), 1);
    /* 3.3 V pk-pk at 1 V/div = 82.5 codes -> smallest with 3300 <= 6 * vdiv: 1 V (idx 6) */
    CHECK_EQ(autoset_vdiv(6, 90, 172), 6);
    /* clipped -> one step less sensitive */
    CHECK_EQ(autoset_vdiv(3, 0, 255), 4);
    CHECK_EQ(autoset_vdiv(9, 0, 255), 9);
    /* flat -> unchanged */
    CHECK_EQ(autoset_vdiv(4, 128, 128), 4);

    /* 1 kHz: need 10*tdiv >= 2 ms -> 200 us/div (idx 8) */
    CHECK_EQ(tdiv_ns[autoset_tdiv(1000.0f)], 200000);
    CHECK_EQ(tdiv_ns[autoset_tdiv(1e6f)], 500);
    CHECK_EQ(tdiv_ns[autoset_tdiv(10.0f)], 20000000);   /* 200 ms screen = 2 periods */
    CHECK_EQ(tdiv_ns[autoset_tdiv(1.0f)], 50000000);    /* clamped to slowest */
    CHECK_EQ(autoset_tdiv(0.0f), -1);
}
```

Register `test_control` and `test_autoset` in `tests/main.c`.

- [ ] **Step 2: Run to verify they fail**

Run: `make -C tests`
Expected: compile error `control.h: No such file or directory`.

- [ ] **Step 3: Implement `control`**

`fw/core/control.h`:
```c
#pragma once
#include <stdint.h>
#include "keys.h"
#include "units.h"

#define PLOT_COLS  300
#define MENU_ITEMS 4

enum { TRIG_AUTO = 0, TRIG_NORMAL, TRIG_SINGLE };
enum { CTL_TIME_VOLT = 0, CTL_POS_OFFSET, CTL_POS_LEVEL };

#define CHG_TIME     0x001u   /* timebase changed: new sample rate */
#define CHG_FE       0x002u   /* range / coupling / offset changed */
#define CHG_TRIG     0x004u   /* trigger level / position / mode / edge or 0 V marker moved */
#define CHG_BARS     0x008u   /* top/bottom status bars need redraw */
#define CHG_SCREEN   0x010u   /* full redraw */
#define CHG_RUN      0x020u   /* run/stop toggled */
#define CHG_MENU     0x040u   /* menu contents changed */
#define ACT_AUTOSET  0x100u
#define ACT_AUTOZERO 0x200u
#define ACT_POWEROFF 0x400u
#define ACT_SAVE     0x800u

typedef struct {
    int8_t tdiv, vdiv;        /* indices into tdiv_ns / vdiv_mv */
    int16_t offset;           /* -100..100 codes: 0 V at code 128 + offset */
    int16_t trig_level;       /* ADC code 0..255 */
    int16_t trig_pos;         /* trigger point, plot column 0..299 */
    uint8_t trig_mode, edge, coupling_dc, probe_x10, run, ctl, menu_open, menu_item;
} scope_state_t;

void state_defaults(scope_state_t *s);
uint32_t control_handle(scope_state_t *s, kev_t ev);
```

`fw/core/control.c`:
```c
#include "control.h"

static int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

void state_defaults(scope_state_t *s) {
    s->tdiv = TDIV_1MS; s->vdiv = 6; s->offset = 0;
    s->trig_level = 128; s->trig_pos = PLOT_COLS / 2;
    s->trig_mode = TRIG_AUTO; s->edge = 0; s->coupling_dc = 1; s->probe_x10 = 0;
    s->run = 1; s->ctl = CTL_TIME_VOLT; s->menu_open = 0; s->menu_item = 0;
}

static uint32_t menu_key(scope_state_t *s, int k, int t, int tap) {
    if ((k == KEY_MODE || k == KEY_WHEEL_PRESS) && t == KEV_SHORT) { s->menu_open = 0; return CHG_SCREEN; }
    if (k == KEY_UP && tap) { s->menu_item = (uint8_t)((s->menu_item + MENU_ITEMS - 1) % MENU_ITEMS); return CHG_MENU; }
    if (k == KEY_DOWN && tap) { s->menu_item = (uint8_t)((s->menu_item + 1) % MENU_ITEMS); return CHG_MENU; }
    if ((k == KEY_WHEEL_L || k == KEY_WHEEL_R) && tap) {
        int d = k == KEY_WHEEL_R ? 1 : -1;
        switch (s->menu_item) {
        case 0: s->coupling_dc ^= 1; return CHG_MENU | CHG_FE | CHG_BARS;
        case 1: s->probe_x10 ^= 1; return CHG_MENU | CHG_BARS;
        case 2: s->trig_mode = (uint8_t)((s->trig_mode + 3 + d) % 3); return CHG_MENU | CHG_BARS | CHG_TRIG;
        case 3: s->edge ^= 1; return CHG_MENU | CHG_BARS | CHG_TRIG;
        }
    }
    return 0;
}

uint32_t control_handle(scope_state_t *s, kev_t ev) {
    int k = ev.key, t = ev.type, tap = (t == KEV_SHORT || t == KEV_REPEAT);
    if (k == KEY_POWER) return t == KEV_LONG ? ACT_POWEROFF : 0;
    if (k == KEY_MODE && t == KEV_LONG) { s->menu_open ^= 1; s->menu_item = 0; return CHG_SCREEN; }
    if (s->menu_open) return menu_key(s, k, t, tap);
    if (k == KEY_MODE && t == KEV_SHORT) { s->ctl = (uint8_t)((s->ctl + 1) % 3); return CHG_BARS; }
    if (k == KEY_RUN && t == KEV_SHORT) { s->run ^= 1; return CHG_RUN | CHG_BARS; }
    if (k == KEY_RUN && t == KEV_LONG) return ACT_SAVE;
    if (k == KEY_AUTO && t == KEV_SHORT) return ACT_AUTOSET;
    if (k == KEY_AUTO && t == KEV_LONG) return ACT_AUTOZERO;
    if (!tap) return 0;
    int h = k == KEY_WHEEL_R ? 1 : k == KEY_WHEEL_L ? -1 : 0;
    int v = k == KEY_UP ? 1 : k == KEY_DOWN ? -1 : 0;
    if (s->ctl == CTL_TIME_VOLT) {
        if (h) {
            int n = clampi(s->tdiv + h, 0, NUM_TDIV - 1);
            if (n == s->tdiv) return 0;
            s->tdiv = (int8_t)n; return CHG_TIME | CHG_BARS;
        }
        if (v) {
            int n = clampi(s->vdiv - v, 0, NUM_VDIV - 1);
            if (n == s->vdiv) return 0;
            s->vdiv = (int8_t)n; return CHG_FE | CHG_BARS;
        }
        return 0;
    }
    if (h) { s->trig_pos = (int16_t)clampi(s->trig_pos + 10 * h, 0, PLOT_COLS - 1); return CHG_TRIG; }
    if (v && s->ctl == CTL_POS_OFFSET) { s->offset = (int16_t)clampi(s->offset + 5 * v, -100, 100); return CHG_FE | CHG_TRIG; }
    if (v && s->ctl == CTL_POS_LEVEL) { s->trig_level = (int16_t)clampi(s->trig_level + 2 * v, 0, 255); return CHG_TRIG; }
    return 0;
}
```

- [ ] **Step 4: Implement `autoset`**

`fw/core/autoset.h`:
```c
#pragma once
/* New V/div index so the signal spans <= 6 divisions (one step less sensitive if clipped). */
int autoset_vdiv(int cur, int vmin, int vmax);
/* Fastest timebase index showing >= 2 periods on the 10-division screen; -1 if freq <= 0. */
int autoset_tdiv(float freq_hz);
```

`fw/core/autoset.c`:
```c
#include "autoset.h"
#include "units.h"

int autoset_vdiv(int cur, int vmin, int vmax) {
    if (vmin <= 2 || vmax >= 253) return cur < NUM_VDIV - 1 ? cur + 1 : cur;
    int pp = vmax - vmin;
    if (pp < 2) return cur;
    uint32_t mv_pp = (uint32_t)pp * vdiv_mv[cur] / CODES_PER_DIV;
    for (int i = 0; i < NUM_VDIV; i++) if (mv_pp <= 6u * vdiv_mv[i]) return i;
    return NUM_VDIV - 1;
}

int autoset_tdiv(float freq_hz) {
    if (freq_hz <= 0.0f) return -1;
    float need_ns = 2e9f / freq_hz;                       /* two periods */
    for (int i = 0; i < NUM_TDIV; i++) if ((float)tdiv_ns[i] * 10.0f >= need_ns * 0.999f) return i;
    return NUM_TDIV - 1;
}
```

- [ ] **Step 5: Run to verify they pass**

Run: `make -C tests`
Expected: `0 failed`.

- [ ] **Step 6: Commit**

```bash
git add fw/core/control.* fw/core/autoset.* tests/test_control.c tests/test_autoset.c tests/main.c
git commit -m "core: scope state + key mapping, auto-set picks"
```

---

### Task 8: 240 MHz clock, SysTick timebase, FPU, weak vectors, fault screen, flash script

**Files:**
- Modify: `fw/common/startup.c` (rewrite), `fw/common/board.h`, `fw/common/board.c`, `fw/Makefile`
- Create: `fw/common/fault.c`, `fw/apps/faulttest/main.c`, `tools/flash.sh`

**Interfaces:**
- Consumes: `keys_read()`, `power_off()`, `backlight()`, LCD API (existing).
- Produces:
  - `void board_init(void);` now: disables all NVIC IRQs left on by the bootloader, VTOR = 0x08007000, 240 MHz clocks, SysTick 1 kHz, enables interrupts, power latch.
  - `uint32_t millis(void);` and `void delay_ms(uint32_t ms);` (SysTick based; reload the watchdog while waiting).
  - `extern uint32_t SystemCoreClock;` (240000000 after `board_init`).
  - Weak handlers in `startup.c`: `NMI_Handler, HardFault_Handler, MemManage_Handler, BusFault_Handler, UsageFault_Handler, SVC_Handler, DebugMon_Handler, PendSV_Handler, SysTick_Handler`; Reset enables the FPU.
  - `fault.c` provides the strong `HardFault_Handler` for every app linked with `common/`.
  - `tools/flash.sh <file.bin>`.

- [ ] **Step 1: Rewrite `fw/common/startup.c`**

```c
#include <stddef.h>
#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
int main(void);

void Reset_Handler(void) {
    *(volatile uint32_t *)0xE000ED88 |= 0xFu << 20;   /* CPACR: enable FPU (CP10, CP11) */
    __asm volatile ("dsb\n isb");
    uint32_t *src = &_sidata, *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;
    for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0;
    main();
    for (;;) { }
}

void Default_Handler(void) { for (;;) { } }

#define WEAK __attribute__((weak, alias("Default_Handler")))
void NMI_Handler(void) WEAK;
void HardFault_Handler(void) WEAK;
void MemManage_Handler(void) WEAK;
void BusFault_Handler(void) WEAK;
void UsageFault_Handler(void) WEAK;
void SVC_Handler(void) WEAK;
void DebugMon_Handler(void) WEAK;
void PendSV_Handler(void) WEAK;
void SysTick_Handler(void) WEAK;

__attribute__((section(".isr_vector"), used))
const void *const vectors[16 + 84] = {
    [0] = &_estack,
    [1] = Reset_Handler,
    [2] = NMI_Handler,
    [3] = HardFault_Handler,
    [4] = MemManage_Handler,
    [5] = BusFault_Handler,
    [6] = UsageFault_Handler,
    [11] = SVC_Handler,
    [12] = DebugMon_Handler,
    [14] = PendSV_Handler,
    [15] = SysTick_Handler,
    [16 ... 16 + 84 - 1] = Default_Handler,
};

/* Minimal libc pieces GCC may emit calls to (we link with -nostdlib). */
void *memset(void *d, int c, size_t n) { unsigned char *p = d; while (n--) *p++ = (unsigned char)c; return d; }
void *memcpy(void *d, const void *s, size_t n) { unsigned char *p = d; const unsigned char *q = s; while (n--) *p++ = *q++; return d; }
```

- [ ] **Step 2: Update `fw/Makefile` for hard float, core, app, libgcc**

Replace the whole file with:
```make
# Build:  make APP=<name>     name = app (the firmware) or a test app in apps/<name>
# Output: build/APP_DSO-510_<APP>.bin  -> copy onto the scope's IAP drive (tools/flash.sh)
APP     ?= app
CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
ARCH    = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS  = $(ARCH) -O2 -g -Wall -Wextra -ffreestanding -ffunction-sections -fdata-sections \
          -fno-tree-loop-distribute-patterns -Icommon -Icore -Iapp
LDFLAGS = -nostdlib -T common/link.ld -Wl,--gc-sections -Wl,-Map=build/$(APP).map

COMMON = $(wildcard common/*.c)
CORE   = $(wildcard core/*.c)
ifeq ($(APP),hello)
SRCS = apps/hello/main.c common/startup.c
else ifeq ($(APP),app)
SRCS = $(wildcard app/*.c) $(COMMON) $(CORE)
else
SRCS = apps/$(APP)/main.c $(COMMON) $(CORE)
endif

ELF = build/$(APP).elf
BIN = build/APP_DSO-510_$(APP).bin
APPS = $(notdir $(wildcard apps/*)) $(if $(wildcard app/main.c),app)

all: $(BIN)

$(ELF): $(SRCS) $(wildcard common/*.h core/*.h app/*.h) common/link.ld | build
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRCS) -lgcc -o $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@
	arm-none-eabi-size $<

build:
	mkdir -p build

everything:
	for a in $(APPS); do $(MAKE) --no-print-directory APP=$$a || exit 1; done

clean:
	rm -rf build

.PHONY: all clean everything
```

- [ ] **Step 3: Update `fw/common/board.h`**

Add after the existing CRM register defines:
```c
#define CRM_MISC3    REG32(0x40021054)
#define SYST_CSR     REG32(0xE000E010)
#define SYST_RVR     REG32(0xE000E014)
#define SYST_CVR     REG32(0xE000E018)
#define NVIC_ICER(i) REG32(0xE000E180 + 4 * (i))
#define NVIC_ICPR(i) REG32(0xE000E280 + 4 * (i))
```
Replace the `board_init` / `delay_ms` declarations with:
```c
extern uint32_t SystemCoreClock;
void board_init(void);           // 240 MHz, SysTick 1 ms, IRQs from bootloader off, power latch
uint32_t millis(void);
void delay_ms(uint32_t ms);      // SysTick based, reloads watchdog
```

- [ ] **Step 4: Update `fw/common/board.c`**

Replace `delay_ms`, `clock_reset_to_hick` and `board_init` with:
```c
uint32_t SystemCoreClock = 8000000;
static volatile uint32_t g_ms;

void SysTick_Handler(void) { g_ms++; }
uint32_t millis(void) { return g_ms; }

void delay_ms(uint32_t ms) {
    uint32_t t0 = g_ms;
    while (g_ms - t0 < ms) WDT_CMD = 0xAAAA;   // reload watchdog in case the bootloader started it
}

static void clock_init_240mhz(void) {
    // Back to HICK first (mirrors stock SystemInit), then PLL = HEXT 8 MHz / 2 * 60 = 240 MHz.
    CRM_CTRL |= 1;
    while (!(CRM_CTRL & 2)) { }
    CRM_CFG &= ~3u;
    while (CRM_CFG & 0xC) { }
    CRM_CTRL &= 0xFEF2FFFFu;
    CRM_CFG = 0;
    CRM_MISC1 = 0;
    CRM_CLKINT = 0x009F0000;

    CRM_CTRL |= 1u << 16;                                 // HEXTEN
    while (!(CRM_CTRL & (1u << 17))) { }                  // HEXTSTBL
    CRM_CFG |= 1u << 16;                                  // PLLRCS = HEXT
    CRM_CFG |= 1u << 17;                                  // HEXT / 2
    CRM_CFG = (CRM_CFG & ~(0xFu << 18)) | (0xBu << 18);   // PLLMULT[3:0]
    CRM_CFG |= 3u << 29;                                  // PLLMULT[5:4] -> x60
    CRM_CFG |= 1u << 31;                                  // PLL range > 72 MHz
    CRM_MISC3 &= ~0x3000u;
    CRM_CTRL |= 1u << 24;                                 // PLLEN
    while (!(CRM_CTRL & (1u << 25))) { }                  // PLLSTBL
    CRM_CFG &= ~0xF0u;                                    // AHB / 1
    CRM_CFG = (CRM_CFG & ~(7u << 11)) | (4u << 11);       // APB2 / 2 = 120 MHz
    CRM_CFG = (CRM_CFG & ~(7u << 8)) | (4u << 8);         // APB1 / 2 = 120 MHz
    CRM_MISC3 |= 0x30;                                    // auto step (as stock)
    CRM_CFG = (CRM_CFG & ~3u) | 2;                        // SCLK = PLL
    while ((CRM_CFG & 0xC) != 8) { }
    CRM_MISC3 &= ~0x30u;
    SystemCoreClock = 240000000;
}

void board_init(void) {
    __asm volatile ("cpsid i");
    SYST_CSR = 0;
    for (int i = 0; i < 8; i++) { NVIC_ICER(i) = 0xFFFFFFFFu; NVIC_ICPR(i) = 0xFFFFFFFFu; }
    SCB_VTOR = 0x08007000;
    clock_init_240mhz();
    SYST_RVR = SystemCoreClock / 1000 - 1;
    SYST_CVR = 0;
    SYST_CSR = 7;                                         // HCLK, interrupt, enable
    __asm volatile ("cpsie i");
    // IOMUX, GPIOA..E, ADC1; XMC.
    CRM_APB2EN |= (1u << 0) | (1u << 2) | (1u << 3) | (1u << 4) | (1u << 5) | (1u << 6) | (1u << 9);
    CRM_AHBEN |= (1u << 8);
    GPIO_SCR(PWR_LATCH_PORT) = 1u << PWR_LATCH_PIN;
    gpio_mode(PWR_LATCH_PORT, PWR_LATCH_PIN, CFG_OUT_PP);
}
```

In `batt_adc_init()`, add as the first line (ADC clock must stay ≤ 28 MHz; stock uses APB2/6 = 20 MHz):
```c
    CRM_CFG = (CRM_CFG & ~((3u << 14) | (1u << 28))) | (2u << 14);   // ADCDIV = /6
```

- [ ] **Step 5: Create `fw/common/fault.c`**

```c
// HardFault: show what happened instead of freezing. Strong override of the weak handler.
#include "board.h"
#include "lcd.h"

void hardfault_report(uint32_t *frame) {
    char b[12];
    backlight(1);
    lcd_fill(0, 0, LCD_W, LCD_H, C_RED);
    lcd_text(1, 1, "HARD FAULT", C_WHITE, C_RED);
    int c = lcd_text(1, 3, "PC   0x", C_WHITE, C_RED);
    lcd_text(c, 3, fmt_x(b, frame[6], 8), C_WHITE, C_RED);
    c = lcd_text(1, 4, "LR   0x", C_WHITE, C_RED);
    lcd_text(c, 4, fmt_x(b, frame[5], 8), C_WHITE, C_RED);
    c = lcd_text(1, 5, "CFSR 0x", C_WHITE, C_RED);
    lcd_text(c, 5, fmt_x(b, REG32(0xE000ED28), 8), C_WHITE, C_RED);
    c = lcd_text(1, 6, "HFSR 0x", C_WHITE, C_RED);
    lcd_text(c, 6, fmt_x(b, REG32(0xE000ED2C), 8), C_WHITE, C_RED);
    c = lcd_text(1, 7, "BFAR 0x", C_WHITE, C_RED);
    lcd_text(c, 7, fmt_x(b, REG32(0xE000ED38), 8), C_WHITE, C_RED);
    lcd_text(1, 10, "Photo this, then press POWER", C_WHITE, C_RED);
    lcd_text(1, 11, "to switch off.", C_WHITE, C_RED);
    for (;;) {
        WDT_CMD = 0xAAAA;
        if (keys_read() & KEY_BIT(KEY_POWER)) power_off();
    }
}

__attribute__((naked)) void HardFault_Handler(void) {
    __asm volatile (
        "tst lr, #4      \n"
        "ite eq          \n"
        "mrseq r0, msp   \n"
        "mrsne r0, psp   \n"
        "b hardfault_report \n");
}
```

- [ ] **Step 6: Create `fw/apps/faulttest/main.c`**

```c
// faulttest: press RUN to execute an undefined instruction; the HardFault screen must appear.
#include "board.h"
#include "lcd.h"

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    lcd_text(1, 1, "faulttest @ 240 MHz", C_CYAN, C_BLACK);
    lcd_text(1, 3, "RUN: trigger a HardFault", C_WHITE, C_BLACK);
    lcd_text(1, 4, "Hold POWER: off", C_WHITE, C_BLACK);
    backlight(1);
    char b[12];
    for (;;) {
        uint32_t k = keys_read();
        if (k & KEY_BIT(KEY_RUN)) __asm volatile (".short 0xde00");   // UDF #0
        if (k & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER))
                if (millis() - t0 > 1000) { backlight(0); power_off(); }
        }
        int c = lcd_text(1, 6, "uptime s ", C_GRAY, C_BLACK);
        lcd_text(c, 6, fmt_u(b, millis() / 1000, 5), C_WHITE, C_BLACK);
        delay_ms(50);
    }
}
```

- [ ] **Step 7: Create `tools/flash.sh`**

```bash
#!/usr/bin/env bash
# Wait for the DSO-510 IAP drive (hold update button + power on), then copy a firmware .bin.
set -euo pipefail
BIN=${1:?usage: tools/flash.sh path/to/APP_DSO-510_x.bin}
[ -f "$BIN" ] || { echo "no such file: $BIN" >&2; exit 1; }
echo "Waiting for the IAP drive (hold update button and press power)..."
until DEV=$(lsblk -dnpo NAME,MODEL | awk '$2=="Msc" && $3=="iap" {print $1}' | head -1) && [ -n "$DEV" ]; do sleep 1; done
sleep 2
MNT=$(lsblk -no MOUNTPOINTS "$DEV" | head -1)
if [ -z "$MNT" ]; then udisksctl mount -b "$DEV" >/dev/null; MNT=$(lsblk -no MOUNTPOINTS "$DEV" | head -1); fi
echo "IAP drive at $MNT ($(ls "$MNT" | tr '\n' ' '))"
cp "$BIN" "$MNT/" && sync
echo "Copied $(basename "$BIN") ($(stat -c %s "$BIN") bytes). The scope flashes and starts the app."
```
Run `chmod +x tools/flash.sh`.

- [ ] **Step 8: Build everything and run host tests**

Run: `make -C fw everything && make -C tests`
Expected: all apps (`caldump` is not there yet; `hello keytest scopetest faulttest`) build with no warnings; tests `0 failed`.

- [ ] **Step 9: On-device check — clock and keys**

Flash keytest: `tools/flash.sh fw/build/APP_DSO-510_keytest.bin` (ask the user to enter update mode).
Ask the user to confirm: screen draws noticeably faster than before; keys show DOWN/up; battery line shows a plausible raw value (2300–2600 on battery); holding K1 for ~3 s switches off on battery.

- [ ] **Step 10: On-device check — fault screen**

Flash faulttest: `tools/flash.sh fw/build/APP_DSO-510_faulttest.bin`.
Ask the user to confirm: "uptime s" counts up once per second (proves SysTick = 1 ms at 240 MHz); pressing RUN shows the red HARD FAULT screen with a PC value; pressing POWER switches off.

- [ ] **Step 11: Commit**

```bash
git add fw tools/flash.sh
git commit -m "fw: 240 MHz PLL, SysTick ms timer, FPU, weak vectors, HardFault screen, flash script"
```

---

### Task 9: Driver split — `adc` (free-running clock), `frontend`, `testsig`; caldump app; scopetest update

**Files:**
- Create: `fw/common/adc.h`, `fw/common/adc.c`, `fw/common/frontend.h`, `fw/common/frontend.c`, `fw/common/testsig.h`, `fw/common/testsig.c`, `fw/apps/caldump/main.c`
- Delete: `fw/common/scope.h`, `fw/common/scope.c`
- Modify: `fw/apps/scopetest/main.c`, `NOTES.md`

**Interfaces:**
- Consumes: `delay_ms` (Task 8), `units_rate_for_tdiv`, `tdiv_ns`, `NUM_TDIV` (Task 1), `cal_parse`, `CAL_PAGE_ADDR` (Task 6).
- Produces:
  - `adc.h`: `void adc_init(void); void adc_set_rate(uint16_t psc, uint16_t arr); void adc_start(uint8_t *buf, uint16_t n); int adc_done(void); void adc_abort(void);`
  - `frontend.h`: `FE_PWM_MAX` 0xFFE, `FE_PC5_FOR_DC` (1), `void fe_init(void); void fe_set_range(int r); void fe_set_coupling_dc(int dc); void fe_set_offset_pwm(uint16_t pwm);`
  - `testsig.h`: `void cal_out_init(void); void dac2_bias_init(void); void gen_sine_init(uint16_t arr); void gen_set_arr(uint16_t arr);` (f = 240 MHz / (arr+1) / 100)

- [ ] **Step 1: Create `fw/common/adc.h` and `adc.c`**

`adc.h`:
```c
// External ADC (3PEAK 3PA1030, pipelined): top 8 bits on PB0..PB7, clock on PB8 = TMR4 CH3.
// The clock runs continuously so the pipeline always holds fresh conversions.
#pragma once
#include <stdint.h>

void adc_init(void);
void adc_set_rate(uint16_t psc, uint16_t arr);   // rate = 240 MHz / ((psc+1)(arr+1)); waits for pipeline flush
void adc_start(uint8_t *buf, uint16_t n);        // arm DMA1 ch7 for n samples (non-blocking)
int adc_done(void);                               // 1 once the armed capture is complete (or none armed)
void adc_abort(void);
```

`adc.c`:
```c
#include "adc.h"
#include "board.h"

#define TMR4_BASE 0x40000800u
#define TMR_CTRL1(t) REG32((t) + 0x00)
#define TMR_IDEN(t)  REG32((t) + 0x0C)
#define TMR_SWEVT(t) REG32((t) + 0x14)
#define TMR_CM2(t)   REG32((t) + 0x1C)
#define TMR_CCTRL(t) REG32((t) + 0x20)
#define TMR_DIV(t)   REG32((t) + 0x28)
#define TMR_PR(t)    REG32((t) + 0x2C)
#define TMR_C3DT(t)  REG32((t) + 0x3C)
#define TMR_BRK(t)   REG32((t) + 0x44)

#define DMA1_STS      REG32(0x40020000)
#define DMA1_CLR      REG32(0x40020004)
#define DMA1_C7CTRL   REG32(0x40020080)
#define DMA1_C7DTCNT  REG32(0x40020084)
#define DMA1_C7PADDR  REG32(0x40020088)
#define DMA1_C7MADDR  REG32(0x4002008C)
#define DMA1_SRC_SEL1 REG32(0x400200A4)
#define DMA_REQ_TMR4_OVF 0x4F   // stock value for the ch7 field

static volatile int busy;

void adc_init(void) {
    CRM_AHBEN |= 1u << 0;            // DMA1
    CRM_APB1EN |= 1u << 2;           // TMR4
    for (int p = 0; p < 8; p++) gpio_mode(GPIOB, p, CFG_IN_FLOAT);
    // Stock: PB8 output, 1 us high pulse, low, then alternate function (TMR4 CH3).
    gpio_mode(GPIOB, 8, CFG_OUT_PP);
    GPIO_SCR(GPIOB) = 1u << 8;
    delay_ms(1);
    GPIO_CLR(GPIOB) = 1u << 8;
    gpio_mode(GPIOB, 8, CFG_AF_PP);

    TMR_CTRL1(TMR4_BASE) = 0;
    TMR_CM2(TMR4_BASE) = (TMR_CM2(TMR4_BASE) & ~0x70u) | 0x60;   // CH3 PWM mode 1
    TMR_CCTRL(TMR4_BASE) |= 0x300;                                // CH3 enable, active low (stock)
    TMR_IDEN(TMR4_BASE) |= 0x100;                                 // overflow DMA request
    TMR_BRK(TMR4_BASE) |= 0x8000;

    DMA1_C7CTRL = 0;
    DMA1_SRC_SEL1 |= 1u << 24;
    DMA1_SRC_SEL1 = (DMA1_SRC_SEL1 & 0xFF00FFFFu) | ((uint32_t)DMA_REQ_TMR4_OVF << 16);
    DMA1_C7PADDR = 0x40010C08;                                    // GPIOB->IDT
    busy = 0;
}

void adc_set_rate(uint16_t psc, uint16_t arr) {
    adc_abort();
    TMR_CTRL1(TMR4_BASE) &= ~1u;
    TMR_DIV(TMR4_BASE) = psc;
    TMR_PR(TMR4_BASE) = arr;
    TMR_C3DT(TMR4_BASE) = (arr + 1u) / 2u;
    TMR_SWEVT(TMR4_BASE) = 1;
    TMR_CTRL1(TMR4_BASE) |= 1;
    // Let >= 64 conversions flush the pipeline: 64 * (psc+1)(arr+1) / 240 MHz.
    uint32_t us = (uint32_t)((uint64_t)64u * (psc + 1u) * (arr + 1u) / 240u);
    delay_ms(us / 1000u + 1u);
}

void adc_start(uint8_t *buf, uint16_t n) {
    DMA1_C7CTRL = 0;
    DMA1_CLR = 0xFu << 24;
    DMA1_C7MADDR = (uint32_t)buf;
    DMA1_C7DTCNT = n;
    busy = 1;
    DMA1_C7CTRL = 0x3000 | 0x80 | 1;   // very high prio, 8-bit, mem increment, enable
}

int adc_done(void) {
    if (!busy) return 1;
    if (DMA1_STS & (1u << 25)) {       // ch7 transfer complete
        DMA1_C7CTRL = 0;
        DMA1_CLR = 0xFu << 24;
        busy = 0;
        return 1;
    }
    return 0;
}

void adc_abort(void) {
    DMA1_C7CTRL = 0;
    DMA1_CLR = 0xFu << 24;
    busy = 0;
}
```

- [ ] **Step 2: Create `fw/common/frontend.h` and `frontend.c`**

`frontend.h`:
```c
// Analog front end: range pins (PC4, PE6, PE5, PE4), coupling (PC5), offset PWM (PA7 = TMR14 CH1).
#pragma once
#include <stdint.h>

#define FE_NUM_RANGES 10
#define FE_PWM_MAX    0xFFE
#define FE_PC5_FOR_DC 1      // PC5 level that selects DC coupling; verified on hardware in Task 11

void fe_init(void);
void fe_set_range(int r);           // r = index into vdiv_mv (stock pin table FUN_08008834)
void fe_set_coupling_dc(int dc);
void fe_set_offset_pwm(uint16_t pwm);
```

`frontend.c`:
```c
#include "frontend.h"
#include "board.h"

#define TMR14_BASE 0x40002000u
#define TMR_CTRL1(t) REG32((t) + 0x00)
#define TMR_SWEVT(t) REG32((t) + 0x14)
#define TMR_CM1(t)   REG32((t) + 0x18)
#define TMR_CCTRL(t) REG32((t) + 0x20)
#define TMR_DIV(t)   REG32((t) + 0x28)
#define TMR_PR(t)    REG32((t) + 0x2C)
#define TMR_C1DT(t)  REG32((t) + 0x34)
#define TMR_BRK(t)   REG32((t) + 0x44)

// range -> {PC4, PE6, PE5, PE4}
static const uint8_t range_pins[FE_NUM_RANGES][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {1, 1, 1, 0}, {1, 0, 0, 1}, {1, 1, 0, 1},
    {0, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}, {0, 1, 0, 1},
};

static void pin(uint32_t port, unsigned p, int v) {
    if (v) GPIO_SCR(port) = 1u << p; else GPIO_CLR(port) = 1u << p;
}

void fe_init(void) {
    gpio_mode(GPIOC, 4, CFG_OUT_PP);
    gpio_mode(GPIOC, 5, CFG_OUT_PP);
    gpio_mode(GPIOE, 4, CFG_OUT_PP);
    gpio_mode(GPIOE, 5, CFG_OUT_PP);
    gpio_mode(GPIOE, 6, CFG_OUT_PP);
    CRM_APB1EN |= 1u << 8;                       // TMR14
    gpio_mode(GPIOA, 7, CFG_AF_PP);
    TMR_PR(TMR14_BASE) = FE_PWM_MAX;
    TMR_DIV(TMR14_BASE) = 0;
    TMR_CM1(TMR14_BASE) = (TMR_CM1(TMR14_BASE) & ~0x70u) | 0x70;   // PWM mode 2 (stock)
    TMR_CCTRL(TMR14_BASE) |= 3;                                     // enable, inverted (stock)
    TMR_C1DT(TMR14_BASE) = 0x834;
    TMR_BRK(TMR14_BASE) |= 0x8000;
    TMR_SWEVT(TMR14_BASE) = 1;
    TMR_CTRL1(TMR14_BASE) |= 1;
}

void fe_set_range(int r) {
    const uint8_t *t = range_pins[r];
    pin(GPIOC, 4, t[0]); pin(GPIOE, 6, t[1]); pin(GPIOE, 5, t[2]); pin(GPIOE, 4, t[3]);
}

void fe_set_coupling_dc(int dc) { pin(GPIOC, 5, dc ? FE_PC5_FOR_DC : !FE_PC5_FOR_DC); }

void fe_set_offset_pwm(uint16_t pwm) { TMR_C1DT(TMR14_BASE) = pwm > FE_PWM_MAX ? FE_PWM_MAX : pwm; }
```

- [ ] **Step 3: Create `fw/common/testsig.h` and `testsig.c`**

`testsig.h`:
```c
// Test signals (stock configuration, at 240 MHz timer clocks). Generator UI is sub-project 3.
#pragma once
#include <stdint.h>

void cal_out_init(void);           // PA3 = TMR9 CH2: 1 kHz, 50 % square on the cal pad
void dac2_bias_init(void);         // PA5 = DAC2 held at 0xF46 like stock (purpose unknown)
void gen_sine_init(uint16_t arr);  // PA4 = DAC1 sine, 100 points: f = 240 MHz / (arr+1) / 100
void gen_set_arr(uint16_t arr);
```

`testsig.c`:
```c
#include "testsig.h"
#include "board.h"
#include "sine100.h"

#define TMR6_BASE 0x40001000u
#define TMR9_BASE 0x40014C00u
#define TMR_CTRL1(t) REG32((t) + 0x00)
#define TMR_CTRL2(t) REG32((t) + 0x04)
#define TMR_SWEVT(t) REG32((t) + 0x14)
#define TMR_CM1(t)   REG32((t) + 0x18)
#define TMR_CCTRL(t) REG32((t) + 0x20)
#define TMR_DIV(t)   REG32((t) + 0x28)
#define TMR_PR(t)    REG32((t) + 0x2C)
#define TMR_C2DT(t)  REG32((t) + 0x38)
#define TMR_BRK(t)   REG32((t) + 0x44)
#define DAC_CTRL     REG32(0x40007400)
#define DAC_SWTRG    REG32(0x40007404)
#define DAC_D1DTH12R REG32(0x40007408)
#define DAC_D2DTH12R REG32(0x40007414)
#define DMA2_C3CTRL  REG32(0x40020430)
#define DMA2_C3DTCNT REG32(0x40020434)
#define DMA2_C3PADDR REG32(0x40020438)
#define DMA2_C3MADDR REG32(0x4002043C)
#define DMA2_SRC_SEL0 REG32(0x400204A0)
#define DMA2_SRC_SEL1 REG32(0x400204A4)
#define DMA_REQ_DAC1 0x05

void cal_out_init(void) {
    CRM_APB2EN |= 1u << 19;                 // TMR9
    gpio_mode(GPIOA, 3, CFG_AF_PP);
    TMR_DIV(TMR9_BASE) = 2399;              // 240 MHz / 2400 = 100 kHz
    TMR_PR(TMR9_BASE) = 99;                 // 1 kHz
    TMR_CM1(TMR9_BASE) = (TMR_CM1(TMR9_BASE) & ~0x7000u) | 0x7000;
    TMR_CCTRL(TMR9_BASE) |= 0x30;
    TMR_C2DT(TMR9_BASE) = 50;
    TMR_BRK(TMR9_BASE) |= 0x8000;
    TMR_SWEVT(TMR9_BASE) = 1;
    TMR_CTRL1(TMR9_BASE) |= 1;
}

void dac2_bias_init(void) {
    CRM_APB1EN |= 1u << 29;
    gpio_mode(GPIOA, 5, CFG_ANALOG);
    DAC_CTRL |= 0x380000;
    DAC_CTRL |= 0x40000;
    DAC_CTRL &= 0xFF3FFFFFu;
    DAC_CTRL |= 0x20000;
    DAC_CTRL &= 0xEFFFFFFFu;
    DAC_CTRL |= 0x10000;
    DAC_D2DTH12R = 0xF46;
    DAC_SWTRG |= 2;
}

void gen_sine_init(uint16_t arr) {
    CRM_APB1EN |= (1u << 29) | (1u << 4);   // DAC, TMR6
    CRM_AHBEN |= 1u << 1;                   // DMA2
    gpio_mode(GPIOA, 4, CFG_ANALOG);
    TMR_CTRL1(TMR6_BASE) = 0;
    TMR_DIV(TMR6_BASE) = 0;
    TMR_PR(TMR6_BASE) = arr;
    TMR_CTRL2(TMR6_BASE) = (TMR_CTRL2(TMR6_BASE) & ~0x70u) | 0x20;
    DAC_CTRL &= ~0x38u;
    DAC_CTRL |= 4;
    DAC_CTRL &= ~0xC0u;
    DAC_CTRL |= 2;
    DAC_CTRL |= 0x1000;
    DMA2_C3CTRL = 0;
    DMA2_C3PADDR = (uint32_t)&DAC_D1DTH12R;
    DMA2_C3MADDR = (uint32_t)sine100;
    DMA2_C3DTCNT = 100;
    DMA2_SRC_SEL1 |= 1u << 24;
    DMA2_SRC_SEL0 = (DMA2_SRC_SEL0 & 0xFF00FFFFu) | ((uint32_t)DMA_REQ_DAC1 << 16);
    DMA2_C3CTRL = 0x10 | 0x20 | 0x80 | 0x200 | 0x400;
    DMA2_C3CTRL |= 1;
    DAC_CTRL |= 1;
    TMR_SWEVT(TMR6_BASE) = 1;
    TMR_CTRL1(TMR6_BASE) |= 1;
}

void gen_set_arr(uint16_t arr) {
    TMR_PR(TMR6_BASE) = arr;
    TMR_SWEVT(TMR6_BASE) = 1;
}
```

Delete `fw/common/scope.h` and `fw/common/scope.c` (`git rm`).

- [ ] **Step 4: Port `fw/apps/scopetest/main.c` to the new drivers**

Replace the file with:
```c
// scopetest @ 240 MHz: raw capture test. Wheel L/R = range, Up/Down = offset PWM,
// Run = timebase, Mode = coupling, Auto = freeze, wheel press = generator frequency, hold Power = off.
#include "board.h"
#include "lcd.h"
#include "adc.h"
#include "frontend.h"
#include "testsig.h"
#include "units.h"
#include "trigger.h"

#define PLOT_X 10
#define PLOT_Y 32
#define PLOT_W 300
#define PLOT_H 176
#define CAP_N  (PLOT_W * 2)

static uint8_t cap[CAP_N];
static int range = 5, tdiv = 7, dc = 1, frozen = 0, gen = 2;
static uint16_t offset = 0x834;
static uint32_t notrig;

static const struct { uint16_t arr; const char *label; } gen_freqs[] = {
    {23999, "100Hz"}, {7199, "333Hz"}, {2399, "1kHz"}, {1799, "1.33kHz"},
    {479, "5kHz"}, {329, "7.27kHz"}, {239, "10kHz"}, {119, "20kHz"},
};
#define NUM_GEN (int)(sizeof gen_freqs / sizeof gen_freqs[0])

static inline uint16_t bg_px(int x, int y) {
    if ((x % 30) == 0 && (y % 4) == 0) return C_GRAY;
    if ((y % 22) == 0 && (x % 6) == 0) return C_GRAY;
    return C_BLACK;
}

static void draw_col(int x, int y0, int y1) {
    lcd_window(PLOT_X + x, PLOT_Y, PLOT_X + x, PLOT_Y + PLOT_H - 1);
    for (int y = 0; y < PLOT_H; y++) lcd_push((y >= y0 && y <= y1) ? C_YELLOW : bg_px(x, y));
}

static int to_y(uint8_t v) { return PLOT_H - 1 - (v * (PLOT_H - 1)) / 255; }

static void apply_rate(void) {
    rate_cfg_t r = units_rate_for_tdiv(tdiv_ns[tdiv]);
    adc_set_rate(r.psc, r.arr);
}

static void draw_status(void) {
    char b[16];
    int c = lcd_text(0, 0, " R", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, fmt_u(b, range, 1), C_WHITE, C_BLACK);
    c = lcd_text(c, 0, "  rate ", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, fmt_u(b, units_rate_for_tdiv(tdiv_ns[tdiv]).rate_hz, 8), C_YELLOW, C_BLACK);
    c = lcd_text(c, 0, " OFS ", C_GRAY, C_BLACK);
    c = lcd_text(c, 0, fmt_u(b, offset, 4), C_WHITE, C_BLACK);
    lcd_text(c, 0, dc ? " DC" : " AC", C_WHITE, C_BLACK);
    c = lcd_text(0, 14, " GEN sine ", C_GRAY, C_BLACK);
    c = lcd_text(c, 14, gen_freqs[gen].label, C_CYAN, C_BLACK);
    lcd_text(c, 14, frozen ? "  HOLD          " : "                ", C_RED, C_BLACK);
}

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    for (int x = 0; x < PLOT_W; x++) draw_col(x, -1, -1);
    backlight(1);
    fe_init();
    fe_set_range(range); fe_set_coupling_dc(dc); fe_set_offset_pwm(offset);
    adc_init();
    dac2_bias_init();
    cal_out_init();
    gen_sine_init(gen_freqs[gen].arr);
    apply_rate();
    draw_status();

    uint32_t prev = 0;
    char b[16];
    for (;;) {
        uint32_t k = keys_read(), press = k & ~prev;
        prev = k;
        int changed = 0;
        if (press & KEY_BIT(KEY_WHEEL_R) && range < FE_NUM_RANGES - 1) { range++; changed = 1; }
        if (press & KEY_BIT(KEY_WHEEL_L) && range > 0) { range--; changed = 1; }
        if (k & KEY_BIT(KEY_UP))   { offset = offset + 16 > FE_PWM_MAX ? FE_PWM_MAX : offset + 16; changed = 1; }
        if (k & KEY_BIT(KEY_DOWN)) { offset = offset < 16 ? 0 : offset - 16; changed = 1; }
        if (press & KEY_BIT(KEY_RUN)) { tdiv = (tdiv + 1) % NUM_TDIV; apply_rate(); changed = 1; }
        if (press & KEY_BIT(KEY_MODE)) { dc ^= 1; changed = 1; }
        if (press & KEY_BIT(KEY_AUTO)) { frozen ^= 1; changed = 1; }
        if (press & KEY_BIT(KEY_WHEEL_PRESS)) { gen = (gen + 1) % NUM_GEN; gen_set_arr(gen_freqs[gen].arr); changed = 1; }
        if (k & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER))
                if (millis() - t0 > 1500) { backlight(0); power_off(); for (;;) { } }
        }
        if (changed) {
            fe_set_range(range); fe_set_coupling_dc(dc); fe_set_offset_pwm(offset);
            draw_status();
        }
        if (frozen) { delay_ms(20); continue; }

        adc_start(cap, CAP_N);
        uint32_t t0 = millis();
        while (!adc_done()) if (millis() - t0 > 3000) { adc_abort(); break; }

        int mn = 255, mx = 0; uint32_t sum = 0;
        for (int i = 0; i < CAP_N; i++) { sum += cap[i]; if (cap[i] < mn) mn = cap[i]; if (cap[i] > mx) mx = cap[i]; }
        int lvl = (int)(sum / CAP_N);
        int t = trigger_find(cap, 0, PLOT_W, lvl, trigger_hysteresis(cap, CAP_N), EDGE_RISING);
        if (t < 0) { notrig++; if (mx - mn > 8) continue; t = 0; }

        int last = to_y(cap[t]);
        for (int x = 0; x < PLOT_W; x++) {
            int y = to_y(cap[t + x]);
            draw_col(x, y < last ? y : last, y < last ? last : y);
            last = y;
        }
        int c = lcd_text(0, 1, " min ", C_GRAY, C_BLACK);
        c = lcd_text(c, 1, fmt_u(b, mn, 3), C_WHITE, C_BLACK);
        c = lcd_text(c, 1, " max ", C_GRAY, C_BLACK);
        c = lcd_text(c, 1, fmt_u(b, mx, 3), C_WHITE, C_BLACK);
        c = lcd_text(c, 1, " notrig ", C_GRAY, C_BLACK);
        lcd_text(c, 1, fmt_u(b, notrig, 5), C_WHITE, C_BLACK);
    }
}
```

- [ ] **Step 5: Create `fw/apps/caldump/main.c`**

```c
// caldump: show the factory settings/calibration page at 0x08006000 (read-only).
#include "board.h"
#include "lcd.h"
#include "cal.h"

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(1);
    const uint32_t *page = (const uint32_t *)CAL_PAGE_ADDR;
    char b[12];
    lcd_text(0, 0, " caldump 0x08006000 (words 00..26)", C_CYAN, C_BLACK);
    for (int row = 0; row < 10; row++) {
        int c = lcd_text(0, row + 1, fmt_x(b, row * 4, 2), C_GRAY, C_BLACK);
        for (int w = 0; w < 4 && row * 4 + w < CAL_PAGE_WORDS; w++) {
            c = lcd_text(c, row + 1, " ", C_BLACK, C_BLACK);
            c = lcd_text(c, row + 1, fmt_x(b, page[row * 4 + w], 8), C_WHITE, C_BLACK);
        }
    }
    cal_t cal;
    int ok = cal_parse(page, &cal);
    lcd_text(0, 12, ok ? " parse: VALID factory data" : " parse: INVALID -> defaults", ok ? C_GREEN : C_RED, C_BLACK);
    int c = lcd_text(0, 13, " slow r6 lo ", C_GRAY, C_BLACK);
    c = lcd_text(c, 13, fmt_x(b, cal.lo[CAL_SET_SLOW][6], 3), C_WHITE, C_BLACK);
    c = lcd_text(c, 13, " hi ", C_GRAY, C_BLACK);
    lcd_text(c, 13, fmt_x(b, cal.hi[CAL_SET_SLOW][6], 3), C_WHITE, C_BLACK);
    lcd_text(0, 14, " Photo this screen. Hold POWER = off", C_GRAY, C_BLACK);
    for (;;) {
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER))
                if (millis() - t0 > 1000) { backlight(0); power_off(); for (;;) { } }
        }
        delay_ms(20);
    }
}
```

- [ ] **Step 6: Build**

Run: `make -C fw everything && make -C tests`
Expected: `hello keytest scopetest faulttest caldump` build with no warnings; `0 failed`.

- [ ] **Step 7: On-device check — calibration page**

`tools/flash.sh fw/build/APP_DSO-510_caldump.bin`. Ask the user for a photo of the screen.
Expected: "parse: VALID factory data"; `slow r6 lo/hi` are different values roughly around 0x834 (e.g. 0x4xx–0xCxx). Transcribe the 39 words into `NOTES.md` under a new heading `## Factory calibration page (caldump)`. If INVALID: record that in `NOTES.md`; the app will run with defaults and auto-zero (Task 11) becomes required.

- [ ] **Step 8: On-device check — free-running ADC at 240 MHz**

`tools/flash.sh fw/build/APP_DSO-510_scopetest.bin`. Probe ×1 on the generator output. Ask the user to confirm: sine visible and stable (no jumping) for each wheel-press frequency at a suitable rate (Run cycles timebase); `notrig` stays near 0 on a periodic signal; the cal pad shows a square wave.

- [ ] **Step 9: Commit**

```bash
git add -A fw NOTES.md
git commit -m "fw: split drivers into adc (free-running clock)/frontend/testsig at 240 MHz; caldump app"
```

---

### Task 10: The scope app — UI and main loop

**Files:**
- Create: `fw/app/ui.h`, `fw/app/ui.c`, `fw/app/main.c`

**Interfaces:**
- Consumes: everything in `fw/core/` (Tasks 1–7) and `adc`, `frontend`, `testsig`, `board`, `lcd` (Tasks 8–9).
- Produces: the firmware `fw/build/APP_DSO-510_app.bin`. UI functions (used only inside `fw/app`):
  - `ui.h`: `PLOT_X 10, PLOT_Y 20, PLOT_W 300, PLOT_H 200`; `void ui_draw_static(void); void ui_draw_trace(const span_t *sp); void ui_draw_markers(const scope_state_t *s); void ui_draw_bars(const scope_state_t *s, int uncal); void ui_draw_meas(const scope_state_t *s, const meas_t *m); void ui_draw_menu(const scope_state_t *s); void ui_draw_batt(int pct); void ui_toast(const char *msg); void ui_toast_clear(void);`
  - Code→pixel: `y = 227 - code`, clamped to 0..199 (codes 28..227 visible, 25 per division).

- [ ] **Step 1: Create `fw/app/ui.h`**

```c
#pragma once
#include "control.h"
#include "render.h"
#include "measure.h"

#define PLOT_X 10
#define PLOT_Y 20
#define PLOT_W 300
#define PLOT_H 200

void ui_draw_static(void);                 /* clear screen + empty grid */
void ui_draw_trace(const span_t *sp);      /* PLOT_W columns */
void ui_draw_markers(const scope_state_t *s);
void ui_draw_bars(const scope_state_t *s, int uncal);
void ui_draw_meas(const scope_state_t *s, const meas_t *m);
void ui_draw_menu(const scope_state_t *s);
void ui_draw_batt(int pct);
void ui_toast(const char *msg);            /* top bar, cols 19..32 */
void ui_toast_clear(void);
```

- [ ] **Step 2: Create `fw/app/ui.c`**

```c
#include "ui.h"
#include "lcd.h"
#include "fmt.h"
#include "units.h"

#define C_ORANGE RGB565(255, 140, 0)

static int code_to_y(int code) {
    int y = 227 - code;
    return y < 0 ? 0 : y > PLOT_H - 1 ? PLOT_H - 1 : y;
}

static inline uint16_t bg_px(int x, int y) {
    if ((x % 30) == 0 && (y % 4) == 0) return C_GRAY;     /* 10 vertical divisions */
    if ((y % 25) == 0 && (x % 6) == 0) return C_GRAY;     /* 8 horizontal divisions */
    if (x == PLOT_W / 2 && (y % 2) == 0) return C_GRAY;   /* centre lines */
    if (y == PLOT_H / 2 && (x % 2) == 0) return C_GRAY;
    return C_BLACK;
}

static void draw_col(int x, int y0, int y1) {
    lcd_window(PLOT_X + x, PLOT_Y, PLOT_X + x, PLOT_Y + PLOT_H - 1);
    for (int y = 0; y < PLOT_H; y++) lcd_push((y >= y0 && y <= y1) ? C_YELLOW : bg_px(x, y));
}

void ui_draw_static(void) {
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    for (int x = 0; x < PLOT_W; x++) draw_col(x, -1, -1);
}

void ui_draw_trace(const span_t *sp) {
    for (int x = 0; x < PLOT_W; x++) {
        if (sp[x].lo < 0) { draw_col(x, -1, -1); continue; }
        draw_col(x, code_to_y(sp[x].hi), code_to_y(sp[x].lo));
    }
}

void ui_draw_markers(const scope_state_t *s) {
    /* 0 V baseline: left margin */
    lcd_fill(0, PLOT_Y, PLOT_X - 1, PLOT_H, C_BLACK);
    int y = code_to_y(ADC_CENTER + s->offset);
    lcd_fill(0, PLOT_Y + (y > 1 ? y - 1 : 0), PLOT_X - 2, 3, C_YELLOW);
    /* trigger level: right margin */
    lcd_fill(PLOT_X + PLOT_W + 1, PLOT_Y, LCD_W - PLOT_X - PLOT_W - 1, PLOT_H, C_BLACK);
    y = code_to_y(s->trig_level);
    lcd_fill(PLOT_X + PLOT_W + 2, PLOT_Y + (y > 1 ? y - 1 : 0), 7, 3, C_CYAN);
    /* trigger position: strip above the plot */
    lcd_fill(PLOT_X, 16, PLOT_W, 4, C_BLACK);
    int x = s->trig_pos - 2 < 0 ? 0 : s->trig_pos - 2;
    lcd_fill(PLOT_X + x, 16, 5, 4, C_ORANGE);
}

void ui_draw_bars(const scope_state_t *s, int uncal) {
    static const char *ctl_names[] = {"[T/V]", "[P/O]", "[P/L]"};
    static const char *trig_names[] = {"Auto  ", "Normal", "Single"};
    char b[16];
    lcd_fill(0, 0, 19 * 8, 16, C_BLACK);
    int c = lcd_text(0, 0, " ", C_WHITE, C_BLACK);
    c = lcd_text(c, 0, fmt_si(b, (float)tdiv_ns[s->tdiv] * 1e-9f, "s", 1), C_WHITE, C_BLACK);
    lcd_text(7, 0, s->run ? "RUN " : "STOP", C_BLACK, s->run ? C_GREEN : C_RED);
    lcd_text(12, 0, ctl_names[s->ctl], C_CYAN, C_BLACK);
    lcd_text(33, 0, uncal ? "UNC" : "   ", C_RED, C_BLACK);

    lcd_fill(0, 224, 21 * 8, 16, C_BLACK);
    c = lcd_text(0, 14, " ", C_WHITE, C_BLACK);
    c = lcd_text(c, 14, fmt_si(b, (float)vdiv_mv[s->vdiv] * 1e-3f * (s->probe_x10 ? 10.0f : 1.0f), "V", 1),
                 C_YELLOW, C_BLACK);
    lcd_text(7, 14, trig_names[s->trig_mode], C_WHITE, C_BLACK);
    lcd_text(14, 14, s->probe_x10 ? "x10" : "x1 ", C_WHITE, C_BLACK);
    lcd_text(17, 14, s->coupling_dc ? "DC" : "AC", C_WHITE, C_BLACK);
    lcd_text(19, 14, s->edge ? "F" : "R", C_CYAN, C_BLACK);
}

void ui_draw_meas(const scope_state_t *s, const meas_t *m) {
    char b[16];
    lcd_fill(21 * 8, 224, LCD_W - 21 * 8, 16, C_BLACK);
    if (!m->valid) { lcd_text(21, 14, "---", C_GRAY, C_BLACK); }
    else {
        int c = lcd_text(21, 14, fmt_si(b, m->freq_hz, "Hz", 0), C_WHITE, C_BLACK);
        c = lcd_text(c + 1, 14, fmt_u(b, (uint32_t)(m->duty_pct + 0.5f), 2), C_WHITE, C_BLACK);
        lcd_text(c, 14, "%", C_WHITE, C_BLACK);
    }
    float vpp = (float)(m->vmax - m->vmin) * units_volts_per_code(s->vdiv, s->probe_x10);
    lcd_text(33, 14, fmt_si(b, vpp, "V", 0), C_YELLOW, C_BLACK);
}

void ui_draw_menu(const scope_state_t *s) {
    static const char *labels[MENU_ITEMS] = {"Coupling", "Probe", "Trigger", "Edge"};
    static const char *trig_names[] = {"Auto", "Normal", "Single"};
    const char *vals[MENU_ITEMS] = {
        s->coupling_dc ? "DC" : "AC", s->probe_x10 ? "x10" : "x1",
        trig_names[s->trig_mode], s->edge ? "Falling" : "Rising",
    };
    lcd_fill(8 * 8 - 4, 3 * 16 - 4, 24 * 8 + 8, 7 * 16 + 8, C_NAVY);
    lcd_text(8, 3, " SETTINGS   Mode=close  ", C_BLACK, C_CYAN);
    for (int i = 0; i < MENU_ITEMS; i++) {
        uint16_t bg = i == s->menu_item ? C_GRAY : C_NAVY;
        lcd_fill(8 * 8, (5 + i) * 16, 24 * 8, 16, bg);
        lcd_text(9, 5 + i, labels[i], C_WHITE, bg);
        lcd_text(20, 5 + i, "<       >", C_CYAN, bg);
        lcd_text(21, 5 + i, vals[i], C_YELLOW, bg);
    }
}

void ui_draw_batt(int pct) {
    char b[8];
    uint16_t col = pct <= 20 ? C_RED : C_GREEN;
    int c = lcd_text(36, 0, fmt_u(b, (uint32_t)pct, 3), col, C_BLACK);
    lcd_text(c, 0, "%", col, C_BLACK);
}

void ui_toast(const char *msg) {
    lcd_fill(19 * 8, 0, 14 * 8, 16, C_BLACK);
    lcd_text(19, 0, msg, C_BLACK, C_YELLOW);
}

void ui_toast_clear(void) { lcd_fill(19 * 8, 0, 14 * 8, 16, C_BLACK); }
```

- [ ] **Step 3: Create `fw/app/main.c`**

```c
// DSO-510 custom firmware: core scope (sub-project 1).
#include "board.h"
#include "lcd.h"
#include "adc.h"
#include "frontend.h"
#include "testsig.h"
#include "units.h"
#include "trigger.h"
#include "render.h"
#include "measure.h"
#include "cal.h"
#include "control.h"
#include "autoset.h"
#include "keys.h"
#include "ui.h"

#define REC_MAX (4 * 1200)

static uint8_t rec[REC_MAX];
static span_t spans[PLOT_W];
static cal_t cal;
static scope_state_t st;
static keys_state_t ks;
static rate_cfg_t rc;
static uint32_t sps;          /* samples per screen */
static int rec_n, armed;
static uint32_t toast_until, armed_at;

static void toast(const char *msg, uint32_t ms) { ui_toast(msg); toast_until = millis() + ms; }

static void apply_fe(void) {
    fe_set_range(st.vdiv);
    fe_set_coupling_dc(st.coupling_dc);
    fe_set_offset_pwm(cal_offset_pwm(&cal, cal_set_for_rate(rc.rate_hz), st.vdiv, st.offset));
}

static void apply_time(void) {
    rc = units_rate_for_tdiv(tdiv_ns[st.tdiv]);
    adc_set_rate(rc.psc, rc.arr);
    sps = units_samples_per_screen(rc.rate_hz, tdiv_ns[st.tdiv]);
    rec_n = (int)(4 * sps);
    if (rec_n > REC_MAX) rec_n = REC_MAX;
    apply_fe();
}

/* Expected capture time plus 1 s margin. */
static uint32_t capture_timeout_ms(void) { return (uint32_t)((uint64_t)rec_n * 1000u / rc.rate_hz) + 1000u; }

static int capture_blocking(void) {
    adc_start(rec, (uint16_t)rec_n);
    uint32_t t0 = millis(), lim = capture_timeout_ms();
    while (!adc_done()) {
        WDT_CMD = 0xAAAA;
        if (millis() - t0 > lim) { adc_abort(); toast(" ADC timeout  ", 2000); return -1; }
    }
    return 0;
}

static void redraw_all(void) {
    ui_draw_static();
    ui_draw_bars(&st, !cal.valid);
    ui_draw_markers(&st);
    if (st.menu_open) ui_draw_menu(&st);
}

static void process_capture(void) {
    int pre = (int)((uint32_t)st.trig_pos * sps / PLOT_W);
    int post = (int)sps - pre;
    int t = trigger_find(rec, pre, rec_n - post, st.trig_level, trigger_hysteresis(rec, rec_n), (edge_t)st.edge);
    int start;
    if (t >= 0) start = t - pre;
    else if (st.trig_mode == TRIG_AUTO) start = 0;
    else return;                                    /* Normal/Single: keep previous trace */
    if (st.trig_mode == TRIG_SINGLE) { st.run = 0; ui_draw_bars(&st, !cal.valid); }
    render_spans(rec, rec_n, start, (sps << 8) / PLOT_W, spans, PLOT_W);
    ui_draw_trace(spans);
    meas_t m;
    measure(rec + start, (int)sps, rc.rate_hz, &m);
    ui_draw_meas(&st, &m);
}

static void minmax(int *mn, int *mx) {
    *mn = 255; *mx = 0;
    for (int i = 0; i < rec_n; i++) { if (rec[i] < *mn) *mn = rec[i]; if (rec[i] > *mx) *mx = rec[i]; }
}

static void do_autoset(void) {
    toast(" AUTO...      ", 60000);
    adc_abort(); armed = 0;
    st.offset = 0; st.trig_mode = TRIG_AUTO; st.trig_pos = PLOT_W / 2; st.run = 1;
    st.tdiv = TDIV_1MS;
    apply_time();
    for (int it = 0; it < 5; it++) {
        capture_blocking();
        int mn, mx; minmax(&mn, &mx);
        int nv = autoset_vdiv(st.vdiv, mn, mx);
        if (nv == st.vdiv) break;
        st.vdiv = (int8_t)nv; apply_fe(); delay_ms(30);
    }
    meas_t m;
    capture_blocking();
    measure(rec, rec_n, rc.rate_hz, &m);
    if (m.valid && m.freq_hz > 20000.0f) {
        st.tdiv = TDIV_10US; apply_time(); capture_blocking();
        measure(rec, rec_n, rc.rate_hz, &m);
    }
    if (m.valid) { int nt = autoset_tdiv(m.freq_hz); if (nt >= 0) st.tdiv = (int8_t)nt; }
    apply_time();
    capture_blocking();
    int mn, mx; minmax(&mn, &mx);
    st.trig_level = (int16_t)((mn + mx) / 2);
    redraw_all();
    ui_toast_clear();
}

static int mean_at_pwm(uint16_t pwm, void *ctx) {
    (void)ctx;
    fe_set_offset_pwm(pwm);
    delay_ms(40);                               /* let the PA7 PWM filter settle */
    capture_blocking();
    uint32_t sum = 0;
    for (int i = 0; i < rec_n; i++) sum += rec[i];
    return (int)(sum / (uint32_t)rec_n);
}

static void do_autozero(void) {
    toast(" ZERO: short! ", 60000);
    adc_abort(); armed = 0;
    int set = cal_set_for_rate(rc.rate_hz), keep = st.vdiv;
    for (int r = 0; r < NUM_VDIV; r++) {
        char b[4];
        ui_toast(" ZERO range   ");
        lcd_text(31, 0, fmt_u(b, (uint32_t)r, 1), C_BLACK, C_YELLOW);
        fe_set_range(r);
        uint16_t p = cal_search_pwm(mean_at_pwm, 0, ADC_CENTER, cal.lo[set][r], cal.hi[set][r]);
        cal_shift_zero(&cal, set, r, p);
    }
    cal.valid = 1;
    st.vdiv = (int8_t)keep;
    apply_fe();
    redraw_all();
    toast(" ZERO done    ", 2000);
}

static void power_down(void) {
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK);
    backlight(0);
    power_off();
    for (;;) WDT_CMD = 0xAAAA;   /* on USB power the scope stays alive; keep the watchdog quiet */
}

int main(void) {
    board_init();
    keys_init();
    lcd_init();
    batt_adc_init();
    cal_parse((const uint32_t *)CAL_PAGE_ADDR, &cal);
    state_defaults(&st);
    keys_reset(&ks, KEY_BIT(KEY_UP) | KEY_BIT(KEY_DOWN) | KEY_BIT(KEY_WHEEL_L) | KEY_BIT(KEY_WHEEL_R));
    fe_init();
    adc_init();
    dac2_bias_init();
    cal_out_init();
    gen_sine_init(2399);          /* 1 kHz test sine until sub-project 3 adds the generator UI */
    apply_time();
    redraw_all();
    backlight(1);
    if (!cal.valid) toast(" no factory cal", 3000);

    uint32_t next_batt = 0;
    for (;;) {
        kev_t ev[8];
        int ne = keys_update(&ks, keys_read(), millis(), ev, 8);
        uint32_t chg = 0;
        for (int i = 0; i < ne; i++) chg |= control_handle(&st, ev[i]);

        if (chg & ACT_POWEROFF) power_down();
        if (chg & CHG_TIME) { adc_abort(); armed = 0; apply_time(); }
        if (chg & CHG_FE) { adc_abort(); armed = 0; apply_fe(); }
        if (chg & CHG_RUN) { adc_abort(); armed = 0; }
        if (chg & CHG_SCREEN) redraw_all();
        else {
            if (chg & CHG_BARS) ui_draw_bars(&st, !cal.valid);
            if (chg & (CHG_TRIG | CHG_FE)) ui_draw_markers(&st);
            if ((chg & CHG_MENU) && st.menu_open) ui_draw_menu(&st);
        }
        if (chg & ACT_SAVE) toast(" CSV: step 2  ", 1500);
        if (chg & ACT_AUTOSET) do_autoset();
        if (chg & ACT_AUTOZERO) do_autozero();

        if (st.run && !st.menu_open) {
            if (!armed) { adc_start(rec, (uint16_t)rec_n); armed = 1; armed_at = millis(); }
            else if (adc_done()) { armed = 0; process_capture(); }
            else if (millis() - armed_at > capture_timeout_ms()) {
                adc_abort(); armed = 0; toast(" ADC timeout  ", 2000);
            }
        }

        uint32_t now = millis();
        if ((int32_t)(now - next_batt) >= 0) {
            next_batt = now + 2000;
            float v = (float)batt_adc_read() * 3.3f / 4095.0f * 2.0f;
            ui_draw_batt(units_batt_pct(v));
        }
        if (toast_until && (int32_t)(now - toast_until) >= 0) { toast_until = 0; ui_toast_clear(); }
        WDT_CMD = 0xAAAA;
    }
}
```

- [ ] **Step 4: Build**

Run: `make -C fw everything && make -C tests`
Expected: all apps including `app` build with no warnings; `0 failed`.

- [ ] **Step 5: On-device smoke test**

`tools/flash.sh fw/build/APP_DSO-510_app.bin`. Probe ×1 on the generator output (1 kHz sine). Ask the user to confirm:
1. Screen: top bar `1ms RUN [T/V] … %`, grid, yellow trace, bottom bar `1V Auto x1 DC R …` with a frequency near `1.00kHz`.
2. Wheel ←/→ changes the timebase label and the number of cycles shown; Up/Down changes V/div and the trace height.
3. Hold Power ~1 s → switches off (on battery).

- [ ] **Step 6: Commit**

```bash
git add fw/app
git commit -m "app: core scope UI, acquisition loop, trigger modes, auto-set, auto-zero"
```

---

### Task 11: On-device verification and calibration of assumptions

**Files:**
- Modify: `fw/common/frontend.h` (only if coupling is inverted), `NOTES.md`, spec `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` (mark sub-project 1 done)

**Interfaces:**
- Consumes: `fw/build/APP_DSO-510_app.bin` (Task 10).
- Produces: verified behaviour recorded in `NOTES.md`.

- [ ] **Step 1: Coupling direction**

Probe ×1 on the **cal pad** (0–3.3 V square), app at 1 V/div, 500 us/div. Long-press Mode → set Coupling DC.
Expected with DC: the square sits between the 0 V marker (left yellow tick) and ~3.3 div above it. With AC: centred on the 0 V marker.
If they are swapped, change `#define FE_PC5_FOR_DC 1` to `0` in `fw/common/frontend.h`, rebuild, reflash, recheck.

- [ ] **Step 2: Gain check on every usable range**

Cal pad, DC, offset −50 (Mode to [P/O], Down ×10) so the square fits. Note the Vpp readout at 1 V/div, 2 V/div, 500 mV/div (3.3 V does not fit below 500 mV/div). Expected ≈ 3.3 V (±5 %). Record results in `NOTES.md`. If off by a constant factor on all ranges, record it as a follow-up item (gain table) in `NOTES.md`; do not change the 25 codes/div constant in this task.

- [ ] **Step 3: Trigger modes and edges**

Generator sine (1 kHz), 200 us/div:
- Auto: trace stable; removing the probe still shows a flat line.
- Normal: stable; removing the probe freezes the last trace.
- Single: Run → captures once and shows STOP; Run again re-arms.
- Edge Falling: trace starts on a falling slope at the trigger marker.
- [P/L] mode Up/Down moves the cyan level marker; moving it above the signal in Normal freezes the display.
- [P/O]/[P/L] wheel moves the orange trigger-position marker and the trace shifts accordingly.

- [ ] **Step 4: Timebase sweep and auto-set**

Wheel through all timebases 500 ns…50 ms on the generator sine and on the cal pad: no hard fault, no hang; 50 ms/div updates about every 2 s. Press Auto with the probe on the cal pad: ends at ≈1 V/div, 200 us/div, stable. Press Auto on the 1 kHz sine: similar.

- [ ] **Step 5: Auto-zero (only if needed)**

Remove probe tip from signals and short probe tip to ground clip. If the flat line is visibly off the 0 V marker on some ranges, long-press Auto and wait for "ZERO done"; the line must sit on the marker afterwards on all ranges. Record whether factory calibration was already good.

- [ ] **Step 6: Record and commit**

Add a `## Sub-project 1 verification (date)` section to `NOTES.md` with the results of Steps 1–5. In the spec, add `Status: sub-project 1 implemented and verified (date)` under the title.

```bash
git add NOTES.md docs/superpowers/specs/2026-11-13-dso510-firmware-design.md fw/common/frontend.h
git commit -m "Verify core scope on hardware; record coupling, gain, trigger results"
```
