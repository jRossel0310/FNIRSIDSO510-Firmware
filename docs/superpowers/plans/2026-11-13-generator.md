# Function Generator (Sub-project 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A 9-waveform function generator (1 Hz–50 kHz, 0.1–3.0 Vpp, offset, duty) on the DAC output, controlled from a generator page in the menu and from a quick `Gen` control mode, running alongside the scope.

**Architecture:** All generator maths is pure and host-tested in `fw/core/gen.[ch]`: settings and clamping, frequency stepping, the frequency planner, and waveform table generation. Key handling for the page and the quick mode extends the existing `fw/core/control.[ch]`. A small hardware driver, `fw/common/gendac.[ch]`, plays a table through DAC1 using TMR6 and DMA2 channel 3 in circular mode. The scope app (`fw/app/main.c`, `ui.c`) applies the settings and draws the page.

**Tech Stack:** arm-none-eabi-gcc (Cortex-M4F hard float, no libm), host `cc` tests.

**Spec:** `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` § "Sub-project 3: function generator". Hardware facts are in `NOTES.md` (generator = DAC1 / PA4, TMR6 trigger, DMA2 ch3 flex request 0x05).

## Global Constraints

- Never write or erase internal flash below 0x08007000. Never touch 0x08006000. SPI flash writes only inside `[0, 0x500000)`.
- Every app implements power-off.
- `fw/core/*` includes no hardware header and compiles with host `cc -std=c99 -Wall -Wextra -Werror`. It uses no libm functions (`sinf`, `expf`, …), because the firmware links without libm.
- Firmware builds with 0 warnings: `make -C fw clean everything 2>&1 | grep -ciE "warning|error"` prints `0`. Host tests: `make -C tests` prints `… 0 failed`.
- Waveforms, in this order: sine, square, triangle, sawtooth (rising), DC, half-wave sine, full-wave sine, exponential rise, exponential fall.
- Frequency 1 Hz … 50 kHz. Timer clock 240 MHz. The DAC update rate is at most 1 MS/s, so the timer divider is at least 240. At least 20 points per period. Table length at most 400 points. Frequency error ≤ 0.1 % (1000 ppm) for every frequency the controls can set (three significant digits: a multiple of 1 % of its decade). Checked at plan time: worst case 625 ppm at 49.9 kHz; unreachable in-between values near 50 kHz can be off by up to 0.13 %.
- Amplitude 0.1 … 3.0 Vpp in 0.1 V steps. Offset (centre) in 0.1 V steps, clamped so the wave stays within 0 … 3.1 V. Full scale: DAC code 4095 = 3100 mV. For DC the level is the offset (0 … 3.1 V).
- Duty 1 … 99 % (square only).
- Exponential waveforms span five time constants per period.
- Power-on default: output on, sine, 1 kHz, 2.0 Vpp, offset 1.5 V. Settings are not persisted.
- The scope, CSV save, USB drive and streaming must keep working unchanged.
- The older test apps keep using `testsig.c`'s `gen_sine_init`/`gen_set_arr`. Do not remove them.
- Commit after every task with targeted `git add` (never `-A`, never `docs/` photos or `.superpowers/`). Every commit message ends with the line `Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp`.

**Plan-time decisions (rulings; update the spec in Task 3):**
1. To meet 0.1 % near 50 kHz, a table may hold `m` whole periods in `n` points (`n/m ≥ 20`). Then f = 240 MHz × m / (divider × n). With a single period the top of the range would be off by up to 0.18 %. The 0.1 % limit is guaranteed for settable frequencies (three significant digits), not for every integer hertz.
2. The bottom bar has no free columns for `G <freq>`. Instead the top bar shows a one-character `G` marker (col 28) while the output is on, and the quick-mode label shows the live frequency and amplitude, e.g. `Gen 1kHz 2.0V` (`~` between them = fine steps); 16 columns do not fit a `|` separator.

## File Structure

| File | Responsibility |
|---|---|
| `fw/core/gen.[ch]` | settings, defaults, clamping, frequency stepping, frequency planner, table generation |
| `fw/core/control.[ch]` | + generator page and quick-mode key handling; `Generator` menu item |
| `fw/common/gendac.[ch]` | DAC1 + TMR6 + DMA2 ch3: play a table, hold the present level, or hold 0 V |
| `fw/app/ui.[ch]` | + generator page, `G` marker, quick-mode label, 5-item menu |
| `fw/app/main.c` | apply generator settings; page open/close; replaces the fixed test sine |
| `tests/test_gen.c`, `tests/test_control.c` | host tests |

---

### Task 1: `gen` core — settings, stepping, planner, tables

**Files:**
- Create: `fw/core/gen.h`, `fw/core/gen.c`, `tests/test_gen.c`
- Modify: `tests/main.c`

**Interfaces:**
- Produces: the `GEN_*` constants, `gen_cfg_t`, `gen_plan_t`, `gen_defaults`, `gen_clamp`, `gen_freq_step`, `gen_plan`, `gen_table`, `gen_code_mv`, `gen_wave_name`.

- [ ] **Step 1: Write the failing test**

`tests/test_gen.c`:
```c
#include "test.h"
#include "gen.h"

static uint16_t tab[GEN_N_MAX];

static int near(int a, int b, int tol) { return a - b <= tol && b - a <= tol; }

void test_gen(void) {
    gen_cfg_t g;
    gen_defaults(&g);
    CHECK_EQ(g.on, 1); CHECK_EQ(g.wave, GEN_SINE); CHECK_EQ(g.freq_hz, 1000);
    CHECK_EQ(g.amp_dv, 20); CHECK_EQ(g.offset_dv, 15); CHECK_EQ(g.duty_pct, 50); CHECK_EQ(g.fine, 0);

    /* volts -> DAC code: 4095 = 3100 mV */
    CHECK_EQ(gen_code_mv(0), 0); CHECK_EQ(gen_code_mv(3100), 4095); CHECK_EQ(gen_code_mv(1550), 2048);
    CHECK_EQ(gen_code_mv(-5), 0); CHECK_EQ(gen_code_mv(5000), 4095);

    /* clamping: offset keeps the wave inside 0..3100 mV */
    g.amp_dv = 30; g.offset_dv = 0; gen_clamp(&g);  CHECK_EQ(g.offset_dv, 15);   /* 3.0 Vpp needs centre 1.5..1.6 */
    g.offset_dv = 31; gen_clamp(&g);                 CHECK_EQ(g.offset_dv, 16);
    g.amp_dv = 10; g.offset_dv = 2; gen_clamp(&g);   CHECK_EQ(g.offset_dv, 5);
    g.offset_dv = 30; gen_clamp(&g);                 CHECK_EQ(g.offset_dv, 26);
    g.amp_dv = 0; gen_clamp(&g);                     CHECK_EQ(g.amp_dv, 1);
    g.amp_dv = 99; gen_clamp(&g);                    CHECK_EQ(g.amp_dv, 30);
    g.duty_pct = 0; gen_clamp(&g);                   CHECK_EQ(g.duty_pct, 1);
    g.duty_pct = 100; gen_clamp(&g);                 CHECK_EQ(g.duty_pct, 99);
    g.freq_hz = 0; gen_clamp(&g);                    CHECK_EQ(g.freq_hz, 1);
    g.freq_hz = 99999; gen_clamp(&g);                CHECK_EQ(g.freq_hz, 50000);
    g.wave = 200; gen_clamp(&g);                     CHECK_EQ(g.wave, GEN_SINE);
    gen_defaults(&g); g.wave = GEN_DC; g.amp_dv = 30; g.offset_dv = 31; gen_clamp(&g);
    CHECK_EQ(g.offset_dv, 31);                        /* DC: level 0..3.1 V regardless of amplitude */
    g.offset_dv = 40; gen_clamp(&g);                 CHECK_EQ(g.offset_dv, 31);

    /* frequency stepping: coarse 1-2-5, fine = 1 % of the decade */
    CHECK_EQ(gen_freq_step(1000, 1, 0), 2000); CHECK_EQ(gen_freq_step(2000, 1, 0), 5000);
    CHECK_EQ(gen_freq_step(5000, 1, 0), 10000); CHECK_EQ(gen_freq_step(1000, -1, 0), 500);
    CHECK_EQ(gen_freq_step(1500, 1, 0), 2000); CHECK_EQ(gen_freq_step(1500, -1, 0), 1000);
    CHECK_EQ(gen_freq_step(50000, 1, 0), 50000); CHECK_EQ(gen_freq_step(1, -1, 0), 1);
    CHECK_EQ(gen_freq_step(20000, 1, 0), 50000);
    CHECK_EQ(gen_freq_step(1000, 1, 1), 1010); CHECK_EQ(gen_freq_step(1010, -1, 1), 1000);
    CHECK_EQ(gen_freq_step(1000, -1, 1), 999);         /* below 1000 the decade is 100: step 1 */
    CHECK_EQ(gen_freq_step(50, 1, 1), 51); CHECK_EQ(gen_freq_step(9990, 1, 1), 10000);
    CHECK_EQ(gen_freq_step(10000, 1, 1), 10100); CHECK_EQ(gen_freq_step(50000, 1, 1), 50000);
    CHECK_EQ(gen_freq_step(1, -1, 1), 1);

    /* planner: limits, and error <= 1000 ppm at every settable frequency (3 significant digits) */
    gen_plan_t p = gen_plan(1000);
    CHECK_EQ(p.m, 1); CHECK(p.n >= 20 && p.n <= GEN_N_MAX);
    CHECK_EQ(p.actual_mhz, 1000000);
    p = gen_plan(1);
    CHECK_EQ(p.actual_mhz, 1000); CHECK_EQ(p.n, GEN_N_MAX);
    p = gen_plan(50000);
    CHECK_EQ(p.actual_mhz, 50000000); CHECK_EQ(p.n / p.m, 20);
    uint32_t worst = 0;
    for (uint32_t f = 1; f <= 50000; f++) {
        uint32_t dec = 1;
        while (dec * 10u <= f) dec *= 10u;
        if (dec >= 100u && f % (dec / 100u)) continue;      /* only frequencies the controls can set */
        p = gen_plan(f);
        uint64_t div = (uint64_t)(p.psc + 1u) * (p.arr + 1u);
        uint64_t want = (uint64_t)f * 1000u;
        uint64_t act = p.actual_mhz;
        uint64_t diff = act > want ? act - want : want - act;
        uint32_t ppm = (uint32_t)(diff * 1000000u / want);
        if (ppm > worst) worst = ppm;
        if (div < GEN_MIN_DIV || p.n > GEN_N_MAX || p.m < 1 || p.n < 20u * p.m) { CHECK(0); break; }
        /* actual_mhz really is 240 MHz * m / (div * n) */
        uint64_t chk = ((uint64_t)GEN_TIMER_HZ * 1000u * p.m + div * p.n / 2) / (div * p.n);
        if (chk != act) { CHECK(0); break; }
    }
    CHECK(worst <= 1000);
    p = gen_plan(49500);                               /* needs several periods per table */
    CHECK(p.m > 1);

    /* tables: 2.0 Vpp around 1.5 V -> 500..2500 mV */
    gen_defaults(&g);
    int lo = gen_code_mv(500), hi = gen_code_mv(2500), mid = gen_code_mv(1500);
    gen_plan_t q = {100, 1, 0, 0, 0};
    gen_table(&g, &q, tab);                            /* sine */
    CHECK(near(tab[0], mid, 2)); CHECK(near(tab[25], hi, 2)); CHECK(near(tab[50], mid, 2)); CHECK(near(tab[75], lo, 2));
    int mn = 4095, mx = 0;
    for (int i = 0; i < 100; i++) { if (tab[i] < mn) mn = tab[i]; if (tab[i] > mx) mx = tab[i]; }
    CHECK(near(mn, lo, 2)); CHECK(near(mx, hi, 2));
    g.wave = GEN_SQUARE; g.duty_pct = 30; gen_table(&g, &q, tab);
    CHECK_EQ(tab[0], hi); CHECK_EQ(tab[29], hi); CHECK_EQ(tab[30], lo); CHECK_EQ(tab[99], lo);
    g.wave = GEN_TRIANGLE; gen_table(&g, &q, tab);
    CHECK_EQ(tab[0], lo); CHECK(near(tab[50], hi, 1)); CHECK(near(tab[25], mid, 1)); CHECK(near(tab[75], mid, 1));
    g.wave = GEN_SAW; gen_table(&g, &q, tab);
    CHECK_EQ(tab[0], lo); CHECK(near(tab[50], mid, 1)); CHECK(tab[99] > tab[98]); CHECK(near(tab[99], hi, 30));
    g.wave = GEN_DC; gen_table(&g, &q, tab);
    CHECK_EQ(tab[0], mid); CHECK_EQ(tab[57], mid); CHECK_EQ(tab[99], mid);
    g.wave = GEN_HALF; gen_table(&g, &q, tab);
    CHECK_EQ(tab[0], lo); CHECK(near(tab[25], hi, 2)); CHECK_EQ(tab[50], lo); CHECK_EQ(tab[75], lo); CHECK_EQ(tab[99], lo);
    g.wave = GEN_FULL; gen_table(&g, &q, tab);         /* one hump per period */
    CHECK_EQ(tab[0], lo); CHECK(near(tab[50], hi, 2)); CHECK(near(tab[25], tab[75], 2)); CHECK(tab[25] > mid);
    g.wave = GEN_EXP_RISE; gen_table(&g, &q, tab);
    CHECK_EQ(tab[0], lo); CHECK(near(tab[99], hi, 12));
    for (int i = 1; i < 100; i++) if (tab[i] < tab[i - 1]) { CHECK(0); break; }
    /* x = 0.2: (1 - e^-1) / (1 - e^-5) = 0.6364 of the span */
    CHECK(near(tab[20], lo + (int)((hi - lo) * 0.6364), 6));
    g.wave = GEN_EXP_FALL; gen_table(&g, &q, tab);
    CHECK_EQ(tab[0], hi); CHECK(near(tab[99], lo, 12));
    CHECK(near(tab[20], lo + (int)((hi - lo) * (1.0 - 0.6364)), 6));
    /* several periods per table repeat exactly */
    g.wave = GEN_SINE;
    gen_plan_t q2 = {40, 2, 0, 0, 0};
    gen_table(&g, &q2, tab);
    for (int i = 0; i < 20; i++) if (tab[i] != tab[i + 20]) { CHECK(0); break; }
    CHECK(near(tab[5], hi, 2));
    /* full scale: 3.0 Vpp around 1.5 V never leaves 0..4095 */
    gen_defaults(&g); g.amp_dv = 30; g.offset_dv = 15; gen_clamp(&g);
    gen_table(&g, &q, tab);
    for (int i = 0; i < 100; i++) if (tab[i] > 4095) { CHECK(0); break; }

    CHECK_STR(gen_wave_name(GEN_SINE), "Sine"); CHECK_STR(gen_wave_name(GEN_EXP_FALL), "Exp fall");
    CHECK_STR(gen_wave_name(99), "?");
}
```
Register `test_gen` in `tests/main.c` (declaration + call).

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile error, `gen.h: No such file or directory`.

- [ ] **Step 3: Implement**

`fw/core/gen.h`:
```c
#pragma once
#include <stdint.h>

/* Function generator maths (spec § Sub-project 3). Pure: no hardware, no libm. */
enum { GEN_SINE = 0, GEN_SQUARE, GEN_TRIANGLE, GEN_SAW, GEN_DC, GEN_HALF, GEN_FULL,
       GEN_EXP_RISE, GEN_EXP_FALL, GEN_NUM_WAVES };

#define GEN_FREQ_MIN 1u
#define GEN_FREQ_MAX 50000u
#define GEN_N_MAX    400u          /* table length limit */
#define GEN_PPP_MIN  20u           /* points per period, at least */
#define GEN_TIMER_HZ 240000000u
#define GEN_MIN_DIV  240u          /* timer divider >= 240: DAC update rate <= 1 MS/s */
#define GEN_FS_MV    3100          /* DAC code 4095 */
#define GEN_AMP_MAX_DV 30          /* 3.0 Vpp */
#define GEN_OFF_MAX_DV 31          /* 3.1 V */

typedef struct {
    uint8_t on;                    /* output enabled */
    uint8_t wave;                  /* GEN_* */
    uint8_t fine;                  /* frequency wheel: 0 = 1-2-5 steps, 1 = 1 % of the decade */
    uint8_t duty_pct;              /* square only, 1..99 */
    uint8_t amp_dv;                /* peak-to-peak in 0.1 V, 1..30 */
    uint8_t offset_dv;             /* centre (DC: level) in 0.1 V */
    uint32_t freq_hz;              /* 1..50000 */
} gen_cfg_t;

/* Timer + table for a frequency: n points hold m whole periods (n/m >= GEN_PPP_MIN);
   f = GEN_TIMER_HZ * m / ((psc + 1) * (arr + 1) * n). */
typedef struct { uint16_t n, m, psc, arr; uint32_t actual_mhz; } gen_plan_t;

void gen_defaults(gen_cfg_t *g);                 /* on, sine, 1 kHz, 2.0 Vpp, 1.5 V, 50 % */
void gen_clamp(gen_cfg_t *g);                    /* force every field into range; offset so the wave fits 0..3.1 V */
uint32_t gen_freq_step(uint32_t f, int dir, int fine);   /* next frequency up (dir > 0) or down */
gen_plan_t gen_plan(uint32_t freq_hz);           /* best plan; error <= 0.1 % at every settable frequency */
void gen_table(const gen_cfg_t *g, const gen_plan_t *p, uint16_t *out);   /* p->n DAC codes */
uint16_t gen_code_mv(int32_t mv);                /* millivolts -> DAC code, clamped 0..4095 */
const char *gen_wave_name(int wave);
```

`fw/core/gen.c`:
```c
#include "gen.h"

void gen_defaults(gen_cfg_t *g) {
    g->on = 1; g->wave = GEN_SINE; g->fine = 0; g->duty_pct = 50;
    g->amp_dv = 20; g->offset_dv = 15; g->freq_hz = 1000;
}

void gen_clamp(gen_cfg_t *g) {
    if (g->wave >= GEN_NUM_WAVES) g->wave = GEN_SINE;
    if (g->freq_hz < GEN_FREQ_MIN) g->freq_hz = GEN_FREQ_MIN;
    if (g->freq_hz > GEN_FREQ_MAX) g->freq_hz = GEN_FREQ_MAX;
    if (g->duty_pct < 1) g->duty_pct = 1;
    if (g->duty_pct > 99) g->duty_pct = 99;
    if (g->amp_dv < 1) g->amp_dv = 1;
    if (g->amp_dv > GEN_AMP_MAX_DV) g->amp_dv = GEN_AMP_MAX_DV;
    if (g->wave == GEN_DC) {
        if (g->offset_dv > GEN_OFF_MAX_DV) g->offset_dv = GEN_OFF_MAX_DV;
        return;
    }
    /* centre*100 - amp*50 >= 0 and centre*100 + amp*50 <= 3100 (mV) */
    int lo = (g->amp_dv + 1) / 2, hi = (GEN_FS_MV - g->amp_dv * 50) / 100;
    if (g->offset_dv < lo) g->offset_dv = (uint8_t)lo;
    if (g->offset_dv > hi) g->offset_dv = (uint8_t)hi;
}

uint32_t gen_freq_step(uint32_t f, int dir, int fine) {
    uint32_t dec = 1;
    while (dec * 10u <= f) dec *= 10u;                 /* largest power of ten <= f */
    uint32_t r;
    if (fine) {
        if (dir > 0) r = f + (dec >= 100u ? dec / 100u : 1u);
        else {
            if (f == dec && dec >= 10u) dec /= 10u;    /* stepping down from 1000 uses the 100s decade */
            uint32_t st = dec >= 100u ? dec / 100u : 1u;
            r = f > st ? f - st : GEN_FREQ_MIN;
        }
    } else if (dir > 0) {
        r = f < 2u * dec ? 2u * dec : f < 5u * dec ? 5u * dec : 10u * dec;
    } else {
        r = f > 5u * dec ? 5u * dec : f > 2u * dec ? 2u * dec : f > dec ? dec : dec / 2u;
    }
    if (r < GEN_FREQ_MIN) r = GEN_FREQ_MIN;
    if (r > GEN_FREQ_MAX) r = GEN_FREQ_MAX;
    return r;
}

gen_plan_t gen_plan(uint32_t f) {
    gen_plan_t best = {0, 0, 0, 0, 0};
    uint32_t best_ppm = 0xFFFFFFFFu;
    if (f < GEN_FREQ_MIN) f = GEN_FREQ_MIN;
    if (f > GEN_FREQ_MAX) f = GEN_FREQ_MAX;
    uint64_t want = (uint64_t)f * 1000u;               /* mHz */
    /* Prefer one period per table and the longest table; add periods only when that is not accurate. */
    for (uint32_t m = 1; m <= GEN_N_MAX / GEN_PPP_MIN; m++) {
        for (uint32_t n = GEN_N_MAX; n >= GEN_PPP_MIN * m; n--) {
            uint64_t fn = (uint64_t)f * n;
            uint64_t num = (uint64_t)GEN_TIMER_HZ * m;
            uint32_t d = (uint32_t)((num + fn / 2) / fn);          /* total timer divider */
            if (d < GEN_MIN_DIV) continue;
            uint32_t ps = (d + 65535u) / 65536u;                   /* prescaler factor so the reload fits 16 bits */
            uint32_t ar = (d + ps / 2) / ps;
            if (ar > 65536u) ar = 65536u;
            uint64_t den = (uint64_t)ps * ar * n;
            uint64_t act = (num * 1000u + den / 2) / den;          /* mHz */
            uint64_t diff = act > want ? act - want : want - act;
            uint32_t ppm = (uint32_t)(diff * 1000000u / want);
            if (ppm < best_ppm) {
                best_ppm = ppm;
                best.n = (uint16_t)n; best.m = (uint16_t)m;
                best.psc = (uint16_t)(ps - 1u); best.arr = (uint16_t)(ar - 1u);
                best.actual_mhz = (uint32_t)act;
            }
            if (ppm <= 200u) return best;                          /* good enough: stop at the smoothest table */
        }
    }
    return best;
}

uint16_t gen_code_mv(int32_t mv) {
    if (mv <= 0) return 0;
    if (mv >= GEN_FS_MV) return 4095;
    return (uint16_t)((mv * 4095 + GEN_FS_MV / 2) / GEN_FS_MV);
}

/* sin(2*pi*x) for x in [0, 1): odd 7th-order polynomial on [-pi/2, pi/2] (error < 2e-4). */
static float gsin(float x) {
    if (x >= 0.75f) x -= 1.0f;                          /* -> [-0.25, 0.75) turns */
    if (x > 0.25f) x = 0.5f - x;                        /* mirror into [-0.25, 0.25] */
    float a = x * 6.2831853f, a2 = a * a;
    return a * (1.0f - a2 / 6.0f * (1.0f - a2 / 20.0f * (1.0f - a2 / 42.0f)));
}

/* e^(-t) for t in [0, 5]: (e^(-t/16))^16 with a 4-term series for the small step. */
static float gexpn(float t) {
    float s = t / 16.0f;
    float e = 1.0f - s * (1.0f - s / 2.0f * (1.0f - s / 3.0f * (1.0f - s / 4.0f)));
    e *= e; e *= e; e *= e; e *= e;
    return e;
}

void gen_table(const gen_cfg_t *g, const gen_plan_t *p, uint16_t *out) {
    int32_t lo_mv = (int32_t)g->offset_dv * 100 - (int32_t)g->amp_dv * 50;
    int32_t hi_mv = (int32_t)g->offset_dv * 100 + (int32_t)g->amp_dv * 50;
    const float e5 = 0.006737947f;                      /* e^-5 */
    uint32_t n = p->n, m = p->m ? p->m : 1u;
    for (uint32_t i = 0; i < n; i++) {
        float x = (float)((i * m) % n) / (float)n;      /* phase within the period, 0..1 */
        float s;                                        /* shape, 0..1 */
        switch (g->wave) {
        case GEN_SQUARE:   s = x * 100.0f < (float)g->duty_pct ? 1.0f : 0.0f; break;
        case GEN_TRIANGLE: s = x < 0.5f ? 2.0f * x : 2.0f - 2.0f * x; break;
        case GEN_SAW:      s = x; break;
        case GEN_DC:       s = 0.5f; break;
        case GEN_HALF:     s = gsin(x); if (s < 0.0f) s = 0.0f; break;
        case GEN_FULL:     s = gsin(x * 0.5f); if (s < 0.0f) s = 0.0f; break;   /* one hump: sin(pi x) */
        case GEN_EXP_RISE: s = (1.0f - gexpn(5.0f * x)) / (1.0f - e5); break;
        case GEN_EXP_FALL: s = (gexpn(5.0f * x) - e5) / (1.0f - e5); break;
        default:           s = 0.5f + 0.5f * gsin(x); break;                   /* sine */
        }
        if (s < 0.0f) s = 0.0f;
        if (s > 1.0f) s = 1.0f;
        int32_t mv = g->wave == GEN_DC ? (int32_t)g->offset_dv * 100
                                       : lo_mv + (int32_t)(s * (float)(hi_mv - lo_mv) + 0.5f);
        out[i] = gen_code_mv(mv);
    }
}

const char *gen_wave_name(int wave) {
    static const char *names[GEN_NUM_WAVES] = {"Sine", "Square", "Triangle", "Sawtooth", "DC",
                                               "Half-wave", "Full-wave", "Exp rise", "Exp fall"};
    return wave >= 0 && wave < GEN_NUM_WAVES ? names[wave] : "?";
}
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `make -C tests`
Expected: `… 0 failed`.
- If the `worst <= 1000` check fails, do not relax it. Report the worst frequency and its ppm (print them from a scratch program).
- If a table check is off by slightly more than its tolerance, check the maths in `gsin`/`gexpn` first. Do not widen the tolerances.

- [ ] **Step 5: Firmware build**

Run: `make -C fw everything 2>&1 | grep -ciE "warning|error"`
Expected: `0`. `gen.c` is compiled into every app and must not pull in libm. The build fails with undefined `sinf`/`expf` if it does.

- [ ] **Step 6: Commit**

```bash
git add fw/core/gen.h fw/core/gen.c tests/test_gen.c tests/main.c
git commit -m "core: function generator maths (settings, 1-2-5/fine stepping, multi-period planner, 9 waveform tables)

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 2: Control — `Generator` menu item, generator page keys, quick `Gen` mode

**Files:**
- Modify: `fw/core/control.h`, `fw/core/control.c`, `tests/test_control.c`
- Modify: `fw/app/ui.c`. These are minimal changes so the firmware stays correct with 5 menu items and 4 control modes. The full page UI comes in Task 3.

**Interfaces:**
- Consumes: `gen_cfg_t`, `gen_defaults`, `gen_clamp`, `gen_freq_step`, `GEN_NUM_WAVES`, `GEN_AMP_MAX_DV`, `GEN_OFF_MAX_DV` (Task 1).
- Produces:
  - `scope_state_t.gen` (a `gen_cfg_t`), `scope_state_t.gen_open` and `scope_state_t.gen_item`.
  - `MENU_ITEMS` 5, with item 4 = Generator.
  - `GEN_ROWS` 6, in the order Output, Waveform, Frequency, Amplitude, Offset, Duty.
  - `CTL_GEN` = 3.
  - `CHG_GEN` 0x1000: generator settings changed, so the hardware must be re-applied.
  - `CHG_GENPAGE` 0x2000: the generator page needs a redraw.

Key rules:
- **Menu:** on item 4 (`Generator`), a wheel left/right tap or a short wheel press opens the page. That sets `menu_open = 0`, `gen_open = 1` and `gen_item = 0`, and returns `CHG_SCREEN`. On the other items the wheel press still closes the menu.
- **Page (`gen_open`)**, handled before everything except Power:
  - Mode short or long closes the page (`CHG_SCREEN`).
  - Up/Down taps move the row with wrap (`CHG_GENPAGE`).
  - A short wheel press on the Frequency row toggles `gen.fine` (`CHG_GENPAGE`).
  - A wheel tap changes the row's value, then `gen_clamp`. It returns `CHG_GEN | CHG_GENPAGE | CHG_BARS`.
  - All other keys are ignored. Run, Auto and Save do nothing while the page is open.
- **Quick mode:** the short-press Mode cycle becomes 4 modes. In `CTL_GEN`:
  - A wheel tap steps the frequency (`CHG_GEN | CHG_BARS`).
  - An Up/Down tap changes the amplitude by ±0.1 V, then `gen_clamp` (`CHG_GEN | CHG_BARS`). It returns 0 if nothing changed.
  - A short wheel press toggles fine (`CHG_BARS`).

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_control.c` a new function, and register it in `tests/main.c`:
```c
void test_control_gen(void) {
    scope_state_t s;
    state_defaults(&s);
    CHECK_EQ(s.gen.on, 1); CHECK_EQ(s.gen.freq_hz, 1000); CHECK_EQ(s.gen_open, 0);

    /* Mode cycle now has four control modes */
    control_handle(&s, K(KEV_SHORT, KEY_MODE)); control_handle(&s, K(KEV_SHORT, KEY_MODE));
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_BARS);
    CHECK_EQ(s.ctl, CTL_GEN);
    /* quick mode: wheel = frequency, Up/Down = amplitude, wheel press = fine */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_GEN | CHG_BARS);
    CHECK_EQ(s.gen.freq_hz, 2000);
    CHECK_EQ(control_handle(&s, K(KEV_REPEAT, KEY_WHEEL_L)), CHG_GEN | CHG_BARS);
    CHECK_EQ(s.gen.freq_hz, 1000);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_PRESS)), CHG_BARS);
    CHECK_EQ(s.gen.fine, 1);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.freq_hz, 1010);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), CHG_GEN | CHG_BARS);
    CHECK_EQ(s.gen.amp_dv, 21);
    s.gen.amp_dv = 30; gen_clamp(&s.gen);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), 0);            /* at the limit: nothing changes */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_DOWN)), CHG_GEN | CHG_BARS);
    CHECK_EQ(s.gen.amp_dv, 29);
    CHECK_EQ(s.tdiv, TDIV_1MS); CHECK_EQ(s.trig_pos, 150);            /* scope settings untouched */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_BARS);
    CHECK_EQ(s.ctl, CTL_TIME_VOLT);

    /* menu: fifth item opens the generator page */
    state_defaults(&s);
    control_handle(&s, K(KEV_LONG, KEY_MODE));
    CHECK_EQ(s.menu_open, 1);
    for (int i = 0; i < 4; i++) control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.menu_item, 4);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.menu_item, 0);                                         /* wraps over 5 items */
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    CHECK_EQ(s.menu_item, 4);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 0); CHECK_EQ(s.gen_open, 1); CHECK_EQ(s.gen_item, 0);

    /* page: rows and values */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_GEN | CHG_GENPAGE | CHG_BARS);
    CHECK_EQ(s.gen.on, 0);                                            /* row 0: output toggle */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L));
    CHECK_EQ(s.gen.on, 1);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_DOWN)), CHG_GENPAGE);
    CHECK_EQ(s.gen_item, 1);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.wave, GEN_SQUARE);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L)); control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L));
    CHECK_EQ(s.gen.wave, GEN_EXP_FALL);                               /* wraps */
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));                       /* row 2: frequency */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.freq_hz, 2000);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_PRESS)), CHG_GENPAGE);
    CHECK_EQ(s.gen.fine, 1);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.freq_hz, 2010);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));                       /* row 3: amplitude */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L));
    CHECK_EQ(s.gen.amp_dv, 19);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));                       /* row 4: offset */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.offset_dv, 16);
    for (int i = 0; i < 40; i++) control_handle(&s, K(KEV_REPEAT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.offset_dv, 21);                                    /* clamped: 1.9 Vpp fits up to 2.1 V centre */
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));                       /* row 5: duty */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L));
    CHECK_EQ(s.gen.duty_pct, 49);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.gen_item, 0);                                          /* wraps over 6 rows */
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    CHECK_EQ(s.gen_item, 5);
    /* page swallows scope keys */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_RUN)), 0);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_RUN)), 0);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_AUTO)), 0);
    CHECK_EQ(s.run, 1);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_POWER)), ACT_POWEROFF);   /* power still works */
    /* Mode closes the page (short or long), and a long press afterwards opens the menu again */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.gen_open, 0); CHECK_EQ(s.menu_open, 0);
    s.gen_open = 1;
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.gen_open, 0); CHECK_EQ(s.menu_open, 0);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 1);
    /* wheel press on a normal menu item still closes the menu */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_PRESS)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 0); CHECK_EQ(s.gen_open, 0);
}
```
The existing `test_control` asserts that the menu wraps after 3 presses (`menu_item == 3`, "wraps"), and that the Mode cycle returns to `CTL_TIME_VOLT` after three presses. Update those existing expectations for 5 items and 4 modes, and say exactly which lines you changed in your report:
- The Mode cycle check needs one more `control_handle(&s, K(KEV_SHORT, KEY_MODE));` before `CHECK_EQ(s.ctl, CTL_TIME_VOLT)`.
- The three `KEY_UP` presses from item 2 now land on `menu_item == 4`.

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `make -C tests`
Expected: compile errors (`s.gen`, `CTL_GEN`, `CHG_GEN` undefined).

- [ ] **Step 3: Implement the control changes**

`fw/core/control.h`:
- Add `#include "gen.h"`.
- `#define MENU_ITEMS 5`, `#define GEN_ROWS 6`.
- `enum { CTL_TIME_VOLT = 0, CTL_POS_OFFSET, CTL_POS_LEVEL, CTL_GEN, CTL_NUM };`
- Add:
```c
#define CHG_GEN      0x1000u  /* generator settings changed: re-apply to the hardware */
#define CHG_GENPAGE  0x2000u  /* generator page needs a redraw */
```
- In `scope_state_t` add `gen_cfg_t gen;` and `uint8_t gen_open, gen_item;`. Their comment should read: "generator page open / selected row (0 Output, 1 Waveform, 2 Frequency, 3 Amplitude, 4 Offset, 5 Duty)".

`fw/core/control.c`:
- In `state_defaults`: `gen_defaults(&s->gen); s->gen_open = 0; s->gen_item = 0;`.
- Add before `menu_key`:
```c
static uint32_t gen_key(scope_state_t *s, int k, int t, int tap) {
    gen_cfg_t *g = &s->gen;
    if (k == KEY_MODE && (t == KEV_SHORT || t == KEV_LONG)) { s->gen_open = 0; return CHG_SCREEN; }
    if (k == KEY_UP && tap) { s->gen_item = (uint8_t)((s->gen_item + GEN_ROWS - 1) % GEN_ROWS); return CHG_GENPAGE; }
    if (k == KEY_DOWN && tap) { s->gen_item = (uint8_t)((s->gen_item + 1) % GEN_ROWS); return CHG_GENPAGE; }
    if (k == KEY_WHEEL_PRESS && t == KEV_SHORT && s->gen_item == 2) { g->fine ^= 1; return CHG_GENPAGE; }
    if ((k == KEY_WHEEL_L || k == KEY_WHEEL_R) && tap) {
        int d = k == KEY_WHEEL_R ? 1 : -1;
        switch (s->gen_item) {
        case 0: g->on ^= 1; break;
        case 1: g->wave = (uint8_t)((g->wave + GEN_NUM_WAVES + d) % GEN_NUM_WAVES); break;
        case 2: g->freq_hz = gen_freq_step(g->freq_hz, d, g->fine); break;
        case 3: g->amp_dv = (uint8_t)clampi(g->amp_dv + d, 1, GEN_AMP_MAX_DV); break;
        case 4: g->offset_dv = (uint8_t)clampi(g->offset_dv + d, 0, GEN_OFF_MAX_DV); break;
        default: g->duty_pct = (uint8_t)clampi(g->duty_pct + d, 1, 99); break;
        }
        gen_clamp(g);
        return CHG_GEN | CHG_GENPAGE | CHG_BARS;
    }
    return 0;
}
```
- In `menu_key`, as its first statement:
```c
    if (s->menu_item == 4 && ((k == KEY_WHEEL_PRESS && t == KEV_SHORT) ||
                              ((k == KEY_WHEEL_L || k == KEY_WHEEL_R) && tap))) {
        s->menu_open = 0; s->gen_open = 1; s->gen_item = 0;
        return CHG_SCREEN;
    }
```
- In `control_handle`:
  - Right after the Power line, add `if (s->gen_open) return gen_key(s, k, t, tap);`.
  - Change the Mode cycle to `% CTL_NUM`.
  - After `if (!tap) return 0;` and the `h`/`v` computation, before the `CTL_TIME_VOLT` block, add:
```c
    if (s->ctl == CTL_GEN) {
        gen_cfg_t *g = &s->gen;
        if (k == KEY_WHEEL_PRESS && t == KEV_SHORT) { g->fine ^= 1; return CHG_BARS; }
        if (h) {
            uint32_t f = gen_freq_step(g->freq_hz, h, g->fine);
            if (f == g->freq_hz) return 0;
            g->freq_hz = f; return CHG_GEN | CHG_BARS;
        }
        if (v) {
            int a = clampi(g->amp_dv + v, 1, GEN_AMP_MAX_DV);
            if (a == g->amp_dv) return 0;
            g->amp_dv = (uint8_t)a; gen_clamp(g); return CHG_GEN | CHG_BARS;
        }
        return 0;
    }
```

- [ ] **Step 4: Keep the firmware UI consistent (minimal)**

In `fw/app/ui.c`:
- `ui_draw_bars`:
  - Give `ctl_names` a fourth entry, `"Gen Freq | Amp       "` (padded to 21 characters like the others; without it `ctl_names[CTL_GEN]` reads past the array). Task 3 replaces it with a live label.
- `ui_draw_menu`:
  - Add the label `"Generator"` and the value `"open >"` as the fifth entries of `labels` and `vals`.
  - Grow the box by one row: the `lcd_fill` height changes from `7 * 16 + 8` to `8 * 16 + 8`.

- [ ] **Step 5: Run the tests and build**

Run: `make -C tests` and `make -C fw clean everything 2>&1 | grep -ciE "warning|error"`
Expected: `… 0 failed` and `0`.

- [ ] **Step 6: Commit**

```bash
git add fw/core/control.h fw/core/control.c tests/test_control.c tests/main.c fw/app/ui.c
git commit -m "control: Generator menu item, generator page keys, quick Gen control mode

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 3: DAC driver, generator page UI, app wiring

**Files:**
- Create: `fw/common/gendac.h`, `fw/common/gendac.c`
- Modify: `fw/app/ui.h`, `fw/app/ui.c`, `fw/app/main.c`, `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md`

**Interfaces:**
- Consumes: `gen_*` (Task 1), `scope_state_t.gen`/`gen_open`/`gen_item`, `CHG_GEN`, `CHG_GENPAGE`, `CTL_GEN` (Task 2).
- Produces: `gendac_init()`, `gendac_load(const uint16_t *tab, uint16_t n, uint16_t psc, uint16_t arr)`, `gendac_hold()`, `gendac_off()`; `ui_draw_gen(const scope_state_t *s, uint32_t actual_mhz)`, `ui_draw_gen_marker(int on)`.

- [ ] **Step 1: The DAC driver**

`fw/common/gendac.h`:
```c
#pragma once
#include <stdint.h>
/* Function generator output: DAC1 on PA4, updated by TMR6, fed by DMA2 channel 3 (flex request 0x05)
   from a table in RAM, circular. Runs without CPU involvement once loaded. */
void gendac_init(void);                 /* clocks, pin, DAC on, output 0 V */
/* Play tab[0..n) repeatedly, one point per TMR6 update: rate = 240 MHz / ((psc + 1) * (arr + 1)).
   tab must stay valid (static) while playing. Restarts cleanly if already playing. */
void gendac_load(const uint16_t *tab, uint16_t n, uint16_t psc, uint16_t arr);
void gendac_hold(void);                 /* stop the table; the output keeps its present level */
void gendac_off(void);                  /* stop the table, hold 0 V */
```

`fw/common/gendac.c`. The register sequence follows the proven `gen_sine_init` in `fw/common/testsig.c`; read that function first:
```c
#include "gendac.h"
#include "board.h"

#define TMR6_BASE 0x40001000u
#define TMR_CTRL1(t) REG32((t) + 0x00)
#define TMR_CTRL2(t) REG32((t) + 0x04)
#define TMR_SWEVT(t) REG32((t) + 0x14)
#define TMR_DIV(t)   REG32((t) + 0x28)
#define TMR_PR(t)    REG32((t) + 0x2C)
#define DAC_CTRL     REG32(0x40007400)
#define DAC_D1DTH12R REG32(0x40007408)
#define DMA2_C3CTRL  REG32(0x40020430)
#define DMA2_C3DTCNT REG32(0x40020434)
#define DMA2_C3PADDR REG32(0x40020438)
#define DMA2_C3MADDR REG32(0x4002043C)
#define DMA2_SRC_SEL0 REG32(0x400204A0)
#define DMA2_SRC_SEL1 REG32(0x400204A4)
#define DMA_REQ_DAC1 0x05

/* DAC_CTRL channel 1 bits: EN 0, output buffer off 1, trigger enable 2, trigger select 5:3, wave 7:6, DMA 12 */
#define DAC1_BITS 0x0000FFFFu

static void stop(void) {
    TMR_CTRL1(TMR6_BASE) &= ~1u;                       /* no more updates */
    DMA2_C3CTRL &= ~1u;
    DAC_CTRL &= ~(0x1000u | 4u);                       /* DMA request off, trigger off: writes take effect at once */
}

void gendac_init(void) {
    CRM_APB1EN |= (1u << 29) | (1u << 4);              /* DAC, TMR6 */
    CRM_AHBEN |= 1u << 1;                              /* DMA2 */
    gpio_mode(GPIOA, 4, CFG_ANALOG);
    TMR_CTRL1(TMR6_BASE) = 0;
    TMR_CTRL2(TMR6_BASE) = (TMR_CTRL2(TMR6_BASE) & ~0x70u) | 0x20;   /* update event -> trigger output */
    DMA2_C3CTRL = 0;
    DMA2_SRC_SEL1 |= 1u << 24;                         /* flexible request mapping on */
    DMA2_SRC_SEL0 = (DMA2_SRC_SEL0 & 0xFF00FFFFu) | ((uint32_t)DMA_REQ_DAC1 << 16);
    DAC_CTRL = (DAC_CTRL & ~DAC1_BITS) | 2u;           /* channel 1: buffer off (as stock), TMR6 trigger select 000 */
    DAC_D1DTH12R = 0;
    DAC_CTRL |= 1u;                                    /* enable: output 0 V */
}

void gendac_load(const uint16_t *tab, uint16_t n, uint16_t psc, uint16_t arr) {
    stop();
    TMR_DIV(TMR6_BASE) = psc;
    TMR_PR(TMR6_BASE) = arr;
    DMA2_C3CTRL = 0;
    DMA2_C3PADDR = (uint32_t)&DAC_D1DTH12R;
    DMA2_C3MADDR = (uint32_t)tab;
    DMA2_C3DTCNT = n;
    DMA2_C3CTRL = 0x10 | 0x20 | 0x80 | 0x200 | 0x400;  /* mem -> periph, circular, mem increment, 16-bit both */
    DMA2_C3CTRL |= 1u;
    DAC_CTRL |= 4u;                                    /* trigger enable (TMR6) */
    DAC_CTRL |= 0x1000u;                               /* DMA request enable */
    TMR_SWEVT(TMR6_BASE) = 1;                          /* load prescaler/reload, first sample */
    TMR_CTRL1(TMR6_BASE) |= 1u;
}

void gendac_hold(void) { stop(); }

void gendac_off(void) {
    stop();
    DAC_D1DTH12R = 0;                                  /* trigger is off: 0 V at once */
}
```
`DAC_CTRL` also holds DAC2's bits (PA5 bias, set by `dac2_bias_init`). Every write here must preserve the upper 16 bits, as the code above does.

- [ ] **Step 2: UI**

`fw/app/ui.h`, add:
```c
void ui_draw_gen(const scope_state_t *s, uint32_t actual_mhz);   /* generator page (over the plot) */
void ui_draw_gen_marker(int on);           /* top bar col 28: "G" while the generator output is on */
```

`fw/app/ui.c`:
- Add `#include "gen.h"`. It also arrives through control.h; add it explicitly anyway.
- Add:
```c
void ui_draw_gen_marker(int on) { lcd_text(28, 0, on ? "G" : " ", C_BLACK, on ? C_GREEN : C_BLACK); }

/* "2.0V" from tenths of a volt */
static char *fmt_dv(char *b, int dv) {
    b[0] = (char)('0' + dv / 10); b[1] = '.'; b[2] = (char)('0' + dv % 10); b[3] = 'V'; b[4] = 0;
    return b;
}

void ui_draw_gen(const scope_state_t *s, uint32_t actual_mhz) {
    static const char *labels[GEN_ROWS] = {"Output", "Waveform", "Frequency", "Amplitude", "Offset", "Duty"};
    const gen_cfg_t *g = &s->gen;
    char v[GEN_ROWS][16], b[16];
    const char *p;
    int k;
    /* row values */
    p = g->on ? "On" : "Off";            for (k = 0; p[k]; k++) v[0][k] = p[k]; v[0][k] = 0;
    p = gen_wave_name(g->wave);          for (k = 0; p[k]; k++) v[1][k] = p[k]; v[1][k] = 0;
    p = fmt_si(b, (float)g->freq_hz, "Hz", 1);
    for (k = 0; p[k]; k++) v[2][k] = p[k];
    if (g->fine) { v[2][k++] = ' '; v[2][k++] = 'f'; v[2][k++] = 'i'; v[2][k++] = 'n'; v[2][k++] = 'e'; }
    v[2][k] = 0;
    fmt_dv(v[3], g->amp_dv);
    fmt_dv(v[4], g->offset_dv);
    p = fmt_u(b, g->duty_pct, 1);        for (k = 0; p[k]; k++) v[5][k] = p[k]; v[5][k++] = '%'; v[5][k] = 0;

    lcd_fill(6 * 8 - 4, 2 * 16 - 4, 28 * 8 + 8, 11 * 16 + 8, C_NAVY);
    lcd_text(6, 2, " GENERATOR      Mode=close  ", C_BLACK, C_CYAN);
    for (int i = 0; i < GEN_ROWS; i++) {
        uint16_t bg = i == s->gen_item ? C_GRAY : C_NAVY;
        int dim = (i == 5 && g->wave != GEN_SQUARE) || (i == 3 && g->wave == GEN_DC);   /* row has no effect */
        lcd_fill(6 * 8, (4 + i) * 16, 28 * 8, 16, bg);
        lcd_text(7, 4 + i, labels[i], C_WHITE, bg);
        lcd_text(18, 4 + i, "<             >", C_CYAN, bg);
        lcd_text(20, 4 + i, v[i], dim ? C_GRAY : C_YELLOW, bg);
    }
    /* what the hardware really produces */
    lcd_text(7, 11, "actual ", C_GRAY, C_NAVY);
    if (g->wave == GEN_DC) lcd_text(14, 11, "DC level     ", C_WHITE, C_NAVY);
    else {
        int c = lcd_text(14, 11, fmt_si(b, (float)actual_mhz * 1e-3f, "Hz", 0), C_WHITE, C_NAVY);
        lcd_text(c, 11, "      ", C_WHITE, C_NAVY);
    }
    lcd_text(7, 12, "press wheel on Freq: fine", C_GRAY, C_NAVY);
}
```
The page box runs from column 6 to column 33; values start at column 20 and are at most 12 characters (`10.1kHz fine`). `fmt_si(buf, v, unit, trim)` and `fmt_u(buf, v, width)` are the existing helpers. Check their signatures in `fw/core/fmt.h` and `fw/common/lcd.h` before use.
- In `ui_draw_bars`, replace the `lcd_text(12, 0, ctl_names[s->ctl], …)` line with a live label for the quick mode:
```c
    if (s->ctl == CTL_GEN) {
        char l[24], t[8];
        int k = 0;
        const char *p = "Gen ";
        while (*p) l[k++] = *p++;
        p = fmt_si(b, (float)s->gen.freq_hz, "Hz", 1);
        while (*p && k < 11) l[k++] = *p++;                 /* at most 7 characters: "10.1kHz" */
        l[k++] = s->gen.fine ? '~' : ' ';
        p = fmt_dv(t, s->gen.amp_dv);
        while (*p) l[k++] = *p++;
        while (k < 16) l[k++] = ' ';
        l[k] = 0;
        lcd_text(12, 0, l, C_CYAN, C_BLACK);
    } else lcd_text(12, 0, ctl_names[s->ctl], C_CYAN, C_BLACK);
```
  - `fmt_dv` must be defined above `ui_draw_bars`: move it, or add a prototype.
  - The label is exactly 16 characters (cols 12..27), e.g. `Gen 1kHz 2.0V   ` or `Gen 10.1kHz~3.0V`. It never reaches the `G` marker (col 28) or the `USB` marker (cols 29..31).
  - Keep `ctl_names` at four entries and make every entry exactly 16 characters (pad with spaces, drop the padding beyond col 27), so each label fully overwrites the previous one and none overwrites the markers. `b` is the existing scratch buffer in `ui_draw_bars`.

- [ ] **Step 3: App wiring in `fw/app/main.c`**

1. Includes: add `#include "gen.h"` and `#include "gendac.h"`.
2. Globals:
```c
static uint16_t gen_tab[GEN_N_MAX];      /* played by DMA: must stay static */
static gen_plan_t gen_pl;

/* Push the generator settings to the hardware. */
static void gen_apply(void) {
    gen_clamp(&st.gen);
    if (!st.gen.on) { gendac_off(); return; }
    gen_pl = gen_plan(st.gen.wave == GEN_DC ? 1000u : st.gen.freq_hz);
    gendac_hold();                        /* DMA must not read the table while it is rewritten */
    gen_table(&st.gen, &gen_pl, gen_tab);
    gendac_load(gen_tab, gen_pl.n, gen_pl.psc, gen_pl.arr);
}
```
3. `draw_bars()`: after the existing drawing, add `if (!toast_until) ui_draw_gen_marker(st.gen.on);`. A toast covers cols 19..32, and the marker must not punch a hole in it; when the toast expires the loop calls `draw_bars()` again, which restores the marker. Keep the existing USB marker call.
4. `redraw_all()`: after the menu line, add `if (st.gen_open) ui_draw_gen(&st, gen_pl.actual_mhz);`.
5. `main()`: replace `gen_sine_init(2399);` and its comment with `gendac_init(); gen_apply();`. This must come after `state_defaults(&st)`, which already runs earlier. Remove the `testsig.h` include only if nothing else from it is used. `cal_out_init` and `dac2_bias_init` are still used, so it stays.
6. Main loop:
   - Directly above the line `if (chg & CHG_SCREEN) redraw_all();`, add `if (chg & CHG_GEN) gen_apply();`. `gen_apply()` must run before anything is drawn, so the page's "actual" line is current.
   - In the `else` block below it, after the `CHG_MENU` line, add `if ((chg & CHG_GENPAGE) && st.gen_open) ui_draw_gen(&st, gen_pl.actual_mhz);`.
7. Capture gating: change the capture condition `if (st.run && !st.menu_open)` to `if (st.run && !st.menu_open && !st.gen_open)`. The trace is held while the page is open, exactly like the menu.
8. Logger hook `hook_log_start`: where it sets `st.menu_open = 0;`, also set `st.gen_open = 0;`.

- [ ] **Step 4: Update the spec (plan-time rulings)**

In `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` § Sub-project 3:
- In "Signal", replace the sentence "One period is a table of N points (20 ≤ N ≤ 400) …" with: "A table of n points (n ≤ 400) holds m whole periods, with at least 20 points per period, played at TMR6's update rate (≤ 1 MS/s): f = 240 MHz × m / (timer divider × n). The planner prefers one period and the longest table and adds periods only when needed for accuracy."
- In "Controls", replace "Bottom bar shows `G <freq>` while the output is on." with: "The top bar shows a `G` marker while the output is on; in the `Gen` control mode the mode label shows the live frequency and amplitude (e.g. `Gen 1kHz 2.0V`; `Gen 1.01kHz~2.0V` = fine steps)."

- [ ] **Step 5: Build and test**

Run: `make -C fw clean everything 2>&1 | grep -ciE "warning|error"`, `make -C tests`, `python3 -m unittest discover -s tools/tests`, `arm-none-eabi-size fw/build/app.elf`
Expected: `0`; `… 0 failed`; `OK`. bss must stay below 64 KiB (it was 62460 before this task, and `gen_tab` adds 800 bytes). If bss would exceed 64 KiB, report it. Do not shrink other buffers on your own.

- [ ] **Step 6: Commit**

```bash
git add fw/common/gendac.h fw/common/gendac.c fw/app/ui.h fw/app/ui.c fw/app/main.c docs/superpowers/specs/2026-11-13-dso510-firmware-design.md
git commit -m "generator: DAC table driver, generator page, live quick-mode label, G marker; replaces the fixed test sine

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```

---

### Task 4: On-device verification (controller with the user)

**Files:**
- Modify: `NOTES.md`, `docs/superpowers/specs/2026-11-13-dso510-firmware-design.md` (status line)

- [ ] **Step 1: Boot.** Flash `APP_DSO-510_app.bin`. Probe on the generator output, then press Auto. Expect a 1 kHz sine of about 2.0 Vpp centred near 1.5 V, and a `G` in the top bar.
- [ ] **Step 2: Page.** Long-press Mode → Generator. Step through all nine waveforms and check each on the scope after closing the page. Use square with duty 30 % and 70 %. For DC, change the offset and watch the level move. Check Output Off (flat 0 V, `G` gone) and On again.
- [ ] **Step 3: Frequency and amplitude.** Check 10 Hz, 1 kHz, 10 kHz, 33.3 kHz (fine steps) and 50 kHz. The scope's frequency reading should agree within its own resolution, and the "actual" line should be plausible. Check amplitude 0.1, 1.0 and 3.0 Vpp against the scope's Vpp, expecting ±5 % plus the scope's ~6 % low reading.
- [ ] **Step 4: Quick mode.** Short-press Mode until the label shows `Gen …`. The wheel sweeps the frequency while the trace follows, and Up/Down changes the amplitude. A wheel press toggles `~` fine steps.
- [ ] **Step 5: Coexistence.** With the generator running, check each of these:
  - CSV save works.
  - The USB drive mounts.
  - `scope_live.py` Frames shows the generated wave.
  - Logger at 100 kS/s shows no drops.
- [ ] **Step 6: Record and commit.** Add the results to `NOTES.md`, mark sub-project 3 done in the spec status line, and commit both:
```bash
git add NOTES.md docs/superpowers/specs/2026-11-13-dso510-firmware-design.md
git commit -m "Notes: sub-project 3 verified on hardware; spec status

Claude-Session: https://claude.ai/code/session_01PdqjwHELxySe7PGVKMZmHp"
```
