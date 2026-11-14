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

    /* square wave keeps every pulse: m rising edges (circular) in the table for any duty */
    {
        static const int duties[] = {1, 2, 30, 50, 70, 98, 99};
        gen_cfg_t sg;
        gen_defaults(&sg); sg.wave = GEN_SQUARE;
        int bad = 0;
        for (uint32_t f = 1; f <= 50000 && !bad; f++) {
            uint32_t dec = 1;
            while (dec * 10u <= f) dec *= 10u;
            if (dec >= 100u && f % (dec / 100u)) continue;
            gen_plan_t sp = gen_plan(f);
            for (unsigned di = 0; di < sizeof duties / sizeof duties[0]; di++) {
                sg.duty_pct = (uint8_t)duties[di];
                gen_table(&sg, &sp, tab);
                int hi_c = tab[0], edges = 0;
                for (int i = 0; i < sp.n; i++) if (tab[i] > hi_c) hi_c = tab[i];
                for (int i = 0; i < sp.n; i++) {
                    int prev = tab[i == 0 ? sp.n - 1 : i - 1];
                    if (prev != hi_c && tab[i] == hi_c) edges++;
                }
                if (edges != sp.m) { CHECK_EQ(edges, sp.m); bad = 1; break; }
            }
        }
    }

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
    mn = 4095; mx = 0;
    for (int i = 0; i < 100; i++) { if (tab[i] < mn) mn = tab[i]; if (tab[i] > mx) mx = tab[i]; }
    CHECK(near(mn, gen_code_mv(0), 2)); CHECK(near(mx, gen_code_mv(3000), 2));

    /* several periods, n not divisible by m: exactly m mid crossings, extremes close to lo/hi */
    gen_defaults(&g);
    gen_plan_t q3 = {101, 5, 0, 0, 0};
    gen_table(&g, &q3, tab);
    {
        int cross = 0;
        mn = 4095; mx = 0;
        for (int i = 0; i < 101; i++) {
            int prev = tab[i == 0 ? 100 : i - 1];
            if (prev < mid && tab[i] >= mid) cross++;
            if (tab[i] < mn) mn = tab[i];
            if (tab[i] > mx) mx = tab[i];
        }
        CHECK_EQ(cross, 5);
        CHECK(near(mn, lo, 12)); CHECK(near(mx, hi, 12));
    }

    CHECK_STR(gen_wave_name(GEN_SINE), "Sine"); CHECK_STR(gen_wave_name(GEN_EXP_FALL), "Exp fall");
    CHECK_STR(gen_wave_name(99), "?");
}
