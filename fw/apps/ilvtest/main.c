// ilvtest: experiment bench for the 24/48 MS/s interleaved capture (sub-project 1b), driven from the PC
// over the serial port. Lines (all numbers hex, space separated):
//   R addr            read a 32-bit register        -> OK "R addr val"
//   W addr val        write a 32-bit register (flash controller refused)
//   F range dc pwm    front end: range index, DC coupling, offset PWM
//   G wave f amp off duty   generator (wave index, Hz, 0.1 V units)
//   P c3dt pol        ADC clock phase override: TMR4 CH3 compare (ff = default) and polarity (1 = active low as stock, 0 = active high)
//   C 0 psc arr       single-stream capture (DMA1 ch7), 1024 samples -> LOG index 9
//   C 1 arr4 [c8 c5 c3 c1]  interleaved, stock sequence (TMR4 stopped during setup)
//   C 2 arr4 [c8 c5 c3 c1]  interleaved, ADC clock kept running (slaves frozen through TMR4's TRGO)
//                     -> four LOG packets, index 0..3 = TMR3, TMR5, TMR1, TMR8 streams, 1024 samples each
// Hold POWER = off.
#include "board.h"
#include "lcd.h"
#include "spiflash.h"
#include "flashmap.h"
#include "vol.h"
#include "msc.h"
#include "usbd.h"
#include "stream_proto.h"
#include "adc.h"
#include "frontend.h"
#include "testsig.h"
#include "gen.h"
#include "gendac.h"

#define TMR1_BASE 0x40012C00u
#define TMR3_BASE 0x40000400u
#define TMR4_BASE 0x40000800u
#define TMR5_BASE 0x40000C00u
#define TMR8_BASE 0x40013400u
#define T_CTRL1(t) REG32((t) + 0x00)
#define T_CTRL2(t) REG32((t) + 0x04)
#define T_STCTRL(t) REG32((t) + 0x08)
#define T_IDEN(t)  REG32((t) + 0x0C)
#define T_SWEVT(t) REG32((t) + 0x14)
#define T_CM2(t)   REG32((t) + 0x1C)
#define T_CCTRL(t) REG32((t) + 0x20)
#define T_CVAL(t)  REG32((t) + 0x24)
#define T_DIV(t)   REG32((t) + 0x28)
#define T_PR(t)    REG32((t) + 0x2C)
#define T_C3DT(t)  REG32((t) + 0x3C)
#define T_BRK(t)   REG32((t) + 0x44)

#define NSTREAM 1024u
/* stream q: timer, trigger select bits, DMA channel registers */
static const struct { uint32_t tmr, ts, dma; } S[4] = {
    {TMR3_BASE, 0x30, 0x40020030},   /* DMA1 ch3 */
    {TMR5_BASE, 0x20, 0x4002041C},   /* DMA2 ch2 */
    {TMR1_BASE, 0x30, 0x40020058},   /* DMA1 ch5 */
    {TMR8_BASE, 0x20, 0x40020408},   /* DMA2 ch1 */
};
#define D_CTRL(d)  REG32((d) + 0x00)
#define D_CNT(d)   REG32((d) + 0x04)
#define D_PADDR(d) REG32((d) + 0x08)
#define D_MADDR(d) REG32((d) + 0x0C)

static msc_t msc;
static const msc_disk_t disk = {vol_read, vol_write, USER_VOL_SECTORS};
void board_idle(void) { usbd_poll(); }

static uint8_t buf[4 * NSTREAM];
static uint16_t gtab[GEN_N_MAX];
static uint32_t caps, timeouts;
static uint32_t ph_c3dt = 0xFF, ph_pol = 1;   /* ADC clock phase override */

static void phase_apply(void) {
    if (ph_c3dt != 0xFF) T_C3DT(TMR4_BASE) = ph_c3dt;
    T_CCTRL(TMR4_BASE) = (T_CCTRL(TMR4_BASE) & ~0x200u) | (ph_pol ? 0x200u : 0u);
}

static void slaves_off(void) {
    for (int q = 0; q < 4; q++) { T_IDEN(S[q].tmr) &= ~0x100u; T_CTRL1(S[q].tmr) &= ~1u; D_CTRL(S[q].dma) = 0; }
    REG32(0x40020004) = 0x000F0F00u;      /* DMA1 clear ch3, ch5 flags */
    REG32(0x40020404) = 0x000000FFu;      /* DMA2 clear ch1, ch2 flags */
}

static void tmr4_clock(uint32_t arr4) {   /* ADC clock = 240 MHz / (arr4 + 1), 50 % PWM, TRGO configured by caller */
    T_CTRL1(TMR4_BASE) &= ~1u;
    T_IDEN(TMR4_BASE) &= ~0x100u;         /* no DMA request of its own */
    T_DIV(TMR4_BASE) = 0;
    T_PR(TMR4_BASE) = arr4;
    T_C3DT(TMR4_BASE) = (arr4 >> 1) + 1u;
    phase_apply();
    T_STCTRL(TMR4_BASE) |= 0x80u;         /* master/slave sync, as stock */
    T_CVAL(TMR4_BASE) = 0;
}

/* Returns 0, or -1 on timeout. cv = start counts of TMR8, TMR5, TMR3, TMR1 (stock 3, 1, 2, 0). */
static int ilv_capture(int keep_clock, uint32_t arr4, const uint32_t *cv) {
    static uint32_t cur_arr4 = 0xFFFFFFFFu;
    const uint32_t cvq[4] = {cv[2] + 4u, cv[1] + 4u, cv[3] + 4u, cv[0] + 4u};   /* +4: first period is 8 ticks */   /* per stream q: TMR3, TMR5, TMR1, TMR8 */
    adc_abort();
    CRM_AHBEN |= 3u;                                         /* DMA1, DMA2 */
    CRM_APB1EN |= (1u << 1) | (1u << 2) | (1u << 3);         /* TMR3, TMR4, TMR5 */
    CRM_APB2EN |= (1u << 11) | (1u << 13);                   /* TMR1, TMR8 */
    slaves_off();
    if (keep_clock) {
        T_CTRL2(TMR4_BASE) &= ~0x70u;                        /* TRGO idle: slaves cannot count */
        phase_apply();
        if (cur_arr4 != arr4) {
            tmr4_clock(arr4);
            T_SWEVT(TMR4_BASE) = 1;
            T_CTRL1(TMR4_BASE) |= 1u;
            cur_arr4 = arr4;
            delay_ms(2);                                     /* pipeline flush */
        }
    } else {
        tmr4_clock(arr4);
        T_SWEVT(TMR4_BASE) = 1;
        T_CVAL(TMR4_BASE) = 0;
        T_CTRL2(TMR4_BASE) = (T_CTRL2(TMR4_BASE) & ~0x70u) | 0x20u;   /* update -> TRGO */
        cur_arr4 = 0xFFFFFFFFu;
    }
    REG32(0x400204A4) |= 1u << 24;                           /* DMA2 flexible mapping */
    REG32(0x400204A0) = (REG32(0x400204A0) & 0xFFFF0000u) | 0x576Fu;   /* ch1 = 0x6F (TMR8 ovf), ch2 = 0x57 (TMR5 ovf) */
    REG32(0x400200A4) |= 1u << 24;                           /* DMA1 flexible mapping */
    REG32(0x400200A0) = (REG32(0x400200A0) & 0xFF00FFFFu) | 0x470000u; /* ch3 = 0x47 (TMR3 ovf) */
    REG32(0x400200A4) = (REG32(0x400200A4) & 0xFFFFFF00u) | 0x37u;     /* ch5 = 0x37 (TMR1 ovf) */
    for (int q = 0; q < 4; q++) {
        uint32_t t = S[q].tmr, d = S[q].dma;
        T_CTRL1(t) = 0x80u;                                  /* reload preload, as the driver */
        T_DIV(t) = 0; REG32(t + 0x30) = 0; T_PR(t) = 7;
        T_SWEVT(t) = 1;
        T_PR(t) = 3;
        T_STCTRL(t) = (T_STCTRL(t) & ~0x77u) | 5u | S[q].ts; /* gated by TMR4's TRGO */
        T_CVAL(t) = cvq[q];
        D_CTRL(d) = 0;
        D_PADDR(d) = 0x40010C08;                             /* GPIOB->IDT */
        D_MADDR(d) = (uint32_t)(buf + (uint32_t)q * NSTREAM);
        D_CNT(d) = NSTREAM;
        D_CTRL(d) = 0x2000u | 0x80u | 1u;                    /* high prio, 8-bit, mem increment, enable */
        T_IDEN(t) |= 0x100u;                                 /* overflow DMA request */
    }
    /* stock enable order: TMR8, TMR5, TMR3, TMR1, then TMR4 */
    T_CTRL1(TMR8_BASE) |= 1u; T_CTRL1(TMR5_BASE) |= 1u; T_CTRL1(TMR3_BASE) |= 1u; T_CTRL1(TMR1_BASE) |= 1u;
    if (keep_clock) T_CTRL2(TMR4_BASE) = (T_CTRL2(TMR4_BASE) & ~0x70u) | 0x20u;   /* go: all slaves see the same ticks */
    else T_CTRL1(TMR4_BASE) |= 1u;
    uint32_t t0 = millis();
    int rc = 0;
    for (;;) {
        if (!D_CNT(S[0].dma) && !D_CNT(S[1].dma) && !D_CNT(S[2].dma) && !D_CNT(S[3].dma)) break;
        if (millis() - t0 > 200) { rc = -1; break; }
    }
    if (keep_clock) T_CTRL2(TMR4_BASE) &= ~0x70u;
    slaves_off();
    return rc;
}

static uint32_t hexarg(const char **pp, uint32_t dflt) {
    const char *p = *pp;
    while (*p == ' ') p++;
    if (!*p) { *pp = p; return dflt; }
    uint32_t v = 0;
    for (; *p && *p != ' '; p++) {
        char c = *p;
        v = v * 16u + (uint32_t)(c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : 0);
    }
    *pp = p;
    return v;
}

static void send(uint8_t type, const uint8_t *a, uint32_t alen, const uint8_t *b, uint32_t blen) {
    uint32_t t0 = millis();
    while (usbd_cdc_open() && sp_put(usbd_cdc_txfifo(), type, a, alen, b, blen) != 0) {
        usbd_cdc_kick(); usbd_poll(); WDT_CMD = 0xAAAA;
        if (millis() - t0 > 2000) return;
    }
    usbd_cdc_kick();
}

static void reply(const char *tag, uint32_t a, uint32_t b) {
    char m[32], x[12];
    int k = 0;
    while (*tag) m[k++] = *tag++;
    m[k++] = ' ';
    for (const char *p = fmt_x(x, a, 8); *p; p++) m[k++] = *p;
    m[k++] = ' ';
    for (const char *p = fmt_x(x, b, 8); *p; p++) m[k++] = *p;
    send(SP_OK, (const uint8_t *)m, (uint32_t)k, 0, 0);
}

static void gen_set(const gen_cfg_t *g) {
    gen_plan_t p = gen_plan(g->freq_hz);
    gendac_hold();
    gen_table(g, &p, gtab);
    gendac_load(gtab, p.n, p.psc, p.arr);
}

static void command(const char *l) {
    char c = l[0];
    const char *p = l + 1;
    if (c == 'R') {
        uint32_t a = hexarg(&p, 0);
        reply("R", a, REG32(a & ~3u));
    } else if (c == 'W') {
        uint32_t a = hexarg(&p, 0) & ~3u, v = hexarg(&p, 0);
        if (a >= 0x40022000u && a < 0x40022400u) { send(SP_ERR, (const uint8_t *)"flash", 5, 0, 0); return; }
        REG32(a) = v;
        reply("W", a, REG32(a));
    } else if (c == 'F') {
        uint32_t r = hexarg(&p, 6), dc = hexarg(&p, 1), pwm = hexarg(&p, 1651);
        fe_set_range((int)r); fe_set_coupling_dc((int)dc); fe_set_offset_pwm((uint16_t)pwm);
        reply("F", r, pwm);
    } else if (c == 'G') {
        gen_cfg_t g;
        gen_defaults(&g);
        g.wave = (uint8_t)hexarg(&p, GEN_SQUARE); g.freq_hz = hexarg(&p, 20000);
        g.amp_dv = (uint8_t)hexarg(&p, 20); g.offset_dv = (uint8_t)hexarg(&p, 15); g.duty_pct = (uint8_t)hexarg(&p, 50);
        gen_clamp(&g); gen_set(&g);
        reply("G", g.wave, g.freq_hz);
    } else if (c == 'P') {
        ph_c3dt = hexarg(&p, 0xFF); ph_pol = hexarg(&p, 1);
        reply("P", ph_c3dt, ph_pol);
    } else if (c == 'C') {
        uint32_t mode = hexarg(&p, 2);
        uint8_t h[8];
        caps++;
        if (mode == 0) {
            uint32_t psc = hexarg(&p, 0), arr = hexarg(&p, 19);
            slaves_off();
            T_CTRL2(TMR4_BASE) = (T_CTRL2(TMR4_BASE) & ~0x70u) | 0x20u;
            adc_init();
            adc_set_rate((uint16_t)psc, (uint16_t)arr);
            phase_apply(); delay_ms(2);
            adc_start(buf, NSTREAM);
            uint32_t t0 = millis();
            while (!adc_done()) if (millis() - t0 > 500) { adc_abort(); timeouts++; send(SP_ERR, (const uint8_t *)"timeout", 7, 0, 0); return; }
            reply("C", 0, arr);
            sp_u32x2(h, 9, 0);
            send(SP_LOG, h, 4, buf, NSTREAM);
        } else {
            uint32_t arr4 = hexarg(&p, 4), cv[4];
            cv[0] = hexarg(&p, 0xFFu); cv[1] = hexarg(&p, 1); cv[2] = hexarg(&p, 2); cv[3] = hexarg(&p, 0);
            if (cv[0] == 0xFFu) { cv[0] = 0xFFFFFFFFu; cv[1] = 1; cv[2] = 2; cv[3] = 0; }   /* default: driver order, TMR8 last */
            if (ilv_capture(mode == 2, arr4, cv) < 0) {
                timeouts++;
                reply("T", D_CNT(S[0].dma) | D_CNT(S[1].dma) << 16, D_CNT(S[2].dma) | D_CNT(S[3].dma) << 16);
                return;
            }
            reply("C", mode, arr4);
            for (uint32_t q = 0; q < 4; q++) { sp_u32x2(h, q, 0); send(SP_LOG, h, 4, buf + q * NSTREAM, NSTREAM); }
        }
    } else send(SP_ERR, (const uint8_t *)"?", 1, 0, 0);
}

int main(void) {
    board_init(); keys_init(); lcd_init();
    lcd_fill(0, 0, LCD_W, LCD_H, C_BLACK); backlight(1);
    lcd_text(0, 0, " ilvtest: interleave bench (serial)", C_CYAN, C_BLACK);
    fe_init(); adc_init(); dac2_bias_init(); cal_out_init();
    fe_set_range(6); fe_set_coupling_dc(1); fe_set_offset_pwm(1651);
    adc_set_rate(0, 19);
    gendac_init();
    { gen_cfg_t g; gen_defaults(&g); g.wave = GEN_SQUARE; g.freq_hz = 20000; gen_set(&g); }
    spiflash_init();
    msc_init(&msc, &disk);
    usbd_init(&msc);
    sp_line_t line = {{0}, 0, 0};
    uint32_t next = 0;
    char b[12];
    for (;;) {
        usbd_poll();
        uint8_t in[32];
        uint32_t n = usbd_cdc_read(in, sizeof in);
        for (uint32_t i = 0; i < n; i++) if (sp_line_feed(&line, in[i])) command(line.buf);
        uint32_t now = millis();
        if ((int32_t)(now - next) >= 0) {
            next = now + 250;
            lcd_text(0, 2, usbd_cdc_open() ? " serial: OPEN  " : " serial: closed", usbd_cdc_open() ? C_GREEN : C_GRAY, C_BLACK);
            int c = lcd_text(0, 4, " captures ", C_GRAY, C_BLACK); lcd_text(c, 4, fmt_x(b, caps, 6), C_WHITE, C_BLACK);
            c = lcd_text(0, 5, " timeouts ", C_GRAY, C_BLACK); lcd_text(c, 5, fmt_x(b, timeouts, 6), C_WHITE, C_BLACK);
            lcd_text(0, 14, " Hold POWER = off", C_GRAY, C_BLACK);
        }
        if (keys_read() & KEY_BIT(KEY_POWER)) {
            uint32_t t0 = millis();
            while (keys_read() & KEY_BIT(KEY_POWER)) {
                usbd_poll(); WDT_CMD = 0xAAAA;
                if (millis() - t0 > 1000) { gendac_off(); backlight(0); usbd_detach(); power_off(); for (;;) WDT_CMD = 0xAAAA; }
            }
        }
        WDT_CMD = 0xAAAA;
    }
}
