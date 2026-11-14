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
