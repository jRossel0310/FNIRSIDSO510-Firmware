#pragma once
#include <stdint.h>
#include "keys.h"
#include "units.h"
#include "gen.h"

#define PLOT_COLS  300
#define MENU_ITEMS 5
#define GEN_ROWS   6

enum { TRIG_AUTO = 0, TRIG_NORMAL, TRIG_SINGLE };
enum { CTL_TIME_VOLT = 0, CTL_POS_OFFSET, CTL_POS_LEVEL, CTL_GEN, CTL_NUM };

#define CHG_TIME     0x001u   /* timebase changed: new sample rate */
#define CHG_FE       0x002u   /* range / coupling / offset changed */
#define CHG_TRIG     0x004u   /* trigger level / position / mode / edge or 0 V marker moved */
#define CHG_BARS     0x008u   /* top/bottom status bars need redraw */
#define CHG_SCREEN   0x010u   /* full redraw */
#define CHG_RUN      0x020u   /* run/stop toggled */
#define CHG_MENU     0x040u   /* menu contents changed */
#define CHG_GEN      0x1000u  /* generator settings changed: re-apply to the hardware */
#define CHG_GENPAGE  0x2000u  /* generator page needs a redraw */
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
    gen_cfg_t gen;
    uint8_t gen_open, gen_item;   /* generator page open / selected row (0 Output, 1 Waveform, 2 Frequency, 3 Amplitude, 4 Offset, 5 Duty) */
} scope_state_t;

void state_defaults(scope_state_t *s);
uint32_t control_handle(scope_state_t *s, kev_t ev);
