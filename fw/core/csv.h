#pragma once
#include <stdint.h>

#define FW_VERSION "0.2.0"
#define CSV_ROW_MAX 40          /* longest row incl. '\n' */

typedef struct {
    uint32_t rate_hz;           /* sample rate of the record */
    uint16_t vdiv_mv;           /* V/div table value (probe factor NOT applied) */
    uint8_t coupling_dc;        /* 1 DC, 0 AC */
    uint8_t probe_x10;          /* 1 = x10 probe */
    uint8_t trig_mode;          /* 0 Auto, 1 Normal, 2 Single */
    uint8_t edge;               /* 0 rising, 1 falling */
    int16_t zero_code;          /* ADC code of 0 V (128 + offset) */
    int16_t trig_level_code;    /* trigger level as ADC code */
    int32_t trig_index;         /* sample index of the trigger point */
    uint16_t offset_pwm;        /* offset PWM used for this record */
} csv_meta_t;

/* All header lines incl. the column-name line. Returns the length written (no NUL),
   or -1 if cap is too small. */
int csv_header(char *buf, int cap, const csv_meta_t *m);
/* Row for sample i with ADC code `code`, '\n'-terminated, no NUL. buf must hold CSV_ROW_MAX.
   Returns the length. */
int csv_row(char *buf, const csv_meta_t *m, int32_t i, uint8_t code);
