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
void ui_draw_gen(const scope_state_t *s, uint32_t actual_mhz);   /* generator page (over the plot) */
void ui_draw_gen_marker(int on);           /* top bar col 29: "G" while the generator output is on */
void ui_draw_usb(int on);                  /* top bar cols 31..33: "USB" while frames stream */
/* Logger status screen (replaces the plot while logging). */
void ui_draw_logger(uint32_t rate_hz, uint32_t sent, uint32_t dropped, float vmin, float vmax, float vmean,
                    float vdiv_v);
