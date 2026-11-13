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
