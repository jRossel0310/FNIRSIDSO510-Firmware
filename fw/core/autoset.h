#pragma once
/* New V/div index so the signal spans <= 6 divisions (one step less sensitive if clipped). */
int autoset_vdiv(int cur, int vmin, int vmax);
/* Fastest timebase index showing >= 2 periods on the 10-division screen; -1 if freq <= 0. */
int autoset_tdiv(float freq_hz);
