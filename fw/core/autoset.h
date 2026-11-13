#pragma once
/* Plot shows ADC codes SCREEN_CODE_LO..SCREEN_CODE_HI (8 divisions). */
#define SCREEN_CODE_LO 28
#define SCREEN_CODE_HI 227

/* One auto-set step. From a capture's min/max ADC codes taken at (*vdiv, *offset), choose the
   V/div and offset that make the signal span <= 6 divisions and sit centred on screen (the
   most sensitive V/div that fits once the +-100-code offset limit is applied). If the ADC
   clipped, shift the offset away from the clipped rail (or go one V/div less sensitive).
   Updates *vdiv and *offset; returns 1 if another capture is needed (V/div changed or offset
   moved by more than 2 codes), 0 when converged or nothing more can be done. */
int autoset_fit(int *vdiv, int *offset, int vmin, int vmax);

/* Fastest timebase index showing >= 2 periods on the 10-division screen; -1 if freq <= 0. */
int autoset_tdiv(float freq_hz);
