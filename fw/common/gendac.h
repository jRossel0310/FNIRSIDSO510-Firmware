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
