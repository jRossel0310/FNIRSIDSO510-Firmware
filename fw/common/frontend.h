// Analog front end: range pins (PC4, PE6, PE5, PE4), coupling (PC5), offset PWM (PA7 = TMR14 CH1).
#pragma once
#include <stdint.h>

#define FE_NUM_RANGES 10
#define FE_PWM_MAX    0xFFE
#define FE_PC5_FOR_DC 1      // PC5 level that selects DC coupling; verified on hardware in Task 11

void fe_init(void);
void fe_set_range(int r);           // r = index into vdiv_mv (stock pin table FUN_08008834)
void fe_set_coupling_dc(int dc);
void fe_set_offset_pwm(uint16_t pwm);
