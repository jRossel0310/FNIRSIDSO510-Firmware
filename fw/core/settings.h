#pragma once
#include <stdint.h>
#include "control.h"
#include "cal.h"

/* Persistent settings file (DSS1, version 2): 158 payload bytes + CRC-16 (low byte first).
   Version 1 (156 bytes, no measurement slots / auto-off) is still read. */
#define SETTINGS_LEN    160u
#define SETTINGS_LEN_V1 156u

/* Serialise the persistent settings into out[SETTINGS_LEN]. zeroed != 0: cal's lo/hi tables are stored as the auto-zero result. */
void settings_pack(uint8_t *out, const scope_state_t *s, const cal_t *cal, int zeroed);
/* Parse a settings file. Returns 1 and updates *s (persistent fields only), and, if the file carries a
   valid auto-zero result, cal->lo/hi and *zeroed = 1. Returns 0 and changes nothing if the file is not
   acceptable (length, magic, version, CRC, or a timebase / V/div value that is not in the tables). */
int settings_unpack(const uint8_t *in, uint32_t len, scope_state_t *s, cal_t *cal, int *zeroed);
