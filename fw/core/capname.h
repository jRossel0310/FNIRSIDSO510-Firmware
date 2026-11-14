#pragma once
/* Capture files: CAP_DIR/CAP0001.CSV .. CAP9999.CSV (8.3 names, FatFs without LFN). */
#define CAP_DIR "/CSV"
#define CAP_MAX 9999

/* "CAP0042.CSV" (any case) -> 42; anything else -> -1. */
int capname_number(const char *name);
/* 42 -> "CAP0042.CSV" (out holds 12 chars + NUL). n must be 0..9999. */
void capname_make(char out[13], int n);
/* Number to use after the highest existing one (0 if none); -1 when 9999 is taken. */
int capname_next(int highest);
