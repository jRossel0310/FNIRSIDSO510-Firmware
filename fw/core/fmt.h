#pragma once
/* Format v with 3 significant digits and an SI prefix (n u m k M), then unit.
 * trim != 0 drops trailing zeros ("1.00us" -> "1us"). buf must hold >= 16 chars. */
char *fmt_si(char *buf, float v, const char *unit, int trim);
