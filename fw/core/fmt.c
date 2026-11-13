#include "fmt.h"
#include <stdint.h>

char *fmt_si(char *buf, float v, const char *unit, int trim) {
    static const char pfx[] = {'n', 'u', 'm', 0, 'k', 'M'};
    char *p = buf;
    if (v < 0) { *p++ = '-'; v = -v; }
    int e = 3;
    if (v != 0.0f) {
        while (v >= 999.5f && e < 5) { v /= 1000.0f; e++; }
        while (v < 0.9995f && e > 0) { v *= 1000.0f; e--; }
    }
    int dec = v >= 99.95f ? 0 : v >= 9.995f ? 1 : 2;
    uint32_t mul = dec == 0 ? 1 : dec == 1 ? 10 : 100;
    uint32_t scaled = (uint32_t)(v * (float)mul + 0.5f);
    uint32_t ip = scaled / mul, fp = scaled % mul;
    char t[12]; int n = 0;
    do { t[n++] = (char)('0' + ip % 10); ip /= 10; } while (ip);
    while (n) *p++ = t[--n];
    if (dec) {
        *p++ = '.';
        if (dec == 2) { *p++ = (char)('0' + fp / 10); *p++ = (char)('0' + fp % 10); }
        else *p++ = (char)('0' + fp);
        if (trim) {
            while (p[-1] == '0') p--;
            if (p[-1] == '.') p--;
        }
    }
    if (pfx[e]) *p++ = pfx[e];
    while (*unit) *p++ = *unit++;
    *p = 0;
    return buf;
}
