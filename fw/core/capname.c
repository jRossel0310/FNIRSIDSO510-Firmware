#include "capname.h"

static char up(char c) { return (c >= 'a' && c <= 'z') ? (char)(c - 32) : c; }

int capname_number(const char *name) {
    static const char pre[] = "CAP", suf[] = ".CSV";
    for (int i = 0; i < 3; i++) if (up(name[i]) != pre[i]) return -1;
    int n = 0;
    for (int i = 3; i < 7; i++) {
        if (name[i] < '0' || name[i] > '9') return -1;
        n = n * 10 + (name[i] - '0');
    }
    for (int i = 0; i < 4; i++) if (up(name[7 + i]) != suf[i]) return -1;
    if (name[11] != 0) return -1;
    return n;
}

void capname_make(char out[13], int n) {
    static const char tmpl[] = "CAP0000.CSV";
    for (int i = 0; i < 12; i++) out[i] = tmpl[i];
    for (int i = 6; i >= 3; i--) { out[i] = (char)('0' + n % 10); n /= 10; }
}

int capname_next(int highest) { return highest >= CAP_MAX ? -1 : highest + 1; }
