#pragma once
#include <math.h>
#include <stdio.h>
#include <string.h>

extern int t_fail, t_run;
#define CHECK(c) do { t_run++; if (!(c)) { t_fail++; \
    printf("%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c); } } while (0)
#define CHECK_EQ(a, b) do { long long _a = (long long)(a), _b = (long long)(b); t_run++; \
    if (_a != _b) { t_fail++; printf("%s:%d: %s == %lld, expected %lld\n", \
    __FILE__, __LINE__, #a, _a, _b); } } while (0)
#define CHECK_NEAR(a, b, eps) do { double _a = (a), _b = (b); t_run++; \
    if (fabs(_a - _b) > (eps)) { t_fail++; printf("%s:%d: %s == %g, expected %g\n", \
    __FILE__, __LINE__, #a, _a, _b); } } while (0)
#define CHECK_STR(a, b) do { const char *_a = (a), *_b = (b); t_run++; \
    if (strcmp(_a, _b)) { t_fail++; printf("%s:%d: %s == \"%s\", expected \"%s\"\n", \
    __FILE__, __LINE__, #a, _a, _b); } } while (0)
