#include "test.h"
int t_fail, t_run;

void test_fmt(void);
void test_units(void);
void test_trigger(void);

int main(void) {
    test_fmt();
    test_units();
    test_trigger();
    printf("%d checks, %d failed\n", t_run, t_fail);
    return t_fail != 0;
}
