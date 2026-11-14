#include "test.h"
int t_fail, t_run;

void test_fmt(void);
void test_units(void);
void test_trigger(void);
void test_render(void);
void test_keys(void);
void test_measure(void);
void test_cal(void);
void test_control(void);
void test_autoset(void);
void test_flashmap(void);
void test_capname(void);
void test_csv(void);

int main(void) {
    test_fmt();
    test_units();
    test_trigger();
    test_render();
    test_keys();
    test_measure();
    test_cal();
    test_control();
    test_autoset();
    test_flashmap();
    test_capname();
    test_csv();
    printf("%d checks, %d failed\n", t_run, t_fail);
    return t_fail != 0;
}
