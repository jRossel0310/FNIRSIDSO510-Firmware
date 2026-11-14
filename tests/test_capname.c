#include "test.h"
#include "capname.h"

void test_capname(void) {
    char b[13];
    CHECK_EQ(capname_number("CAP0001.CSV"), 1);
    CHECK_EQ(capname_number("CAP0042.CSV"), 42);
    CHECK_EQ(capname_number("CAP9999.CSV"), 9999);
    CHECK_EQ(capname_number("cap0042.csv"), 42);              /* case-insensitive */
    CHECK_EQ(capname_number("CAP0042.TXT"), -1);
    CHECK_EQ(capname_number("CAP042.CSV"), -1);
    CHECK_EQ(capname_number("CAPX042.CSV"), -1);
    CHECK_EQ(capname_number("LOG0042.CSV"), -1);
    CHECK_EQ(capname_number("CAP00420.CSV"), -1);
    CHECK_EQ(capname_number(""), -1);
    capname_make(b, 7);    CHECK_STR(b, "CAP0007.CSV");
    capname_make(b, 1234); CHECK_STR(b, "CAP1234.CSV");
    capname_make(b, 9999); CHECK_STR(b, "CAP9999.CSV");
    CHECK_EQ(capname_next(0), 1);
    CHECK_EQ(capname_next(41), 42);
    CHECK_EQ(capname_next(9998), 9999);
    CHECK_EQ(capname_next(9999), -1);                         /* exhausted */
}
