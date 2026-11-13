#include "test.h"
#include "control.h"

static kev_t K(int type, int key) { kev_t e; e.type = (uint8_t)type; e.key = (uint8_t)key; return e; }

void test_control(void) {
    scope_state_t s;
    state_defaults(&s);
    CHECK_EQ(s.tdiv, TDIV_1MS); CHECK_EQ(s.vdiv, 6); CHECK_EQ(s.trig_pos, 150);
    CHECK_EQ(s.trig_mode, TRIG_AUTO); CHECK_EQ(s.run, 1); CHECK_EQ(s.ctl, CTL_TIME_VOLT);

    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_TIME | CHG_BARS);
    CHECK_EQ(s.tdiv, TDIV_1MS + 1);
    s.trig_level = 153;                                    /* +1 div at 1 V/div = +1 V */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), CHG_FE | CHG_BARS | CHG_TRIG);
    CHECK_EQ(s.vdiv, 5);                                   /* up = more sensitive */
    CHECK_EQ(s.trig_level, 178);                           /* still +1 V = 2 div at 500 mV */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), CHG_FE | CHG_BARS | CHG_TRIG);
    CHECK_EQ(s.trig_level, 253);                           /* +1 V = 5 div at 200 mV */
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    CHECK_EQ(s.trig_level, 255);                           /* clamped to ADC range */
    s.trig_level = 128;
    s.vdiv = 0;
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), 0); /* clamped */

    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_BARS);
    CHECK_EQ(s.ctl, CTL_POS_OFFSET);
    CHECK_EQ(control_handle(&s, K(KEV_REPEAT, KEY_UP)), CHG_FE | CHG_TRIG);
    CHECK_EQ(s.offset, 5);
    CHECK_EQ(s.trig_level, 133);                           /* level follows the trace */
    s.offset = 98;
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    CHECK_EQ(s.offset, 100); CHECK_EQ(s.trig_level, 135);  /* moves by the clamped amount */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), 0); /* at limit: nothing moves */
    CHECK_EQ(s.trig_level, 135);
    s.offset = 5; s.trig_level = 133;
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L)), CHG_TRIG);
    CHECK_EQ(s.trig_pos, 140);
    control_handle(&s, K(KEV_SHORT, KEY_MODE));
    CHECK_EQ(s.ctl, CTL_POS_LEVEL);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.trig_level, 131);
    control_handle(&s, K(KEV_SHORT, KEY_MODE));
    CHECK_EQ(s.ctl, CTL_TIME_VOLT);

    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 1);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_MENU | CHG_FE | CHG_BARS);
    CHECK_EQ(s.coupling_dc, 0);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.menu_item, 2);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.trig_mode, TRIG_NORMAL);
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    CHECK_EQ(s.menu_item, 3);                              /* wraps */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 0);

    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_RUN)), CHG_RUN | CHG_BARS);
    CHECK_EQ(s.run, 0);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_RUN)), ACT_SAVE);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_AUTO)), ACT_AUTOSET);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_AUTO)), ACT_AUTOZERO);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_POWER)), 0);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_POWER)), ACT_POWEROFF);
}
