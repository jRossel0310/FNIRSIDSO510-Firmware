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
    CHECK_EQ(s.menu_item, 4);                              /* wraps */
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

void test_control_gen(void) {
    scope_state_t s;
    state_defaults(&s);
    CHECK_EQ(s.gen.on, 1); CHECK_EQ(s.gen.freq_hz, 1000); CHECK_EQ(s.gen_open, 0);

    /* Mode cycle now has four control modes */
    control_handle(&s, K(KEV_SHORT, KEY_MODE)); control_handle(&s, K(KEV_SHORT, KEY_MODE));
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_BARS);
    CHECK_EQ(s.ctl, CTL_GEN);
    /* quick mode: wheel = frequency, Up/Down = amplitude, wheel press = fine */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_GEN | CHG_BARS);
    CHECK_EQ(s.gen.freq_hz, 2000);
    CHECK_EQ(control_handle(&s, K(KEV_REPEAT, KEY_WHEEL_L)), CHG_GEN | CHG_BARS);
    CHECK_EQ(s.gen.freq_hz, 1000);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_PRESS)), CHG_BARS);
    CHECK_EQ(s.gen.fine, 1);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.freq_hz, 1010);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), CHG_GEN | CHG_BARS);
    CHECK_EQ(s.gen.amp_dv, 21);
    s.gen.amp_dv = 30; gen_clamp(&s.gen);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_UP)), 0);            /* at the limit: nothing changes */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_DOWN)), CHG_GEN | CHG_BARS);
    CHECK_EQ(s.gen.amp_dv, 29);
    CHECK_EQ(s.tdiv, TDIV_1MS); CHECK_EQ(s.trig_pos, 150);            /* scope settings untouched */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_BARS);
    CHECK_EQ(s.ctl, CTL_TIME_VOLT);

    /* menu: fifth item opens the generator page */
    state_defaults(&s);
    control_handle(&s, K(KEV_LONG, KEY_MODE));
    CHECK_EQ(s.menu_open, 1);
    for (int i = 0; i < 4; i++) control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.menu_item, 4);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.menu_item, 0);                                         /* wraps over 5 items */
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    CHECK_EQ(s.menu_item, 4);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 0); CHECK_EQ(s.gen_open, 1); CHECK_EQ(s.gen_item, 0);

    /* page: rows and values */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_GEN | CHG_GENPAGE | CHG_BARS);
    CHECK_EQ(s.gen.on, 0);                                            /* row 0: output toggle */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L));
    CHECK_EQ(s.gen.on, 1);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_DOWN)), CHG_GENPAGE);
    CHECK_EQ(s.gen_item, 1);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.wave, GEN_SQUARE);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L)); control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L));
    CHECK_EQ(s.gen.wave, GEN_EXP_FALL);                               /* wraps */
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));                       /* row 2: frequency */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.freq_hz, 2000);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_PRESS)), CHG_GENPAGE | CHG_BARS);
    CHECK_EQ(s.gen.fine, 1);
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.freq_hz, 2010);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));                       /* row 3: amplitude */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L));
    CHECK_EQ(s.gen.amp_dv, 19);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));                       /* row 4: offset */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.offset_dv, 16);
    for (int i = 0; i < 40; i++) control_handle(&s, K(KEV_REPEAT, KEY_WHEEL_R));
    CHECK_EQ(s.gen.offset_dv, 21);                                    /* clamped: 1.9 Vpp fits up to 2.1 V centre */
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));                       /* row 5: duty */
    control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L));
    CHECK_EQ(s.gen.duty_pct, 49);
    control_handle(&s, K(KEV_SHORT, KEY_DOWN));
    CHECK_EQ(s.gen_item, 0);                                          /* wraps over 6 rows */
    control_handle(&s, K(KEV_SHORT, KEY_UP));
    CHECK_EQ(s.gen_item, 5);
    /* page swallows scope keys */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_RUN)), 0);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_RUN)), 0);
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_AUTO)), 0);
    CHECK_EQ(s.run, 1);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_POWER)), ACT_POWEROFF);   /* power still works */
    /* Mode closes the page (short or long), and a long press afterwards opens the menu again */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.gen_open, 0); CHECK_EQ(s.menu_open, 0);
    s.gen_open = 1;
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.gen_open, 0); CHECK_EQ(s.menu_open, 0);
    CHECK_EQ(control_handle(&s, K(KEV_LONG, KEY_MODE)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 1);
    /* wheel press on a normal menu item still closes the menu */
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_PRESS)), CHG_SCREEN);
    CHECK_EQ(s.menu_open, 0); CHECK_EQ(s.gen_open, 0);

    /* no-op wheel edits report no change */
    state_defaults(&s); s.gen_open = 1; s.gen_item = 5; s.gen.duty_pct = 1;
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_L)), 0);
    CHECK_EQ(s.gen.duty_pct, 1);
    state_defaults(&s); s.gen_open = 1; s.gen_item = 2; s.gen.freq_hz = 50000;
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), 0);
    CHECK_EQ(s.gen.freq_hz, 50000);
    state_defaults(&s); s.gen_open = 1; s.gen_item = 5;
    CHECK_EQ(control_handle(&s, K(KEV_SHORT, KEY_WHEEL_R)), CHG_GEN | CHG_GENPAGE | CHG_BARS);
    CHECK_EQ(s.gen.duty_pct, 51);
}
