/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Battery.h"

#include "Colors.h"
#include "AdaptUGC.h"
#include "setup/SetupNG.h"

#include "logdefnone.h"

#include <algorithm>


extern AdaptUGC *MYUCG;

Battery::Battery(int16_t cx, int16_t cy, bool sbs) :
    ScreenElement(cx, cy),
    _side_text(sbs)
{
    setThresholds();
}

void Battery::setThresholds()
{
    _yellow =  (int)(( bat_yellow_volt.get() - bat_low_volt.get() )*100)/( bat_full_volt.get() - bat_low_volt.get() );
    _red = (int)(( bat_red_volt.get() - bat_low_volt.get() )*100)/( bat_full_volt.get() - bat_low_volt.get() );
}

void Battery::blank()
{
    if( battery_display.get() != BAT_VOLTAGE_BIG ){
        MYUCG->setColor( COLOR_BLACK );
        MYUCG->drawBox( _ref.x-40, _ref.y-2, 40, 12  );
    } else {
        MYUCG->setColor( COLOR_BLACK );
        MYUCG->drawBox( _ref.x-55, _ref.y-12, 65, 22  );
    }
    _dirty = true;
}


void Battery::draw(float volt)
{
    if (volt < bat_red_volt.get() && !_dirty) {
        blank();
        return;
    }
    int chargev = (int)(volt * 10);
    if (_charge == chargev && !_dirty) {
        return;
    }
    _charge = chargev;
    _dirty = false;
    float chargef = chargev / 10.0f;

    int chargep = (int)((chargef - bat_low_volt.get())*100)/( bat_full_volt.get() - bat_low_volt.get());
    ESP_LOGI(FNAME,"Charge: %d , Volt: %.1f V, Delta %.1f V", chargep, chargef, (bat_full_volt.get() - bat_low_volt.get()) );

    chargep = std::clamp(chargep, 0, 100);
    if ( battery_display.get() != BAT_VOLTAGE_BIG ){
        MYUCG->setColor( COLOR_HEADER );
        MYUCG->drawBox( _ref.x-40,_ref.y-2, 36, 12  );  // Bat body square
        MYUCG->drawBox( _ref.x-4, _ref.y+1, 3, 6  );      // Bat pluspole pimple
        float v_red    = bat_red_volt.get();
        float v_yellow = bat_yellow_volt.get();
        if (v_yellow < v_red)
            v_yellow = v_red;
        if (volt >= v_yellow) {
            MYUCG->setColor( COLOR_GREEN );  // green = moderate to full 
        } else if (volt >= v_red) {
            MYUCG->setColor( COLOR_YELLOW ); // yellow = critical to moderate
        } else {
            MYUCG->setColor( COLOR_RED );    // red = empty to critical
        }
        int chgpos=(chargep*32)/100;
        if(chgpos <= 4) {
            chgpos = 4;
        }
        MYUCG->drawBox( _ref.x-40+2, _ref.y, chgpos, 8  );  // Bat charge state
        MYUCG->setColor( DARK_GREY );
        MYUCG->drawBox( _ref.x-40+2+chgpos, _ref.y, 32-chgpos, 8 );  // Empty bat bar
        MYUCG->setFont(ucg_font_fub11_hr, true);
    }
    MYUCG->setColor(COLOR_WGREY);
    bool right_align = _side_text && battery_display.get() != BAT_VOLTAGE_BIG;
    int16_t tpos_x = right_align ? _ref.x - 58 : _ref.x - 40; // side by side and right alignment
    int16_t tpos_y = (_side_text || battery_display.get() == BAT_VOLTAGE_BIG) ? _ref.y + 12 : _ref.y - 6; // sbs or on top of symbol
    char *s, buf[32];
    s = buf;
    if( battery_display.get() == BAT_PERCENTAGE ) {
        sprintf(s, " %3d", chargep);
        if ( right_align ) {
            tpos_x -= MYUCG->getStrWidth(s);
        } else {
            s++; // skip leading space for left alignment
        }
        MYUCG->setPrintPos(tpos_x, tpos_y);
    }
    else {
        if ( battery_display.get() == BAT_VOLTAGE_BIG ) {
            MYUCG->setFont(ucg_font_fub14_hr, true);
        }
        sprintf(s, " %2.1f", chargef);
        if ( ! right_align ) {
            s++; // skip leading space for left alignment
        }
        MYUCG->setPrintPos(tpos_x - (right_align ? MYUCG->getStrWidth(s) : 0), tpos_y);
    }
    MYUCG->print(s);
    MYUCG->setColor( COLOR_HEADER );
    if( battery_display.get() == BAT_PERCENTAGE ) {
        MYUCG->print("%");
    }
    else {
        MYUCG->print("V");
    }
    if ( ! right_align ) {
        MYUCG->print(" ");
    }
}
