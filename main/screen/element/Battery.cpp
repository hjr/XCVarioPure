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

Battery::Battery(int16_t cx, int16_t cy) :
    ScreenElement(cx, cy)
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
    MYUCG->setColor( COLOR_BLACK );
    MYUCG->drawBox( _ref.x-36, _ref.y-19, 55, 18);
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

    // MYUCG->setColor( COLOR_HEADER );
    // MYUCG->drawFrame(_ref.x-36,_ref.y-20, 56, 19);
    MYUCG->startBuffering(_ref.x-36,_ref.y-19, 55, 18);
    chargep = std::clamp(chargep, 0, 100);
    char buf[32];
    char unit;
    if ( battery_display.get() == BAT_PERCENTAGE ){
        MYUCG->setColor( COLOR_HEADER );
        MYUCG->drawBox( _ref.x-29,_ref.y-19, 43, 18  ); // Bat body square
        MYUCG->drawBox( _ref.x+14, _ref.y-14, 3, 6  ); // Bat pluspole pimple
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
        int chgpos=(chargep*39)/100;
        if(chgpos <= 4) {
            chgpos = 4;
        }
        MYUCG->drawBox( _ref.x-29+2, _ref.y-17, chgpos, 14  );  // Bat charge state
        MYUCG->setColor( DARK_GREY );
        MYUCG->drawBox( _ref.x-29+2+chgpos, _ref.y-17, 39-chgpos, 14 );  // Empty bat bar
        MYUCG->setFont(ucg_font_fub11_hr);
        MYUCG->setColor(COLOR_WHITE);
        sprintf(buf, "%3d", chargep);
        unit = '%';
    }
    else {
        MYUCG->setColor(COLOR_WGREY);
        MYUCG->setFont(ucg_font_fub14_hr);
        sprintf(buf, "%2.1f", chargef);
        unit = 'V';
    }

    // draw value and unit
    MYUCG->setPrintPos(_ref.x - MYUCG->getStrWidth(buf), _ref.y);
    MYUCG->print(buf);
    MYUCG->setFont(ucg_font_fub11_hr);
    MYUCG->setColor( COLOR_HEADER );
    MYUCG->print(unit);
    MYUCG->finishBuffering();
}
