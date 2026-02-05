/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Temperature.h"

#include "sensor/imu/ImuSensor.h"
#include "Colors.h"
#include "AdaptUGC.h"

#include <cstdio>

extern AdaptUGC *MYUCG;


void Temperature::draw(kelvin_t t, temp_status_t mputemp)
{
    if ( std::abs(t - _temp) > 0.1 || _dirty ) {
        _temp = t;

        MYUCG->setFont(ucg_font_fub20_hn, false);
        char s[32];
        if (t > -1000.) {
            float tu = std::roundf(TempUnit->apply(t) * 10.f) / 10.f;
            if ( tu < 100.f ) {
                std::sprintf(s, "  %.1f ", tu);
            } else {
                std::sprintf(s, "  %.0f ", tu);
            }
            if (t < -0.05f) {
                MYUCG->setColor(COLOR_BBLUE);
            } else {
                MYUCG->setColor(COLOR_WHITE);
            }
        } else {
            strcpy(s, "  ---");
            MYUCG->setColor(COLOR_WHITE);
        }
        // ESP_LOGI(FNAME,"drawTemperature: %d,%d %s", _ref_x, _ref_y, s);
        int16_t fl = MYUCG->getStrWidth(s);
        MYUCG->setPrintPos(_ref_x - fl, _ref_y);
        MYUCG->print(s);
        // the figure covers the little ° symbol, so alway repaint
    // }
    // if ( mputemp != _imut || _dirty ) {
        _imut = mputemp;

        // Color of unit shows if MPU silicon temperature is locked, too high or too low
        switch (mputemp) {
            case temp_status_t::MPU_T_LOCKED:
                MYUCG->setColor(COLOR_HEADER);
                break;
            case temp_status_t::MPU_T_LOW:
                MYUCG->setColor(COLOR_LBLUE);
                break;
            case temp_status_t::MPU_T_HIGH:
                MYUCG->setColor(COLOR_RED);
                break;
            default:
                MYUCG->setColor(COLOR_HEADER);
        }
        MYUCG->setFont(ucg_font_fub11_hr, false);
        MYUCG->setPrintPos(_ref_x - 4, _ref_y - 9);
        MYUCG->print(TempUnit->getName());
        _dirty = false;
    }
}
