/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ScreenElement.h"
#include "math/Units.h"

#include <cstdint>


// a speed to fly indicator bars
class S2FBar : public ScreenElement
{
public:
    static constexpr const mps_t LEVEL_DELTA = Units::kmh_to_mps(10.0f); // 10 km/h per level

    S2FBar(int16_t cx, int16_t cy, int16_t width, int16_t gap);

    // API
    void setRef(int16_t x, int16_t y) { _ref_x=x; _ref_y=y; }
    void setWidth(int16_t width) { _width_h=width/2; stepFromWidth(width); }
    void setGap(int16_t gap) { _gap_v=gap/2; }
    void draw(mps_t s2fd, mps_t s2f_speed = -1.0f);

private:
    void stepFromWidth(int16_t width) { _step = (width+4)/8; }
    void drawSpeed(mps_t v);
    void drawArrow(int16_t x, int16_t y, int16_t level, bool del);

private: // attributes
    int16_t _prev_s2f_level = 0;
    int16_t _prev_s2f_speed = 0;
    int16_t _width_h;
    int16_t _gap_v;
    int16_t _step;
};