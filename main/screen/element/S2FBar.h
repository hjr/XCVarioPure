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
    constexpr void setWidth(int16_t width) { _width_half=width/2; _step = stepFromWidth(width); }
    constexpr void setGap(int16_t gap) { _gap_half=gap/2; }
    using ScreenElement::draw;
    void draw(mps_t s2fd, bool cruise);

private:
    constexpr int16_t stepFromWidth(int16_t width) { return (width+4)/8; }
    // void drawSpeed(mps_t v);
    void drawBlock(int16_t level);
    void drawCircle(int16_t score);
    void drawArrow(int16_t x, int16_t y, int16_t level, bool del);

private: // attributes
    union Hash {
        struct {
            int8_t _cruise_mode = 1;
            int8_t _s2f_level = 0;
            int8_t _score = 0;
        };
        int32_t _raw;
    } _prev_hash;
    int16_t _width_half;
    int16_t _gap_half;
    int16_t _step;
};