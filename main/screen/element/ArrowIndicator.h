/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "AdaptUGC.h"

#include <cstdint>

class PolarGauge;


////////////////////////////
// ArrowIndicator

// A pointer linked to a polar gauge

struct ArrowPoints
{
    int16_t x_0 = 0, y_0 = 0, x_1 = 0, y_1 = 1, x_2 = 1, y_2 = 0, x_3 = 1, y_3 = 0;
    int16_t x_4 = 0, y_4 = 0, x_5 = 0, y_5 = 0, x_6 = 0, y_6 = 0;
};

class ArrowIndicator
{
public:
    ArrowIndicator() = delete;
    ArrowIndicator(PolarGauge &g, int16_t tipradius, int16_t length, int16_t half_base, int16_t headlen = 0);

    // API
    void setColor(ucg_color_t c) { color = c; }
    bool draw(int16_t val);
    bool drawOver(int16_t val, float a);
    void testDraw(int16_t val);
    int16_t getBase() { return _root; }
    int16_t getPos() { return _needle_pos; }

private: // attributes
    PolarGauge &_gauge;             // ref to gauge the pointer belonges to
    const int16_t _tip;             // distance from gauge ref. center to arrow tip
    const int16_t _arrowhead;       // length of the head of the arrow
    const int16_t _root;            // distance from gauge ref. center to the tail of the indicator
    const int16_t _shoulder_val_offset; // .5deg steps from root point to shoulder point of arrow
    const int16_t _half_width;
    ucg_color_t color;
    // live data
    int16_t _needle_pos = 0;        // .5deg diced up
    float _last_a = 0.f;            // last drawn position in scale values
    ArrowPoints prev;
};

extern ArrowIndicator *testarrow;
