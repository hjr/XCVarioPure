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

extern const ucg_color_t ndl_color[3];

class ScreenElement
{
public:
    ScreenElement() = delete;
    ScreenElement(int16_t refx, int16_t refy) : _ref_x(refx), _ref_y(refy) {}
    ~ScreenElement() = default;
    void setRef(int16_t x, int16_t y) { _ref_x=x; _ref_y=y; }
    void forceRedraw() { _dirty = true; }

public:
    int16_t _ref_x;
    int16_t _ref_y;
    bool _dirty = true;
};
