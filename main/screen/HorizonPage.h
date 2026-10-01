/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "IpsDisplay.h"
#include "math/Quaternion.h"

#include <cstdint>

class BasePage
{
public:
    static bool _DIRTY; // valid for all pages, set when page cycles
};

class HorizonPage : public BasePage
{
public:
    static HorizonPage* HORIZON();
    ~HorizonPage() = default;

    void rot(int count);
    void draw(Quaternion q);
    void leave();

    static constexpr const int16_t BOX_SIZE = 200;

private:
    HorizonPage();
    static HorizonPage* instance;
    Line previous_horizon_line;
    Point horizon_box[4];
    int16_t _show_adjustment = 0; // show adjustment for some seconds
};
