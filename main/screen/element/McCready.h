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

// MC screen element
class McCready : public ScreenElement
{
public:
    McCready(int16_t cx, int16_t cy) : ScreenElement(cx, cy) {}
    // API
    void setLarge(bool l) { _large = l; }
    void draw(mps_t mc);

    // attributes
private:
    bool _large = true;
    mps_t _mcval = -1.;
};