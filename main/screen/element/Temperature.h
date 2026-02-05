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

enum class temp_status_t : uint8_t;

// Temperature screen element
class Temperature : public ScreenElement
{
public:
    // right-aligned value to cx, incl. unit to the right of cx
    Temperature(int16_t cx, int16_t cy) : ScreenElement(cx, cy) {}
    // API
    void draw(kelvin_t t, temp_status_t mputemp);

    // attributes
private:
    kelvin_t _temp = -1001.;
    temp_status_t _imut = temp_status_t(0);
};
