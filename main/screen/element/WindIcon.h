/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ScreenElement.h"

#include "wind/Wind.h"
#include "IpsDisplay.h"

#include <cstdint>

class PolarGauge;


// Small wind indicator, showing a direction icon and a number with the strength.
// This is only about synoptic wind.

class WindIcon : public ScreenElement
{
  public:
    WindIcon() = delete;
    WindIcon(int16_t cx, int16_t cy, int16_t radius);

    // API
    using ScreenElement::draw;
    bool draw(WindData w);

  private:
    void drawIcon() const;
    void drawUnit() const;

  private: // attributes
    WindData _wind; // current wind data
    int16_t _radius; // radius of the wind icon
    int16_t _str_width; // width of the string for the wind strength
    Point _arrow[4];
};
