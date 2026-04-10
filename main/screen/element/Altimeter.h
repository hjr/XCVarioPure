/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ScreenElement.h"
#include "sensor/Filters.h"
#include "math/Units.h"

#include <cstdint>

enum class alt_unit_t : uint8_t;

// altimeter screen element
class Altimeter : public ScreenElement
{
public:
    Altimeter(int16_t cx, int16_t cy, bool big=true);

    // API
    using AltitudeDisplay = enum { MODE_QNH, MODE_QFE };
    using AltQuantisation = enum { ALT_QUANT_DISABLE, ALT_QUANT_2, ALT_QUANT_5, ALT_QUANT_10, ALT_QUANT_20 };
    using AltAttr = enum : uint8_t { ATTR_SMALL = 1 };

    // void setSmall() { _aattr |= 1; }
    void drawUnit();
    void draw(meter_t alt);

private: // attributes
    LowPassFilterT<float> _alt_lpf{0.15f};
    int   _alt_prev = 0;
    uint8_t _aattr = 0;
    bool  _isa_alt = false;
    alt_unit_t _unit;
    alt_unit_t _unit_drawn;
    // for roling altimeter
	int16_t _char_width;
	int16_t _char_height;
    int16_t _quant = 0;
    uint8_t _last_quant = 0;
    float fraction_prev = 1.;
    char altpart_prev_s[32] = "";
};