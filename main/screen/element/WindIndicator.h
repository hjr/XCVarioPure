/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "wind/Wind.h"
#include "IpsDisplay.h"
#include "AdaptUGC.h"

#include <cstdint>

class PolarGauge;


// Wind indicator around gauge center

class WindIndicator
{
public:
    WindIndicator() = delete;
    WindIndicator(PolarGauge &g, bool live);

    // API
    // void setWind(int16_t wdir, int16_t wval) { _dir = wdir; _val = wval; }
    void setColor(ucg_color_t c) { _color = c; }
    // a degraded wind (not valid any more, still the best estimate) is drawn grey; true when that changed
    bool setDegraded(bool d) {
        const bool changed = d != _degraded;
        _degraded = d;
        return changed;
    }
    bool changed(WindData w);
    bool draw(WindData w);
    void drawWind(bool erase=false);
    void redrawBG();

  private: // attributes
    const PolarGauge &_gauge;  // ref to gauge the indicator belonges to
    WindData _wind; // current wind data
    bool _live = false;
    bool _degraded = false;
    ucg_color_t _color;
    Point _indgeom[5]; // geometry of the indicator at 0° (north wind)
    Point _arrow[5];
    BoundingBox _abox;
    static int16_t _cheight;
    static int16_t _cwidth;
};
