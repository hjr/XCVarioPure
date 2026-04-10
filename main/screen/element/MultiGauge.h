/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ScreenElement.h"

template <typename T>
class SetupNG;

// multi purpose screen element
class MultiGauge : public ScreenElement
{
public:
    using MultiDisplay = enum { GAUGE_NONE, GAUGE_IAS_SPEED, GAUGE_TAS_SPEED, GAUGE_GND_SPEED, GAUGE_S2F, GAUGE_NETTO, GAUGE_HEADING, GAUGE_OAT, GAUGE_SLIP, GAUGE_MC };

    MultiGauge(int16_t cx, int16_t cy, MultiDisplay d, bool large=true);
    ~MultiGauge() = default;

    // API
    void setDisplay(MultiDisplay d);
    void draw();
    void drawUnit() const;

    // attributes
private:
    void update_nvs();
    MultiDisplay _display;
    bool _large;
    SetupNG<float> *_nvsvar = nullptr;
    int val_prev = -1;
};