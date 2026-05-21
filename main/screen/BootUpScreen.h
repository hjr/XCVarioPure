/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "driver/time/ClockIntf.h"

#include <cstdint>

// draw a startup XCV logo
// precondition:
// - ucg adapter for the connected display
// - Clock timer

class BootUpScreen final : public Clock_I
{
public:
    static constexpr int16_t DIVIDER = 4;

    static BootUpScreen *create();
    static void terminate();
    static bool isActive() { return inst != nullptr; }

    // this will fill the logo completely
    void finish(int16_t part);
    static void draw();

    // Clock tick callback
    bool tick() override;

private:
    BootUpScreen();
    ~BootUpScreen();
    static BootUpScreen *inst;
    void animate();

    int16_t x_offset;
    int16_t y_offset;
    float _fadein = 0.f;
};
