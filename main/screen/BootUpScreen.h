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

class SetupMenuDisplay;

// draw a startup XCV logo
// precondition:
// - ucg adapter for the connected display
// - Clock timer

class BootUpScreen final : public Clock_I
{
public:
    static constexpr int16_t DIVIDER = 4;

    static BootUpScreen *create();
    ~BootUpScreen();
    // static bool isActive() { return inst != nullptr; }

    // update hook for the draw display context
    static void draw();

    // Clock tick callback
    bool tick() override;

private:
    BootUpScreen();
    static BootUpScreen *inst;
    void animate();

    int16_t x_offset;
    int16_t y_offset;
    float _fadein = 0.f;
    SetupMenuDisplay *_boot_log = nullptr;
};
