/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "setup/MenuEntry.h"
#include "driver/time/ClockIntf.h"

class SensorTest final : public MenuEntry, public Clock_I
{
private:
    SensorTest();

public:
    ~SensorTest();

    // simple use
    static SensorTest* create();
    static void update();

    // MenuEntry interface
    void display(int mode = 0) override;
    void draw(); // own

    // Receiver interface
    const char *value() const override { return nullptr; }
    void rot(int count) override {}
    void press() override;
    void longPress() override { press(); }

    // Clock tick callback
    bool tick() override;
};

