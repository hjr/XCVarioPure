/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "setup/primitive/SetupMenuDisplay.h"

class WiperControl : public SetupMenuDisplay
{
public:
    WiperControl();
    virtual ~WiperControl();

    void enter() override;
    void exit(int ups=1) override;
    void display(int mode = 0) override;
    void rot(int count) override;
    void press() override;
    void longPress() override;
    void release() override;

    void callback(int par);

private:
    uint8_t _active_button = 0;
    const int16_t display_width;
};

extern WiperControl* WPControl;
