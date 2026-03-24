/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "TempVSensor.h"

class TempSim : public TempVSensor
{
public:
    TempSim() : TempVSensor() {}
    ~TempSim() = default;

    const char* name() const override { return "TempSim"; }
    bool setup() override { return true; }
    bool doRead(float &val) override { return false; } // never used
};
