/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "sensor/SensorBase.h"

#include "../SensorMgr.h"


class TempVSensor : public SensorTP<float> {
public:
    TempVSensor();
    TempVSensor& operator=(const TempVSensor&) = delete;
    TempVSensor(const TempVSensor&) = delete;
    virtual ~TempVSensor();

    bool probe() override { return true; }; // not used
    
    // For sim purposes
    const char* name() const override { return "Temp"; }
    bool setup() override { return true; }
    bool doRead(float &val) override { return false; } // never used

private:
    LowPassFilterT<float> _lpf;
};


extern TempVSensor* oatSensor;
