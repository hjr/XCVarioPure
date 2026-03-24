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
    ~TempVSensor() = default;
    bool probe() override { return true; };
private:
    LowPassFilterT<float> _lpf;
};


extern TempVSensor* oatSensor;
