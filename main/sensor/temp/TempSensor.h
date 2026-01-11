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


class TempSensor : public SensorTP<float> {
public:
    TempSensor();
    ~TempSensor() = default;
    bool probe() override { return false; };
};


extern TempSensor* OATSensor;
