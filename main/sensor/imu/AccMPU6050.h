/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ImuSensor.h"


class AccMPU6050 final : public ImuSensor
{
public:
    AccMPU6050();

    bool doRead(vector_f& val) override;
    void postProcess() override;
    
private:
    // static constexpr float ACCEL_SCALE = 4096.0f; // LSB/g for ±8g
};

