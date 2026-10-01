/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "AirspeedSensor.h"

#include <cstdint>

class ASclient final : public AirspeedSensor
{
public:
    ASclient() : AirspeedSensor() {};

    const char *name() const override { return "ASclient"; }
    bool probe() override { return true; }
    bool setup() override { return true; }
    void changeConfig() override {}

protected:
    bool fetch_pressure(int32_t &p, uint16_t &t) override { return false; }
    bool offsetPlausible(int32_t offset) override { return false; }
    int  getMaxACOffset() override { return 0; }
};
