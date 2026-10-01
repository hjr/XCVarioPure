/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "PressureSensor.h"

class PSclient : public PressureSensor {
   public:
    PSclient(SensorType typ) : PressureSensor(SensorId(typ, 4)) {};
    virtual ~PSclient() = default;

    const char* name() const override { return _id.type == SensorType::STATIC_PRESSURE ? "STclient" : "TEclient"; }
    bool probe() override { return true; }
    bool setup() override { return true; }
    bool selfTest(celsius_t& t, pascal_t& p) override { return true; }
    bool doRead(pascal_t &val) override { return false; }
    celsius_t readTemperature(bool& success) override { success = false; return 0.0; }
};
