/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "AnalogInput.h"


class BatteryVoltage final : public AnalogInput {
   public:
    explicit BatteryVoltage();
    virtual ~BatteryVoltage() {};

    // sensor API
    const char* name() const override { return "Battery"; }
    bool setup() override;
    bool doRead(volt_t& val) override;
    // specific API
    void setAdjust(float adj);

   private:
    LowPassFilterT<float> _lpf_volt;
    static const float _Multiplier;
    float _adjust_factor;  // reverse mV after voltage divider 22K/1.2K to Volt
};

extern BatteryVoltage *batSensor;
