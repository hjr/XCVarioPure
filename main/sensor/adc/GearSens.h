/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "SensorBase.h"

#include <driver/gpio.h>

// Digital gear warning input on S2 pin 6.
// An external logic drives the pin high (or low, when inverted) on a gear alert.
class GearSens final : public SensorBase {
private:
    explicit GearSens(bool inverted);

public:
    static GearSens* create(bool inverted);
    virtual ~GearSens();

    // sensor API
    const char* name() const override { return "GearSens"; }
    bool probe() override { return true; }
    bool setup() override;
    bool update(uint32_t now_ms) override;

    // specific API
    bool isAlarm() const { return _alarm; }
    void setInverted(bool inv) { _inverted = inv; }
    bool isInverted() const { return _inverted; }

private:
    static constexpr gpio_num_t GEAR_IO = GPIO_NUM_34; // S2 pin 6
    bool _inverted;
    volatile bool _alarm = false;
};

extern GearSens *gearSensor;
