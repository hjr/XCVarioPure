/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "sensor/temp/OwSens.h"

struct onewire_bus_t;

constexpr int DS18B20_FAMILY = 0x28;

class DS18B20 final : public OwSens
{
public:
    DS18B20(onewire_device_address_t addr);
    virtual ~DS18B20() = default;
    const char* name() const override { return "DS18B20"; }
    bool setup() override;
    bool doRead(float &val) override;

    // OW
    uint8_t family() override { return DS18B20_FAMILY; }
    bool primeRead(uint32_t now_ms) override;
};
