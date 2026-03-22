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

// Model IDs
constexpr uint8_t DS18S20MODEL   = 0x10;  // also DS1820
constexpr uint8_t DS18B20MODEL   = 0x28;
constexpr uint8_t DS1822MODEL    = 0x22;
constexpr uint8_t DS1825MODEL    = 0x3B;
constexpr uint8_t DS28EA00MODEL  = 0x42;

class DS18B20 final : public OwSens
{
public:
    DS18B20(onewire_device_address_t addr);
    virtual ~DS18B20() = default;
    const char* name() const override { return "DS18B20"; }
    bool setup() override;
    bool doRead(float &val) override;

    // OW
    uint8_t family() override { return (uint8_t)(_address && 0xff); }
    bool primeRead(uint32_t now_ms) override;

private:
    int16_t calculateTemperature(uint8_t* scratchPad);
};
