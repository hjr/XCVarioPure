/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "TempVSensor.h"

#include <onewire_types.h>
#include <cstdint>


class OwSens : public TempVSensor
{
public:
    OwSens() = delete;
    OwSens(onewire_device_address_t addr) : TempVSensor(), _address(addr) {}
    virtual uint8_t family() = 0;
    virtual bool primeRead(uint32_t now_ms) = 0;
    onewire_device_address_t getAddress() const { return _address; }
    bool isConverting() const { return _converting; }
    uint32_t getConvertStartMs() const { return _convert_start_ms; }

protected:
    onewire_device_address_t _address;
    bool _converting = false;
    uint32_t _convert_start_ms = 0;
};
