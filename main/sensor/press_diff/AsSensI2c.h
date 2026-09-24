/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "AirspeedSensor.h"

#include <driver/i2c_types.h>

namespace i2cbus {
    class I2C;
}

class AsSensI2c : public AirspeedSensor
{
public:
    AsSensI2c() : AirspeedSensor() {};
    virtual ~AsSensI2c() {};

    bool probe_i2c(uint8_t addr);

protected:
    bool fetch_pressure(int32_t &p, uint16_t &t) override;
    i2c_master_dev_handle_t _dev = NULL;
};
