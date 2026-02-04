/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "AirspeedSensor.h"

namespace i2cbus {
    class I2C;
}

class AsSensI2c : public AirspeedSensor
{
public:
    AsSensI2c(i2cbus::I2C *b, char addr) : AirspeedSensor(), _bus(b), _address(addr) {};
    virtual ~AsSensI2c() {};

    bool probe() override;

protected:
    bool fetch_pressure(int32_t &p, uint16_t &t) override;
    virtual void setSubType(bool positive) {} // dedicated to ABPMRR and MS4525DO
    i2cbus::I2C *_bus;
    const uint8_t _address;
    int _sign_read_count = 0;
};
