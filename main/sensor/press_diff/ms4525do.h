/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once


#include "AsSensI2c.h"

#include <cstdint>

class MS4525DO final : public AsSensI2c
{
public:
    // instance methods
    MS4525DO(bool is_abpmrr = false);
    virtual ~MS4525DO() = default;

    const char *name() const override;
    bool probe() override;
    void changeConfig() override;

protected:
    bool offsetPlausible(int32_t offset) override;
    int getMaxACOffset() override;

private:
    uint8_t _is_abpmrr :1;
    float getTemperature(); // returns temperature of last measurement
    uint16_t t_dat; // 11 bit temperature data
};
