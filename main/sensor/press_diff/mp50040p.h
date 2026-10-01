/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "AirspeedSensor.h"

#include "../adc/mcp3221.h"

class MP5004DP : public AirspeedSensor
{
public:
    MP5004DP();
    const char *name() const override { return "MP5004DP"; }
    bool probe() override;
    void changeConfig() override;

protected:
	bool fetch_pressure(int32_t &p, uint16_t &t) override;
    bool offsetPlausible(int32_t offset) override;
    int  getMaxACOffset() override;

private:
    MCP3221 _mcp;
};
