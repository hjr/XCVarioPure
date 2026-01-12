/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "../SensorBase.h"
#include "../SensorMgr.h"
#include "math/vector_3d.h"

class GpsVSensor final : public SensorTP<vector_f> {
private:
    GpsVSensor();

public:
    static GpsVSensor* createGpsVSensor();

    const char* name() const override { return "GPS"; }
    bool probe() override { return true; }
    bool setup() override { return true; }

    // Never used
    bool doRead(vector_f &val) override {
        return false;
    }

    // Injection API
    void inject(float lat, float lon);
    void setExternalAltitude(float);

private:
    float _lat_ref = 0.f;
    float _lon_ref = 0.f;
    float _alt = 0.f;
};

extern GpsVSensor* GpsSensor;