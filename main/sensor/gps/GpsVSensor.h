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
#include "vector.h"
#include "math/Units.h"

class GpsVSensor;
extern GpsVSensor* gpsSensor;

class GpsVSensor final : public SensorTP<vector_f> {
private:
    GpsVSensor();

public:
    static GpsVSensor* createGpsVSensor();

    const char* name() const override { return "GPS"; }
    bool probe() override { return true; }
    bool setup() override { return true; }

    // Main API
    bool doRead(vector_f &val) override { return false; } // not used
    void postProcess() override;
    static bool getValid() { return gpsSensor && gpsSensor->getHeadValid(); }
    inline bool isValid() const { return getHeadValid(); }
    int getNumSat() const { return _numSat; }
    void setNumSat(int num) { _numSat = num; return; }
    rps_t getOmega() const { return _omega; }

    // Injection API
    void inject(float lat, float lon, mps_t gndSpeed, rad_t gndCourse);
    void setExternalAltitude(float);

private:
    int16_t _numSat;
    float _lat_ref = 0.f;
    float _lon_ref = 0.f;
    float _alt = 0.f;
    rps_t _omega = 0.f;
    Vector _prev_diff = {};
};

