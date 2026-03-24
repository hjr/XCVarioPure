/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "../SensorBase.h"
// #include "Compass.h"
#include "math/vector_3d_fwd.h"
#include "sensor/Filters.h"


class MagVSensor final : public SensorTP<vector_f> {
private:
    MagVSensor();

public:
    static MagVSensor* createMagVSensor();

    const char* name() const override { return "MAG"; }
    bool probe() override { return true; }
    bool setup() override;
    bool doRead(vector_f& val) override { return false; } // not used
    void postProcess() override;

    // Injection API
    void inject(int16_t x, int16_t y, int16_t z);

    // Legacy calibration API
    bool calibrate( void (*reporter)(const CompassCalibrationData &data, bool print), bool only_show);

private:
    // bool calcCalibration(CompassCalibrationData &data);
    bool loadCalibration();
    // void resetCalibration();
    // void saveCalibration(const vector_f &bias, const vector_f &scale);

    vector_f _bias;
    vector_f _scale;
    LowPassFilterT<float> _lpf_heading;
};

extern MagVSensor* magSensor;
