/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "sensor/SensorBase.h"
#include "math/vector_3d.h"
#include "math/Quaternion.h"

class FlightAHRS final : public SensorTP<vector_f>
{
public:
    FlightAHRS();
    virtual ~FlightAHRS();

    const char *name() const override { return "ahrs"; }
    bool probe() override { return false; }
    bool setup() override;
    bool doRead(vector_f& val) override { return false; }
    void postProcess() override;

    void startCalibration(int8_t duration = 50); // duration in cycles
    degree_t getRollDeg() const;
    rad_t    getRoll() const;
    degree_t getPitchDeg() const;
    rad_t    getPitch() const;
    degree_t getYawDeg() const;
    degree_t getMagnHeadingDeg() const;
    dps_t getCircleOmegaENUDeg() const; // ENU: pos. -> left turn; neg. -> right turn
    Quaternion getBody2Nav() const;
    Quaternion getNav2Body() const;

private:
    // AHRS variables
    // rad_t fused_mag_heading = 0;
    // rad_t filtered_mag_heading = 0;
    rps_t circle_omega = 0.f;
    rad_t _circle_footing = 0.f; // a relative heading (ENU) w/o north reference
    Quaternion _q = Quaternion(); // a body to nav frame rotation! both NED frames!!
    vector_f euler_rad = {};
    // slip angle
    LowPassFilterT<float> _lpf_slip_angle;
    int8_t _calibrating = 0; // calibration counter
};

extern FlightAHRS *ahrsSensor;
