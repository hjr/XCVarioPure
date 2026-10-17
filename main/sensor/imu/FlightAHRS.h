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
#include "math/Trigonometry.h"
#include "math/Quaternion.h"

class FlightAHRS final : public SensorTP<vector_f>
{
public:
    FlightAHRS();
    virtual ~FlightAHRS();

    const char *name() const override { return "ahrs"; }
    bool probe() override { return false; }
    bool setup() override { return false; }
    bool doRead(vector_f& val) override { return false; }
    void postProcess() override;

    degree_t getRollDeg() const;
    rad_t getRoll() const;
    degree_t getPitchDeg() const;
    degree_t getYawDeg() const;
    degree_t getMagnHeadingDeg() const;
    dps_t getCircleOmegaENUDeg() const;
    Quaternion getNav2Body() const;

private:
    // AHRS variables
    rad_t fused_mag_heading = 0;
    rad_t filtered_mag_heading = 0;
    vector_f petal = {0,0,0};
    rps_t circle_omega = 0.f;
    rad_t circle_footing = 0.f; // a relative heading (ENU) w/o north reference
    Quaternion att_quat = Quaternion(); // a nav to body frame rotation! both NED frames!!
    Quaternion d_gyro = Quaternion();
    vector_f euler_rad = {};
    // slip angle
    LowPassFilterT<float> _lpf_slip_angle;
};

extern FlightAHRS *ahrsSensor;
