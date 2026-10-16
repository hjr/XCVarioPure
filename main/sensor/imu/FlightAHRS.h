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

    inline degree_t getRollDeg() { return rad2deg(euler_rad.Roll()); }
    inline rad_t getRoll() { return euler_rad.Roll(); }
    inline degree_t getPitchDeg() { return rad2deg(euler_rad.Pitch()); }
    inline degree_t getYawDeg() { return rad2deg(euler_rad.Yaw()); }
    inline degree_t getMagnHeadingDeg() { return rad2deg(filtered_mag_heading); }
    inline dps_t getCircleOmegaENUDeg() { return rad2deg(-circle_omega); }
    inline Quaternion getNav2Body() { return att_quat; }

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
