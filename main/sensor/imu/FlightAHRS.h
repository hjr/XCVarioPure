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
#include "math/MahonyAHRS.h"

// Compact 6-axis tilt AHRS, no heap or dependencies.
// NED, body X forward Y right Z down;
// Accelerometer SPECIFIC FORCE in g: at rest (0,0,-1). 
// Gyro body rad/s.
// Quaternion rotates BODY/NED -> NAV/NED.

class FlightAHRS  final : public SensorTP<vector_f> {
public:
    FlightAHRS();
    virtual ~FlightAHRS();
    // void setConfig(Config c);

    const char *name() const override { return "ahrs"; }
    bool probe() override { return false; }
    bool setup() override;
    bool doRead(vector_f& val) override { return false; }
    void postProcess() override;

    void startCalibration(int8_t duration = 50); // duration in cycles
    float rollRad() const;
    float pitchRad() const;
    // Yaw unobservable without heading source.
    // Gravity-compensated inertial acceleration in BODY axes (m/s^2).
    // This is NOT necessarily TAS derivative (wind, sideslip, rotations).
    // vector_f linearAccelBody(vector_f accelG, float g=9.80665f) const;

    degree_t getRollDeg() const;
    rad_t getRoll() const;
    degree_t getPitchDeg() const;
    rad_t getPitch() const;
    degree_t getYawDeg() const;
    degree_t getMagnHeadingDeg() const;
    dps_t getCircleOmegaENUDeg() const;
    Quaternion getBody2Nav() const; // Conventional NED aerospace roll/pitch, radians.
    Quaternion getNav2Body() const;

private:
    MahonyAHRS::Config _cfg = {};
    Quaternion _q = {};
    rad_t _circle_footing = 0.f; // a relative heading (ENU) w/o north reference
    rps_t _omega = 0.f;
    int8_t _calibrating = 0;
    uint32_t _prev_time = 0;
};


extern FlightAHRS *ahrsSensor;
