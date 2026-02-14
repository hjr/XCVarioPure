/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ImuSensor.h"
#include "../SensorBase.h"
#include "math/vector_3d_fwd.h"
#include "math/Trigonometry.h"
#include "math/Quaternion.h"

class MpuImu;

class AccMPU6050 final : public SensorTP<vector_f>
{
public:
    AccMPU6050(MpuImu &mmpu);

    const char *name() const override { return _my_mpu.name(); }
    bool probe() override { return false; } // probe is done in MpuImu;
    bool setup() override { return _my_mpu.setup(); } // setup is done in MpuImu;
    bool doRead(vector_f& val) override;
    void postProcess() override;
    const vector_f& getAccel() const { return accel; }

    temp_status_t getTempStatus() const { return _my_mpu.getTempStatus(); }
    inline float getRollDeg() { return rad2deg(euler_rad.Roll()); }
    inline float getRoll() { return euler_rad.Roll(); }
    inline float getPitchDeg() { return rad2deg(euler_rad.Pitch()); }
    inline float getYawDeg() { return rad2deg(euler_rad.Yaw()); }
    inline float getCircleOmegaDeg() { return rad2deg(circle_omega); }
    inline Quaternion getAHRSQuaternion() { return att_quat; }
    float getVerticalAcceleration();
    MpuImu& getMpu() const { return _my_mpu; }

private:
    MpuImu &_my_mpu;
    // static constexpr float ACCEL_SCALE = 4096.0f; // LSB/g for ±8g
    // AHRS variables
    vector_f accel = {0,0,0};
    float fused_yaw = 0;
    rad_t filterYaw = 0;
    vector_f petal = {0,0,0};
    float circle_omega = 0.f;
    Quaternion att_quat = Quaternion();
    Quaternion omega_step = Quaternion();
    vector_f att_vector = {};
    vector_f euler_rad = {};
    //AHRS helpers
    float PitchFromAccelRad();
};

extern AccMPU6050 *accSensor;
