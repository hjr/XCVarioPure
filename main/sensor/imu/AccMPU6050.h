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
#include "math/Units.h"
#include "math/vector_3d_fwd.h"
#include "math/Trigonometry.h"
#include "math/Quaternion.h"

class MpuImu;

class AccMPU6050 final : public SensorTP<vector_f>
{
public:
    AccMPU6050(MpuImu &mmpu);

    static constexpr float ACCVAR_THRESHOLD2 = Units::ms2_to_g(0.02f * 0.02f); // variance threshold for rest detection (VQF-typical)
    static constexpr float ACCEL_THRESHOLD = Units::ms2_to_g(1.5f); // thresholds (VQF-typical)
    static constexpr float ACCEL_THRESHOLD2 = ACCEL_THRESHOLD * ACCEL_THRESHOLD;

    const char *name() const override { return _my_mpu.name(); }
    bool probe() override { return false; } // probe is done in MpuImu;
    bool setup() override { return _my_mpu.setup(); } // setup is done in MpuImu;
    bool doRead(vector_f& val) override;
    void postProcess() override;
    bool detectRest();
    bool isResting() const override { return _isResting; }
    void resetRest();

    temp_status_t getTempStatus() const { return _my_mpu.getTempStatus(); }
    inline degree_t getRollDeg() { return rad2deg(euler_rad.Roll()); }
    inline rad_t getRoll() { return euler_rad.Roll(); }
    inline degree_t getPitchDeg() { return rad2deg(euler_rad.Pitch()); }
    inline degree_t getYawDeg() { return rad2deg(euler_rad.Yaw()); }
    float getGyroFooting() const;
    inline degree_t getMagnHeadingDeg() { return rad2deg(filtered_mag_heading); }
    inline degree_t getCircleOmegaENUDeg() { return rad2deg(-circle_omega); }
    inline Quaternion getAHRSQuaternion() { return att_quat; }
    inline vector_f getAttVector() { return att_vector; }
    inline float getGload() const { return -getRef().z; }
    float getVerticalAcceleration();
    MpuImu& getMpu() const { return _my_mpu; }
    void resetBias() { _my_mpu.setAccelOffset({}); }
    void pushBias(const vector_f& bias);


private:
    MpuImu &_my_mpu;
    // static constexpr float ACCEL_SCALE = 4096.0f; // LSB/g for ±8g
    // AHRS variables
    rad_t fused_mag_heading = 0;
    rad_t filtered_mag_heading = 0;
    vector_f petal = {0,0,0};
    rps_t circle_omega = 0.f;
    rad_t circle_footing = 0.f; // a relative heading (ENU) w/o north reference
    Quaternion att_quat = Quaternion(); // a nav to body frame rotation! both NED frames!!
    Quaternion d_gyro = Quaternion();
    vector_f att_vector = {};
    vector_f euler_rad = {};
    // g-load
    LowPassFilterT<vector_f> _lpf_accel;
    // slip angle
    LowPassFilterT<float> _lpf_slip_angle;
    // calm counter
    int _restTimer = 0; // milliseconds since last movement
    bool _isResting = false;
    // vector_f _soft_bias; todo later
};

extern AccMPU6050 *accSensor;
