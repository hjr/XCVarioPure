/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "../SensorBase.h"
#include "math/Quaternion.h"
#include "math/Units.h"

#include <MPU.hpp>
#include <mpu/types.hpp>

#include <cstdint>

class AccMPU6050;
class GyroMPU6050;
struct PIController;
enum class ImuType : uint8_t {UNKNOWN, MPU6050, MPU6500, ICM20602, ICM20689};
enum class temp_status_t : uint8_t { MPU_T_UNKNOWN, MPU_T_LOCKED, MPU_T_LOW, MPU_T_HIGH };


class MpuImu
{
public:
    MpuImu();
    ~MpuImu();
    const char *name() const;
    bool probe();
    bool setup();
    bool hasHeatCtlr() const { return _pictrl != nullptr; }

    ImuType getImuType() const { return _who_typ; }
    temp_status_t getTempStatus() const;
    static Quaternion getDefaultImuReference();
    void applyImuReference(const degree_t gAA, const Quaternion& basic) {
        _ref_rot = concatGaaAndImuReference(gAA, basic);
    }
    int getAccelSamplesAndCalib(vector_f gyro_integral, rad_t& wing_angle);
    inline void setLeverArm(float la) { _leverarm = la; }
    inline float getLeverArm() const { return _leverarm; }

    friend class AccMPU6050;
    friend class GyroMPU6050;

protected:
    ImuType getImuId();
    static Quaternion concatGaaAndImuReference(const degree_t gAA, const Quaternion& basic);
    mpud::MPU& _MPUdev;
    ImuType _who_typ; // cached IMU type
    inline vector_f rotate(const vector_f& v) const { return _ref_rot.rotate(v); }
    inline esp_err_t rotation(mpud::raw_axes_t *r) const { return _MPUdev.rotation(r); }
    inline esp_err_t acceleration(mpud::raw_axes_t *a) const { return _MPUdev.acceleration(a); }
    inline mpud::raw_axes_t getGyroOffset() const { return _MPUdev.getGyroOffset(); }
    inline esp_err_t setGyroOffset(mpud::raw_axes_t bias) { return _MPUdev.setGyroOffset(bias); }

    // Heat control & parameters
    void initHeatCtrl();
    void temp_control();   // Tick hook
    void clearpwm(); // ensure heating is off

private:
    // IMU reference calibration
    Quaternion _ref_rot;
    float _leverarm = 0.f; // distance of the accelerometer to the CG in m, used for acceleration compensation during rotation

    celsius_t _mpu_t_delta = 0; // difference to target temp, positive means too hot
    PIController *_pictrl = nullptr;
};

