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
#include "MPU.hpp"

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
    float getTemperature() const { return _MPUdev.getTemperature(); }
    ImuType getImuType() const { return _who_typ; }
    temp_status_t getTempStatus() const;
    static Quaternion getDefaultImuReference(bool include_factory_ref=true);
    void applyImuReference(const degree_t gAA, const Quaternion& basic) {
        _ref_rot = concatGaaAndImuReference(gAA, basic);
    }
    void resetImuReference(bool save_nvs=true);
    void resetCalibProgress() { progress = 0; }
    void zeroGyroBias();
    void zeroAccBias();
    int getAccelSamplesAndCalib(vector_f gyro_integral, rad_t& wing_angle, rad_t& ground_angle);
    vector_f extractAccBias(vector_f *samples, int nr, float *res0, float *res);
    void calculateFactoryReference(vector_f *corr_samples, int nr);
    void setLeverArm(float la) { _leverarm = la; }
    inline float getLeverArm() const { return _leverarm; }
    void setRefRot(const Quaternion& ref) { _ref_rot = ref; }
    inline Quaternion getRefRot() const { return _ref_rot; }
    void restoreAccelOffset() const; // restore accel offset from NVS

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
    inline mpud::raw_axes_t getAccelOffset() const { return _MPUdev.getAccelOffset(); }
    inline esp_err_t setAccelOffset(mpud::raw_axes_t bias) { return _MPUdev.setAccelOffset(bias); }
    inline bool isIcm20602() const { return _who_typ == ImuType::ICM20602; }

    // Heat control & parameters
    void initHeatCtrl();
    void temp_control();   // Tick hook
    void clearpwm(); // ensure heating is off

private:
    // IMU reference calibration
    Quaternion _ref_rot; // maps sensor to body frame
    float _leverarm = 0.f; // distance of the accelerometer to the CG in m, used for acceleration compensation during rotation
    int16_t progress = 0; // bit-wise 0 -> 1 -> 3 -> 0 // start -> right -> left -> finish
    vector_f bob_right_wing, bob_left_wing, bob_level;
    vector_f gyro_axis_right, gyro_axis_left;

    celsius_t _mpu_t_delta = 0; // difference to target temp, positive means too hot
    PIController *_pictrl = nullptr;

    constexpr static mpud::accel_fs_t ACCEL_SCALE = mpud::ACCEL_FS_8G;
    constexpr static mpud::gyro_fs_t GYRO_SCALE = mpud::GYRO_FS_250DPS;
};

