/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "../SensorBase.h"
#include "math/vector_3d.h"
#include "math/Quaternion.h"

#include <MPU.hpp>
#include <mpu/types.hpp>

#include <cstdint>

struct PIController;
enum class ImuType : uint8_t {UNKNOWN, MPU6050, MPU6500, ICM20602, ICM20689};

class ImuSensor : public SensorTP<vector_f>
{
public:
    using temp_status_t = enum : uint8_t  { MPU_T_UNKNOWN, MPU_T_LOCKED, MPU_T_LOW, MPU_T_HIGH };

public:
    ImuSensor(SensorId id);
    ~ImuSensor() override;
    const char *name() const override;
    bool probe() override;
    bool setup() override;
    bool hasHeatCtlr() const { return _pictrl != nullptr; }


    ImuType getImuType() const { return _who_typ; }
    temp_status_t getTempStatus() const;
    static Quaternion setDefaultImuReference();
    void applyImuReference(const float gAA, const Quaternion& basic) {
        _ref_rot = concatGaaAndImuReference(gAA, basic);
    }

protected:
    ImuType getImuId();
    static Quaternion concatGaaAndImuReference(const float gAA, const Quaternion& basic);
    mpud::MPU& _MPUdev;
    ImuType _who_typ; // cached IMU type
    static Quaternion _ref_rot;

    // Heat control & parameters
    void initHeatCtrl();
    void temp_control();   // Tick hook
    void clearpwm(); // ensure heating is off
private:
    float _mpu_t_delta = 0;
    PIController *_pictrl = nullptr;
};

extern ImuSensor *accSensor;
extern ImuSensor *gyroSensor;
