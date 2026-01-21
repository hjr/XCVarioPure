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

// extern mpud::MPU MPU;
enum class ImuType : uint8_t {UNKNOWN, MPU6050, MPU6500, ICM20602, ICM20689};

class ImuSensor : public SensorTP<vector_f>
{
public:
    ImuSensor(SensorId id);
    const char *name() const override;
    bool probe() override;
    bool setup() override;
    void initHeatCtrl();


    ImuType getImuType() const { return _who_typ; }
    inline temp_status_t getTempStatus() const { return _MPUdev.getSiliconTempStatus(); }
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
};

extern ImuSensor *accSensor;
extern ImuSensor *gyroSensor;
