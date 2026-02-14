/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "../SensorBase.h"
#include "math/vector_3d_fwd.h"


class MpuImu;

class GyroMPU6050 final : public SensorTP<vector_f>
{
public:
    GyroMPU6050(const MpuImu &mmpu);

    const char *name() const override;
    bool probe() override { return false; } // probe is done in MpuImu;
    bool setup() override { return false; } // setup is done in MpuImu;
    bool doRead(vector_f& val) override;
    void postProcess() override;
    float getAxD() const { return _gyro_lpf_ayd.get(); }
    const vector_f& getGyro() const { return gyro; }

private:
    const MpuImu& _my_mpu;
    const float _scale;
    // low-pass filter for gyro y-axis to get dw/dt
    LowPassFilter _gyro_lpf_ayd{0.5f}; // to compensate the accelerometer mounting position in front of CG
    vector_f gyro = {0,0,0};
};

extern GyroMPU6050 *gyroSensor;