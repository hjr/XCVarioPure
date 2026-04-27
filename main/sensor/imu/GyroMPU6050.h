/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "vqfekf.h"
#include "../SensorBase.h"
#include "math/vector_3d_fwd.h"


class MpuImu;

class GyroMPU6050 final : public SensorTP<vector_f>
{
public:
    GyroMPU6050(MpuImu &mmpu);

    static constexpr float GYRO_THRESHOLD = Units::deg_to_rad(0.2f); // thresholds (VQF-typical)
    static constexpr float GYRO_THRESHOLD2 = GYRO_THRESHOLD * GYRO_THRESHOLD; // squared for variance comparison
    static constexpr int GYRO_REST_DURATION_MS = 30000; // 30 seconds

    const char *name() const override;
    bool probe() override { return false; } // probe is done in MpuImu;
    bool setup() override { return false; } // setup is done in MpuImu;
    bool doRead(vector_f& val) override;
    void postProcess() override;
    bool isResting() const override { return _isResting > 1; }
    void resetRest();
    inline float getAxD() const { return _gyro_lpf_dwydt.get(); }
    bool detectRest();
    void pushBias(vector_f& bias);
    inline const vector_f& getBias() const { return _bias_estimator.getBias(); }

private:
    MpuImu& _my_mpu;
    const float _scale;
    // low-pass filter for gyro y-axis to get dw/dt
    LowPassFilterT<float> _gyro_lpf_dwydt{0.5f}; // to compensate the accelerometer mounting position in front of CG
    // vqf rest detection and bias estimation
    BiasEstimatorEKF _bias_estimator;
    uint8_t _bias_update = 0;
    int _restTimer = 0; // milliseconds since last movement
    uint8_t _isResting = 0; // goes in three stages: 0 = moving, 1 = long half rest (bias update enforced), 2 = rest 

};

extern GyroMPU6050 *gyroSensor;