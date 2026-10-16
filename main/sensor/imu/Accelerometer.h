/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ImuSensor.h"
#include "sensor/SensorBase.h"
#include "math/Units.h"
#include "math/vector_3d_fwd.h"

class Accelerometer final : public SensorTP<vector_f>
{
public:
    Accelerometer();
    virtual ~Accelerometer();

    static constexpr float ACCVAR_THRESHOLD2 = Units::ms2_to_g(0.04f * 0.04f); // variance threshold for rest detection (VQF-typical)
    static constexpr float ACCEL_THRESHOLD = Units::ms2_to_g(1.5f); // thresholds (VQF-typical)
    static constexpr float ACCEL_THRESHOLD2 = ACCEL_THRESHOLD * ACCEL_THRESHOLD;

    const char *name() const override;
    bool probe() override { return false; } // probe is done in MpuImu
    bool setup() override { return false; } // setup is done in MpuImu
    vector_f selftest() const;
    bool doRead(vector_f& val) override;
    void postProcess() override;
    bool detectRest();
    bool isResting() const override { return _isResting; }
    void resetRest();

    temp_status_t getTempStatus() const { return imuSensor ? imuSensor->getTempStatus() : temp_status_t::MPU_T_UNKNOWN; }
    inline float getGload() const { return -getRef().z; }
    float getVerticalAcceleration();
    void resetBias() { if (imuSensor) imuSensor->setAccelOffset({}); }
    void pushBias(const vector_f& bias);


private:
    // virtually it has an aggregate MpuImu* imuSensor; // but this is global
    // g-load
    LowPassFilterT<vector_f> _lpf_accel;
    // calm counter
    int _restTimer = 0; // milliseconds since last movement
    bool _isResting = false;
    // vector_f _soft_bias; todo later
};

extern Accelerometer *accSensor;
