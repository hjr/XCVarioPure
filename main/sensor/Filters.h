/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "math/Units.h"

// Simple filter not knowing the signal history

class SensorBase;

class BaseFilterItf
{
public:
    virtual ~BaseFilterItf() = default;
    virtual void reset(float init_val) = 0;
    virtual float filter(float input) = 0;
    virtual float get() const { return 0.0f; }
};

// A simple low-pass filter (exponential moving average)
//
// Links to N-sample average
// in time:
// alpha = dt / (tau + dt)
// with tau => N/2 * dt
// in samples:
// alpha = 2 / (N + 2)
// example for tau = 3sec and dt = 0.1sec:
// N = (tau/dt) * 2 = 60 samples
// alpha = 2 / (60 + 2) = 0.032258
class LowPassFilter : public BaseFilterItf
{
public:
    explicit LowPassFilter(float alpha) : _alpha(alpha), _last_output(0.0f) {}
    void setAlpha(float alpha) { _alpha = alpha; }
    float getAlpha() const { return _alpha; }
    void reset(float init_val) override { _last_output = init_val; }
    float filter(float input) override;
    float get() const override { return _last_output; }
private:
    float _alpha;
    float _last_output;
};


// // TE Variometer 
// class TEVariometerFilter : public BaseFilterItf
// {
// public:
//     TEVariometerFilter() = default;
//     float filter(float input) override;
// private:
//     float _oldte = 0.0f;
// };
