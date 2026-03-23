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

template <typename T>
class FilterItf {
public:
    virtual ~FilterItf() = default;
    virtual void reset(T init_val) = 0;
    virtual T filter(T input) = 0;
    virtual T get() const = 0;
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
template <typename T>
class LowPassFilterT : public FilterItf<T>
{
public:
    explicit LowPassFilterT(float alpha) : _alpha(alpha), _last_output(T{}) {}
    static inline float alphaFromTau(second_t tau, second_t dt) {
        return dt / (tau + dt);
    }
    void setTau(second_t tau, second_t dt) { _alpha = alphaFromTau(tau, dt); }
    void setAlpha(float alpha) { _alpha = alpha; }
    float getAlpha() const { return _alpha; }
    void reset(T init_val) { _last_output = init_val; }
    T filter(T input);
    T get() const { return _last_output; }
    const T& getRef() const { return _last_output; }
private:
    float _alpha;
    T _last_output;
};


class ZeroOutGatingLPFilter : public LowPassFilterT<float>
{
public:
    ZeroOutGatingLPFilter(float alpha, float threshold) : LowPassFilterT<float>(alpha), _threshold(threshold) {}
    void setThreshold(float threshold) { _threshold = threshold; }
    float filter(float input) override;
private:
    float _threshold;
};