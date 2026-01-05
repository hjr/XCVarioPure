/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

// A simple filter design not knowing the signal history

class SensorBase;

class BaseFilterItf
{
public:
    virtual ~BaseFilterItf() = default;
    virtual float filter(float input) = 0;
};

// A simple low-pass filter
class LowPassFilter : public BaseFilterItf
{
public:
    explicit LowPassFilter(float alpha) : _alpha(alpha), _last_output(0.0f) {}
    void reset(float init_val);
    float filter(float input) override;
private:
    float _alpha;
    float _last_output;
};


// A air speed converter and low-pass filter
class AirSpeedFilter : public BaseFilterItf
{
public:
    explicit AirSpeedFilter(float alpha) : _lpf(alpha) {}
    float filter(float input) override;
private:
    LowPassFilter _lpf;
};

// Altimeter 
class AltimeterFilter : public BaseFilterItf
{
public:
    AltimeterFilter() = default;
    float filter(float input) override;
};

// TE Variometer 
class TEVariometerFilter : public BaseFilterItf
{
public:
    TEVariometerFilter() = default;
    float filter(float input) override;
private:
    float _oldte = 0.0f;
};
