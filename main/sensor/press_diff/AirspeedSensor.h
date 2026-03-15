/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "../SensorBase.h"
#include "Units.h"

class AirspeedSensor : public SensorTP<float> {
public:
    using ASens_Type = enum : uint8_t { ABPMRR, TE4525, MP3V5004, MCPH21, MAX_TYPES, NONE = 0xff };

    static constexpr float DYNP_THRESHOLD = Units::mps_to_pascal(8.0f / 3.6f); // ca. 8 km/h

    AirspeedSensor();
    virtual ~AirspeedSensor() {};

    static AirspeedSensor *autoSetup();

    bool setup() override;
    bool doRead(float &val) override;
    void postProcess() override;
    virtual void changeConfig() = 0;

protected:
    virtual bool fetch_pressure(int32_t &p, uint16_t &t) = 0;
    virtual bool offsetPlausible(int32_t offset) = 0;
    virtual int getMaxACOffset() = 0;
    void setMultiplier(float multiplier) { _multiplier = multiplier; }
    inline float getMultiplier() const { return _multiplier; }

private:
    LowPassFilterT<float> _dynp_lpf;
    int32_t _offset = 0; // raw adc offset value (float in nvs storage)
    float _multiplier = 1.0f;
    // rest counter
    int _restTimer = 0;
    bool _isResting = false;
    int _counter = 0;
};

extern AirspeedSensor *asSensor;

