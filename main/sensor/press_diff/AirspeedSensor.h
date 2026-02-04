#pragma once

#include "../SensorBase.h"

class AirspeedSensor : public SensorTP<float> {
public:
    using ASens_Type = enum : uint8_t { PS_ABPMRR, PS_TE4525, PS_MP3V5004, PS_MCPH21, PS_MAX_TYPES, PS_NONE = PS_MAX_TYPES };

    AirspeedSensor();
    virtual ~AirspeedSensor() {};

    static AirspeedSensor *autoSetup();

    bool setup() override;
    bool doRead(float &val) override;
    virtual void changeConfig() = 0;

protected:
    virtual bool fetch_pressure(int32_t &p, uint16_t &t) = 0;
    virtual bool offsetPlausible(int32_t offset) = 0;
    virtual int getMaxACOffset() = 0;
    int32_t _offset = 0.; // raw adc offset value (float in nvs storage)
    float _multiplier = 1.0f;
};

extern AirspeedSensor *asSensor;

