#pragma once

#include "../SensorBase.h"


class PressureSensor : public SensorTP<float>
{
   public:
    using PSens_Type = enum : uint8_t { SPL06_007, BME280_SPI, PS_MAX_TYPES };

    PressureSensor(SensorId id);
    virtual ~PressureSensor() {};

    virtual bool selfTest(float& p, float& t) = 0;
    virtual float readTemperature(bool& success) = 0;
    float readAltitude(float qnh, bool& success);

    static PressureSensor* autoSetup(SensorId id);

   protected:
    // float _multiplier = 1.0f;
};
