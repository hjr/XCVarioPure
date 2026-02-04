#pragma once

#include "../SensorBase.h"


class PressureSensor : public SensorTP<pascal_t>
{
   public:
    using PSens_Type = enum : uint8_t { SPL06_007, BME280_SPI, PS_MAX_TYPES };

    PressureSensor(SensorId id);
    virtual ~PressureSensor() {};

    virtual bool selfTest(celsius_t& t, pascal_t& p) = 0;
    virtual celsius_t readTemperature(bool& success) = 0;
    meter_t readAltitude(pascal_t qnh, bool& success);
    meter_t readAltitudeISA(bool& success);

    static PressureSensor* autoSetup(SensorId id);
};

extern PressureSensor *baroSensor;
extern PressureSensor *teSensor;

