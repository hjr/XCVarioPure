#pragma once

#include "PressureSensor.h"

#include <cstdint>

namespace i2cbus {
    class I2C;
}

class SPL06_007 : public PressureSensor {
   public:
    SPL06_007(SensorId id);
    virtual ~SPL06_007() {};
    const char* name() const override { return "SPL06_007"; }
    bool probe() override;
    bool setup() override;
    bool selfTest(float& t, pascal_t& p) override;
    bool doRead(float &val) override;
    float readTemperature(bool& success) override;

   private:
    bool get_raw(int32_t& val, int32_t* tvalptr);

	i2cbus::I2C* _bus;
    uint8_t _address;

    int32_t c00, c10;
    int16_t c0, c1;
    int16_t c01, c11, c20, c21, c30;

    float _scale_factor_p;
    float _scale_factor_t;
    int32_t _traw;
    uint32_t tick = 0;
};
