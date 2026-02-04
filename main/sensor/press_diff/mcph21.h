
#pragma once

#include "AsSensI2c.h"

#include <cstdint>

class MCPH21 final : public AsSensI2c
{
public:
    // instance methods
    MCPH21();
    ~MCPH21() = default;

    const char *name() const override { return "MCPH21"; }
    bool probe() override;
    bool setup() override;
    void changeConfig() override;

protected:
    bool fetch_pressure(int32_t &p, uint16_t &t) override;
    bool offsetPlausible(int32_t offset) override;
    int  getMaxACOffset() override;

private:
    float getTemperature(void); // returns temperature of last measurement
};
