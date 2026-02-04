
#pragma once


#include "AsSensI2c.h"

#include <cstdint>

class MS4525DO final : public AsSensI2c
{
public:
    // instance methods
    MS4525DO(bool is_abpmrr = false);
    virtual ~MS4525DO() = default;

    const char *name() const override;
    void changeConfig() override;
    inline void setAbpmrr() { setSubType(true); }

protected:
    void setSubType(bool positive) override;
    bool offsetPlausible(int32_t offset) override;
    int getMaxACOffset() override;

private:
    float getTemperature(); // returns temperature of last measurement
    uint16_t t_dat; // 11 bit temperature data
    bool _is_abpmrr = false;
};
