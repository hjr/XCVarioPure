#pragma once

#include "Poti.h"

constexpr uint8_t MCP4018_I2C_ADDR = 0x2f; // 0101111

// MCP4018 7 bit digital potentiometer.

class MCP4018 : public Poti
{
public:
    // Connect module using I2C port
    MCP4018() = delete;
    explicit MCP4018(void (*mute_cb)(), void (*unmute_cb)());
    virtual ~MCP4018();

    bool probe(i2c_master_bus_handle_t bus) override;
    e_poti_type getType() const override { return POTI_MCP4018; }
    bool writeWiper(uint16_t val) override;

private:
    bool readWiper(uint16_t &val) override;
    static constexpr int MCP4018RANGE = 40; // do not use the full range of 127
};

