#pragma once

#include "driver/time/ClockIntf.h"

#include <driver/i2c_types.h>

#include <cstdint>

namespace i2cbus {
    class I2C;
}

enum e_poti_type
{
    POTI_NONE = 0,
    POTI_CAT5171,
    POTI_MCP4018
};

// digital poti interface
class Poti : public Clock_I
{
public:
    Poti() = delete;
    explicit Poti(void (*mcb)(), void (*ucb)());
    virtual ~Poti() {};
    bool begin();
    virtual bool probe(i2c_master_bus_handle_t bus) = 0;
    virtual e_poti_type getType() const = 0;
    bool tick() override;

    void softSetVolume(float val); // the important and only to use API
    // virtual bool readVolume( float& val );
    bool writeVolume(float val); // 0..100 %, only for test purposes, it does not drive the amplifier mute cbs

protected:
    static constexpr int calcDbFromVolume(float val);
    i2c_master_dev_handle_t _dev = nullptr;
    i2cbus::I2C *bus = nullptr;

    int errorcount = 0;
    static float RANGE;

private:
    virtual bool writeWiper(uint16_t val) = 0;
    virtual bool readWiper(uint16_t &val) = 0;

    uint16_t lastWiperValue = 0;
    uint16_t targetWiperValue = 0;
    void (*_mute_cb)(); // optional callback to mute the audio during volume changes
    void (*_unmute_cb)();
};
