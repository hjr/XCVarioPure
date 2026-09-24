#pragma once

#include <esp_err.h>
#include <driver/i2c_master.h>


// Connect module using I2C port pins sda and scl. The output is referenced to the supply voltage which can be
// 2.7v to 5.0v. The read will return the correct voltage, if you supply the correct supplyVoltage when instantiating.

class MCP3221
{
public:
    MCP3221() = default;
    ~MCP3221();

    // check for reply with I2C bus address
    bool probe(i2c_master_bus_handle_t bus);

    // raw read value of airspeed sensor
    int readVal();

    // Reads the analog register of the MCP3221 and converts it to a useable value. (a voltage)
    esp_err_t readRaw(uint16_t &val);

private:
    i2c_master_dev_handle_t _dev = nullptr;
};

