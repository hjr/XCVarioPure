#include "mcp4018.h"

#include "I2Cbus.hpp"
#include "logdefnone.h"

#include <esp_system.h>

MCP4018::MCP4018(i2cbus::I2C* i2cbus, void (*mcb)(), void (*ucb)()) : Poti(i2cbus, MPC4018_I2C_ADDR, mcb, ucb)
{
    RANGE = MCP4018RANGE;
}

bool MCP4018::readWiper(uint16_t& val) {
    // uint16_t i16val;
    esp_err_t err = bus->readBits(MPC4018_I2C_ADDR, 0, 0, 2, (uint8_t*)&val);
    if (err == ESP_OK) {
        // ESP_LOGI(FNAME,"MCP4018 read wiper val=%d  OK", i16val );
        // val = i16val;
        return true;
    } else {
        ESP_LOGE(FNAME, "MCP4018 Error reading wiper, error count %d", errorcount);
        errorcount++;
        return false;
    }
}

bool MCP4018::writeWiper(uint16_t val) {
    // ESP_LOGI(FNAME,"MCP4018 write wiper %d", val );
    // uint8_t v = (uint8_t)val;
    esp_err_t err = bus->writeByte(MPC4018_I2C_ADDR, 0, (uint8_t)val);
    if (err == ESP_OK) {
        // ESP_LOGI(FNAME,"MCP4018 write wiper OK val=%d", val );
        return true;
    } else {
        ESP_LOGE(FNAME, "MCP4018 Error writing wiper, error count %d", errorcount);
        errorcount++;
        return false;
    }
}
