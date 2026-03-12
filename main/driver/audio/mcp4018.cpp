#include "mcp4018.h"

#include "I2Cbus.hpp"
#include "logdefnone.h"

#include <esp_system.h>

MCP4018::MCP4018(i2cbus::I2C* i2cbus, void (*mcb)(), void (*ucb)()) : Poti(i2cbus, MPC4018_I2C_ADDR, mcb, ucb)
{
    RANGE = MCP4018RANGE;
}

// esp_err_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, int32_t timeout = -1);

bool MCP4018::readWiper(uint16_t& val) {
    esp_err_t err = bus->read8bit(MPC4018_I2C_ADDR, &val);
    if (err == ESP_OK) {
        ESP_LOGI(FNAME,"MCP4018 read wiper val=%d  OK", val );
        return true;
    } else {
        ESP_LOGE(FNAME, "MCP4018 Error reading wiper, error count %d", errorcount);
        errorcount++;
        return false;
    }
}

bool MCP4018::writeWiper(uint16_t val) {
    ESP_LOGI(FNAME,"MCP4018 write wiper %d", val );
    esp_err_t err = bus->write8bit(MPC4018_I2C_ADDR, val );
    if (err == ESP_OK) {
        ESP_LOGI(FNAME,"MCP4018 write wiper OK val=%d", val );
        return true;
    } else {
        ESP_LOGE(FNAME, "MCP4018 Error writing wiper, error count %d", errorcount);
        errorcount++;
        return false;
    }
}
