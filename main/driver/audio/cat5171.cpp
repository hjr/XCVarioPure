#include "cat5171.h"

#include "I2Cbus.hpp"
#include "logdefnone.h"

#include <esp_system.h>

CAT5171::CAT5171(i2cbus::I2C* i2cbus, void (*mcb)(), void (*ucb)()) : Poti(i2cbus, CAT5171_I2C_ADDR, mcb, ucb)
{
    RANGE = CAT5171RANGE;
}

bool CAT5171::readWiper(uint16_t& val) {
    // uint8_t rv;
    esp_err_t err = bus->readByte(CAT5171_I2C_ADDR, 0, (uint8_t*)&val);
    if (err == ESP_OK) {
        // ESP_LOGI(FNAME,"CAT5171 read wiper val=%d  OK", rv );
        // val = rv;
        return true;
    }
    // else
    ESP_LOGE(FNAME, "CAT5171 Error reading wiper, error count %d", errorcount);
    errorcount++;
    return false;
}

bool CAT5171::writeWiper(uint16_t val) {
    ESP_LOGI(FNAME, "CAT5171 write wiper %d", val);
    esp_err_t err = bus->writeByte(CAT5171_I2C_ADDR, 0, (uint8_t)val);
    if (err != ESP_OK) {
        ESP_LOGE(FNAME, "CAT5171 Error writing wiper, error count %d", errorcount);
        errorcount++;
        return false;
    }
    // ESP_LOGI(FNAME,"CAT5171 write wiper OK");
    return true;
}

bool CAT5171::reset() {
    ESP_LOGI(FNAME, "CAT5171 reset");
    uint16_t data = 128;
    esp_err_t err = bus->writeBytes(CAT5171_I2C_ADDR, 0x60, 2, (uint8_t*)&data);
    if (err != ESP_OK) {
        // ESP_LOGV(FNAME,"CAT5171 write wiper OK");
        ESP_LOGE(FNAME, "CAT5171 Error writing wiper, error count %d", errorcount);
        errorcount++;
        return false;
    }
    ESP_LOGI(FNAME, "CAT5171 reset OK");
    return true;
}
