#include "mcp4018.h"

#include "logdefnone.h"

#include <driver/i2c_master.h>
#include <esp_system.h>

MCP4018::MCP4018(void (*mcb)(), void (*ucb)()) : Poti(mcb, ucb)
{
    RANGE = MCP4018RANGE;
}

MCP4018::~MCP4018()
{
    if (_dev) {
        i2c_master_bus_rm_device(_dev);
        _dev = NULL;
    }
}

bool MCP4018::readWiper(uint16_t& val) {
    uint8_t rx;
    esp_err_t err = i2c_master_receive(_dev, &rx, 1, 10);

    if (err == ESP_OK) {
        val = rx;
        ESP_LOGI(FNAME,"MCP4018 read wiper val=%d  OK", val );
        return true;
    }
    ESP_LOGE(FNAME, "MCP4018 Error reading wiper, error count %d", errorcount);
    errorcount++;
    return false;
}

bool MCP4018::writeWiper(uint16_t val) {
    uint8_t *tx = reinterpret_cast<uint8_t*>(&val);
    ESP_LOGI(FNAME,"MCP4018 write wiper %d", *tx);
    esp_err_t err = i2c_master_transmit(_dev, tx, 1, 10);

    if (err == ESP_OK) {
        ESP_LOGI(FNAME,"MCP4018 write wiper OK val=%d", val );
        return true;
    }
    ESP_LOGE(FNAME, "MCP4018 Error writing wiper, error count %d", errorcount);
    errorcount++;
    return false;
}

bool MCP4018::probe(i2c_master_bus_handle_t bus) {
    ESP_LOGI(FNAME, "MCP4018 probe & reset");
    if ( i2c_master_probe(bus, MCP4018_I2C_ADDR, 10) != ESP_OK ) {
        ESP_LOGE(FNAME, "MCP4018 probe FAIL");
        return false;
    }

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MCP4018_I2C_ADDR,
        .scl_speed_hz    = 100000,
        .scl_wait_us     = 0,
        .flags = {
            .disable_ack_check = 1,
        }
    };

    if (i2c_master_bus_add_device(bus, &cfg, &_dev) != ESP_OK) {
        _dev = NULL;
        ESP_LOGE(FNAME, "MCP4018 add device FAIL");
        return false;
    }

    ESP_LOGI(FNAME, "MCP4018 found OK");
    return true;
}
