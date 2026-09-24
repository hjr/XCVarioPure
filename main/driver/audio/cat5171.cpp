#include "cat5171.h"

#include "logdefnone.h"

#include <driver/i2c_master.h>
#include <esp_system.h>

CAT5171::CAT5171(void (*mcb)(), void (*ucb)()) : Poti(mcb, ucb)
{
    RANGE = CAT5171RANGE;
}

CAT5171::~CAT5171()
{
    if (_dev) {
        i2c_master_bus_rm_device(_dev);
        _dev = NULL;
    }
}

bool CAT5171::readWiper(uint16_t& val) {
    uint8_t reg = 0x00;
    uint8_t rx = 0;
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, &rx, 1, 10);
    if (err == ESP_OK) {
        val = rx;
        ESP_LOGI(FNAME,"CAT5171 read wiper val=%d  OK", val );
        return true;
    }
    ESP_LOGE(FNAME, "CAT5171 Error reading wiper, error count %d", errorcount);
    errorcount++;
    return false;
}

bool CAT5171::writeWiper(uint16_t val) {
    uint8_t tx[2] = {0x00, static_cast<uint8_t>(val)};
    ESP_LOGI(FNAME, "CAT5171 write wiper %d", tx[1]);
    esp_err_t err = i2c_master_transmit(_dev, tx, sizeof(tx), 10);
    if (err != ESP_OK) {
        ESP_LOGE(FNAME, "CAT5171 Error writing wiper, error count %d", errorcount);
        errorcount++;
        return false;
    }
    // ESP_LOGI(FNAME,"CAT5171 write wiper OK");
    return true;
}

bool CAT5171::probe(i2c_master_bus_handle_t bus) {
    ESP_LOGI(FNAME, "CAT5171 probe & reset");
    if ( i2c_master_probe(bus, CAT5171_I2C_ADDR, 10) != ESP_OK ) {
        ESP_LOGE(FNAME, "CAT5171 probe FAIL");
        return false;
    }

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = CAT5171_I2C_ADDR,
        .scl_speed_hz    = 100000,
        .scl_wait_us     = 0,
        .flags = {
            .disable_ack_check = 1,
        }
    };

    if (i2c_master_bus_add_device(bus, &cfg, &_dev) != ESP_OK) {
        _dev = NULL;
        ESP_LOGE(FNAME, "CAT5171 add device FAIL");
        return false;
    }

    uint8_t tx[2] = { 0, 128 };
    esp_err_t err = i2c_master_transmit(_dev, tx, 2, 10);

    if (err != ESP_OK) {
        ESP_LOGE(FNAME, "CAT5171 Error reseting wiper, error count %d", errorcount);
        errorcount++;
    }
    ESP_LOGI(FNAME, "CAT5171 found and reset OK");
    return true;
}
