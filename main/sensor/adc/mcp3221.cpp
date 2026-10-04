
#include "mcp3221.h"

#include "logdefnone.h"

#include <driver/i2c_master.h>

#define MCP3221_CONVERSE 0x4d   //10011010 NOTE IF IT ENDS IN 1, this is the READ ADDRESS. This is all this device does.
                                //It opens a conversation via this specific READ address

// Library for the MCP3221 12 BIT ADC.
//   MCP3221  Top View

//   +  -o   o- SCL
//   -  -o
//   S  -o   o- SDA

MCP3221::~MCP3221()
{
    if (_dev) {
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
    }
}

// scan bus for I2C address
bool MCP3221::probe(i2c_master_bus_handle_t bus)
{
    if ( i2c_master_probe(bus, MCP3221_CONVERSE, 10) != ESP_OK ) {
        ESP_LOGI(FNAME, "Could not probe a MCP3221 device");
        return false;
    }

    ESP_LOGI(FNAME, "MCP3221 probe");
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MCP3221_CONVERSE,
        .scl_speed_hz    = 100000,
        .scl_wait_us     = 0,
        .flags = {
            .disable_ack_check = 1,
        }
    };

    if (i2c_master_bus_add_device(bus, &cfg, &_dev) != ESP_OK) {
        _dev = nullptr;
        ESP_LOGE(FNAME, "MCP3221 add device FAIL");
        return false;
    }

    uint8_t data[2];
    esp_err_t err = i2c_master_receive(_dev, data, sizeof(data), 10);
    if (err != ESP_OK) {
        ESP_LOGI(FNAME, "MCP3221 selftest, scan for I2C address %02x FAILED", MCP3221_CONVERSE);
        i2c_master_bus_rm_device(_dev);
        _dev = nullptr;
        return false;
    }
    ESP_LOGI(FNAME, "MCP3221 selftest, scan for I2C address %02x PASSED", MCP3221_CONVERSE);

    return true;
}

// You cannot write to an MCP3221, it has no writable registers.
// MCP3221 also requires an ACKnowledge between each byte sent, before it will send the next byte. So we need to be a bit manual with how we talk to it.
// It also needs an (NOT) ACKnowledge after the second byte or it will keep sending bytes (continuous sampling)
// 
// From the datasheet.
// 
// I2C.START
// Send 8 bit device/ part address to open conversation.   (See .h file for part explanation)
// read a byte (with ACK)
// read a byte (with NAK)
// I2C.STOP

int MCP3221::readVal()
{
    int retval = 0;
    int samples = 0;
    uint16_t as_last = 0;
    for (int i = 0; i < 8; i++)
    {
        uint16_t as;
        if (readRaw(as) == ESP_OK)
        {
            if ( as_last == 0 ) as_last = as;
            if (abs(as - as_last) > 1000) {
                ESP_LOGE(FNAME, "REREAD AS delta OOB dropped, cur:%04x  last:%04x", as, as_last);
            }
            else {
                retval += as;
                samples++;
            }
            as_last = as;
        }
        else {
            ESP_LOGE(FNAME, "Airspeed I2C read error");
        }
    }
    if (samples)
    {
        return retval / samples;
    }
    return 0;
}

esp_err_t MCP3221::readRaw(uint16_t &val)
{
    uint8_t data[2];
    esp_err_t err = i2c_master_receive(_dev, data, sizeof(data), 10);
    if (err != ESP_OK) {
        val = 0;
    } else {
        val = (data[0] << 8) + data[1];
    }

    return err;
}


