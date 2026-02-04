/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/


#include "AsSensI2c.h"
#include "logdefnone.h"

#include <I2Cbus.hpp>

bool AsSensI2c::probe()
{
    uint8_t data[4];
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < 4; i++)
    {
        err = _bus->readBytes(_address, 0, 4, data);
        if (err == ESP_OK)
            break;
    }
    if (err != ESP_OK)
    {
        ESP_LOGW(FNAME, "%s probing I2C for address %02x FAILED", name(), _address);
        return false;
    }
    return true;
}

// #define RANDOM_TEST
#define Press_H data[0]
#define Press_L data[1]
#define Temp_H  data[2]
#define Temp_L  data[3]

bool AsSensI2c::fetch_pressure(int32_t &p, uint16_t &t)
{
    // ESP_LOGI(FNAME,"fetch_pressure");
    uint8_t data[4];
    esp_err_t err = _bus->readBytes(_address, 0, 4, data);
    if (err != ESP_OK)
    {
        // i2c error detected
        ESP_LOGW(FNAME, "fetch_pressure() I2C error");
        return false;
    }
#ifdef RANDOM_TEST
    Press_H = esp_random() % 255;
    Press_L = esp_random() % 255;
    Temp_L = esp_random() % 255;
#endif
    // ESP_LOG_BUFFER_HEXDUMP(FNAME,data,4, ESP_LOG_INFO);
    uint8_t stat = (Press_H >> 6) & 0x03;
    p = ((Press_H & 0x3f) << 8) | Press_L;
    t = (Temp_H << 3) | (Temp_L >> 5);
    ESP_LOGI(FNAME,"fetch_pressure() status: %d, err %d,  P:%d T: %d Off:%d, SC:%d",  (int)stat, (int)err, (int)p, (int)t, (int)_offset, _sign_read_count);
    if ( abs(p - _offset) > 10 ) {
        _sign_read_count += (p > _offset) ? 1 : -1;
        if ( abs(_sign_read_count) == 100) {
            ESP_LOGW(FNAME, "fetch_pressure() sign pressure read count: %d", _sign_read_count);
            setSubType(_sign_read_count > 0);
        }
    }
    return stat == 0;
}


