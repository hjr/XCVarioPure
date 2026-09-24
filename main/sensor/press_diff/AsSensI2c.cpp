/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/


#include "AsSensI2c.h"
#include "sensor.h"
#include "logdefnone.h"

#include <driver/i2c_master.h>

bool AsSensI2c::probe_i2c(uint8_t addr)
{
    if ( i2c_master_probe(i2c_bus, addr, 10) != ESP_OK ) {
        ESP_LOGE(FNAME, "I2C probe FAIL");
        return false;
    }

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000,
        .scl_wait_us     = 0,
        .flags = {
            .disable_ack_check = 1,
        }
    };
    if (i2c_master_bus_add_device(i2c_bus, &cfg, &_dev) != ESP_OK) {
        _dev = NULL;
        ESP_LOGE(FNAME, "I2C add device FAIL");
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
    uint8_t reg = 0x0;
    uint8_t data[4];
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, data, 4, 10);
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
    ESP_LOGI(FNAME,"fetch_pressure() status: %d, err %d,  P:%d T: %d",  (int)stat, (int)err, (int)p, (int)t);
    return stat == 0;
}


