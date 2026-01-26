/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ds18b20.h"

#include "comm/OneWireBus.h"
#include "math/Units.h"
// #include "sensor.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <onewire_bus.h>
#include <onewire_crc.h>

#include <cmath>

#define EXAMPLE_ONEWIRE_BUS_GPIO    GPIO_NUM_23
constexpr int OW_SENSOR_ID = 0x28;
constexpr int DS18B20_RESOLUTION_12B = 0x7f; // 12-bit resolution

#define DS18B20_CMD_CONVERT_TEMP      0x44
#define DS18B20_CMD_WRITE_SCRATCHPAD  0x4E
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE

// OneWIRE need to be created earlier and live longer than the sensor

DS18B20::DS18B20(onewire_device_address_t addr) : OwSens(addr)
{
    _latency_ms = 800;
}

bool DS18B20::setup()
{
    OneWIRE->busReset();

    // send command: DS18B20_CMD_WRITE_SCRATCHPAD
    OneWIRE->sendCommand(_address, DS18B20_CMD_WRITE_SCRATCHPAD);

    // write new resolution to scratchpad
    // const uint8_t resolution_data[] = {0x1F, 0x3F, 0x5F, 0x7F};
    uint8_t tx_buffer[3] = {0};
    // tx_buffer[0] = ds18b20->th_user1;
    // tx_buffer[1] = ds18b20->tl_user2;
    tx_buffer[2] = DS18B20_RESOLUTION_12B;

    return OneWIRE->writeBytes(tx_buffer, sizeof(tx_buffer)) == ESP_OK;
}

// call to start a conversion
bool DS18B20::primeRead(uint32_t now_ms)
{
    // 1. reset
    OneWIRE->busReset();

    // 2. send command: DS18B20_CMD_CONVERT_TEMP
    if ( OneWIRE->sendCommand(_address, DS18B20_CMD_CONVERT_TEMP) == ESP_OK ) {
        _converting = true;
    }
    _convert_start_ms = now_ms; // allways keep timing for conversion
    return _converting;
}

// call when conversion is over to read the temperature
bool DS18B20::doRead(float &val)
{
    uint8_t scratch[9];
    _converting = false;

    // 3. Reset again
    OneWIRE->busReset();

    // 4. Read scratchpad
    esp_err_t err = OneWIRE->sendCommand(_address, DS18B20_CMD_READ_SCRATCHPAD);
    err |= OneWIRE->readBytes(scratch, sizeof(scratch));

    // 5. Validate CRC
    if (err != ESP_OK || scratch[8] != OneWIRE->crc8(scratch, 8)) {
        ESP_LOGE(FNAME, "DS18B20 error, return NAN");
        val = NAN;
        return false;
    }

    // 6. Decode temp
    val = (float)((scratch[1] << 8) | scratch[0]) / 16.0f;
    val += 273.15f; // convert to Kelvin
    return true;
}

