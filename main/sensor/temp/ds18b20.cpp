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
#include "logdef.h"

#include <onewire_bus.h>
#include <onewire_crc.h>

#include <cmath>

#define EXAMPLE_ONEWIRE_BUS_GPIO    GPIO_NUM_23
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

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

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
        ESP_LOGE(FNAME, "DS18B20 error, return false");
        return false;
    }

    // 6. Decode temp
    val = (float)calculateTemperature(scratch) / 128.f;
    val += Units::C2K; // convert to Kelvin

    // 7. Plausibility check
    if (val < 235.f || val > 325.f) {
        ESP_LOGE(FNAME, "DS18B20 implausible temperature: %f", val);
        return false;
    }
    return true;
}

// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
int16_t DS18B20::calculateTemperature(uint8_t* scratchPad)
{
	int16_t fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11) | (((int16_t) scratchPad[TEMP_LSB]) << 3);
	
    // DS1820 and DS18S20 have a 9-bit temperature register.
    // Resolutions greater than 9-bit can be calculated using the data from
    // the temperature, and COUNT REMAIN and COUNT PER °C registers in the
    // scratchpad.  The resolution of the calculation depends on the model.
    // While the COUNT PER °C register is hard-wired to 16 (10h) in a
    // DS18S20, it changes with temperature in DS1820.
    // After reading the scratchpad, the TEMP_READ value is obtained by
    // truncating the 0.5°C bit (bit 0) from the temperature data. The
    // extended resolution temperature can then be calculated using the
    // following equation:
    //                                 COUNT_PER_C - COUNT_REMAIN
    // TEMPERATURE = TEMP_READ - 0.25 + --------------------------
    //                                         COUNT_PER_C
    // Hagai Shatz simplified this to integer arithmetic for a 12 bits
    // value for a DS18S20, and James Cameron added legacy DS1820 support.
    // See - http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html

	if (family() == DS18B20MODEL) {
		fpTemperature = ((fpTemperature & 0xfff0) << 3) - 16 +
				(((scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) << 7) / scratchPad[COUNT_PER_C]);
	}
	return fpTemperature;
}

