#include "mcph21.h"

#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <driver/i2c_master.h>


#define I2C_ADDRESS_MCPH21    0x7F  //  Datasheet testcode: IC_Send(0xFE,..) what is 7F shifted right 1 bit

// Long term stability of Sensor as from datasheet FS* 0.15 + 0.3 (dT) % per year -> 16777216 * 0.00015 = 2516
// pressure for minimum of 60 Pa: 911868 Offset according to datasheet: 838861, difference: ~73000 and ~1% FS of 7549746
// raised to 100000 to be on the safe side, which is ~1.3% FS, but still much less than 10% FS which is the minimum step 
// size of the sensor output according to the datasheet (10% of 2^24 counts = 1677721 counts) and also much less than the 
// typical long term drift of the sensor (0.15% FS per year = 2516 counts per year)
#define MAX_AUTO_CORRECTED_OFFSET 100000

// MCPH21D sensor full scale range and units
// const int16_t MCPH21FullScaleRange = 0.725;  //  psi
 
// MCPH21D Sensor type (10% to 90%)
// Output (% of 2^14 counts) = P max. 80% x (Pressure applied – P min. ) + 10%

// const int16_t MCPH21MinScaleCounts = 0;
// const float MCPH21multiplier =  2 * 6894.76 / MCPH21Span;

MCPH21::MCPH21() : AsSensI2c()
{
    changeConfig();
}

MCPH21::~MCPH21()
{
    if (_dev) {
        i2c_master_bus_rm_device(_dev);
        _dev = NULL;
    }
}

bool MCPH21::probe()
{
    ESP_LOGI(FNAME, "MCPH21 probe");
    
    if ( !probe_i2c(I2C_ADDRESS_MCPH21) ) {
        ESP_LOGE(FNAME, "MCPH21 probe FAIL");
        return false;
    }

    uint8_t reg = 0x01;
    uint8_t byte;
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, &byte, 1, 10);
    if (err != ESP_OK) {
        ESP_LOGI(FNAME, "MCPH21 selftest read Chip ID reg 0x01 failed");
        return false;
    }
    else {
        ESP_LOGI(FNAME, "MCPH21 selftest read Chip ID reg 0x01: %x ", byte);
    }
    reg = 0xA8;
    err = i2c_master_transmit_receive(_dev, &reg, 1, &byte, 1, 10);
    if (err != ESP_OK) {
        ESP_LOGI(FNAME, "MCPH21 selftest read Chip ID reg 0xA8 failed");
        return false;
    }
    else {
        ESP_LOGI(FNAME, "MCPH21 selftest read Chip ID reg 0xA8: %x", byte);
    }
    return true;
}

bool MCPH21::setup()
{
    // set continuous mode
    uint8_t tx[2] = { 0x30, 0x0B }; // continuous mode, sleep 62.5msec
    esp_err_t err = i2c_master_transmit(_dev, tx, 2, 10);

    if (err != ESP_OK) {
        ESP_LOGE(FNAME, "MCPH21: failed to set continuous mode: %s", esp_err_to_name(err));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(6));

    return AirspeedSensor::setup();
}

void MCPH21::changeConfig()
{
    setMultiplier(6250.f / 8388608.f * ((100.0 + speedcal.get()) / 100.0));
    ESP_LOGI(FNAME, "changeConfig, speed multiplier %f, speed cal: %f ", getMultiplier(), speedcal.get());
}


// #define RANDOM_TEST
#define Press_H data[0]
#define Press_L data[1]
#define Temp_H  data[2]
#define Temp_L  data[3]

bool MCPH21::fetch_pressure(int32_t &p, uint16_t &t)
{
    // ESP_LOGI(FNAME,"MCPH21::fetch_pressure");
    uint8_t reg = 0x06;
    uint8_t pres[3];
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, pres, 3, 10);
    if (err != ESP_OK)
    {
        ESP_LOGW(FNAME, "fetch_pressure readBytes I2C error");
        return false;
    }

#ifdef RANDOM_TEST
    Press_H = esp_random() % 255;   
    Press_L = esp_random() % 255;
    Temp_L = esp_random() % 255;
#endif
    p = (int32_t(pres[0]) << 16) + (int32_t(pres[1]) << 8) + int32_t(pres[2]);
    if (p & 0x800000) p |= 0xFF000000; // sign extend negative numbers

    return true;
}

bool MCPH21::offsetPlausible(int32_t offset)
{
    constexpr int lower_val = 838861 - MAX_AUTO_CORRECTED_OFFSET;
    constexpr int upper_val = 838861 + MAX_AUTO_CORRECTED_OFFSET;
    bool plausible = (offset > lower_val) && (offset < upper_val);
    ESP_LOGI(FNAME, "offsetPlausible( %ld ) Deviation: %.1f%% RET:%d", offset, (((float)offset - 838861.0f) / MAX_AUTO_CORRECTED_OFFSET) * 100.0f, plausible);
    return plausible;
}

int MCPH21::getMaxACOffset()
{
    return MAX_AUTO_CORRECTED_OFFSET;
}

float MCPH21::getTemperature()
{
    uint8_t reg = 0x09;
    uint8_t t_buf[2];
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, t_buf, 2, 10);

    float temp = 0.f;
    if (err != ESP_OK)
    {
        ESP_LOGI(FNAME, "MCPH21 read temperature 0x09 failed");
        return 0.0;
    }
    else
    {
        int t = t_buf[0] * 256 + t_buf[1];
        temp = t / 256.0;
        ESP_LOGI(FNAME, "MCPH21 T val read ok: T: %.2f", temp);
    }
    return temp;
}
