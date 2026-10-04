
#include "spl06_007.h"

#include "../SensorMgr.h"
#include "math/Units.h"
#include "sensor.h"
#include "logdefnone.h"

#include <driver/i2c_master.h>


#define SPL06_007_BARO 0x77
#define SPL06_007_TE   0x76


SPL06_007::SPL06_007(SensorType typ) :
    PressureSensor(SensorId(typ, 4)),
    _address( (typ == SensorType::STATIC_PRESSURE) ? SPL06_007_BARO : SPL06_007_TE )
{
}

SPL06_007::~SPL06_007()
{
    if (_dev) {
        i2c_master_bus_rm_device(_dev);
        _dev = NULL;
    }
}

bool SPL06_007::probe()
{
    if ( i2c_master_probe(i2c_bus, _address, 10) != ESP_OK ) {
        ESP_LOGE(FNAME, "I2C probe FAIL");
        return false;
    }

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = _address,
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
    
    uint8_t reg = 0x0D;
    uint8_t id;
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, &id, 1, 10);

    if (err != ESP_OK || id != 0x10) {
        i2c_master_bus_rm_device(_dev);
        _dev = NULL;
        return false;
    }

    return true;
}

// Setup for continuous mode (background) 64Hz 8x oversampling and 1Hz temperature
//
// ---- Oversampling of >8x for temperature or pressuse requires FIFO operational mode which is not implemented ---
// ---- Use rates of 8x or less until feature is implemented ---
bool SPL06_007::setup() {
    // Addr. 0x06 PM_RATE Bits 6-4:    0110 - 64 measurements pro sec.
    //            PM_PRC  Bis  3-0:    0011 - 8 times oversampling
    // Addr. 0x08 MEAS_CTRL Bits 2-0:  111  - Continuous pressure and temperature measurement

    // Pressure    6=64 samples per second and 3 = 8x oversampling
    uint8_t tx[2] = {0X06, 0x63};
    esp_err_t err = i2c_master_transmit(_dev, tx, 2, 10);
    // Temperature 0=1   sample  per second and 0 = no oversampling
    tx[0] = 0X07;
    tx[1] = 0X80;
    err |= i2c_master_transmit(_dev, tx, 2, 10);
    // continuous temp and pressure measurement
    tx[0] = 0X08;
    tx[1] = 0B0111;
    err |= i2c_master_transmit(_dev, tx, 2, 10);
    // FIFO Pressure measurement
    tx[0] = 0X09;
    tx[1] = 0x00;
    err |= i2c_master_transmit(_dev, tx, 2, 10);
    if (err != ESP_OK) {
        ESP_LOGE(FNAME, "Error I2C write during setup");
        return false;
    }

    uint8_t reg = 0x08;
    uint8_t status = 0xff;
    for (int i = 0; i < 10; i++) {
        // mandatory wait time for calib data to be ready
        vTaskDelay(pdMS_TO_TICKS(10));
        err = i2c_master_transmit_receive(_dev, &reg, 1, &status, 1, 10);
        if (err == ESP_OK) {
            if (status & 0x80) {
                break;
            }
        }
    }
    if (!(status & 0x80)) {
        ESP_LOGW(FNAME, "Coefficients not ready, status byte %02x", status);
        return false;
    }

    reg = 0x10;
    uint8_t calib[18];
    if (i2c_master_transmit_receive(_dev, &reg, 1, calib, sizeof(calib), 10) != ESP_OK) {
        ESP_LOGE(FNAME, "Error I2C read calibration data");
        return false;
    }
    c0  = (calib[0] << 4) | (calib[1] >> 4);
    if (c0 & (1 << 11)) c0 = c0 | 0xF000; // 2's complement conversion of negitive number
    c1 = ((calib[1] & 0x0F) << 8) | calib[2];
    if (c1 & (1 << 11)) c1 = c1 | 0xF000;
    c00 = (calib[3] << 12) | (calib[4] << 4) | (calib[5] >> 4);
    if (c00 & (1 << 19)) c00 = c00 | 0xFFF00000;
    c10 = ((calib[5] & 0x0F) << 16) | (calib[6] << 8) | calib[7];
   	if (c10 & (1 << 19)) c10 = c10 | 0xFFF00000;

    reg = 0x18;
    if (i2c_master_transmit_receive(_dev, &reg, 1, calib, 10, 10)!= ESP_OK) {
        ESP_LOGE(FNAME, "Error I2C read calibration data");
        return false;
    }
    c01 = (int16_t)((calib[0] << 8) | calib[1]);
    c11 = (int16_t)((calib[2] << 8) | calib[3]);
    c20 = (int16_t)((calib[4] << 8) | calib[5]);
    c21 = (int16_t)((calib[6] << 8) | calib[7]);
    c30 = (int16_t)((calib[8] << 8) | calib[9]);

    // oversampling rate
    constexpr const float oversampling_factor[8] = { 524288.0, 1572864.0, 3670016.0, 7864320.0, 253952.0, 516096.0, 1040384.0, 2088960.0 };
    reg = 0x06;
    if (i2c_master_transmit_receive(_dev, &reg, 1, calib, 2, 10) != ESP_OK) {
        ESP_LOGE(FNAME, "Error I2C read calibration data");
        return false;
    }
    _scale_factor_p = oversampling_factor[calib[0] & 0x7];
    _scale_factor_t = oversampling_factor[calib[1] & 0x7];
    return true;
}

celsius_t SPL06_007::readTemperature(bool& success) {
    success = true;
    return c0 * 0.5f + c1 * _traw / _scale_factor_t;
}

bool SPL06_007::selfTest( celsius_t& t, pascal_t& p ){
    uint8_t rdata = 0xFF;
    vTaskDelay(pdMS_TO_TICKS(100));                          // give first measurement time to settle
    uint8_t reg = 0x0D;
    esp_err_t err = i2c_master_transmit_receive(_dev, &reg, 1, &rdata, 1, 10);
    if (err != ESP_OK) {
        ESP_LOGE(FNAME, "Error I2C read, status :%d", err);
        return false;
    }
    ESP_LOGI(FNAME, "SPL06_007 selftest, scan for I2C address %02x PASSED, Product ID: %d, Revision ID:%d", _address, rdata >> 4, rdata & 0x0F);
    p = 0.;
    for (int i = 0; i < 10; i++) {
        pascal_t tmp;
        doRead(tmp);
        pushAndPublish(tmp, Clock::getMillis());
        p += tmp;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    p = p / 10.;
    t = 0.;
    bool ok;
    t = readTemperature(ok);
    ESP_LOGI(FNAME, "SPL06_007 selftest, p=%f t=%f", p, t);

    if (p < 120000.f && p > 0.f) {
        ESP_LOGI(FNAME, "SPL06_007 selftest addr: %d PASSED, p=%f t=%f", _address, p, t);
        return true;
    } else {
        ESP_LOGI(FNAME, "SPL06_007 selftest addr: %d FAILED, p=%f t=%f", _address, p, t);
        return false;
    }
}

bool SPL06_007::doRead(pascal_t &val)
{
    int32_t *t_rawptr = nullptr;
    int32_t p_raw;
    
    if ( !(tick%10) ) { // read temperature every 10th pressure reading
        t_rawptr = &_traw;
    }
    if ( ! get_raw(p_raw, t_rawptr) ) {
        ESP_LOGW(FNAME, "Sensor reading failed");
        return false;
    }
    float praw_sc = p_raw / _scale_factor_p;
    float traw_sc = _traw / _scale_factor_t;

    val = c00 + praw_sc * (c10 + praw_sc * (c20 + praw_sc * c30)) 
                  + traw_sc * (c01 + praw_sc * (c11 + praw_sc * c21));

    tick++;
    return true;
}


bool SPL06_007::get_raw(int32_t& pval, int32_t *tvalptr) {
    uint8_t reg = 0x00;
    uint8_t data[6];
    if (i2c_master_transmit_receive(_dev, &reg, 1, data, tvalptr?6:3, 10) != ESP_OK) {
        ESP_LOGE(FNAME, "Error I2C read raw sensor data");
        return false;
    }

    pval = data[0] << 16 | data[1] << 8 | data[2];
    if (pval & (1 << 23)) {
        pval = pval | 0XFF000000;  // set left bits to one for 2's complement conversion of negative number
    }
    if (tvalptr) {
        *tvalptr = data[3] << 16 | data[4] << 8 | data[5];
        if (*tvalptr & (1 << 23)) {
            *tvalptr = *tvalptr | 0XFF000000; 
        }
    }

    return true;
}

