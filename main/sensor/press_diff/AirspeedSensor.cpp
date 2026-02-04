/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "AirspeedSensor.h"

#include "mcph21.h"
#include "mp50040p.h"
#include "ms4525do.h"
#include "Atmosphere.h"
#include "../SensorMgr.h"
#include "setup/SetupNG.h"
#include "math/Floats.h"
#include "logdefnone.h"

#include <freertos/FreeRTOS.h>

AirspeedSensor *asSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
static float as_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ]; // history buffer for airspeed sensor

AirspeedSensor::AirspeedSensor() : SensorTP<float>(as_buffer, DUTY_CYCLE_MS)
{
    _id = SensorId::DIFFPRESSURE | SensorId::LocalSensor;
    setNVSVar(&dynp);
    setFilter(new LowPassFilter(0.25f));
}

static AirspeedSensor* factory(AirspeedSensor::ASens_Type type)
{
    AirspeedSensor* tmp = nullptr;
    switch (type) {
    case AirspeedSensor::PS_ABPMRR:
        tmp = new MS4525DO(true);
        break;
    case AirspeedSensor::PS_TE4525:
        tmp = new MS4525DO();
        break;
    case AirspeedSensor::PS_MP3V5004:
        tmp = new MP5004DP();
        break;
    case AirspeedSensor::PS_MCPH21:
        tmp = new MCPH21();
        break;
    default:
        ESP_LOGI(FNAME, "Not supported sensor");
        break;
    }
    if ( tmp ) {
        ESP_LOGI(FNAME, "%s created", tmp->name());
    }
    return tmp;
}

AirspeedSensor* AirspeedSensor::autoSetup()
{
    ESP_LOGI(FNAME, "Airspeed sensor init..  type configured: %d", airspeed_sensor.get());
    AirspeedSensor *as_sens = nullptr;
    if (airspeed_sensor.get() != PS_NONE)
    {
        as_sens = factory((ASens_Type)airspeed_sensor.get());

        // there is a configured sensor
        ESP_LOGI(FNAME, "There is valid config for airspeed sensor: check this one first...");
        if (!as_sens->probe()) {
            delete as_sens;
            as_sens = nullptr;
        }
    }

    // Probe any kind of ever known sensors
    if (!as_sens)
    {
        ESP_LOGI(FNAME, "Configured AS sensor not found, probing all known types...");
        // behaves same as above, so we can't detect this, needs to be setup in factory
        for ( ASens_Type t = PS_ABPMRR; t < PS_MAX_TYPES; t = static_cast<ASens_Type>(t + 1) ) {
            as_sens = factory(t);
            ESP_LOGI(FNAME, "Try %s", as_sens->name());
            if ( as_sens && as_sens->probe() ) {
                airspeed_sensor.set( t );
                break;
            }
            else {
                ESP_LOGI(FNAME, "Sensor not found");
                delete as_sens;
                as_sens = nullptr;
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }

    return as_sens;
}

bool AirspeedSensor::setup()
{
    ESP_LOGI(FNAME, "Airspeed sensor %s", name());

    _offset = as_offset.get();
    if (_offset < 0) {
        ESP_LOGI(FNAME, "offset not yet done: need to recalibrate");
    }
    else {
        ESP_LOGI(FNAME, "offset from NVS: %d", (int)_offset);
    }

    int32_t adcval = 0;
    uint16_t temp;
    fetch_pressure(adcval, temp);

    ESP_LOGI(FNAME, "offset from ADC %ld", adcval);

    bool plausible = offsetPlausible(adcval);
    if (plausible) {
        ESP_LOGI(FNAME, "offset from ADC is plausible");
    }
    else {
        ESP_LOGI(FNAME, "offset from ADC is NOT plausible");
    }

    bool deviation_ok = std::abs(adcval - _offset) < getMaxACOffset();
    if (deviation_ok) {
        ESP_LOGI(FNAME, "Deviation in bounds");
    }
    else {
        ESP_LOGI(FNAME, "Deviation out of bounds");
    }

    // Long term stability of Sensor as from datasheet 0.5% per year -> 4000 * 0.005 = 20
    if (_offset < 0 || (plausible && deviation_ok))
    {
        ESP_LOGI(FNAME, "Airspeed OFFSET correction ongoing, calculate new _offset...");
        int32_t rawOffset = 0;
        int samples = 0;
        for (int i = 0; i < 100; i++)
        {
            if ( fetch_pressure(adcval, temp) ) {
                samples++;
                rawOffset += adcval;
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        _offset = fast_iroundf(float(rawOffset) / samples);
        if (offsetPlausible(_offset)) {
            ESP_LOGI(FNAME, "Offset procedure finished, offset: %d", (int)_offset);
            as_offset.set(_offset);
        }
        else {
            ESP_LOGW(FNAME, "Offset out of tolerance, ignore odd offset value");
        }
    }
    else
    {
        ESP_LOGI(FNAME, "No new Calibration: flying with plausible pressure");
    }
    return true;
}

bool AirspeedSensor::doRead(float &val)
{
    int32_t p_raw;
    uint16_t t_dat;
    bool ok = fetch_pressure(p_raw, t_dat);
    if (!ok)
    {
        ESP_LOGE(FNAME, "Retry measure, status :%d  p=%ld", ok, p_raw);
        ok = fetch_pressure(p_raw, t_dat);
        if (!ok)
        {
            ESP_LOGE(FNAME, "Warning, status :%d  p=%ld, bad even retry", ok, p_raw);
            val = NAN;
            return false;
        }
    }
    int raw_diff = p_raw - _offset;
    float tmpv = static_cast<float>(raw_diff) * _multiplier;
    if (tmpv < 0.f) {
        val = 0.f;
        return true;
    }
    val = tmpv;
    ESP_LOGI(FNAME,"P:%f offset:%d raw:%d  raw-off:%d m:%f T:%u", val, (int)_offset, (int)p_raw, raw_diff, _multiplier, (unsigned)t_dat);
    return true;
}

