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
#include "../SensorMgr.h"
#include "../Filters.h"
#include "setup/SetupNG.h"
#include "S2F.h"
#include "math/Floats.h"
#include "logdefnone.h"

#include <freertos/FreeRTOS.h>

AirspeedSensor *asSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
static float as_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ]; // history buffer for airspeed sensor

AirspeedSensor::AirspeedSensor() :
    SensorTP<float>(as_buffer, DUTY_CYCLE_MS),
    _dynp_lpf(0.25f)
{
    _id = SensorId::DIFFPRESSURE | SensorFlags::SENSOR_LOCAL;
    setNVSVar(&dynp);
    setFilter(&_dynp_lpf);
}

static AirspeedSensor* factory(AirspeedSensor::ASens_Type type)
{
    AirspeedSensor* tmp = nullptr;
    switch (type) {
    case AirspeedSensor::ABPMRR:
        tmp = new MS4525DO(); // Only the multiplier has different sign, alias swapped tubes
        break;
    case AirspeedSensor::TE4525:
        tmp = new MS4525DO();
        break;
    case AirspeedSensor::MP3V5004:
        tmp = new MP5004DP();
        break;
    case AirspeedSensor::MCPH21:
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
    if (airspeed_sensor.get() != AirspeedSensor::NONE)
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
        for ( ASens_Type t = ABPMRR; t < MAX_TYPES; t = static_cast<ASens_Type>(t + 1) ) {
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
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    return as_sens;
}

bool AirspeedSensor::setup()
{
    ESP_LOGI(FNAME, "Airspeed sensor %s", name());

    _counter = 0; // start a series of auto offset corrections
    if (as_offset.get() > 0.) {
        _offset = as_offset.get();
        printf("AS offset from NVS: %d\n", (int)_offset);
        return true;
    }
    ESP_LOGI(FNAME, "offset not yet done: init auto calibration");

    bool plausible = false;
    int trials = 10;
    while ( !plausible && trials-- > 0) {
        int32_t adcval = 0;
        uint16_t temp;
        if ( fetch_pressure(adcval, temp) ) {
            plausible = offsetPlausible(adcval);
            ESP_LOGI(FNAME, "AS from ADC %ld, plausible=%d", adcval, plausible);
            if ( plausible ) {
                _offset = adcval;
                printf("AS initial offset from sensor: %d\n", (int)_offset);
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return plausible;
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
    // ESP_LOGI(FNAME,"P:%f offset:%d raw:%d  raw-off:%d m:%f T:%u", val, (int)_offset, (int)p_raw, raw_diff, _multiplier, (unsigned)t_dat);
    return true;
}

void AirspeedSensor::postProcess()
{
    // rest detection and potential reset of the offset
    // ESP_LOGI(FNAME, "dynp: %f, integ: %f < %f", getHead(), getIntegral(1000), DYNP_THRESHOLD);
    bool rest_old = _isResting;
    if (std::fabsf(getHead()) < DYNP_THRESHOLD && std::fabsf(getIntegral(1000)) < DYNP_THRESHOLD) {
         // min. 3 sec below threshold, consider as rest
        _restTimer += getDutyCycle();
        if ( _restTimer > 3000) {
            _isResting = true;
        }
    } else {
        _restTimer = 0;
        _isResting = false;
    }
    
    if ( _isResting != rest_old) {
        ESP_LOGI(FNAME, "rest state changed: %c -> %c", rest_old ? 'R' : 'F', _isResting ? 'R' : 'F');
    }

    // airborne status update
    if ( !(_counter++ % 5) ) {
        if (!airborne.get() && (ias.get() > Speed2Fly.getStallSpeed())) {
            airborne.set(true);
            ESP_LOGI(FNAME, "Airborne detected by airspeed sensor");
        } else if (airborne.get() && _isResting) {
            airborne.set(false);
            ESP_LOGI(FNAME, "Landed detected by airspeed sensor");
        }
    }

    if (_counter < 3001) {
        if ( !(_counter%50) ) {
            // check every 5 seconds for a rest phase and do an auto offset correction
            ESP_LOGI(FNAME,"AS check for offset calib");
            if (_isResting) {
                float avg = getAVG(1000);
                _offset = fast_iroundf((float(3 * _offset) + avg / getMultiplier()) / 3.f);
                printf("AS new offset calib %ld\n", _offset);
            }
            if (_counter == 3000 && offsetPlausible(_offset)) {
                as_offset.set(_offset);
                printf("AS offset correction applied, new offset: %d\n", (int)_offset);
            }
        }
    }
}