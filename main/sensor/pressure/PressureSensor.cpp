/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "PressureSensor.h"

#include "bme280_spi.h"
#include "spl06_007.h"
#include "Atmosphere.h"
#include "../SensorMgr.h"
#include "setup/SetupNG.h"
#include "math/Floats.h"
#include "logdef.h"

#include <I2Cbus.hpp>

#include <freertos/FreeRTOS.h>


#define SPL06_007_BARO 0x77
#define SPL06_007_TE   0x76

constexpr int DUTY_CYCLE_MS = 100; // 100ms cycle time for pressure sensors
static float pstat_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 4 ];
static float te_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 4 ];

PressureSensor::PressureSensor(SensorId id) : SensorTP<float>((id == SensorId::STATIC_PRESSURE) ? pstat_buffer : te_buffer, DUTY_CYCLE_MS, id)
{
    if (id == SensorId::STATIC_PRESSURE) {
        setNVSVar(&altitude);
        setFilter(new AltimeterFilter());
    }
    // else {
    //     setNVSVar(&te_vario);
    //     setFilter(new TEVariometerFilter());
    // }
}

float PressureSensor::readAltitude(float qnh, bool& success) {
    success = getValid();
    return Atmosphere::calcAltitude( qnh, getHead() );
}

static PressureSensor* factory(PressureSensor::PSens_Type type, SensorId id)
{
    PressureSensor* ret = nullptr;
    switch (type) {
    case PressureSensor::SPL06_007:
    {
        SPL06_007 *tmp = new SPL06_007(id);
        ret = tmp;
        break;
    }
    case PressureSensor::BME280_SPI:
        ret = new BME280_SPI(id);
        break;
    default:
        ESP_LOGI(FNAME, "Not supported sensor");
        break;
    }
    return ret;
}

PressureSensor* PressureSensor::autoSetup(SensorId id) {
    PressureSensor* p_sens = nullptr;
    // Probe any kind of ever known sensors
    for ( PSens_Type t = SPL06_007; t < PS_MAX_TYPES; t = static_cast<PSens_Type>(t + 1) ) {
        p_sens = factory(t, id);
        if ( p_sens && p_sens->probe() ) {
            ESP_LOGI(FNAME, "Found %s as sensor %d", p_sens->name(), id);
            p_sens->setup();
            break;
        }
        else {
            delete p_sens;
            p_sens = nullptr;
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    if ( ! p_sens ) {
        ESP_LOGW(FNAME, "No sensor not found for id %d", id);
    }

    return p_sens;
}
