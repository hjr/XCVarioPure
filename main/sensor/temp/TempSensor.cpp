/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "sensor/temp/TempSensor.h"

TempSensor* OATSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 1000;
static float temp_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ];

TempSensor::TempSensor() : SensorTP<float>(temp_buffer, DUTY_CYCLE_MS)
{
    _id = SensorId::TEMPERATURE | SensorId::ExternalSensor;
    setNVSVar(&OAT);
    setFilter(new LowPassFilter(0.3f));
    OATSensor = this;
}