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

TempSensor::TempSensor() :
    SensorTP<float>(temp_buffer, DUTY_CYCLE_MS),
    _lpf(0.5f)
{
    _id = SensorId::TEMPERATURE;
    _latency_ms = 750;      // 0.75 seconds
    _valid_time_ms = 10000; // 10 seconds
    setNVSVar(&OAT);
    _lpf.reset(Units::C_to_K(15.f)); // initial guess for OAT
    setFilter(&_lpf);
    OATSensor = this;
}