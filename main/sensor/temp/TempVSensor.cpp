/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "TempVSensor.h"

TempVSensor* oatSensor = nullptr;

constexpr int SENSOR_HISTORY_DURATION_MS = 10000;  // milliseconds
constexpr int DUTY_CYCLE_MS = 1000;
constexpr size_t HSIZE = SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) float temp_buffer[ HSIZE + 1 ];

TempVSensor::TempVSensor() :
    SensorTP<float>(temp_buffer, HSIZE, DUTY_CYCLE_MS),
    _lpf(0.5f)
{
    _id = SensorId::TEMPERATURE;
    _latency_ms = 750;      // 0.75 seconds
    _valid_time_ms = 10000; // 10 seconds
    setNVSVar(&OAT);
    _lpf.reset(Units::C_to_K(15.f)); // initial guess for OAT
    setFilter(&_lpf);
    oatSensor = this;
}