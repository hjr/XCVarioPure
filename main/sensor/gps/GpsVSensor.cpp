/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/
#include "GpsVSensor.h"


constexpr int DUTY_CYCLE_MS = 1000; // 1Hz Flarm update rate
static vector_f gps_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ];

GpsVSensor* GpsSensor = nullptr;

GpsVSensor::GpsVSensor() : SensorTP<vector_f>(gps_buffer, DUTY_CYCLE_MS)
{
    _id = SensorId::POSITION;
    _latency_ms = 750; // classical Flarm latency
}

GpsVSensor* GpsVSensor::createGpsVSensor() {
    if ( !GpsSensor ) {
        GpsSensor = new GpsVSensor();
    }
    return GpsSensor;
}

void GpsVSensor::inject(float lat, float lon)
{
    vector_f v;
    v.x = lat;
    v.y = lon;
    v.z = _alt;
    _alt = 0.f;
    int time = Clock::getMillis();
    pushToHistory(v, time);
    if ( _lat_ref == 0.f && _lon_ref == 0.f ) {
        _lat_ref = lat;
        _lon_ref = lon;
    }
}

void GpsVSensor::setExternalAltitude(float alt) {
    vector_f *last = getHeadPtr();
    if (last) {
        if (last->z == 0.f && (_last_update_time_ms + _update_interval_ms/2) > Clock::getMillis()) {
            last->z = alt;
        } else {
            _alt = alt; // for next injection
        }
    }
}
