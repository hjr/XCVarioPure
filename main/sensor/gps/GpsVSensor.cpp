/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/
#include "GpsVSensor.h"

#include "math/Floats.h"
#include "math/Trigonometry.h"
#include "math/Units.h"
#include "wind/Wind.h"
#include "vector.h"


#include "logdef.h"

constexpr int SENSOR_HISTORY_DURATION_MS = 10000;  // 10 sec
constexpr int DUTY_CYCLE_MS = 1000; // 1Hz Flarm update rate
constexpr size_t HSIZE = SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) vector_f gps_buffer[ HSIZE + 1 ];

GpsVSensor* gpsSensor = nullptr;


GpsVSensor::GpsVSensor() : SensorTP<vector_f>(gps_buffer, HSIZE, DUTY_CYCLE_MS)
{
    _id = SensorId::POSITION;
    _latency_ms = 750;      // classical Flarm latency
    _valid_time_ms = 5000;  // 5 seconds
}

GpsVSensor* GpsVSensor::createGpsVSensor() {
    if ( !gpsSensor ) {
        gpsSensor = new GpsVSensor();
    }
    return gpsSensor;
}

void GpsVSensor::inject(float lat, float lon, mps_t gndSpeed, rad_t gndCourse)
{
    // only the position goes into the sensor history
    vector_f v;
    v.x = lat;
    v.y = lon;
    v.z = _alt;
    _alt = 0.f;
    int time = Clock::getMillis();
    pushAndPublish(v, time);
    if ( _lat_ref == 0.f && _lon_ref == 0.f ) {
        _lat_ref = lat;
        _lon_ref = lon;
    }
    // publish current speed an course through blackboard for other consumers, e.g. wind calc
    gnd_speed.set(gndSpeed);
    gnd_course.set(gndCourse);
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


void GpsVSensor::postProcess() {
    // check if we have a valid fix
    if ( ! getHeadValid() ) {
        // invalidate derived black board values
        gnd_speed.setInvalid();
        gnd_course.setInvalid();
        heading_tru.setInvalid();
        heading_wca.set(0.f);
        return;
    }

    // calculate an omega when we have at lease 3 valid points in the history
    if ( _history.level() >= 3 ) {
        Vector vec(getHead() - getIdx(1));
        float d = _prev_diff.dot(vec);
        float c = _prev_diff.cross(vec);
        constexpr float one_div_by_dt = 1.f / (DUTY_CYCLE_MS / 1000.f);
        _omega = atan2f(c, d) * one_div_by_dt;
        _prev_diff = vec;
    }

    // check on avg wind
    rad_t track = gnd_course.get();
    if( ! tas.getValid() || ! synoptic_wind.getValid() ) {
        heading_tru.set(track); // fall back
        heading_wca.set(0.f);
        return;
    }

    // calc heading (incl wind)
    mps_t groundspeed = gnd_speed.get();
    WindData wind = static_cast<WindData>(synoptic_wind.get());

    // wind vector
    vector_f wind_vec = wind.getVector();

    // ground vector
    vector_f ground_vec = {groundspeed * fast_cos_rad(track), groundspeed * fast_sin_rad(track), .0f};

    // air vector = ground - wind
    vector_f air_vec = ground_vec - wind_vec;
    ESP_LOGI(FNAME, "wvec: (%.1f, %.1f) gvec: (%.1f, %.1f) -> avec: (%.1f, %.1f)", 
        Units::mps_to_kmh(wind_vec.x), Units::mps_to_kmh(wind_vec.y), 
        Units::mps_to_kmh(ground_vec.x), Units::mps_to_kmh(ground_vec.y), 
        Units::mps_to_kmh(air_vec.x), Units::mps_to_kmh(air_vec.y));

    // and for plausible airspeed result to prevent wild heading indications
    float true_airspeed = tas.get();
    if (!floatEqualFastAbs(air_vec.get_norm(), true_airspeed, 0.1f * true_airspeed)) {
        heading_tru.set(track); // fallback to track if airspeed calculation seems off
        ESP_LOGW(FNAME, "postProcess: airspeed mismatch: air_vec=%.1f vs TAS=%.1f, fallback to track", 
            Units::mps_to_kmh(air_vec.get_norm()), Units::mps_to_kmh(true_airspeed));
        return;
    }
    float headg = Vector::polar(air_vec.y, air_vec.x);
    ESP_LOGI(FNAME, "computeHeading: track=%.1f groundspeed=%.1f tas=%.1f wind=%d@%.1f -> head=%.1f airspeed=%.1f wca=%.1f", 
        Units::rad_to_deg(track), Units::mps_to_kmh(groundspeed), Units::mps_to_kmh(true_airspeed), wind.getDeg(), 
        Units::mps_to_kmh(wind.getVal()), Units::rad_to_deg(headg), Units::mps_to_kmh(air_vec.get_norm()), 
        Units::rad_to_deg(Vector::angleDiff(headg, track)));
    heading_tru.set(headg);
    heading_wca.set(Vector::angleDiff(headg, track));

}

