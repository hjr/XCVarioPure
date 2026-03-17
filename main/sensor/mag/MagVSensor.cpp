/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "MagVSensor.h"

#include "Units.h"
#include "sensor/SensorMgr.h"
#include "vector.h"
#include "protocol/Clock.h"
#include "sensor/imu/AccMPU6050.h"
#include "logdefnone.h"


constexpr int DUTY_CYCLE_MS = 500; // 2Hz
// provide a 10Hz sized buffer, but only process it with 2Hz
static vector_f mag_buffer[ (SENSOR_HISTORY_DURATION_MS / 100) + 1 ];

MagVSensor* magSensor = nullptr;

MagVSensor::MagVSensor() : SensorTP<vector_f>(mag_buffer, DUTY_CYCLE_MS),
    _lpf_heading(LowPassFilterT<float>::alphaFromTau(1.f, DUTY_CYCLE_MS / 1000.f))
{
    _id = SensorId::MAGNETO;
    _latency_ms = 20; // estimated latency of CAN sensor
}

MagVSensor* MagVSensor::createMagVSensor() {
    if ( !magSensor ) {
        magSensor = new MagVSensor();
        magSensor->setup();
    }
    return magSensor;
}

bool MagVSensor::setup() {
    loadCalibration();

    mag_hdm.setInvalid();
    mag_hdt.setInvalid();

    return true;
}

void MagVSensor::postProcess()
{
    if ( ! getHeadValid() ) {
        mag_hdm.setInvalid();
        mag_hdt.setInvalid();
        ESP_LOGI(FNAME, "mag reading not valid");
        return;
    }
    const vector_f mv = getAVG(800); // avg over the last ca. 0.5sec
    ESP_LOGI(FNAME, "mag reading AVG(0.5s): (%.2f,%.2f,%.2f)", mv.x, mv.y, mv.z);
    assert(accSensor);
    vector_f d = accSensor->getAttVector().get_normalized();  // a down vector, relative to glider
    _processed = mv - d * mv.dot(d); // a north vector
    // vector_f north = accSensor->getAHRSQuaternion().conjugate().rotate(mv); // works but is more expensive than the above calculation
    rad_t heading = Vector::polar(-_processed.y, _processed.x);  // the "-" translates into a ENU nav frame
    ESP_LOGI(FNAME, "mag (%.2f,%.2f,%.2f) heading %.2f", _processed.x, _processed.y, _processed.z, Units::rad_to_deg(heading));

    // publish the heading
    mag_hdm.set(_lpf_heading.filter(heading));
    mag_hdt.set(_lpf_heading.get() - Units::deg_to_rad(compass_declination.get()));
}

// this is called from the protocol handler when a new magnetometer reading is received from CAN
void MagVSensor::inject(int16_t x, int16_t y, int16_t z)
{
    vector_f mv;
    // rotate -90° around Z (the data sheet of the used QMC5883L sensor shows a wrong axes orientation)
    // the move to NED reference system does not need any other rotation anymore
    mv.x = -((float)y - _bias.y) * _scale.y;
    mv.y =  ((float)x - _bias.x) * _scale.x;
    mv.z =  ((float)z - _bias.z) * _scale.z;

    // backdate the timestamp to approximate the actual measurement time
    int time = Clock::getMillis() - _latency_ms;
    pushAndPublish(mv, time);
}



//////////////////////////////
// helpers
//////////////////////////////


// Loads a stored compass calibration. Returns true, if valid calibration
// data could be loaded, otherwise false.
bool MagVSensor::loadCalibration() {
    if (compass_calibrated.get() == 0) {
        ESP_LOGI(FNAME, "Compass calibration is not valid");
        // no stored calibration available
        return false;
    }

    _bias.x = compass_x_bias.get();
    _bias.y = compass_y_bias.get();
    _bias.z = compass_z_bias.get();
    _scale.x = compass_x_scale.get();
    _scale.y = compass_y_scale.get();
    _scale.z = compass_z_scale.get();

    ESP_LOGI(FNAME, "Read calibration: %f, %f, %f, %f, %f, %f, valid=%d", 
        _bias.x, _bias.y, _bias.z, _scale.x, _scale.y, _scale.z, compass_calibrated.get());

    return true;
}