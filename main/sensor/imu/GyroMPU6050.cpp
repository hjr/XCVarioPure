/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "GyroMPU6050.h"

#include "ImuSensor.h"
#include "AccMPU6050.h"
#include "../SensorMgr.h"
#include "SetupNG.h"
#include "logdef.h"

#include "mpu/math.hpp"

GyroMPU6050 *gyroSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
static vector_f gyro_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ];

GyroMPU6050::GyroMPU6050(MpuImu &mmpu) :
    SensorTP<vector_f>(gyro_buffer, DUTY_CYCLE_MS),
    _my_mpu(mmpu),
    _scale(Units::deg_to_rad(mpud::gyroResolution(mpud::GYRO_FS_250DPS))) // scale factor for raw gyro data to rad/s
{
    _id = SensorId::GYRO_INERTIAL | SensorFlags::SENSOR_LOCAL;
    // push a single previous value
    pushAndPublish(vector_f(0,0,0), 0);
}
const char *GyroMPU6050::name() const { return _my_mpu.name(); }

bool GyroMPU6050::doRead(vector_f& val) {
    // Get new gyro values from MPU6050
    mpud::raw_axes_t imuRaw;
    if (_my_mpu.rotation(&imuRaw) == ESP_OK) {
        // raw data to rad/s
        vector_f tmpvec = vector_f::make_vector(imuRaw.x, imuRaw.y, imuRaw.z, _scale);

        // Check on irrational changes
        if ((tmpvec - *getHeadPtr()).get_norm2() > Units::deg_to_rad(30000.f)) {
            ESP_LOGE(FNAME, "gyro angle >300 deg/s in 0.1 sec");
            // return false;
        }

        // // Gating ignores Gyro drift < 1 deg per second (default)
        // float gate = gyro_gating.get();
        // nogate_gyro = ref_rot * tmpvec;
        // tmpvec.x = abs(tmpvec.x) < gate ? 0.0 : tmpvec.x;
        // tmpvec.y = abs(tmpvec.y) < gate ? 0.0 : tmpvec.y;
        // tmpvec.z = abs(tmpvec.z) < gate ? 0.0 : tmpvec.z;
        // into glider reference system
        val = _my_mpu.rotate(tmpvec);
        // ESP_LOGI(FNAME, "gyro raw: %d/%d/%d scaled: %f/%f/%f", imuRaw.x, imuRaw.y, imuRaw.z, val.x, val.y, val.z);
        _gyro_lpf_dwydt.filter((val.y - getHeadPtr()->y) / getDutyCycleS()); // diverenciate and filter to get dwy/dt for accelerometer compensation
        return true;
    }

    ESP_LOGE(FNAME, "gyro I2C error, X:%d Y:%d Z:%d", imuRaw.x, imuRaw.y, imuRaw.z);
    return false;
}


void GyroMPU6050::postProcess() {
    // calm status
    static bool rest_old = false;
    const vector_f& gyro = getHead();

    _processed = gyro - _bias_estimator.getBias(); // a corrected measurment
    rps_t gate = Units::deg_to_rad(gyro_gating.get());
    _processed.x = abs(_processed.x) < gate ? 0.0 : _processed.x;
    _processed.y = abs(_processed.y) < gate ? 0.0 : _processed.y;
    _processed.z = abs(_processed.z) < gate ? 0.0 : _processed.z;
    bool rest = detectRest() && accSensor->isResting();

    // feed the bias filter
    _bias_estimator.update(gyro, rest);
    vector_f bias = _bias_estimator.getBias();
    if ( rest != rest_old) {
        ESP_LOGI(FNAME, "rest state changed: %c -> %c", rest_old ? 'R' : 'M', rest ? 'R' : 'M');
        rest_old = rest;
    }
    if ( rest ) {
        // ESP_LOGI(FNAME, "gyro bEKF: %c - %f/%f/%f (%f/%f/%f)", rest ? 'R' : 'M', bias.x, bias.y, bias.z, _processed.x, _processed.y, _processed.z);
        if ( !airborne.get() // only on the ground
            && _restTimer > 30000 // only after a long rest (30sec)
            && bias.get_norm() > GyroMPU6050::GYRO_THRESHOLD // only push if bias is > threshold
            && _bias_update < 3 ) { // only push if we have a stable bias and are airborne
            pushGyroBias(bias);
            resetRest(); // to avoid multiple pushes in a row, only update bias once per rest phase
            _bias_update++;
        }
    }
}

void GyroMPU6050::resetRest() {
    _bias_estimator.reset();
    _restTimer = 0;
    _isResting = false;
}

// rest - detection
bool GyroMPU6050::detectRest() {
    // gyro variance
    vector_f gyrVar = getVariance(1000);
    
    // ESP_LOGI(FNAME, "rest detection: gyrVar=(%f, %f, %f) accDev=%f accNormLP=%f, dt=%f", gyrVar.x, gyrVar.y, gyrVar.z, accDev, accNormLP, dt);
    
    if (gyrVar.get_norm2() < GYRO_THRESHOLD2 && _processed.get_norm2() < GYRO_THRESHOLD * 2.f) {
         // min. 1.5–3 sec below threshold → consider as rest
        _restTimer += getDutyCycle();
        if ( _restTimer > 3000) {
            _isResting = true;
        }
    } else {
        _restTimer = 0;
        _isResting = false;
    }
    return _isResting;
}

void GyroMPU6050::pushGyroBias(vector_f& bias) {
    // rotate back into sensor frame
    // ESP_LOGI(FNAME, "-> bias: %f/%f/%f", bias.x, bias.y, bias.z);
    vector_f bias_sensor = _my_mpu._ref_rot.get_conjugate().rotate(bias);
    ESP_LOGI(FNAME, "backrot: %f/%f/%f", bias_sensor.x, bias_sensor.y, bias_sensor.z);

    // to raw units and scale to 1000 dps
    mpud::raw_axes_t measured;
    measured.x = bias_sensor.x / _scale / 4.f;
    measured.y = bias_sensor.y / _scale / 4.f;
    measured.z = bias_sensor.z / _scale / 4.f;
    ESP_LOGI(FNAME, "measured: %d/%d/%d", measured.x, measured.y, measured.z);

    // subtract the new bias from the current bias to get the new offset
    mpud::raw_axes_t current_bias = _my_mpu.getGyroOffset();
    ESP_LOGI(FNAME, "current gyro bias: %d/%d/%d", current_bias.x, current_bias.y, current_bias.z);
    // ESP_LOGI(FNAME, "delta gyro bias: %d/%d/%d", bias_delta.x, bias_delta.y, bias_delta.z);
    mpud::raw_axes_t new_bias = current_bias;
    new_bias -= measured;
    ESP_LOGI(FNAME, "new gyro bias: %d/%d/%d", new_bias.x, new_bias.y, new_bias.z);
    _my_mpu.setGyroOffset(new_bias);
    // save to nvs
    gyro_bias.set(axes_i16_abi(new_bias.x, new_bias.y, new_bias.z), false);
}