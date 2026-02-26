/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "GyroMPU6050.h"

#include "ImuSensor.h"
#include "../SensorMgr.h"
#include "logdef.h"

#include "mpu/math.hpp"

GyroMPU6050 *gyroSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
static vector_f gyro_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ];

GyroMPU6050::GyroMPU6050(const MpuImu &mmpu) :
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
        _gyro_lpf_ayd.filter((val.y - getHeadPtr()->y) / getDutyCycleS());
        return true;
    }

    ESP_LOGE(FNAME, "gyro I2C error, X:%d Y:%d Z:%d", imuRaw.x, imuRaw.y, imuRaw.z);
    return false;
}


void GyroMPU6050::postProcess() {
    // calm status
    vector_f gyro = getHead();
    if (gyro.get_norm2() < Units::deg_to_rad(3.f * 3.f)) { // within 3 deg/s
        _calm_counter++;
    }
    else {
        _calm_counter = 0;
    }

    // feed the drift filter
}