/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "GyroMPU6050.h"
#include "mpu/math.hpp"
#include "../SensorMgr.h"
#include "logdef.h"

GyroMPU6050::GyroMPU6050() : ImuSensor(SensorId::GYRO_INERTIAL | SensorId::LocalSensor)
{
    // push a single previous value
    pushAndPublish(vector_f(0,0,0), 0);
}

bool GyroMPU6050::doRead(vector_f& val) {
    // Get new gyro values from MPU6050
    mpud::raw_axes_t imuRaw;
    if (_MPUdev.rotation(&imuRaw) == ESP_OK) {
        // raw data to rad/s
        float scale = Units::deg_to_rad(mpud::gyroResolution(mpud::GYRO_FS_250DPS));
        vector_f tmpvec = vector_f::make_vector(imuRaw.x, imuRaw.y, imuRaw.z, scale);

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
        val = _ref_rot.rotate(tmpvec);
        return true;
    }

    ESP_LOGE(FNAME, "gyro I2C error, X:%d Y:%d Z:%d", imuRaw.x, imuRaw.y, imuRaw.z);
    return false;
}
