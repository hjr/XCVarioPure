/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "AccMPU6050.h"
#include "KalmanMPU6050.h"
#include "mpu/math.hpp"
#include "../SensorMgr.h"
#include "../pressure/PressureSensor.h"
#include "logdef.h"

AccMPU6050::AccMPU6050() : ImuSensor(SensorId::ACC_INERTIAL | SensorId::LocalSensor)
{
    // push a single previous value
    pushToHistory(vector_f(1,0,0), 0);

    // Load 
}

bool AccMPU6050::doRead(vector_f& val) {
    // mpud::raw_axes_t a;
    // MPU.getAcceleration(&a);
    // val.x = static_cast<int16_t>(a.x / ACCEL_SCALE);
    // val.y = static_cast<int16_t>(a.y / ACCEL_SCALE);
    // val.z = static_cast<int16_t>(a.z / ACCEL_SCALE);
    // return true;

    // Get new accelerometer values from MPU6050
    mpud::raw_axes_t imuRaw;
    // fetch raw data from the registers
    if (_MPUdev.acceleration(&imuRaw) == ESP_OK) {
        mpud::float_axes_t tmp = mpud::math::accelGravity(imuRaw, mpud::ACCEL_FS_8G);  // raw data to gravity
        vector_f tmpvec(tmp.x, tmp.y, tmp.z);

        // Check on irrational changes
        if ((tmpvec - *getHeadPtr()).get_norm2() > 25) {
            // vector_f d(tmpvec-prev_accel);
            ESP_LOGE(FNAME, "accelaration change > 5 g in 0.1 sec");
            // return false;
        }
        val = _ref_rot * tmpvec;
        return true;
    }

    ESP_LOGE(FNAME, "accel I2C error, X:%d Y:%d Z:%d", imuRaw.x, imuRaw.y, imuRaw.z);
    return false;
}


// Call temp regulation
void AccMPU6050::postProcess() {
    static int count = 0;
    if ( hasHeatCtlr() && ! (++count%10) ) { // every second
        temp_control();
    }
    IMU::Process(); // fixme
}
