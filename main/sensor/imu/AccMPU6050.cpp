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
#include "logdefnone.h"

AccMPU6050::AccMPU6050() : ImuSensor(SensorId::ACC_INERTIAL | SensorId::LocalSensor)
{
    // push a single previous value
    pushAndPublish(vector_f(1,0,0), 0);

    // Load 
}

bool AccMPU6050::doRead(vector_f& val) {

    // Get new accelerometer values from MPU6050
    mpud::raw_axes_t imuRaw;
    // fetch raw data from the registers
    if (_MPUdev.acceleration(&imuRaw) == ESP_OK) {
        // raw data to gravity
        vector_f tmp_ned = vector_f::make_vector(imuRaw.x, imuRaw.y, imuRaw.z, -mpud::math::accelResolution(mpud::ACCEL_FS_8G));
        // ESP_LOGI(FNAME, "accel X:%d Y:%d Z:%d", imuRaw.x, imuRaw.y, imuRaw.z);
        // ESP_LOGI(FNAME, "tmpvec X:%f Y:%f Z:%f", tmp_ned.x, tmp_ned.y, tmp_ned.z);

        // Check on irrational changes
        if ((tmp_ned - *getHeadPtr()).get_norm2() > 25) {
            // vector_f d(tmpvec-prev_accel);
            ESP_LOGE(FNAME, "accelaration change > 5 g in 0.1 sec");
            // return false;
        }
        val = _ref_rot.rotate(tmp_ned);
        ESP_LOGI(FNAME, "val X:%f Y:%f Z:%f", val.x, val.y, val.z);
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
