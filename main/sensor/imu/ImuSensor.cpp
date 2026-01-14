/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ImuSensor.h"
#include "../SensorMgr.h"
#include "math/Trigonometry.h"
#include "setup/SetupNG.h"

#include "logdef.h"

#include <string_view>

ImuSensor *accSensor = nullptr;
ImuSensor *gyroSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
static vector_f acc_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ];
static vector_f gyro_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ];
static const std::string_view TYPES[] = { "UNKNOWN", "MPU6050", "MPU6500", "ICM20602", "ICM20689" };

Quaternion ImuSensor::_ref_rot;

// ImuSensor* IMUSensor = nullptr;
mpud::MPU myMPU; // TODO as optional resource


ImuSensor::ImuSensor(SensorId id) : SensorTP<vector_f>((id == SensorId::ACC_INERTIAL) ? acc_buffer : gyro_buffer, DUTY_CYCLE_MS),
    _MPUdev(myMPU)
{
    _id = id;
    if ( id == SensorId::GYRO_INERTIAL ) {
        // set long term zero tracker filter
    }
    else {
        // set the MPU temperature regulation filter
    }
    
    // _invalid = vector_f(NAN, NAN, NAN);
}

const char *ImuSensor::name() const {
    return TYPES[static_cast<int>(_who_typ)].data();
}

bool ImuSensor::probe() {
    // probe on MPU
	myMPU.setBus(i2c1);  // set communication bus
	myMPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  // set address
	if (myMPU.reset() != ESP_OK) {
		ESP_LOGI( FNAME,"MPU not avail");
        return false;
	}
    vTaskDelay(pdMS_TO_TICKS(100)); // wait after reset
    _who_typ = getImuId();
    if (_who_typ != ImuType::UNKNOWN) {
        ESP_LOGI(FNAME, "found %s", name());
        myMPU.clearpwm(); // Stop MPU heating, before we know the hardware details
        return true;
    }
    return false;
}

bool ImuSensor::setup() {
    ESP_LOGI(FNAME, "MPU initialize");
    myMPU.initialize();       // this will initialize the chip and set default configurations
    myMPU.setSampleRate(50);  // in (Hz)
    myMPU.setAccelFullScale(mpud::ACCEL_FS_8G);
    myMPU.setGyroFullScale(mpud::GYRO_FS_250DPS);
    myMPU.setDigitalLowPassFilter(mpud::DLPF_5HZ);  // smoother data
    mpud::raw_axes_t gb = gyro_bias.get();
    mpud::raw_axes_t ab = accl_bias.get();
    if (gb.isZero() && ab.isZero()) {
        ESP_LOGI(FNAME, "MPU computeOffsets");
        myMPU.computeOffsets(&ab, &gb);  // returns Offsets in 16G scale
        accl_bias.set(ab);
        gyro_bias.set(gb);
        myMPU.setGyroOffset(gb);
        ESP_LOGI(FNAME, "MPU new offsets accl:%d/%d/%d gyro:%d/%d/%d ZERO:%d", ab.x, ab.y, ab.z, gb.x, gb.y, gb.z, gb.isZero());
    } else {
        myMPU.setAccelOffset(ab);
        myMPU.setGyroOffset(gb);
    }
    ESP_LOGI(FNAME, "MPU current offsets accl:%d/%d/%d gyro:%d/%d/%d ZERO:%d", ab.x, ab.y, ab.z, gb.x, gb.y, gb.z, gb.isZero());

    // Load reference calibration
    Quaternion basic_ref = imu_reference.get();
    if (basic_ref == Quaternion()) {
        // If unset, set to a rough default
        basic_ref = setDefaultImuReference();
    }
    _ref_rot = concatGaaAndImuReference(glider_ground_aa.get(), basic_ref);

    return true;
}

void ImuSensor::initHeatCtrl() {
    _MPUdev.pwm_init(mpu_temperature.get());
}

ImuType ImuSensor::getImuId() {
    switch (myMPU.whoAmI()) {
    case 0x68: return ImuType::MPU6050;
    case 0x70: return ImuType::MPU6500;
    case 0x12: return ImuType::ICM20602;
    case 0x98: return ImuType::ICM20689;
    default:   return ImuType::UNKNOWN;
    }
}

// Setup the rotation for the "upright", "topdown" and "ninety" vario mounting positions
Quaternion ImuSensor::setDefaultImuReference()
{
	// Revert from calibrated IMU to default mapping, which fits
	// roughly to an upright or top down installation.
	Quaternion accelDefaultRef = Quaternion(deg2rad(90.0f), vector_f(0,1,0)).get_conjugate();

	if ( display_orientation.get() == DISPLAY_TOPDOWN ) {
		accelDefaultRef = Quaternion(deg2rad(180.0f), vector_f(1,0,0)) * accelDefaultRef;
	}
	else if ( display_orientation.get() == DISPLAY_NINETY ) {
		accelDefaultRef = Quaternion(deg2rad(-90.0f), vector_f(1,0,0)) * accelDefaultRef;
	}
	imu_reference.set(accelDefaultRef, false); // nvs
	// imu_reference.commit(); should not be needed
    return accelDefaultRef;
}

// Concatenation of ground angle of attack and the basic reference calibration rotation
Quaternion ImuSensor::concatGaaAndImuReference(const float gAA, const Quaternion& basic)
{
	Quaternion rot = Quaternion(deg2rad(gAA), vector_f(0,1,0)) * basic; // rotate positive around Y
	return rot.normalize();
}
