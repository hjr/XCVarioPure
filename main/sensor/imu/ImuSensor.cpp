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
#include "comm/CanBus.h"
#include "logdef.h"

#include "driver/ledc.h"

#include <string_view>
#include <algorithm> // for std::clamp

ImuSensor *accSensor = nullptr;
ImuSensor *gyroSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
static vector_f acc_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ];
static vector_f gyro_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ];
static const std::string_view TYPES[] = { "UNKNOWN", "MPU6050", "MPU6500", "ICM20602", "ICM20689" };

Quaternion ImuSensor::_ref_rot;

// ImuSensor* IMUSensor = nullptr;
mpud::MPU myMPU; // TODO as optional resource

// for heat control
struct PIController {
    const float Kp = 0.3f;
    const float Ki = 0.02f;
    const float dt = 1.0f; // seconds
    float I  = 0.0f;

    float update(float set, float meas) {
        float e = set - meas;
        float u = Kp * e + Ki * I;

        // anti-windup
        if ((u > 0.0f && u < 1.0f) || (e > 0))
            I += e * dt;

        I = std::clamp(I, -20.0f, 20.0f);
        return std::clamp(u, 0.0f, 1.0f);
    }
};


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

ImuSensor::~ImuSensor() {
    if ( _pictrl ) {
        clearpwm(); // ensure heating is off
        delete _pictrl;
    }
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

    // Check on heat control availability
    if ( CAN && !CAN->hasSlopeSupport() ) {
        initHeatCtrl();
    }

    // Load reference calibration
    Quaternion basic_ref = imu_reference.get();
    if (basic_ref == Quaternion()) {
        // If unset, set to a rough default
        basic_ref = setDefaultImuReference();
    }
    _ref_rot = concatGaaAndImuReference(glider_ground_aa.get(), basic_ref);

    return true;
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

ImuSensor::temp_status_t ImuSensor::getTempStatus() const {
    if( abs(_mpu_t_delta) < 0.5)
        return MPU_T_LOCKED;
    else if( _mpu_t_delta < -0.5 )
        return MPU_T_LOW;
    else if( _mpu_t_delta > 0.5 )
        return MPU_T_HIGH;
    else
        return MPU_T_UNKNOWN;
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


//
// PI control to regulate temperature
//

void ImuSensor::initHeatCtrl() {
    if (!_pictrl) {
        ESP_LOGI(FNAME, "Initialize AHRS heating PWM control");
        ledc_timer_config_t pwm_timer = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                         .duty_resolution = LEDC_TIMER_8_BIT,
                                         .timer_num = LEDC_TIMER_1,
                                         .freq_hz = 500,
                                         .clk_cfg = LEDC_AUTO_CLK,
                                         .deconfigure = false};
        ledc_channel_config_t pwm_ch = {.gpio_num = GPIO_NUM_2,
                                        .speed_mode = LEDC_LOW_SPEED_MODE,
                                        .channel = LEDC_CHANNEL_0,
                                        .intr_type = LEDC_INTR_DISABLE,
                                        .timer_sel = LEDC_TIMER_1,
                                        .duty = 0,
                                        .hpoint = 0,
                                        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
                                        .flags = {0}};
        ledc_timer_config(&pwm_timer);
        ledc_channel_config(&pwm_ch);
        _pictrl = new PIController();
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void ImuSensor::temp_control() {
    float mpu_target_temp = mpu_temperature.get();
    float temp = _MPUdev.getTemperature();
    _mpu_t_delta = temp - mpu_target_temp;
    unsigned pwm = (unsigned)(_pictrl->update(mpu_target_temp, temp) * 255.f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(FNAME, "MPU Temp Control: T=%.2f Target=%.2f PWM=%d", temp, mpu_target_temp, pwm);
}

void ImuSensor::clearpwm() {
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);  // heating off
}
