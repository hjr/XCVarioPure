/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ImuSensor.h"

#include "math/Quaternion.h"
#include "math/Trigonometry.h"
#include "math/Floats.h"
#include "setup/SetupNG.h"
#include "GyroMPU6050.h"
#include "AccMPU6050.h"
#include "comm/CanBus.h"
#include "simplex.h"
#include "logdef.h"

#include <driver/ledc.h>

#include <string_view>
#include <algorithm> // for std::clamp


static const std::string_view TYPES[] = { "UNKNOWN", "MPU6050", "MPU6500", "ICM20602", "ICM20689" };


mpud::MPU myMPU; // TODO as optional resource

// for heat control
struct PIController {
    const float Kp = 0.3f;
    const float Ki = 0.033f;
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


MpuImu::MpuImu() :
    _MPUdev(myMPU)
{
}

MpuImu::~MpuImu() {
    if ( _pictrl ) {
        clearpwm(); // ensure heating is off
        delete _pictrl;
    }
}

const char *MpuImu::name() const {
    return TYPES[static_cast<int>(_who_typ)].data();
}

bool MpuImu::probe() {
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

bool MpuImu::setup() {
    ESP_LOGI(FNAME, "MPU initialize");
    esp_err_t err = myMPU.reset();
    err |= myMPU.resetFIFO();
    err |= myMPU.initialize(_who_typ == ImuType::ICM20602);       // this will initialize the chip and set default configurations
    // err |= myMPU.setSampleRate(50);  // in (Hz)
    // err |= myMPU.setAccelFullScale(MpuImu::ACCEL_SCALE);
    // err |= myMPU.setGyroFullScale(MpuImu::GYRO_SCALE);
    // err |= myMPU.setDigitalLowPassFilter(mpud::DLPF_5HZ);  // smoother data
    axes_i16_abi tmp = gyro_bias.get(); // will get refined on Rest condition while on the ground
    err |= myMPU.setGyroOffset(mpud::raw_axes_t(tmp.x, tmp.y, tmp.z));
    ESP_LOGI(FNAME, "MPU current gyro bias: %d/%d/%d", tmp.x, tmp.y, tmp.z);
    tmp = accl_bias.get();
    err |= myMPU.setAccelOffset(mpud::raw_axes_t(tmp.x, tmp.y, tmp.z));
    ESP_LOGI(FNAME, "MPU current accel bias: %d/%d/%d", tmp.x, tmp.y, tmp.z);

    // Check on heat control availability
    if ( CAN && !CAN->hasSlopeSupport() ) {
        initHeatCtrl();
    }

    // Load reference calibration
    Quaternion basic_ref = imu_reference.get();
    _ref_rot = concatGaaAndImuReference(glider_ground_aa.get(), basic_ref);
    if (basic_ref == Quaternion()) {
        // If unset, set the default, but do not apply the GAA (!)
        _ref_rot = getDefaultImuReference();
    }

    return err == ESP_OK;
}

ImuType MpuImu::getImuId() {
    switch (myMPU.whoAmI()) {
    case 0x68: return ImuType::MPU6050;
    case 0x70: return ImuType::MPU6500;
    case 0x12: return ImuType::ICM20602;
    case 0x98: return ImuType::ICM20689;
    default:   return ImuType::UNKNOWN;
    }
}

temp_status_t MpuImu::getTempStatus() const {
    if( abs(_mpu_t_delta) < 0.5) {
        return temp_status_t::MPU_T_LOCKED;
    } else if( _mpu_t_delta < -0.5 ) {
        return temp_status_t::MPU_T_LOW;
    } else if( _mpu_t_delta > 0.5 ) {
        return temp_status_t::MPU_T_HIGH;
    } else {
        return temp_status_t::MPU_T_UNKNOWN;
    }
}

//
// IMU reference
//

// Calc the rotation for the "upright", "topdown" and "ninety" vario mounting positions
Quaternion MpuImu::getDefaultImuReference() {

    // Revert from calibrated IMU to default mapping, which fits
    // roughly to an upright, top down, or ninety degree installation.
    // IMU on PCB raw: X up, Y left, Z backwards
    // IMU on PCB to "NED" reference: X forward, Y right, Z down
    Quaternion accelDefaultRef = Quaternion(deg2rad(-90.0f), vector_f(0, 1, 0)); // towards "NED"

    if (display_orientation.get() == DISPLAY_NORMAL) {
        accelDefaultRef = Quaternion(deg2rad(180.0f), vector_f(1, 0, 0)) * accelDefaultRef;
    } else if (display_orientation.get() == DISPLAY_NINETY) {
        accelDefaultRef = Quaternion(deg2rad(90.0f), vector_f(1, 0, 0)) * accelDefaultRef;
    }
    return accelDefaultRef;
}

void MpuImu::resetImuReference(bool save_nvs) {
    Quaternion base = getDefaultImuReference();
    applyImuReference(0.f, base); // do not add the GAA onto the default reference
    if (save_nvs) {
        imu_reference.set(Quaternion(), false); // nvs
    }
}

void MpuImu::zeroGyroBias() {
    gyro_bias.set({});
    myMPU.setGyroOffset({});
}

void MpuImu::zeroAccBias() {
    // never get this into the hands of a user. This is only for development and factory setup purposes.
    accl_bias.set({});
    myMPU.setAccelOffset({});
}

// Concatenation of ground angle of attack and the basic reference calibration rotation
Quaternion MpuImu::concatGaaAndImuReference(const degree_t gAA, const Quaternion& basic) {
    Quaternion rot = Quaternion(deg2rad(-gAA), vector_f(0, 1, 0)) * basic;  // rotate positive around Y
    return rot.normalize();
}

// Callback for the two/three vector samples needed for the reference calibration
// Returns progress in case of success
// 0 := not yet started
// 1 := right wing completed
// 2 := left wing completed
// 3 := level bob, and calibration completed
// else returns -1
int MpuImu::getAccelSamplesAndCalib(vector_f gyro_integral, rad_t& wing_angle, rad_t& ground_angle) {
    vector_f* bob = &bob_level;
    int16_t side = 0;
    ESP_LOGI(FNAME, "gyro integral: %f/%f/%f n:%f", gyro_integral.x, gyro_integral.y, gyro_integral.z, gyro_integral.get_norm());
    if ( progress == 3 ) {
        progress = 4;
    }
    else if (gyro_integral.x > 0.) {
        bob = &bob_right_wing;
        gyro_axis_right = gyro_integral;
        side = 1;
        ESP_LOGI(FNAME, "Right wing down");
    } else {
        bob = &bob_left_wing;
        gyro_axis_left = gyro_integral;
        side = 2;
        ESP_LOGI(FNAME, "Left wing down");
    }

    float var_norm2 = 2.;
    while (var_norm2 > GyroMPU6050::GYRO_THRESHOLD2) {
        vTaskDelay(pdMS_TO_TICKS(200));
        var_norm2 = gyroSensor->getVariance(2000).get_norm2();
        ESP_LOGI(FNAME, "Waiting for calm to sample accel, var=%f", var_norm2);
    }
    // pick average samples
    *bob = accSensor->getAVG(2000);

    ESP_LOGI(FNAME, "wing down bob: %f/%f/%f", bob->x, bob->y, bob->z);
    progress |= side;  // accumulate progress
    if (progress == 4) {
        // Calculate the rotation to the glieder reference from the two measurments
        ESP_LOGI(FNAME, "BobR:\t%f\t%f\t%f \tL%.2f", bob_right_wing.x, bob_right_wing.y, bob_right_wing.z, bob_right_wing.get_norm());
        ESP_LOGI(FNAME, "BobL:\t%f\t%f\t%f \tL%.2f", bob_left_wing.x, bob_left_wing.y, bob_left_wing.z, bob_left_wing.get_norm());
        ESP_LOGI(FNAME, "level:\t%f\t%f\t%f \tL%.2f", bob_level.x, bob_level.y, bob_level.z, bob_level.get_norm());

        // Check on wing angle is at least 4 degree
        wing_angle = Quaternion::AlignVectors(vector_f(bob_right_wing.x, bob_right_wing.y, bob_right_wing.z),
                                                vector_f(bob_left_wing.x, bob_left_wing.y, bob_left_wing.z)).getAngle();
        ESP_LOGI(FNAME, "Wing Angle: %f degree.", rad2deg(wing_angle / 2.));
        if (wing_angle < deg2rad(8.f)
            || gyro_axis_right.get_norm() < deg2rad(8.f)
            || gyro_axis_left.get_norm() < deg2rad(8.f)) {
            progress = 0;  // resert the progress
            return -1;
        }

        // A vector from skid touch points towards the main wheel touch point
        // The X in glider reference (points towards the nose)
        vector_f X = bob_right_wing.cross(bob_left_wing);  // Br x Bl
        X.normalize();
        ESP_LOGI(FNAME, "X: %f,%f,%f", X.x, X.y, X.z);
        // The Z in NED glider reference (points down)
        // The bob in glider ref, skid stil on the ground (points up)
        vector_f Z_plane_normal = bob_left_wing.cross(bob_right_wing) + bob_left_wing.cross(bob_level) + bob_level.cross(bob_right_wing); // Bl x Br + Bl x Blev + Blev x Br
        Z_plane_normal.normalize();
        // project bob_level into plane
        vector_f Z(bob_level - Z_plane_normal * bob_level.dot(Z_plane_normal)); // Blev - (Blev dot plane normal) * plane normal
        Z = Z.normalize() * -1.f; // towards ground
        // The Y in glider reference points towards the right wing
        vector_f Y = Z.cross(X);
        Y.normalize();
        ESP_LOGI(FNAME, "Y: %f,%f,%f", Y.x, Y.y, Y.z);
        ESP_LOGI(FNAME, "Z: %f,%f,%f", Z.x, Z.y, Z.z);

        // process gyro integral to compensate for ground level off the hirozontal plane
        // gyro rotation (to right wing) axes captures the straight from the skid to main gear (in sudo NED space)
        // leverage between gyro rotation to the left wing, which captures the opposite.
        vector_f wiggle_axes = gyro_axis_right - gyro_axis_left;
        ESP_LOGI(FNAME, "gyro_axesr: %f,%f,%f", gyro_axis_right.x, gyro_axis_right.y, gyro_axis_right.z);
        ESP_LOGI(FNAME, "gyro_axesl: %f,%f,%f", gyro_axis_left.x, gyro_axis_left.y, gyro_axis_left.z);
        ESP_LOGI(FNAME, "wiggle_axes: %f,%f,%f", wiggle_axes.x, wiggle_axes.y, wiggle_axes.z);
        // the XY-plane represents the measured horizontal ground plane
        // calc the angle of the wggle axes wrt. the ground plane
        ground_angle = atan2f(-wiggle_axes.z, sqrtf(wiggle_axes.x*wiggle_axes.x + wiggle_axes.y*wiggle_axes.y));
        ESP_LOGI(FNAME, "level correction: %f degree", rad2deg(ground_angle));
        Quaternion level_correction = Quaternion(-ground_angle, vector_f(0., 1., 0.));

        // Concat level correction, calib rotation, on top of the current reference
        Quaternion basic_reference = level_correction * Quaternion::fromRotationMatrix(X, Y).get_conjugate() * _ref_rot;

        // Concatenated with ground angle
        applyImuReference(glider_ground_aa.get(), basic_reference);

        // Save the basic part to nvs storage
        imu_reference.set(basic_reference, false);
    }
    return progress;
}


class ACC_Bias {
   public:
    void set(vector_f *samples, int nr) { smpls = samples; _nr = nr; }
    float operator()(const std::vector<float> x) const {
        const vector_f tmp(x[0], x[1], x[2]);
        float rms = 0.;
        for (int i = 0; i < _nr; i++) {
            rms += powf((smpls[i] - tmp).get_norm() - 1.f, 2.f);
        }
        return rms;
    }

   private:
    vector_f *smpls;
    int _nr;
};

vector_f MpuImu::extractAccBias(vector_f *samples, int nr, float *res0, float *res) {
    // Extract the current bias from samples
    std::vector<float> start{.0, .0, .0};
    std::vector<std::vector<float> > acc_simp{{0.05, 0, 0}, {0, -0.05, 0}, {0, 0, 0.05}, {0, 0, 0}};
    ACC_Bias bias_min;
    bias_min.set(samples, nr);
    std::vector<float> x = MATH::Simplex(bias_min, start, 1e-6f, acc_simp);
    vector_f bias(x[0], x[1], x[2]);
    ESP_LOGI(FNAME, "ACC bias: %f,%f,%f res:%f", x[0], x[1], x[2], bias_min.operator()(x));
    if (res0) *res0 = bias_min.operator()(start);
    if (res) *res = bias_min.operator()(x);
    return bias;
}

void MpuImu::restoreAccelOffset() const {
    axes_i16_abi tmp = accl_bias.get(); 
    accSensor->getMpu().setAccelOffset(mpud::raw_axes_t(tmp.x, tmp.y, tmp.z));
}


//
// PI control to regulate temperature
//

void MpuImu::initHeatCtrl() {
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

void MpuImu::temp_control() {
    float mpu_target_temp = mpu_temperature.get();
    float temp = _MPUdev.getTemperature();
    _mpu_t_delta = temp - mpu_target_temp;
    unsigned pwm = (unsigned)(_pictrl->update(mpu_target_temp, temp) * 255.f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    // ESP_LOGI(FNAME, "MPU Temp Control: T=%.2f Target=%.2f PWM=%d", temp, mpu_target_temp, pwm);
}

void MpuImu::clearpwm() {
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 0);  // heating off
}
