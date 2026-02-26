/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ImuSensor.h"

#include "math/Trigonometry.h"
#include "setup/SetupNG.h"
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
    myMPU.initialize();       // this will initialize the chip and set default configurations
    myMPU.setSampleRate(50);  // in (Hz)
    myMPU.setAccelFullScale(mpud::ACCEL_FS_8G);
    myMPU.setGyroFullScale(mpud::GYRO_FS_250DPS);
    myMPU.setDigitalLowPassFilter(mpud::DLPF_5HZ);  // smoother data
    axes_i16_abi tmp = gyro_bias.get();
    mpud::raw_axes_t gb(tmp.x, tmp.y, tmp.z);
    tmp = accl_bias.get();
    mpud::raw_axes_t ab(tmp.x, tmp.y, tmp.z);
    if (gb.isZero() && ab.isZero()) {
        ESP_LOGI(FNAME, "MPU computeOffsets");
        myMPU.computeOffsets(&ab, &gb);  // returns Offsets in 16G scale
        accl_bias.set(axes_i16_abi(ab.x, ab.y, ab.z));
        gyro_bias.set(axes_i16_abi(gb.x, gb.y, gb.z));
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
        basic_ref = getDefaultImuReference();
    }
    _ref_rot = concatGaaAndImuReference(glider_ground_aa.get(), basic_ref);

    return true;
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
    Quaternion accelDefaultRef = Quaternion(deg2rad(90.0f), vector_f(0, 1, 0)).conjugate(); // towards "NED"

    if (display_orientation.get() == DISPLAY_NORMAL) {
        accelDefaultRef = Quaternion(deg2rad(180.0f), vector_f(1, 0, 0)) * accelDefaultRef;
    } else if (display_orientation.get() == DISPLAY_NINETY) {
        accelDefaultRef = Quaternion(deg2rad(-90.0f), vector_f(1, 0, 0)).conjugate() * accelDefaultRef;
    }
    return accelDefaultRef;
}

// Concatenation of ground angle of attack and the basic reference calibration rotation
Quaternion MpuImu::concatGaaAndImuReference(const degree_t gAA, const Quaternion& basic) {
    Quaternion rot = Quaternion(deg2rad(-gAA), vector_f(0, 1, 0)) * basic;  // rotate positive around Y
    return rot.normalize();
}

// IMU reference calibration
class IMU_Ref {
   public:
    void setBr(vector_d& p) { Br = p; }
    void setBl(vector_d& p) { Bl = p; }
    void setBh(vector_d& p) { Bh = p; }
    double operator()(const std::vector<double> x) const {
        const vector_d tmp(x[0], x[1], x[2]);
        double Nr = (Br - tmp).get_norm();  // |Mr-B|
        double Nl = (Bl - tmp).get_norm();
        double Nh = (Bh - tmp).get_norm();

        // ESP_LOGI(FNAME, "iter: %f,%f,%f", x[0],x[1],x[2]);
        return pow(Nr - 1, 2) + pow(Nl - 1, 2) + pow(Nh - 1, 2) + (tmp).get_norm2() + pow(Nl - Nr, 2) / 10.;
    }

   private:
    vector_d Br, Bl, Bh;
};

// Callback for the two vector samples needed for the reference calibration
// Returns progress in case of success
// 0 := not yet started
// 1 := right wing completed
// 2 := left wing completed
// 3 := calibration completed
// else returns -1
int MpuImu::getAccelSamplesAndCalib(vector_f gyro_integral, rad_t& wing_angle) {
    esp_err_t err;
    vector_d* bob = &bob_level;
    mpud::axes_t<int>* gbias = &gyro_bias_three;
    int16_t side = 0;
    ESP_LOGI(FNAME, "gyro integral: %f/%f/%f", gyro_integral.x, gyro_integral.y, gyro_integral.z);
    if ( progress == 3 ) {
        side = 3;
    }
    else if (gyro_integral.x > 0.) {
        bob = &bob_right_wing;
        gbias = &gyro_bias_one;
        gyro_axis_one = gyro_integral;
        side = 1;
        ESP_LOGI(FNAME, "Right wing down");
    } else {
        bob = &bob_left_wing;
        gbias = &gyro_bias_two;
        gyro_axis_two = gyro_integral;
        side = 2;
        ESP_LOGI(FNAME, "Left wing down");
    }

    err = myMPU.getMPUSamples(bob->x, bob->y, bob->z, *gbias);
    ESP_LOGI(FNAME, "wing down bob: %f/%f/%f", bob->x, bob->y, bob->z);
    if (err == 0) {
        progress |= side;  // accumulate progress
        if (side == 3) {
            // Extract the current bias from wing down measurments
            std::vector<double> start{.0, .0, .0};
            std::vector<std::vector<double> > imu_simp{{0.05, 0, 0}, {0, -0.05, 0}, {0, 0, 0.05}, {0, 0, 0}};
            IMU_Ref bias_min;
            bias_min.setBr(bob_right_wing);
            bias_min.setBl(bob_left_wing);
            bias_min.setBh(bob_level);
            std::vector<double> x = MATH::Simplex(bias_min, start, 1e-10, imu_simp);
            vector_d bias(x[0], x[1], x[2]);
            ESP_LOGI(FNAME, "Bias: %f,%f,%f", x[0], x[1], x[2]);

            // Calculate the rotation to the glieder reference from the two measurments
            // Corrected wing down bob vectores
            vector_d pureBr = bob_right_wing - bias;
            vector_d pureBl = bob_left_wing - bias;
            ESP_LOGI(FNAME, "pureBr:\t%f\t%f\t%f \tL%.2f", pureBr.x, pureBr.y, pureBr.z, pureBr.get_norm());
            ESP_LOGI(FNAME, "pureBl:\t%f\t%f\t%f \tL%.2f", pureBl.x, pureBl.y, pureBl.z, pureBl.get_norm());

            // Check on wing angle is at least 4 degree
            wing_angle = Quaternion::AlignVectors(vector_f(bob_right_wing.x, bob_right_wing.y, bob_right_wing.z),
                                                  vector_f(bob_left_wing.x, bob_left_wing.y, bob_left_wing.z))
                             .getAngle();
            ESP_LOGI(FNAME, "Wing Angle: %f degree.", rad2deg(wing_angle / 2.));
            if (wing_angle < deg2rad(8.f)) {
                progress = 0;  // resert the progress
                return -1;
            }

            // A vector from skid touch points towards the main wheel touch point
            // The X in glider reference (points towards the nose)
            vector_d X = pureBr.cross(pureBl);  // Br x Bl
            X.normalize();
            ESP_LOGI(FNAME, "X: %f,%f,%f", X.x, X.y, X.z);
            // The Z in glider reference
            // The bob in glider ref, skid stil on the ground (points up)
            vector_d Z(pureBr + pureBl);
            Z.normalize();
            // The Y in glider reference
            vector_d Y = Z.cross(X);
            Y.normalize();
            ESP_LOGI(FNAME, "Y: %f,%f,%f", Y.x, Y.y, Y.z);
            ESP_LOGI(FNAME, "Z: %f,%f,%f", Z.x, Z.y, Z.z);

            // The inverted rotation we need to apply
            Quaternion basic_reference = Quaternion::fromRotationMatrix(X, Y).get_conjugate();
            // Concatenated with ground angle
            applyImuReference(glider_ground_aa.get(), basic_reference);

            // Save the basic part to nvs storage
            imu_reference.set(basic_reference, false);

            // Save the accel bias
            mpud::raw_axes_t raw_bias(bias.x * -2048., bias.y * -2048., bias.z * -2048.);
            ESP_LOGI(FNAME, "raw  Bias: %d,%d,%d", raw_bias.x, raw_bias.y, raw_bias.z);
            accl_bias.set(axes_i16_abi(raw_bias.x, raw_bias.y, raw_bias.z), false);
            // Reprogam MPU bias
            myMPU.setAccelOffset(raw_bias);

            // Additionaly use gyro sample to calc offset and save it
            raw_bias = mpud::raw_axes_t((gyro_bias_one.x + gyro_bias_two.x) / -2, (gyro_bias_one.y + gyro_bias_two.y) / -2,
                                        (gyro_bias_one.z + gyro_bias_two.z) / -2);
            ESP_LOGI(FNAME, "raw  Gyro: %d,%d,%d", raw_bias.x, raw_bias.y, raw_bias.z);
            mpud::raw_axes_t curr_bias = myMPU.getGyroOffset();
            ESP_LOGI(FNAME, "curr Gyro: %d,%d,%d", curr_bias.x, curr_bias.y, curr_bias.z);
            raw_bias += curr_bias;
            ESP_LOGI(FNAME, "new  Gyro: %d,%d,%d", raw_bias.x, raw_bias.y, raw_bias.z);
            gyro_bias.set(axes_i16_abi(raw_bias.x, raw_bias.y, raw_bias.z), false);
            // Reprogam MPU bias
            myMPU.setGyroOffset(raw_bias);

            // log the gyro axis for debug
            ESP_LOGI(FNAME, "gyro_axis_one: %f,%f,%f", gyro_axis_one.x, gyro_axis_one.y, gyro_axis_one.z);
            ESP_LOGI(FNAME, "gyro_axis_two: %f,%f,%f", gyro_axis_two.x, gyro_axis_two.y, gyro_axis_two.z);
        }
        return progress;
    }
    return -1;
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
