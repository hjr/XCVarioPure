/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Accelerometer.h"

#include "ImuSensor.h"
#include "Gyroscope.h"
#include "FlightAHRS.h"
#include "math/Units.h"
#include "math/Trigonometry.h"
#include "vector.h"
#include "sensor/SensorMgr.h"
#include "screen/element/ThermalAssist.h"
#include "logdefnone.h"

#include "mpu/math.hpp"

#include <algorithm>
#include <cmath>

Accelerometer *accSensor = nullptr;

constexpr int SENSOR_HISTORY_DURATION_MS = 10000;  // 10 sec
constexpr int DUTY_CYCLE_MS = 100; // 10Hz
constexpr size_t HSIZE = SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) vector_f acc_buffer[ HSIZE + 1 ];

Accelerometer::Accelerometer() : 
    SensorTP<vector_f>(acc_buffer, HSIZE, DUTY_CYCLE_MS, 1),
    _lpf_accel(LowPassFilterT<vector_f>::alphaFromTau(0.3, DUTY_CYCLE_MS / 1000.f))
{
    _id = SensorId(SensorType::ACC_INERTIAL, 1);
    if (imuSensor) {
        _id.flags |= SensorId::SENSOR_LOCAL;
    }

    // accelerometer filter init.
    setFilter(&_lpf_accel);
    _lpf_accel.reset({0.f,0.f,1.f});

    // push a single previous value
    pushAndPublish(vector_f(1,0,0), 0);
}
Accelerometer::~Accelerometer() {
    accSensor = nullptr;
}

const char *Accelerometer::name() const { return imuSensor ? imuSensor->name() : "vMPU"; }

// a quick ten samples self-test of the accelerometer
vector_f Accelerometer::selftest() const
{
    vector_f result, tmp;
    int samples = 0;
    for (auto i = 0; i < 50; i++) {
        accSensor->doRead(tmp); // drop the swing-in samples 
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (auto i = 0; i < 10; i++) {
        if (!accSensor->doRead(tmp)) {
            ESP_LOGE(FNAME, "AHRS acceleration I2C read error");
            continue;
        }
        samples++;
        result += tmp;
        ESP_LOGI(FNAME, "Sample %d: %f/%f/%f", i, tmp.x, tmp.y, tmp.z);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    result /= samples;
    return result;
}

bool Accelerometer::doRead(vector_f& val) {

    // Get new accelerometer values from MPU6050
    mpud::raw_axes_t imuRaw;
    // fetch raw data from the registers
    if (imuSensor->acceleration(&imuRaw) == ESP_OK) {
        // raw data to gravity
        vector_f tmp_ned = vector_f::make_vector(imuRaw.x, imuRaw.y, imuRaw.z, mpud::accelResolution(MpuImu::ACCEL_SCALE));
        // ESP_LOGI(FNAME, "accraw\tX:%d Y:%d Z:%d", imuRaw.x, imuRaw.y, imuRaw.z);
        // ESP_LOGI(FNAME, "tmpvec\tX:%f Y:%f Z:%f", tmp_ned.x, tmp_ned.y, tmp_ned.z);

        // Check on irrational changes
        if ((tmp_ned - getHead()).get_norm2() > 25.f) {
            // vector_f d(tmpvec-prev_accel);
            ESP_LOGE(FNAME, "accelaration change > 5 g in 0.1 sec");
            // return false;
        }
        val = imuSensor->rotate(tmp_ned);
        // ESP_LOGI(FNAME, "val X:%f Y:%f Z:%f", val.x, val.y, val.z);
        return true;
    }

    ESP_LOGE(FNAME, "accel I2C error, X:%d Y:%d Z:%d", imuRaw.x, imuRaw.y, imuRaw.z);
    return false;
}

float Accelerometer::getVerticalAcceleration() {
    // orthonormal projection of the current accelerometer reading to the attitude vector
    // https://de.wikipedia.org/wiki/Orthogonalprojektion
    // lamda := (accel * att_vector) / norm(att_vector)^2 // att_vector is normalized
    // lamda * att_vector would be the projection point, but here is only the g multiple requested

    return getRef().dot(ahrsSensor->get());
}

// compensate for this bias
void Accelerometer::pushBias(const vector_f& bias) {
    // rotate back into sensor frame
    vector_f bias_sensor = imuSensor->_ref_rot.get_conjugate().rotate(bias);
    ESP_LOGI(FNAME, "backrot: %f/%f/%f", bias_sensor.x, bias_sensor.y, bias_sensor.z);

    ESP_LOGI(FNAME, "Mpu scale: %d", mpud::accelSensitivity(MpuImu::ACCEL_SCALE));
    ESP_LOGI(FNAME, "Mpu scale: %f", static_cast<float>(mpud::accelSensitivity(mpud::ACCEL_FS_16G)) / static_cast<float>(mpud::accelSensitivity(MpuImu::ACCEL_SCALE)));
    float scale = static_cast<float>(mpud::accelSensitivity(MpuImu::ACCEL_SCALE)); // to raw units
    scale *= static_cast<float>(mpud::accelSensitivity(mpud::ACCEL_FS_16G)) / static_cast<float>(mpud::accelSensitivity(MpuImu::ACCEL_SCALE));
    // compensate for different scale of bias and current setting, because bias is always in +-16G format
    scale *= static_cast<float>(mpud::accelSensitivity(mpud::ACCEL_FS_16G)) / static_cast<float>(mpud::accelSensitivity(MpuImu::ACCEL_SCALE));
    bias_sensor *= scale;
    mpud::raw_axes_t raw_bias(bias_sensor.x, bias_sensor.y, bias_sensor.z);
    ESP_LOGI(FNAME, "raw  bias: %d,%d,%d", raw_bias.x, raw_bias.y, raw_bias.z);
    // Reprogam MPU bias (acc bias never incremental)
    imuSensor->_MPUdev.setAccelOffset(raw_bias);
    // save to nvs
    accl_bias.set(axes_i16_abi(raw_bias.x, raw_bias.y, raw_bias.z), false);
}

/////////////////////////////////////////
// Post Processing and AHRS fusion code
/////////////////////////////////////////

static void update_fused_vector(vector_f& fused, float gyro_trust, vector_f& petal_force, const Quaternion& omega_step)
{
    // move the previos fused attitude by the new omega step
    vector_f virtual_gravity = omega_step.rotate(fused);
    virtual_gravity.normalize();
    virtual_gravity *= gyro_trust;
    // ESP_LOGI(FNAME,"fused/virtual %.4f,%.4f,%.4f/%.4f,%.4f,%.4f", fused.a, fused.b, fused.c, virtual_gravity.a, virtual_gravity.b, virtual_gravity.c);

    // fuse the centripetal and gyro estimation
    petal_force.normalize();
    fused = virtual_gravity + petal_force;
    // ESP_LOGI(FNAME,"fused %.4f,%.4f,%.4f", fused.a, fused.b, fused.c);
    fused.normalize();
    // ESP_LOGI(FNAME,"fusedn %.4f,%.4f,%.4f", fused.a, fused.b, fused.c);
}


void Accelerometer::postProcess() {
    if (imuSensor) {
        static int count = 0;
        if ( imuSensor->hasHeatCtlr() && ! (++count%10) ) { // every second
            imuSensor->temp_control(); // Call temp regulation
        }
    }

    // calm status
    if (imuSensor) {
        // only needed if we have a real sensor
        detectRest();
    }
}

// rest - detection
bool Accelerometer::detectRest() {
    // accel variance
    vector_f accVar = getVariance(1000);
    const vector_f& accel = getHead();
    
    // ESP_LOGI(FNAME, "rest detection: accVar=(%f, %f, %f) n2=%f < %f", accVar.x, accVar.y, accVar.z, accVar.get_norm2(), ACCVAR_THRESHOLD2);
    // ESP_LOGI(FNAME, "rest detection: accel=(%f, %f, %f) n2=%f < %f", accel.x, accel.y, accel.z, accel.get_norm()-1., ACCEL_THRESHOLD);
    
    if (accVar.get_norm2() < ACCVAR_THRESHOLD2 && std::fabsf(accel.get_norm() - 1.f) < ACCEL_THRESHOLD) {
         // min. 3 sec below threshold, consider as rest
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

void Accelerometer::resetRest() {
    _restTimer = 0;
    _isResting = false;
}

// rad_t AccMPU6050::PitchFromAccelRad()
// {
// 	return std::atan2(-accel.x, accel.z); // neglecting accel.y, because of minor influence on pitch and more noise
// }

