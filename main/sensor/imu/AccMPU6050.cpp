/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "AccMPU6050.h"

#include "ImuSensor.h"
#include "GyroMPU6050.h"
#include "Units.h"
#include "math/Trigonometry.h"
#include "vector.h"
#include "../SensorMgr.h"
#include "logdefnone.h"

#include "mpu/math.hpp"

#include <algorithm>
#include <cmath>

AccMPU6050 *accSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
static vector_f acc_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ];

AccMPU6050::AccMPU6050(MpuImu &mmpu) : 
    SensorTP<vector_f>(acc_buffer, DUTY_CYCLE_MS),
    _my_mpu(mmpu),
    _lpf_slip_angle(LowPassFilterT<float>::alphaFromTau(1.5, DUTY_CYCLE_MS / 1000.f))
{
    _id = SensorId::ACC_INERTIAL | SensorFlags::SENSOR_LOCAL;

    // push a single previous value
    pushAndPublish(vector_f(1,0,0), 0);

    // Load 
}

bool AccMPU6050::doRead(vector_f& val) {

    // Get new accelerometer values from MPU6050
    mpud::raw_axes_t imuRaw;
    // fetch raw data from the registers
    if (_my_mpu.acceleration(&imuRaw) == ESP_OK) {
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
        val = _my_mpu.rotate(tmp_ned);
        val.z -= gyroSensor->getAxD() * _my_mpu.getLeverArm(); // compensate the accelerometer mounting position in front of CG
        // ESP_LOGI(FNAME, "val X:%f Y:%f Z:%f", val.x, val.y, val.z);
        return true;
    }

    ESP_LOGE(FNAME, "accel I2C error, X:%d Y:%d Z:%d", imuRaw.x, imuRaw.y, imuRaw.z);
    return false;
}

float AccMPU6050::getVerticalAcceleration() {
    // orthonormal projection of the current accelerometer reading to the attitude vector
    // https://de.wikipedia.org/wiki/Orthogonalprojektion
    // lamda := (accel * att_vector) / norm(att_vector)^2 // att_vector is normalized
    // lamda * att_vector would be the projection point, but here is only the g multiple requested

    return getHead().dot(att_vector);
}

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


void AccMPU6050::postProcess() {
    static int count = 0;
    if ( _my_mpu.hasHeatCtlr() && ! (++count%10) ) { // every second
        _my_mpu.temp_control(); // Call temp regulation
    }
    
    // AHRS code
    constexpr const float dt = 0.1f; // seconds, a reasonable assumption

    float gravity_trust = 1;
    const vector_f accel = getHead();
    _processed = accel; // todo subtraced soft bias
    const vector_f& gyro = gyroSensor->getRef();
    // ESP_LOGI( FNAME, " Accel: %.3f,%.3f,%.3f Gyro: %.3f,%.4f,%.4f dt: %.4f", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, dt );

    // create a gyro base rotation delta
    d_gyro = Quaternion::fromGyro(gyro, dt);

    rad_t roll = 0, pitch = 0;
    if (airborne.get()) {
        float loadFactor = accel.get_norm();
        float lf = loadFactor > 2.0 ? 2.0 : loadFactor;
        loadFactor = lf < 0 ? 0 : lf;  // limit to 0..2g
        // rotation from navigation to body frame
        vector_f z_nav_in_body = att_quat.rotate(vector_f{0,0,1});
        circle_omega = gyro.dot(z_nav_in_body);
        // tan(roll):= petal force/G = m w v / m g
        float tanw = circle_omega * tas.get() / Units::g0;
        roll = atan(tanw);
        if (ahrs_roll_check.get()) {
            // expected extra load c = sqrt(aa+bb) - 1, here a = 1/9.81 x atan, b=1
            float loadz_exp = std::sqrtf(tanw * tanw / (Units::g0 * Units::g0) + 1.f) - 1.f;
            float loadz_check = (loadz_exp > 0.f) ? std::min(std::max((accel.z - .99f) / loadz_exp, 0.f), 1.f) : 0.f;
            // ESP_LOGI( FNAME,"tanw: %f loadexp: %.2f loadf: %.2f c:%.2f", tanw, loadz_exp, loadFactor, loadz_check );
            // Scale according to real experienced load factor with x 0..1
            roll *= loadz_check;
        }
        // get pitch from accelerometer
        pitch = atan2f(-accel.x, accel.z); // neglecting accel.y, because of minor influence on pitch and more noise

        // Centripetal forces to keep angle of bank while circling
        petal.x = -sin(pitch);             // Nose up (positive Y turn) results in negative X force
        petal.y = sin(roll) * cos(pitch);  // Right wing down (or positive X roll) results in positive Y force
        petal.z = cos(roll) * cos(pitch);  // Any roll or pitch creates a smaller positive Z, gravity Z is positive
        // trust in gyro at load factors unequal 1 g
        gravity_trust =
            (ahrs_min_gyro_factor.get() + (ahrs_gyro_factor.get() * (pow(10, abs(loadFactor - 1) * ahrs_dynamic_factor.get()) - 1)));
        // ESP_LOGI(FNAME, "Omega roll: %f Pitch: %f W_yz: %f Gyro Trust: %f", rad2deg(roll), rad2deg(pitch), circle_omega, gravity_trust);
    } else {
        // For still stand centripetal forces are taken from the accelerometer
        petal = accel;
        circle_omega = 0.f;
    }
    // ESP_LOGI( FNAME, " ax1:%f ay1:%f az1:%f Gx:%f Gy:%f GZ:%f dT:%f", petal.x, petal.y, petal.z, gyro.x, gyro.y, gyro.z, dt );
    // vector_f att_prev = att_vector;
    update_fused_vector(att_vector, gravity_trust, petal, d_gyro.get_conjugate());
    // ESP_LOGI(FNAME,"attv: %.3f %.3f %.3f ProjAccel: %f", att_vector.x, att_vector.y, att_vector.z, accel.dot(att_vector));
    att_quat = Quaternion::fromAccelerometer(att_vector);
    // ESP_LOGI(FNAME,"attq: %.3f %.3f %.3f %.3f", att_quat._x, att_quat._y, att_quat._z, att_quat._w );
    // ESP_LOGI(FNAME,"Circle Omega: %f", circle_omega );
    euler_rad = att_quat.toEulerRad() * -1.f;
    // debug
    // float attvd = (att_vector - att_prev).get_norm2();
    // if (attvd > 0.5) {
    //     [[maybe_unused]] vector_f euler = euler_rad * rad2deg(1.f);
    //     ESP_LOGI(FNAME, "Euler R:%.1f P:%.1f OR:%.1f IMUP:%.1f", euler.Roll(), euler.Pitch(), rad2deg(roll), rad2deg(pitch));
    // }

    // treat gimbal lock, limit to 88 deg
    constexpr const float limit = deg2rad(88.);
    euler_rad.clamp(-limit, limit);

    rad_t gyro_heading_step = circle_omega * dt; // gyro heading change in this step (NED)
    circle_footing = Vector::normalizePI2(circle_footing + gyro_heading_step); // integrate gyro heading change to get the current circle footing

    rad_t curh = 0;
    if (theCompass && theCompass->cur_heading(&curh)) {
        // tuned to plus 7% what gave the best timing swing in response, 2% for compass is far enough
        // gyro and compass are time displaced, gyro comes immediate, compass a second later
        fused_mag_heading += Vector::angleDiff(curh, fused_mag_heading) * 0.02 + gyro_heading_step;
        filtered_mag_heading = Vector::normalizePI2(fused_mag_heading);
        theCompass->setGyroHeading(filtered_mag_heading);
        // ESP_LOGI( FNAME,"cur magn head %.2f gyro yaw: %.4f fused: %.1f Gyro(%.3f/%.3f/%.3f)", curh, gyroYaw, gh, gyroX, gyroY, gyroZ );
    } else {
        fused_mag_heading +=  gyro_heading_step;
        Vector::normalizePI2( fused_mag_heading );
    }

    // ESP_LOGI( FNAME,"GV-Pitch=%.1f  GV-Roll=%.1f filtered_mag_heading: %.2f curh: %.2f GX:%.3f GY:%.3f GZ:%.3f AX:%.3f AY:%.3f AZ:%.3f  FP:%.1f
    // FR:%.1f", euler.Pitch(), euler.Roll(), filtered_mag_heading, curh, gyro.a,gyro.b,gyro.c, accel.a, accel.b, accel.c, filterPitch_rad,
    // filterRoll_rad  );

    // update slip angle
    if (airborne.get()) {
        // the slip angle: accel in NED, slip angle in ENU (nav frame)
        constexpr const float K = rad2deg(4000.f); // airplane constant and Ay correction factor
        rad_t slip = accel.y * K / (tas.get() * tas.get());
        slip = std::clamp(slip, -deg2rad(15.f), deg2rad(15.f));
        slip_angle.set( _lpf_slip_angle.filter( slip ) );  // with atan(x) = x for small x
        // ESP_LOGI(FNAME,"AS: %f m/s, CURSL: %f°, SLIP: %f", tas.get(), -accel.y*K / (tas.get() * tas.get()), slip_angle.get() );
    }

    // calm status
    detectRest();
}

// rest - detection
bool AccMPU6050::detectRest() {
    // accel variance
    vector_f accVar = getVariance(1000);
    const vector_f* accel = getHeadPtr();
    
    // ESP_LOGI(FNAME, "rest detection: accVar=(%f, %f, %f) n2=%f < %f", accVar.x, accVar.y, accVar.z, accVar.get_norm2(), ACCVAR_THRESHOLD2);
    // ESP_LOGI(FNAME, "rest detection: accel=(%f, %f, %f) n2=%f < %f", accel->x, accel->y, accel->z, accel->get_norm()-1., ACCEL_THRESHOLD);
    
    if (accVar.get_norm2() < ACCVAR_THRESHOLD2 && std::fabsf(accel->get_norm() - 1.f) < ACCEL_THRESHOLD) {
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

void AccMPU6050::resetRest() {
    _restTimer = 0;
    _isResting = false;
}

// rad_t AccMPU6050::PitchFromAccelRad()
// {
// 	return atan2f(-accel.x, accel.z); // neglecting accel.y, because of minor influence on pitch and more noise
// }

float AccMPU6050::getGyroFooting() const {
    return circle_footing;
}

