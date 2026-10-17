/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "FlightAHRS.h"

#include "Accelerometer.h"
#include "Gyroscope.h"
#include "math/Units.h"
#include "math/Trigonometry.h"
#include "vector.h"
#include "screen/element/ThermalAssist.h"
#include "logdefnone.h"

#include <algorithm>
#include <cmath>

FlightAHRS *ahrsSensor = nullptr;

constexpr int SENSOR_HISTORY_DURATION_MS = 1000;  // 1 sec
constexpr int DUTY_CYCLE_MS = 100; // 10Hz
constexpr size_t HSIZE = SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) vector_f ahrs_buffer[ HSIZE + 1 ];

FlightAHRS::FlightAHRS() : 
    SensorTP<vector_f>(ahrs_buffer, HSIZE, DUTY_CYCLE_MS, 1),
    _lpf_slip_angle(LowPassFilterT<float>::alphaFromTau(0.3, DUTY_CYCLE_MS / 1000.f))
{
    _id = SensorId(SensorType::VIRTUAL, 2);
}
FlightAHRS::~FlightAHRS() {
    ahrsSensor = nullptr;
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


void FlightAHRS::postProcess() {
    
    // AHRS code
    constexpr const float dt = 0.1f; // seconds, a reasonable assumption

    float gravity_trust = 1;
    const vector_f& accel = accSensor->getHead();

    // todo track and subtraced soft bias
    // todo _processed.z -= gyroSensor->getAxD() * imuSensor.getLeverArm(); // compensate the accelerometer mounting position in front of CG
    const vector_f& gyro = gyroSensor->getRef();
    // ESP_LOGI( FNAME, " Accel: %.3f,%.3f,%.3f Gyro: %.3f,%.4f,%.4f dt: %.4f", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, dt );
    // ESP_LOGI( FNAME, " Accel: %.3f,%.3f,%.3f dt: %.4f TS:%d", accel.x, accel.y, accel.z, dt, imuSensor?imuSensor->getTempStatus():temp_status_t::MPU_T_UNKNOWN);

    // create a gyro base rotation delta
    vector_f petal;
    Quaternion d_gyro = Quaternion::fromGyro(gyro, dt);
    rad_t roll = 0, pitch = 0;
    if (airborne.get()) {
        float loadFactor = std::clamp(_processed.get_norm(), 0.f, 2.f);
        // rotation around the nav Z axis
        circle_omega = -gyro.dot(_processed);
        // ESP_LOGI(FNAME,"attv: %.3f,%.3f,%.3f - %.3f,%.3f,%.3f - omega %f", _processed.x, _processed.y, _processed.z, z_nav_in_body.x, z_nav_in_body.y, z_nav_in_body.z, circle_omega);
        // ESP_LOGI(FNAME,"attv: %.3f,%.3f,%.3f - omega %f", _processed.x, _processed.y, _processed.z, circle_omega);
        // tan(roll):= petal force/G = m w v / m g
        float tanw = circle_omega * tas.get() / Units::g0;
        roll = atan(tanw);
        // get pitch from accelerometer
        pitch = atan2f(accel.x, -accel.z); // neglecting accel.y, because of minor influence on pitch and more noise
        // Virtual sum of forces (centrifugaland gravity), serving similar to the accel on the ground 
        // for this complementary filter
        petal.x = sin(pitch);              // Nose up (positive Y turn) results in positive X force
        petal.y = -sin(roll) * cos(pitch); // Right wing down (or positive X roll) results in negative Y force
        petal.z = -cos(roll) * cos(pitch); // Any roll or pitch creates a smaller negative Z, gravity Z is negative
        // trust in gyro at load factors unequal 1 g, trust notches down to "min" at +/-1g
        gravity_trust = (ahrs_min_gyro_factor.get() + 
            (ahrs_gyro_factor.get() * (pow(10, abs(loadFactor - 1) * ahrs_dynamic_factor.get()) - 1) ));
        ESP_LOGI(FNAME, "Petal: %f/%f/%f Gyro Trust: %f", petal.x, petal.y, petal.z, gravity_trust);
    } else {
        // For still stand centripetal forces are taken from the accelerometer
        petal = accel;
        circle_omega = 0.f;
    }
    // ESP_LOGI( FNAME, " ax1:%f ay1:%f az1:%f Gx:%f Gy:%f GZ:%f dT:%f", petal.x, petal.y, petal.z, gyro.x, gyro.y, gyro.z, dt );
    update_fused_vector(_processed, gravity_trust, petal, d_gyro.get_conjugate());
    // ESP_LOGI(FNAME,"attv: %.3f %.3f %.3f ProjAccel: %f", _processed.x, _processed.y, _processed.z, accel.dot(_processed));
    att_quat = Quaternion::fromAccelerometer(_processed.get_negated()).get_conjugate();
    // vector_f img = att_quat.rotate(vector_f{0,0,1});
    // ESP_LOGI(FNAME,"attv: %.3f,%.3f,%.3f - %.3f,%.3f,%.3f", _processed.x, _processed.y, _processed.z, img.x, img.y, img.z);
    // ESP_LOGI(FNAME,"attq: %.3f %.3f %.3f %.3f", att_quat._x, att_quat._y, att_quat._z, att_quat._w );
    // ESP_LOGI(FNAME,"Circle Omega: %f", circle_omega );
    euler_rad = att_quat.toEulerRad();
    // vector_f euler = euler_rad * rad2deg(1.f);
    // ESP_LOGI(FNAME, "Euler R:%.1f P:%.1f WR:%.1f IMUP:%.1f", euler.Roll(), euler.Pitch(), rad2deg(roll), rad2deg(pitch));

    rad_t gyro_heading_step = circle_omega * dt; // gyro heading change in this step (NED)
    _circle_footing = Vector::normalizePI2(_circle_footing + gyro_heading_step); // integrate gyro heading change to get the current circle footing

    // rad_t curh = 0;
    // if (theCompass && theCompass->cur_heading(&curh)) {
    //     // tuned to plus 7% what gave the best timing swing in response, 2% for compass is far enough
    //     // gyro and compass are time displaced, gyro comes immediate, compass a second later
    //     fused_mag_heading += Vector::angleDiff(curh, fused_mag_heading) * 0.02 + gyro_heading_step;
    //     filtered_mag_heading = Vector::normalizePI2(fused_mag_heading);
    //     theCompass->setGyroHeading(filtered_mag_heading);
    //     // ESP_LOGI( FNAME,"cur magn head %.2f gyro yaw: %.4f fused: %.1f Gyro(%.3f/%.3f/%.3f)", curh, gyroYaw, gh, gyroX, gyroY, gyroZ );
    // } else {
    //     fused_mag_heading +=  gyro_heading_step;
    //     Vector::normalizePI2( fused_mag_heading );
    // }

    // ESP_LOGI( FNAME,"GV-Pitch=%.1f  GV-Roll=%.1f filtered_mag_heading: %.2f curh: %.2f GX:%.3f GY:%.3f GZ:%.3f AX:%.3f AY:%.3f AZ:%.3f  FP:%.1f
    // FR:%.1f", euler.Pitch(), euler.Roll(), filtered_mag_heading, curh, gyro.a,gyro.b,gyro.c, accel.a, accel.b, accel.c, filterPitch_rad,
    // filterRoll_rad  );

    // update slip angle and thermal assist
    if (airborne.get()) {
        // the slip angle: accel in NED, slip angle in ENU (nav frame)
        constexpr const float K = rad2deg(4000.f); // airplane constant and Ay correction factor
        rad_t slip = accel.y * K / (tas.get() * tas.get());
        slip = std::clamp(slip, -deg2rad(15.f), deg2rad(15.f));
        slip_angle.set( _lpf_slip_angle.filter( slip ) );  // with atan(x) = x for small x
        // ESP_LOGI(FNAME,"AS: %f m/s, CURSL: %f°, SLIP: %f", tas.get(), -accel.y*K / (tas.get() * tas.get()), slip_angle.get() );

        if ( thrmAssist ) {
            thrmAssist->checkHeading(_circle_footing, circle_omega, euler_rad.Roll());
        }
    }

}


degree_t FlightAHRS::getRollDeg() const { return rad2deg(euler_rad.Roll()); }
rad_t FlightAHRS::getRoll() const { return euler_rad.Roll(); }
degree_t FlightAHRS::getPitchDeg() const { return rad2deg(euler_rad.Pitch()); }
degree_t FlightAHRS::getYawDeg() const { return rad2deg(euler_rad.Yaw()); }
degree_t FlightAHRS::getMagnHeadingDeg() const { return rad2deg(filtered_mag_heading); }
dps_t FlightAHRS::getCircleOmegaENUDeg() const { return rad2deg(-circle_omega); }
Quaternion FlightAHRS::getBody2Nav() const { return att_quat; }
Quaternion FlightAHRS::getNav2Body() const { return att_quat.get_conjugate(); }
