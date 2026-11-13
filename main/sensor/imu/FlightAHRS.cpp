/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "FlightAHRS.h"

#include "imu/Accelerometer.h"
#include "imu/Gyroscope.h"
#include "math/Units.h"
#include "math/Trigonometry.h"
#include "math/Quaternion.h"
#include "vector.h"
#include "screen/element/ThermalAssist.h"
#include "logdefnone.h"
#include "setup/SetupNG.h"

#include <algorithm>
#include <cmath>

FlightAHRS *ahrsSensor = nullptr;

constexpr int SENSOR_HISTORY_DURATION_MS = 1000;  // 1 sec
constexpr int DUTY_CYCLE_MS = 100; // 10Hz
constexpr size_t HSIZE = SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) vector_f ahrs_buffer[ HSIZE + 1 ];

FlightAHRS::FlightAHRS() :
    SensorTP<vector_f>(ahrs_buffer, HSIZE, DUTY_CYCLE_MS, 1),
    _cfg({})
{
    _id = SensorId(SensorType::VIRTUAL, 2);
    ESP_LOGI(FNAME, "FlightAHRS config: maxDtS=%.3f, accelNormGateG=%.3f, innovationGateRad=%.3f, tiltTauS=%.3f, dynamicTauS=%.3f",
             _cfg.maxDt, _cfg.accelNormGate, _cfg.innovationGate, _cfg.tiltTau, _cfg.dynamicTau);
}
FlightAHRS::~FlightAHRS() {
    ahrsSensor = nullptr;
}

bool FlightAHRS::setup() {
    assert(accSensor);
    // specific force to hold against G in NED system (body axes), typically (0,0,-1) at rest
    vector_f d = accSensor->getRef().get_negated(); // body direction of NED down
    d.normalize();
    ESP_LOGI(FNAME, "FlightAHRS reset: specific force direction=(%.3f, %.3f, %.3f)", d.x, d.y, d.z);
    // Minimal quaternion rotating body 'd' onto nav (0,0,1): yaw set to zero by convention.
    _q = Quaternion::fromDownVector(d);
    if ( airborne.get() ) {
        _calibrating = 7;
    }
    ESP_LOGI(FNAME, "FlightAHRS reset: quaternion=(%.3f, %.3f, %.3f, %.3f)", _q._w, _q._x, _q._y, _q._z);
    return true;
}

// void FlightAHRS::setConfig(Config c) { _cfg = c; }


/////////////////////////////////////////
// Post Processing and AHRS fusion code
/////////////////////////////////////////

void FlightAHRS::postProcess() {

    // Body X forward, Y right, Z down
    // _q is Body -> NAV (NED)
    // gyro already bias corrected

    // Gyrointegration -> predicted down vector -> reference formation -> Mahony correction

    // todo track and subtraced soft bias
    // todo _processed.z -= gyroSensor->getAxD() * imuSensor.getLeverArm(); // compensate the accelerometer mounting position in front of CG
    const vector_f& accel = accSensor->getHead();
    const vector_f& gyro = gyroSensor->getRef();
    uint32_t now = Clock::getMillis();
    const second_t dt = (now - _prev_time) / 1000.0f;
    _prev_time = now;
    ESP_LOGI( FNAME, " Accel: %.3f,%.3f,%.3f Gyro: %.3f,%.4f,%.4f dt: %.4f", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, dt );
    
    // if (!(dt>0.f && dt<=_cfg.maxDt) || !finite(accel) || !finite(gyro)) return;

    if (!MahonyAHRS::update(_q, _omega, _calibrating, _cfg, accel, gyro, tas.get(), dt)) {
        return;
    }

    vector_f ahrs{getRoll(), getPitch(), _omega};
    pushAndPublish(ahrs, Clock::getMillis()); // an artificial attitude vector for the history

    // ESP_LOGI("AHRS", "q=%.3f %.3f %.3f %.3f roll=%.1f pitch=%.1f omega=%.1f", _q._w, _q._x, _q._y, _q._z, getRollDeg(), getPitchDeg(), rad2deg(_omega));

    // a pseudo heading integration using the AHRS omega
    rad_t gyro_heading_step = _omega * dt; // gyro heading change in this step (NED)
    _circle_footing = Vector::normalizePI2(_circle_footing + gyro_heading_step); // integrate gyro heading change to get the current circle footing

    // drive the thermal assistent
    if ( thrmAssist ) {
        thrmAssist->checkHeading(_circle_footing, _omega, ahrs.x);
    }
}

void FlightAHRS::startCalibration(int8_t duration) {
    _calibrating = duration;
}

// Conventional NED aerospace roll/pitch, radians. Yaw unobservable without heading source.
float FlightAHRS::rollRad() const { return std::atan2(2.f*(_q._w * _q._x + _q._y * _q._z), 1.f - 2.f * (_q._x * _q._x + _q._y * _q._y)); }
float FlightAHRS::pitchRad() const { return std::asin(std::clamp(2.f * (_q._w * _q._y - _q._z * _q._x), -1.f, 1.f)); }
// Gravity-compensated inertial acceleration in BODY axes (m/s^2).
// This is NOT necessarily TAS derivative (wind, sideslip, rotations).
// vector_f FlightAHRS::linearAccelBody(vector_f accelG, float g) const {
//     const Vec downBody=rotateInv(_q,{0,0,1});
//     return {g*(accelG.x+downBody.x),g*(accelG.y+downBody.y),g*(accelG.z+downBody.z)};
// }

degree_t FlightAHRS::getRollDeg() const { return rad2deg(rollRad()); }
rad_t FlightAHRS::getRoll() const { return rollRad(); }
degree_t FlightAHRS::getPitchDeg() const { return rad2deg(pitchRad()); }
rad_t FlightAHRS::getPitch() const { return pitchRad(); }
degree_t FlightAHRS::getYawDeg() const { return 0.f; }
degree_t FlightAHRS::getMagnHeadingDeg() const { return 0.f; }
dps_t FlightAHRS::getCircleOmegaENUDeg() const { return rad2deg(-_omega); }
Quaternion FlightAHRS::getBody2Nav() const { return _q; }
Quaternion FlightAHRS::getNav2Body() const { return _q.get_conjugate(); }
