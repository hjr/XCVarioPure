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

    // extended Mahoney filter update step
    const float an = accel.get_norm();
    const float loadError = std::fabs(an - 1.f);
    if (!(an>0.1f && an<5.f)) return;
    
    // gyro integration
    // q body->nav: q(t+dt) = q(t) * exp(omega_body*dt/2).
    _q = _q * Quaternion::fromGyro(gyro, dt);
    const vector_f predicted = _q.rotateInv({0,0,1});

    // Coordinated-turn estimate
    // Yaw rate around NED down, expressed using body axes
    _omega = gyro.dot(predicted);
    ESP_LOGI("AHRS", "q=(%.3f,%.3f,%.3f,%.3f) down=(%.1f,%.1f,%.1f) omega=%.1f/%.1f", _q._w, _q._x, _q._y, _q._z, 
        predicted.x, predicted.y, predicted.z, rad2deg(_omega), rad2deg(gyro.get_norm()));

    // Only trust turn model for approximately steady turns.
    // Avoid interpreting quick roll/pitch maneuvers as steady turns.
    const bool steadyTurn = (tas.get() > 12.f 
        && std::fabs(_omega) > Units::deg_to_rad(2.4f) 
        && std::fabs(gyro.x) < Units::deg_to_rad(6.5f) 
        && std::fabs(gyro.y) < Units::deg_to_rad(6.5f));

    // Current attitude
    vector_f measured = accel.get_negated();
    measured.normalize();
    const float pitch = pitchRad();

    // Select reference and correction time constant
    vector_f reference = measured;
    float tau = _cfg.tiltTau;

    if (_calibrating) {
        // reference = measured;
        tau = 0.3f;  // Fast correction during adjustment
        if (_calibrating > 0) {
            _calibrating--;
        }
    } else if (steadyTurn) {
        // Coordinated-turn roll estimate
        const float rollTurn = std::atan2(  tas.get() * _omega, Units::g0);
        reference = {-std::sin(pitch), std::sin(rollTurn) * std::cos(pitch), std::cos(rollTurn) * std::cos(pitch)};
        ESP_LOGI(FNAME, "steadyTurn rollTurn=%.1f reference=(%.3f,%.3f,%.3f)", rad2deg(rollTurn), reference.x, reference.y, reference.z);
    } else if (loadError > _cfg.accelNormGate) {
        // Unreliable acceleration reference: gyro only
        tau = 1.0e6f;
    } else {
        // Standard Mahony correction, with innovation gating
        const float angle = std::acos(std::clamp(predicted.dot(measured),-1.f,1.f));
        const float dyn = std::max(std::clamp(loadError / _cfg.accelNormGate,0.f,1.f),
                        std::clamp(angle / _cfg.innovationGate,0.f,1.f));
        tau = _cfg.tiltTau + (_cfg.dynamicTau - _cfg.tiltTau) * dyn;
    }
    const float alpha = -std::expm1f(-dt / tau); // 1.f - std::exp(-dt / tau);

    // Mahony-style attitude correction:
    // e = reference x predicted (both in body coordinates).
    // Right-multiply the body->NED quaternion by a positive correction.
    const vector_f e = reference.cross(predicted);
    Quaternion correction = Quaternion(1.f, 0.5f * alpha * e.x, 0.5f * alpha * e.y, 0.5f * alpha * e.z);
    correction.normalize();
    _q = _q * correction;

    vector_f ahrs{getRoll(), getPitch(), _omega};
    pushAndPublish(ahrs, Clock::getMillis()); // an artificial attitude vector for the history

    // ESP_LOGI("AHRS", "q=%.3f %.3f %.3f %.3f roll=%.1f pitch=%.1f omega=%.1f", _q._w, _q._x, _q._y, _q._z, getRollDeg(), getPitchDeg(), rad2deg(_omega));
    ESP_LOGI("AHRS", "turn=%d loadErr=%.2f tau=%.4f alpha=%f roll=%.1f pitch=%.1f", steadyTurn, loadError, tau, alpha, rad2deg(ahrs.x), rad2deg(ahrs.y));

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
