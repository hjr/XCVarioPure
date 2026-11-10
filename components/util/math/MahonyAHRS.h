#pragma once

// The device's extended Mahony tilt filter, shared with flight replay. Body and nav axes are NED;
// accel is specific force [g], gyro is body rate [rad/s], tas [m/s], dt [s].
#include "math/Quaternion.h"
#include "math/vector_3d.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace MahonyAHRS {

struct Config {
    second_t tiltTau = 12.f;         // low-dynamics tilt correction time constant (sec)
    second_t dynamicTau = 60.f;      // weak correction during suspected acceleration
    gee_t    accelNormGate = 0.12f;  // deviation from 1g considered dynamic
    rad_t    innovationGate = 0.18f; // ~10deg accel/gyro disagreement
    second_t maxDt = 0.3f;
};

inline float pitchRad(const Quaternion& q) {
    return std::asin(std::clamp(2.f * (q._w * q._y - q._z * q._x), -1.f, 1.f));
}

inline bool update(Quaternion& q, float& omega, int8_t& calibrating, const Config& cfg,
                   const vector_f& accel, const vector_f& gyro, float tas, float dt) {
    // extended Mahoney filter update step
    const float an = accel.get_norm();
    const float loadError = std::fabs(an - 1.f);
    if (!(an>0.1f && an<5.f)) return false;

    // gyro integration
    // q body->nav: q(t+dt) = q(t) * exp(omega_body*dt/2).
    q = q * Quaternion::fromGyro(gyro, dt);
    const vector_f predicted = q.rotateInv({0,0,1});

    // Coordinated-turn estimate
    // Yaw rate around NED down, expressed using body axes
    omega = gyro.dot(predicted);

    // Only trust turn model for approximately steady turns.
    // Avoid interpreting quick roll/pitch maneuvers as steady turns.
    const bool steadyTurn = (tas > 12.f
        && std::fabs(omega) > Units::deg_to_rad(2.4f)
        && std::fabs(gyro.x) < Units::deg_to_rad(6.5f)
        && std::fabs(gyro.y) < Units::deg_to_rad(6.5f));

    // Current attitude
    vector_f measured = accel.get_negated();
    measured.normalize();
    const float pitch = pitchRad(q);

    // Select reference and correction time constant
    vector_f reference = measured;
    float tau = cfg.tiltTau;

    if (calibrating) {
        // reference = measured;
        tau = 0.3f;  // Fast correction during adjustment
        if (calibrating > 0) {
            calibrating--;
        }
    } else if (steadyTurn) {
        // Coordinated-turn roll estimate
        const float rollTurn = std::atan2(  tas * omega, Units::g0);
        reference = {-std::sin(pitch), std::sin(rollTurn) * std::cos(pitch), std::cos(rollTurn) * std::cos(pitch)};
    } else if (loadError > cfg.accelNormGate) {
        // Unreliable acceleration reference: gyro only
        tau = 1.0e6f;
    } else {
        // Standard Mahony correction, with innovation gating
        const float angle = std::acos(std::clamp(predicted.dot(measured),-1.f,1.f));
        const float dyn = std::max(std::clamp(loadError / cfg.accelNormGate,0.f,1.f),
                        std::clamp(angle / cfg.innovationGate,0.f,1.f));
        tau = cfg.tiltTau + (cfg.dynamicTau - cfg.tiltTau) * dyn;
    }
    const float alpha = -std::expm1f(-dt / tau); // 1.f - std::exp(-dt / tau);

    // Mahony-style attitude correction:
    // e = reference x predicted (both in body coordinates).
    // Right-multiply the body->NED quaternion by a positive correction.
    const vector_f e = reference.cross(predicted);
    Quaternion correction = Quaternion(1.f, 0.5f * alpha * e.x, 0.5f * alpha * e.y, 0.5f * alpha * e.z);
    correction.normalize();
    q = q * correction;
    return true;
}

} // namespace MahonyAHRS
