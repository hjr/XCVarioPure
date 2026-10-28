#pragma once

// Attitude for the AirMass estimator: the down vector in body coordinates, propagated by the gyros and
// slowly corrected (with gyro bias estimation) towards an air data aided gravity reference. The specific
// force is cleared from the known accelerations of the flight path, the airspeed change along the air
// velocity and the centripetal part rate x air velocity (the air velocity in body axes from the angle of
// attack), so speed changes (pull-ups, zooms) and turns do not tilt the reference like they tilt a plain
// accelerometer reference. Plain C++, verified by tools/airmass_sim.

#include "math/Units.h"

#include <algorithm>
#include <cmath>

namespace airmass {

class Attitude {
public:
    static constexpr float TAU = 6.f;          // s, time constant of the gravity reference correction
    static constexpr float KI = 0.003f;        // 1/s², gyro bias integration
    static constexpr float TAS_DOT_TAU = 0.5f; // s, low pass of the airspeed derivative
    static constexpr float GUST_ACC = 1.f;     // m/s², airspeed change not explained by inertia: no reference
    static constexpr float GUST_TAU = 10.f;    // s, a lasting disagreement is an attitude error, not a gust
    static constexpr float BIAS_RATE = 0.1f;   // rad/s, the gyro bias is only learned below this rotation rate
    static constexpr float BIAS_MAX = 0.03f;   // rad/s, plausible residual gyro bias incl. temperature drift
    static constexpr float G0 = Units::g0;

    // start from an external attitude (the device AHRS)
    void reset(const float down[3]) {
        for (int i = 0; i < 3; ++i) {
            _d[i] = down[i];
            _bias[i] = 0.f;
        }
        normalize(_d);
        _tas_prev = NAN;
        _tas_dot = 0.f;
        _gust_slow = 0.f;
    }

    // gyro: body rates [rad/s], f: specific force [g] at the CG, tas: calibrated true airspeed [m/s],
    // alpha: angle of attack of the body x axis [rad]
    void update(const float gyro[3], const float f[3], float tas, float alpha, float dt) {
        if (std::isfinite(_tas_prev)) {
            _tas_dot += ((tas - _tas_prev) / dt - _tas_dot) * dt / (TAS_DOT_TAU + dt);
        }
        _tas_prev = tas;

        float w[3];
        for (int i = 0; i < 3; ++i) {
            w[i] = gyro[i] - _bias[i];
        }

        // gravity reference: g = a - f, with the flight path acceleration a = V' u + w x (V u),
        // u = (cos alpha, 0, sin alpha) the air velocity direction in body axes (constant wind)
        const float ca = std::cos(alpha), sa = std::sin(alpha);
        const float a[3] = {_tas_dot * ca + w[1] * tas * sa, w[2] * tas * ca - w[0] * tas * sa, _tas_dot * sa - w[1] * tas * ca};
        const float ref[3] = {a[0] / G0 - f[0], a[1] / G0 - f[1], a[2] / G0 - f[2]};
        const float norm = std::sqrt(ref[0] * ref[0] + ref[1] * ref[1] + ref[2] * ref[2]);
        // less trust when the reference does not look like gravity (unmodelled accelerations), and in gusts:
        // a horizontal gust changes the airspeed without the inertial acceleration along the air velocity.
        // Along u, not along the body x axis: the centripetal part w x (V u) is perpendicular to u, but has
        // a body x component of wy V sin(alpha), about 1 m/s² in a 40° turn.
        // Only the transient part counts: the inertial acceleration is computed with the estimated attitude, so an
        // attitude error shows up as a lasting disagreement, which must not lock out the correction that removes it.
        const float acc_u = ((f[0] + _d[0]) * ca + (f[2] + _d[2]) * sa) * G0;
        const float disagree = _tas_dot - acc_u;
        _gust_slow += (disagree - _gust_slow) * dt / (GUST_TAU + dt);
        const float gust = std::fabs(disagree - _gust_slow);
        const float trust = norm > 0.1f ? std::clamp(1.f - 4.f * std::fabs(norm - 1.f), 0.f, 1.f) * std::clamp(1.f - gust / GUST_ACC, 0.f, 1.f) : 0.f;
        float e[3] = {};
        if (trust > 0.f) {
            const float r[3] = {ref[0] / norm, ref[1] / norm, ref[2] / norm};
            // rotation that moves the estimate towards the reference
            cross(r, _d, e);
            // in turns the reference depends on the modelled angle of attack, its error must not go into the bias
            const float rate = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
            const float learn = std::clamp(1.f - rate / BIAS_RATE, 0.f, 1.f);
            for (int i = 0; i < 3; ++i) {
                e[i] *= trust;
                _bias[i] = std::clamp(_bias[i] - KI * learn * e[i] * dt, -BIAS_MAX, BIAS_MAX);
                w[i] += e[i] / TAU;
            }
        }

        // a nav fixed vector seen from the rotating body: d' = -w x d
        float wd[3];
        cross(w, _d, wd);
        for (int i = 0; i < 3; ++i) {
            _d[i] -= wd[i] * dt;
        }
        normalize(_d);
    }

    const float* down() const { return _d; }
    float roll() const { return std::atan2(_d[1], _d[2]); }
    float pitch() const { return std::asin(std::clamp(-_d[0], -1.f, 1.f)); }
    // heading (Euler yaw) rate, clockwise positive: (q sin(roll) + r cos(roll)) / cos(pitch). The rotation
    // around the nav down axis (gyro . down) differs by roll rate * sin(pitch), which shifts the heading in
    // every roll in/out. The raw gyro: the estimator tracks the yaw rate bias itself, the gravity reference
    // cannot observe the bias around the vertical axis.
    float yawRate(const float gyro[3]) const {
        const float cp = std::sqrt(_d[1] * _d[1] + _d[2] * _d[2]); // cos(pitch)
        if (cp < 0.1f) {
            return 0.f;
        }
        // sin(roll) = d1 / cos(pitch), cos(roll) = d2 / cos(pitch)
        return (gyro[1] * _d[1] + gyro[2] * _d[2]) / (cp * cp);
    }

private:
    static void cross(const float a[3], const float b[3], float out[3]) {
        out[0] = a[1] * b[2] - a[2] * b[1];
        out[1] = a[2] * b[0] - a[0] * b[2];
        out[2] = a[0] * b[1] - a[1] * b[0];
    }
    static void normalize(float v[3]) {
        const float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (n > 1e-6f) {
            for (int i = 0; i < 3; ++i) {
                v[i] /= n;
            }
        }
    }

    float _d[3] = {0.f, 0.f, 1.f};
    float _bias[3] = {};
    float _tas_prev = NAN;
    float _tas_dot = 0.f;
    float _gust_slow = 0.f;
};

} // namespace airmass
