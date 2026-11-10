#pragma once
// The device's extended Mahony AHRS (components/util/math/MahonyAHRS.h) as the attitude of a replay (replay_core.h,
// Replayer::setMahony), instead of the pipeline's air-data-aided airmass::Attitude: fed with the frames' fresh IMU and
// airspeed readings, propagated by the gyro alone while a reading is stale or invalid.

#include "AirMassPipeline.h"
#include "math/MahonyAHRS.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace airmass::replay {

class MahonySource {
public:
    // start (or restart) with the start attitude: down axis in body coordinates
    void start(uint32_t time_ms, const float down[3], const float gyro[3]) {
        _q = Quaternion::fromDownVector({down[0], down[1], down[2]});
        _supported_q = _q;
        _omega = 0.f;
        // The start already supplies gravity. Fast accelerometer leveling would erase a banked start.
        _calibrating = 0;
        _time = time_ms;
        _gyro = {std::isfinite(gyro[0]) ? gyro[0] : 0.f, std::isfinite(gyro[1]) ? gyro[1] : 0.f, std::isfinite(gyro[2]) ? gyro[2] : 0.f};
        _force = {-down[0], -down[1], -down[2]};
        _tas = 0.f;
        for (auto& clock : _samples) clock = {};
    }

    // the attitude for the pipeline's step() with this frame
    ExternalAttitude step(const Frame& f) {
        const uint32_t elapsed_ms = f.time_ms - _time;
        const float dt = elapsed_ms ? std::min(elapsed_ms * 1e-3f, 0.3f) : 0.05f;
        _time = f.time_ms;
        bool accel_ok = f.valid & Frame::ACCEL, gyro_ok = f.valid & Frame::GYRO;
        for (int i = 0; i < 3; ++i) {
            accel_ok = accel_ok && std::isfinite(f.f_body[i]);
            gyro_ok = gyro_ok && std::isfinite(f.gyro[i]);
        }
        const uint32_t accel_ms = f.sample_times ? f.sample_ms[0] : f.time_ms;
        const uint32_t gyro_ms = f.sample_times ? f.sample_ms[1] : f.time_ms;
        const uint32_t airspeed_ms = f.sample_times ? f.sample_ms[3] : f.time_ms;
        const bool new_accel = _samples[0].observe(f.time_ms, accel_ms, accel_ok);
        const bool new_gyro = _samples[1].observe(f.time_ms, gyro_ms, gyro_ok);
        if (new_accel) _force = {f.f_body[0], f.f_body[1], f.f_body[2]};
        if (new_gyro) _gyro = {f.gyro[0], f.gyro[1], f.gyro[2]};
        if ((new_accel && _samples[0].interval_ms > 10000) || (new_gyro && _samples[1].interval_ms > 10000)) {
            _q = _supported_q;
            _omega = 0.f;
        }
        if (_samples[2].observe(f.time_ms, airspeed_ms, (f.valid & Frame::AIRSPEED) && std::isfinite(f.tas))) _tas = f.tas;
        if (accel_ok && gyro_ok && _samples[0].age(f.time_ms) <= 300 && _samples[1].age(f.time_ms) <= 300) {
            MahonyAHRS::update(_q, _omega, _calibrating, {}, _force, _gyro, _tas, dt);
            _supported_q = _q;
        } else {
            _q = _q * Quaternion::fromGyro(_gyro, dt);
        }
        const vector_f down = _q.rotateInv({0.f, 0.f, 1.f});
        ExternalAttitude att;
        for (int i = 0; i < 3; ++i) att.down[i] = down[i];
        att.roll = Attitude::roll(att.down);
        att.pitch = MahonyAHRS::pitchRad(_q);
        // Euler heading rate, as expected by AirMass (not gyro dot down).
        att.yaw_rate = Attitude::yawRate(att.down, _gyro.y, _gyro.z);
        return att;
    }

private:
    Quaternion _q, _supported_q; // the attitude, and its last one supported by the accelerometer
    float _omega = 0.f;
    int8_t _calibrating = 0;
    uint32_t _time = 0;
    vector_f _force, _gyro; // the last fresh readings
    float _tas = 0.f;
    SampleClock _samples[3]; // accelerometer, gyro, airspeed
};

} // namespace airmass::replay
