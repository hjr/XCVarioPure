#pragma once

// One sensor cycle of the AirMass estimator: raw measurements in, estimator update out. The device (AirMassSensor),
// the synthetic flights (tools/airmass_sim) and the replay of recorded flights (tools/airmass_sim/replay) all run this,
// so what is tested and replayed is what flies. Plain C++ without ESP-IDF dependencies.

#include "AirMassAttitude.h"
#include "AirMassEKF.h"
#include "AirMassFlaps.h"
#include "AirMassInput.h"
#include "AirMassSampling.h"
#include "AirMassWindAverage.h"
#include "glider/PolarModel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace airmass {

// The damping of the air mass vario from the TE vario damping [s]: it does not take horizontal gusts for lift, so half
// the damping is just as calm, and a thermal entry shows about a second earlier.
inline float displayDamping(float te_damping) { return std::max(1.f, te_damping * 0.5f); }

// the device's glider polar (S2F): sink at the IAS and load factor
using Polar = glider::Polar;

// installation and settings
struct Setup {
    float lever_arm = 0.f; // m, IMU ahead of the CG
    float mass = 0.f;      // kg, gross weight
    float wing_area = 0.f; // m²
    Polar polar;
    float wind_resp = 1.f;   // scales the horizontal wind process noise, fixed on the device
    float vario_resp = 1.f;  // scales the vertical wind process noise, fixed on the device
    float te_adjust = 100.f; // TE "eAdjustment" [%]

    bool operator==(const Setup& o) const {
        return lever_arm == o.lever_arm && mass == o.mass && wing_area == o.wing_area && polar == o.polar && wind_resp == o.wind_resp &&
               vario_resp == o.vario_resp && te_adjust == o.te_adjust;
    }
    bool operator!=(const Setup& o) const { return !(*this == o); }
};

// learned values to start with
struct Warm {
    float k_tas = 1.f, pitch_ofs = 0.f, latency = AirMassEKF::START_LATENCY, k_aoa = 1.f;
    float flap_ofs[FlapOffsets::N];
    Warm() { std::fill(std::begin(flap_ofs), std::end(flap_ofs), FlapOffsets::UNKNOWN); }
};

// the raw measurements of one cycle
struct Frame {
    enum : uint8_t { ACCEL = 1, GYRO = 2, BARO = 4, AIRSPEED = 8, ALL = 15 };
    uint32_t time_ms = 0;
    float f_body[3] = {0.f, 0.f, -1.f}; // g, specific force at the IMU, body NED
    float gyro[3] = {};                 // rad/s, body rates (bias corrected, without the gating dead zone)
    float ias = 0.f, tas = 0.f;         // m/s, as measured
    float baro_alt = NAN;               // m, static port altitude (ISA)
    float flap_lever = NAN;             // lever position 0 .. N-1 (fractional between notches), NAN without flaps
    // Optional source timestamps, in ACCEL/GYRO/BARO/AIRSPEED order. Old logs treat each frame as fresh.
    uint32_t sample_ms[4] = {};
    bool sample_times = false;
    uint8_t valid = ALL;
};

// Optional attitude supplied by replay experiments instead of the air-data-aided attitude.
struct ExternalAttitude {
    float down[3] = {0.f, 0.f, 1.f};
    float roll = 0.f, pitch = 0.f, yaw_rate = 0.f;
};

// Implausible readings (a garbled log line, a sensor glitch) are held out like a missing sample: the previous value
// stays and the channel's source clock does not advance. A reading that steps and stays (three samples in a row that
// agree) is the sensor's new value. The step limits are per 100 ms and scale with the sample interval up to three times;
// real flights stay well below them (airspeed 7 m/s, altitude 9 m, 2 g, 190 °/s between 10 Hz samples).
struct Plausibility {
    static constexpr float TAS_MAX = 150.f, TAS_STEP = 15.f;                     // m/s
    static constexpr float ALT_MIN = -1000.f, ALT_MAX = 15000.f, ALT_STEP = 25.f; // m
    static constexpr float F_MAX = 8.f, F_STEP = 4.f;                            // g
    static constexpr float GYRO_MAX = 8.f, GYRO_STEP = 5.f;                      // rad/s

    struct Guard {
        float last[3] = {}, raw[3] = {}; // the last accepted and the last seen values
        uint32_t raw_ms = 0;
        uint8_t rejects = 0;
        bool have = false;

        void reset() {
            have = false;
            rejects = 0;
        }
        // v[n] sampled at stamp_ms: false to hold the sample out
        bool accept(const float* v, int n, float step, uint32_t stamp_ms) {
            bool jump = false, same = rejects > 0;
            if (have) {
                // the sample interval (missing samples widen the step), not the time since the last accepted one:
                // held out samples must not widen it
                const float lim = step * std::clamp((stamp_ms - raw_ms) * 0.01f, 1.f, 3.f);
                for (int i = 0; i < n; ++i) {
                    if (std::fabs(v[i] - last[i]) > lim) jump = true;
                    if (std::fabs(v[i] - raw[i]) > step) same = false;
                }
            }
            std::copy(v, v + n, raw);
            raw_ms = stamp_ms;
            if (jump && !(same && rejects >= 2)) {
                rejects = std::min<uint8_t>(rejects + 1, 10);
                return false;
            }
            std::copy(v, v + n, last);
            have = true;
            rejects = 0;
            return true;
        }
    };
};

class Pipeline {
public:
    static constexpr float GYRO_DOT_TAU = 0.2f; // s, low pass of the differentiated gyro (lever arm)

    // simulation experiments; estimator settings go by name (tools/airmass_sim/replay_settings.h)
    struct Options {
        bool slip_aid = true; // sideslip uncertainty from the lateral acceleration
    };

    void configure(const Setup& s) {
        _setup = s;
        AirMassEKF::Config cfg;
        cfg.q_wind_h *= s.wind_resp * s.wind_resp;
        cfg.q_wind_v *= s.vario_resp * s.vario_resp;
        cfg.static_comp = s.te_adjust / 100.f;
        cfg.learn_calibration = _learn;
        _ekf.setConfig(cfg);
    }
    void setOptions(const Options& o) {
        _opt = o;
        configure(_setup);
    }
    void setLearning(bool learn) {
        _learn = learn;
        configure(_setup);
    }

    // start (or restart) with the device AHRS attitude: down axis in body coordinates
    void start(uint32_t time_ms, float baro_alt, const float down[3], const float gyro[3], const Warm& w) {
        if (!std::isfinite(baro_alt)) baro_alt = 0.f;
        _ekf.reset(baro_alt, w.k_tas, w.pitch_ofs, w.latency, w.k_aoa);
        _flaps.load(w.flap_ofs);
        _att.reset(down);
        std::copy(_att.down(), _att.down() + 3, _supported_down);
        for (int i = 0; i < 3; ++i) {
            _gyro_prev[i] = std::isfinite(gyro[i]) ? gyro[i] : 0.f;
            _gyro_dot[i] = 0.f;
        }
        _roll_mem = 0.f;
        _prev_time = time_ms;
        _avg_wind.reset(0.f, 0.f);
        _open = false;
        _have_frame = false;
        _have_attitude = false;
        for (auto& clock : _samples) clock = {};
        for (auto& guard : _guards) guard.reset();
        _values = Frame();
        _values.baro_alt = baro_alt;
        for (int i = 0; i < 3; ++i) _values.gyro[i] = _gyro_prev[i];
    }

    // one cycle: the estimator update, then the cycle's gnss() and heading(), then endCycle()
    void step(const Frame& source, const ExternalAttitude* attitude = nullptr) {
        Frame f = source;
        if (!f.sample_times) {
            for (auto& stamp : f.sample_ms) stamp = f.time_ms;
            f.sample_times = true;
        }
        endCycle();
        const int32_t elapsed = (int32_t)(f.time_ms - _prev_time);
        if (_have_frame && elapsed == 0) return;
        bool source_gap = false;
        for (int i = 0; i < 2; ++i) {
            source_gap = source_gap || (_samples[i].have && (f.valid & (1 << i)) &&
                (int32_t)(f.time_ms - f.sample_ms[i]) >= 0 && (int32_t)(f.sample_ms[i] - _samples[i].time_ms) > 10000);
        }
        // the clock went back (an instrument replay returning to real time): the estimate cannot wait until it
        // catches up, held and still flagged valid
        const bool clock_back = elapsed < 0;
        if (clock_back || elapsed > 10000 || source_gap) {
            // No usable motion history remains after a long outage. Restart dynamics, retaining calibration
            // and the last attitude; do not integrate seconds of unobserved acceleration as if measured.
            Warm w;
            w.k_tas = _ekf.kTas();
            w.pitch_ofs = _ekf.pitchOffset();
            w.latency = _ekf.gnssLatency();
            w.k_aoa = _ekf.aoaScale();
            for (int i = 0; i < FlapOffsets::N; ++i) w.flap_ofs[i] = _flaps.offset(i);
            const float down[3] = {_supported_down[0], _supported_down[1], _supported_down[2]};
            start(f.time_ms, f.baro_alt, down, f.gyro, w);
        }
        const bool gap = clock_back || elapsed > 300 || source_gap;
        if (gap && elapsed <= 10000) {
            // Causal reconstruction: hold the previous sample through missing time, not the future endpoint.
            // Inferred history is marked unusable for GNSS fusion; trust recovers only on measured cycles.
            Frame held = _have_frame ? _last_frame : f;
            while ((uint32_t)(f.time_ms - _prev_time) > 100) {
                held.time_ms = _prev_time + 100;
                stepOnce(held, _have_attitude ? &_last_attitude : nullptr, false);
                endCycle();
            }
        }
        stepOnce(f, attitude, !gap);
        _last_frame = f;
        _have_frame = true;
        _have_attitude = attitude != nullptr;
        if (attitude) _last_attitude = *attitude;
    }

private:
    void stepOnce(const Frame& f, const ExternalAttitude* attitude, bool continuous) {
        AirMassEKF::Input& in = _in;
        in = AirMassEKF::Input();
        in.time_ms = f.time_ms;
        const uint32_t elapsed_ms = f.time_ms - _prev_time;
        in.dt = elapsed_ms ? std::min(elapsed_ms * 1e-3f, 0.3f) : 0.05f;
        _prev_time = f.time_ms;
        bool fresh[4];
        uint8_t valid = f.valid;
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(f.f_body[i])) valid &= ~Frame::ACCEL;
            if (!std::isfinite(f.gyro[i])) valid &= ~Frame::GYRO;
        }
        if (!std::isfinite(f.baro_alt)) valid &= ~Frame::BARO;
        if (!std::isfinite(f.ias) || !std::isfinite(f.tas)) valid &= ~Frame::AIRSPEED;
        for (int i = 0; i < 4; ++i) {
            const uint8_t bit = 1 << i;
            // a new source sample (not a held reading polled again) has to be plausible
            const bool is_new = (valid & bit) && (int32_t)(f.time_ms - f.sample_ms[i]) >= 0 &&
                                (!_samples[i].have || (int32_t)(f.sample_ms[i] - _samples[i].time_ms) > 0);
            if (is_new && !plausible(i, f)) {
                valid &= ~bit;
                ++_held_out[i];
            }
            fresh[i] = _samples[i].observe(f.time_ms, f.sample_ms[i], valid & bit);
        }
        for (int i = 0; i < 3; ++i) {
            if (fresh[0]) _values.f_body[i] = f.f_body[i];
            if (fresh[1]) _values.gyro[i] = f.gyro[i];
        }
        if (fresh[2]) _values.baro_alt = f.baro_alt;
        if (fresh[3]) { _values.ias = f.ias; _values.tas = f.tas; }
        // a reading missing or held out for one cycle is bridged by the held one, like a slower sensor polled at 20 Hz
        const uint32_t imu_age = std::max(_samples[0].age(f.time_ms), _samples[1].age(f.time_ms));
        const bool motion = imu_age <= 300;
        const bool airdata = _samples[2].age(f.time_ms) <= 300 && _samples[3].age(f.time_ms) <= 300;
        in.continuous = continuous && motion && !(fresh[0] && _samples[0].interval_ms > 300) &&
                        !(fresh[1] && _samples[1].interval_ms > 300);
        in.imu_age = imu_age == UINT32_MAX ? 10.f : imu_age * 1e-3f;
        in.baro_fresh = fresh[2] && _samples[2].age(f.time_ms) <= 300;
        in.kinematic_fresh = fresh[0] || fresh[1] || fresh[3];
        in.energy_fresh = fresh[3];
        in.baro_alt = _values.baro_alt;

        // the flap position decides the pitch/AoA offset the estimator works with
        _flaps.update(_ekf, f.flap_lever);

        // IMU and air data
        Raw raw;
        const float gyro_dt = _samples[1].interval_ms ? std::min(_samples[1].interval_ms * 1e-3f, 0.3f) : in.dt;
        const float a = gyro_dt / (GYRO_DOT_TAU + gyro_dt);
        for (int i = 0; i < 3; ++i) {
            raw.f_body[i] = _values.f_body[i];
            raw.gyro[i] = _values.gyro[i];
            if (!in.continuous) _gyro_dot[i] = 0.f;
            else if (fresh[1]) _gyro_dot[i] += ((raw.gyro[i] - _gyro_prev[i]) / gyro_dt - _gyro_dot[i]) * a;
            if (fresh[1]) _gyro_prev[i] = raw.gyro[i];
            raw.gyro_dot[i] = _gyro_dot[i];
        }
        raw.lever_arm = _setup.lever_arm;
        raw.ias = _values.ias;
        raw.tas = _values.tas;
        raw.mass = _setup.mass;
        raw.wing_area = _setup.wing_area;
        const AirData air = airData(raw, _ekf);
        raw.polar_sink_ias = _setup.polar.sink(_ekf.config().polar_calibrated > 0.5f ? air.ias : raw.ias, air.n_lift);

        // attitude: aided by the air velocity direction, with the estimator's body z gyro bias (the gravity reference
        // cannot learn it in straight flight)
        if (attitude) {
            in.roll = attitude->roll;
            in.pitch = attitude->pitch;
            in.yaw_rate = attitude->yaw_rate;
            std::copy(std::begin(attitude->down), std::end(attitude->down), raw.down_body);
        } else {
            _att.update(raw.gyro, air.f_cg, _ekf.kTas() * raw.tas, air.ca, air.sa, in.dt, _ekf.yawRateBias(), in.continuous);
            in.roll = _att.roll();
            in.pitch = _att.pitch();
            in.yaw_rate = _att.yawRate(raw.gyro);
            for (int i = 0; i < 3; ++i) {
                raw.down_body[i] = _att.down()[i];
            }
        }
        fillInput(raw, air, in);
        if (in.continuous) std::copy(std::begin(raw.down_body), std::end(raw.down_body), _supported_down);
        // Wheel support is not wing lift. Below the flying-speed cutoff the air-vector/AoA
        // model cannot distinguish ground roll from flying; let the caller use TE instead.
        const float polar_ias = _ekf.config().polar_calibrated > 0.5f ? air.ias : raw.ias;
        in.flying_speed = airdata && std::isfinite(polar_ias) && polar_ias >= std::max(10.f, _setup.polar.v_stall * 0.8f);
        // sideslip while rolling (adverse yaw), the side force shows little of it
        _roll_mem = std::max(std::fabs(raw.gyro[0]), _roll_mem * (1.f - in.dt / ROLL_SLIP_TAU));
        in.slip_sigma = std::min(std::max(in.slip_sigma, ROLL_SLIP * _roll_mem), SLIP_MAX);
        if (!_opt.slip_aid) {
            in.slip_sigma = 0.f;
        }
        _ekf.update(in);
        _open = true;
    }

    // the reading of a channel (Frame::ACCEL .. AIRSPEED bit index) is within range and not a one-off jump
    bool plausible(int channel, const Frame& f) {
        using P = Plausibility;
        switch (channel) {
            case 0:
                for (float v : f.f_body) if (std::fabs(v) > P::F_MAX) return false;
                return _guards[0].accept(f.f_body, 3, P::F_STEP, f.sample_ms[0]);
            case 1:
                for (float v : f.gyro) if (std::fabs(v) > P::GYRO_MAX) return false;
                return _guards[1].accept(f.gyro, 3, P::GYRO_STEP, f.sample_ms[1]);
            case 2:
                if (f.baro_alt < P::ALT_MIN || f.baro_alt > P::ALT_MAX) return false;
                return _guards[2].accept(&f.baro_alt, 1, P::ALT_STEP, f.sample_ms[2]);
            default:
                if (f.ias < 0.f || f.tas < 0.f || f.ias > P::TAS_MAX || f.tas > P::TAS_MAX) return false;
                return _guards[3].accept(&f.tas, 1, P::TAS_STEP, f.sample_ms[3]);
        }
    }

    // the end of the cycle, after its GNSS and compass fusion: the averaged wind follows the valid instant wind
    // (step() ends a cycle left open)
public:
    void endCycle() {
        if (!_open) {
            return;
        }
        _open = false;
        if (_ekf.status() & AirMassEKF::ST_WIND_VALID) {
            _avg_wind.update(_ekf.windN(), _ekf.windE(), _ekf.heading(), _ekf.headingRate(), _in.dt);
        } else {
            _avg_wind.reset(_ekf.windN(), _ekf.windE());
        }
    }

    // GNSS ground velocity, with a trusted wind as prior for the alignment
    void gnss(const AirMassEKF::GnssVelocity& g, bool has_prior, float wn0, float we0) { _ekf.fuseGnss(g, has_prior, wn0, we0); }
    // absolute true heading (compass)
    void heading(float psi, float sigma) { _ekf.fuseHeading(psi, sigma); }

    AirMassEKF& ekf() { return _ekf; }
    const AirMassEKF& ekf() const { return _ekf; }
    const Attitude& attitude() const { return _att; }
    FlapOffsets& flaps() { return _flaps; }
    const AirMassEKF::Input& input() const { return _in; } // the estimator input of the last cycle
    const Setup& setup() const { return _setup; }
    const WindAverage& avgWind() const { return _avg_wind; } // low pass, heading sectors while circling
    // readings held out as implausible since the start, per channel (Frame::ACCEL .. AIRSPEED bit index)
    uint32_t heldOut(int channel) const { return _held_out[channel]; }

private:
    AirMassEKF _ekf;
    Attitude _att;
    FlapOffsets _flaps;
    Setup _setup;
    Options _opt;
    AirMassEKF::Input _in;
    WindAverage _avg_wind;
    bool _open = false; // a cycle stepped, not ended yet

    float _gyro_prev[3] = {};
    float _gyro_dot[3] = {};
    float _roll_mem = 0.f; // rad/s, recent roll rate
    uint32_t _prev_time = 0;
    bool _learn = true;
    Frame _last_frame;
    ExternalAttitude _last_attitude;
    bool _have_frame = false, _have_attitude = false;
    SampleClock _samples[4];
    Plausibility::Guard _guards[4];
    uint32_t _held_out[4] = {};
    Frame _values;
    float _supported_down[3] = {0.f, 0.f, 1.f};
};

} // namespace airmass
