#pragma once

// One sensor cycle of the AirMass estimator: raw measurements in, estimator update out. The device (AirMassSensor),
// the synthetic flights (tools/airmass_sim) and the replay of recorded flights (tools/airmass_sim/replay) all run this,
// so what is tested and replayed is what flies. Plain C++ without ESP-IDF dependencies.

#include "AirMassAttitude.h"
#include "AirMassEKF.h"
#include "AirMassFlaps.h"
#include "AirMassInput.h"
#include "AirMassWindAverage.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace airmass {

// the glider polar like S2F::getSink: sink at the IAS and load factor, negative = sink, 0 below the stall speed
struct Polar {
    float a0 = 0.f, a1 = 0.f, a2 = 0.f;
    float v_stall = 0.f; // m/s
    bool valid = false;

    float sink(float v, float n) const {
        const float vs = v_stall * 0.9f;
        if (v < vs || !valid) {
            return 0.f;
        }
        const float sqn = std::sqrt(n);
        return a0 * n * sqn + a1 * v * n + a2 * v * v * sqn;
    }
    bool operator==(const Polar& o) const { return a0 == o.a0 && a1 == o.a1 && a2 == o.a2 && v_stall == o.v_stall && valid == o.valid; }
};

// installation and settings
struct Setup {
    float lever_arm = 0.f; // m, IMU ahead of the CG
    float mass = 0.f;      // kg, gross weight
    float wing_area = 0.f; // m²
    Polar polar;
    float wind_resp = 1.f;   // "Wind Agility"
    float vario_resp = 1.f;  // "Vario Agility"
    float te_adjust = 100.f; // "Pull-up Comp." [%]

    bool operator==(const Setup& o) const {
        return lever_arm == o.lever_arm && mass == o.mass && wing_area == o.wing_area && polar == o.polar && wind_resp == o.wind_resp &&
               vario_resp == o.vario_resp && te_adjust == o.te_adjust;
    }
    bool operator!=(const Setup& o) const { return !(*this == o); }
};

// learned values to start with
struct Warm {
    float k_tas = 1.f, pitch_ofs = 0.f, lat_ofs = 0.f, k_aoa = 1.f;
    float flap_ofs[FlapOffsets::N];
    Warm() { std::fill(std::begin(flap_ofs), std::end(flap_ofs), FlapOffsets::UNKNOWN); }
};

// the raw measurements of one cycle
struct Frame {
    uint32_t time_ms = 0;
    float f_body[3] = {0.f, 0.f, -1.f}; // g, specific force at the IMU, body NED
    float gyro[3] = {};                 // rad/s, body rates (bias corrected, without the gating dead zone)
    float ias = 0.f, tas = 0.f;         // m/s, as measured
    float baro_alt = NAN;               // m, static port altitude (ISA)
    float flap_lever = NAN;             // lever position 0 .. N-1 (fractional between notches), NAN without flaps
};

class Pipeline {
public:
    static constexpr float GYRO_DOT_TAU = 0.2f; // s, low pass of the differentiated gyro (lever arm)
    static constexpr float DUTY = 0.1f;         // s, nominal cycle

    // simulation experiments
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
    void setOptions(const Options& o) { _opt = o; }
    void setLearning(bool learn) {
        _learn = learn;
        configure(_setup);
    }

    // start (or restart) with the device AHRS attitude: down axis in body coordinates
    void start(uint32_t time_ms, float baro_alt, const float down[3], const float gyro[3], const Warm& w) {
        _ekf.reset(baro_alt, w.k_tas, w.pitch_ofs, w.lat_ofs, w.k_aoa);
        _flaps.load(w.flap_ofs);
        _att.reset(down);
        for (int i = 0; i < 3; ++i) {
            _gyro_prev[i] = gyro[i];
            _gyro_dot[i] = 0.f;
        }
        _prev_time = time_ms;
        _avg_wind.reset(0.f, 0.f);
        _open = false;
    }

    // one cycle: the estimator update, then the cycle's gnss() and heading(), then endCycle()
    void step(const Frame& f) {
        endCycle();
        AirMassEKF::Input& in = _in;
        in = AirMassEKF::Input();
        in.time_ms = f.time_ms;
        in.dt = std::clamp((f.time_ms - _prev_time) / 1000.f, 0.05f, 0.3f);
        _prev_time = f.time_ms;
        in.baro_alt = f.baro_alt;

        // the flap position decides the pitch/AoA offset the estimator works with
        _flaps.update(_ekf, f.flap_lever);

        // IMU and air data
        Raw raw;
        const float a = DUTY / (GYRO_DOT_TAU + DUTY);
        for (int i = 0; i < 3; ++i) {
            raw.f_body[i] = f.f_body[i];
            raw.gyro[i] = f.gyro[i];
            _gyro_dot[i] += ((f.gyro[i] - _gyro_prev[i]) / in.dt - _gyro_dot[i]) * a;
            _gyro_prev[i] = f.gyro[i];
            raw.gyro_dot[i] = _gyro_dot[i];
        }
        raw.lever_arm = _setup.lever_arm;
        raw.ias = f.ias;
        raw.tas = f.tas;
        raw.mass = _setup.mass;
        raw.wing_area = _setup.wing_area;
        const AirData air = airData(raw, _ekf);
        raw.polar_sink_ias = _setup.polar.sink(air.ias, air.n_lift);

        // attitude: aided by the air velocity direction, with the estimator's body z gyro bias (the gravity reference
        // cannot learn it in straight flight)
        _att.update(raw.gyro, air.f_cg, _ekf.kTas() * raw.tas, air.alpha, in.dt, _ekf.yawRateBias());
        in.roll = _att.roll();
        in.pitch = _att.pitch();
        in.yaw_rate = _att.yawRate(raw.gyro);
        for (int i = 0; i < 3; ++i) {
            raw.down_body[i] = _att.down()[i];
        }
        fillInput(raw, air, in);
        if (!_opt.slip_aid) {
            in.slip_sigma = 0.f;
        }
        _ekf.update(in);
        _open = true;
    }

    // the end of the cycle, after its GNSS and compass fusion: the averaged wind follows the valid instant wind
    // (step() ends a cycle left open)
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
    uint32_t _prev_time = 0;
    bool _learn = true;
};

} // namespace airmass
