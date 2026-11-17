#pragma once

// Horizontal gusts: a sudden change of the head wind, from the airspeed change the inertial acceleration does not
// explain (AirMassEKF::gustAcc). Only in straight flight; in circling the inertial acceleration is not good enough.
// Their slow disagreement is attitude error. The estimator's own low pass of it (kinetic_tau) is fast enough to take a
// third of a gust for it: the detector starts from it and follows it slower. A gust first changes the airspeed (the
// glider's inertia keeps its ground speed): the disagreement without that is an error of the inertial acceleration,
// on real flights mostly after rolling out of a turn.

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace airmass {

class GustDetector {
public:
    static constexpr float STEP = 0.1f;      // s, bucket of the integrated gust acceleration
    static constexpr int N = 50;             // buckets: the head wind change within 5 s
    static constexpr float THRESHOLD = 5.f;  // m/s, head wind change
    static constexpr float HOLD = 6.f;       // s, between two events
    static constexpr float SLOW_TAU = 30.f;  // s, attitude error
    static constexpr float TAS_SHARE = 0.5f; // of the threshold, the airspeed change with it
    static constexpr float STRAIGHT = 0.9f;  // AirMassEKF::straightWeight from which it detects (about 14° bank)
    static constexpr float TRUST = 0.5f;     // AirMassEKF::inertialTrust from which it detects

    void reset() { *this = GustDetector(); }

    // gust_acc NAN while the estimator does not compute it (not flying, a gap)
    void update(float gust_acc, float slow, float tas, float dt, bool active) {
        _event = 0;
        _since += dt;
        if (!active || !std::isfinite(gust_acc) || !std::isfinite(slow) || !std::isfinite(tas)) {
            clear();
            _value = 0.f;
            _slow = NAN;
            return;
        }
        if (!std::isfinite(_slow)) _slow = slow;
        _slow += (gust_acc - _slow) * dt / (SLOW_TAU + dt);
        _cur += (gust_acc - _slow) * dt;
        _cur_t += dt;
        while (_cur_t >= STEP) {
            _dv[_head] = _cur;
            _tas[_head] = tas;
            _head = (_head + 1) % N;
            _n = std::min(_n + 1, N);
            _cur = 0.f;
            _cur_t -= STEP;
        }
        // the change since the lowest (highest) point of the window
        float sum = 0.f, lo = 0.f, hi = 0.f, tas_lo = tas, tas_hi = tas;
        for (int i = 0; i < _n; ++i) {
            const int j = (_head - _n + i + N) % N;
            sum += _dv[j];
            lo = std::min(lo, sum);
            hi = std::max(hi, sum);
            tas_lo = std::min(tas_lo, _tas[j]);
            tas_hi = std::max(tas_hi, _tas[j]);
        }
        sum += _cur;
        const float up = sum - lo, down = sum - hi;
        _value = up >= -down ? up : down;
        const int8_t sign = _value > 0.f ? 1 : -1;
        if (_since >= HOLD && std::fabs(_value) >= THRESHOLD && sign * (tas - (sign > 0 ? tas_lo : tas_hi)) >= TAS_SHARE * THRESHOLD) {
            _event = sign;
            _since = 0.f;
            clear();
        }
    }

    // +1 head wind increase, -1 head wind drop, in this cycle only
    int8_t event() const { return _event; }
    // m/s, the largest head wind change within the window (at an event: the gust's)
    float value() const { return _value; }

private:
    void clear() {
        _n = _head = 0;
        _cur = _cur_t = 0.f;
    }

    float _dv[N] = {};
    float _tas[N] = {};
    int _n = 0, _head = 0;
    float _cur = 0.f, _cur_t = 0.f, _value = 0.f;
    float _since = HOLD;
    float _slow = NAN;
    int8_t _event = 0;
};

} // namespace airmass
