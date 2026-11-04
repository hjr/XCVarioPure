#pragma once

// Averaged wind of the AirMass estimator, for the navigation and as synoptic fallback. In straight flight a 30s low
// pass. While circling the mean over heading sectors: wind errors that depend on the heading (residual airspeed
// scale, heading or latency errors, sideslip) cancel when every heading counts the same, which a low pass only gives
// for whole numbers of circles in its window, and not when the circle is flown unevenly. Each sector averages its
// visits of the last circles (the latest visit weighs VISIT_WEIGHT), about as much turbulence as the low pass
// averages. Plain C++, verified by tools/airmass_sim.

#include "math/Units.h"

#include <algorithm>
#include <cmath>

namespace airmass {

class WindAverage {
public:
    static constexpr int SECTORS = 16;
    static constexpr float TAU = 30.f;                             // s, straight flight low pass
    static constexpr float VISIT_WEIGHT = 0.5f;                    // weight of a sector's latest visit, ~2 circles memory
    static constexpr float RATE_HIGH = Units::deg_to_rad(4.f);     // rad/s, turning
    static constexpr float RATE_LOW = Units::deg_to_rad(0.5f);     // rad/s, straight
    static constexpr float RATE_TAU = 0.5f;                        // s, low pass of the turn rate
    static constexpr float CIRCLE_TIME = 10.f;                     // s of turning into the circling state, and of straight flight out of it
    static constexpr float NEW_VISIT = 5.f;                        // s, a sector entered again after this long is a new circle

    void reset(float wn, float we) {
        _lpf[0] = _out[0] = wn;
        _lpf[1] = _out[1] = we;
        _rate = 0.f;
        _counter = 0.f;
        _circling = false;
    }

    // wn, we: instant wind [m/s], psi: heading [rad], turn_rate: around the down axis [rad/s], dt [s]
    void update(float wn, float we, float psi, float turn_rate, float dt) {
        const float a = dt / (TAU + dt);
        _lpf[0] += (wn - _lpf[0]) * a;
        _lpf[1] += (we - _lpf[1]) * a;

        // circling state with hysteresis
        _rate += (turn_rate - _rate) * dt / (RATE_TAU + dt);
        if (std::fabs(_rate) > RATE_HIGH) {
            _counter = std::min(_counter + dt, CIRCLE_TIME);
        } else if (std::fabs(_rate) < RATE_LOW) {
            _counter = std::max(_counter - dt, 0.f);
        }
        if (!_circling && _counter >= CIRCLE_TIME) {
            startCircling();
        } else if (_circling && _counter <= 0.f) {
            // straight flight continues from the circle mean
            _circling = false;
            _lpf[0] = _out[0];
            _lpf[1] = _out[1];
        }

        if (!_circling) {
            _out[0] = _lpf[0];
            _out[1] = _lpf[1];
            return;
        }
        constexpr float TWO_PI = Units::deg_to_rad(360.f);
        float h = std::fmod(psi, TWO_PI);
        if (h < 0.f) {
            h += TWO_PI;
        }
        const int s = std::min(static_cast<int>(h / TWO_PI * SECTORS), SECTORS - 1);
        _time += dt;
        if (_time - _last[s] > NEW_VISIT) {
            // a sector visited again in the next circle: its last visit goes into the sector average (back and forth
            // across a sector edge is the same visit)
            commitVisit(s);
        }
        _last[s] = _time;
        _sum[s][0] += wn;
        _sum[s][1] += we;
        _n[s]++;
        _out[0] = _out[1] = 0.f;
        for (int i = 0; i < SECTORS; ++i) {
            for (int k = 0; k < 2; ++k) {
                _out[k] += _n[i] ? _avg[i][k] + (_sum[i][k] / _n[i] - _avg[i][k]) * VISIT_WEIGHT : _avg[i][k];
            }
        }
        _out[0] /= SECTORS;
        _out[1] /= SECTORS;
    }

    float windN() const { return _out[0]; }
    float windE() const { return _out[1]; }
    bool circling() const { return _circling; }

private:
    void startCircling() {
        _circling = true;
        for (int i = 0; i < SECTORS; ++i) {
            // sectors not flown yet in this climb hold the average from before
            _avg[i][0] = _lpf[0];
            _avg[i][1] = _lpf[1];
            _n[i] = 0;
            _sum[i][0] = _sum[i][1] = 0.f;
            _last[i] = -NEW_VISIT;
        }
        _time = 0.f;
    }

    void commitVisit(int s) {
        if (_n[s]) {
            for (int k = 0; k < 2; ++k) {
                _avg[s][k] += (_sum[s][k] / _n[s] - _avg[s][k]) * VISIT_WEIGHT;
            }
        }
        _n[s] = 0;
        _sum[s][0] = _sum[s][1] = 0.f;
    }

    float _lpf[2] = {};
    float _out[2] = {};
    float _avg[SECTORS][2] = {}; // per sector: average of the committed visits
    float _sum[SECTORS][2] = {}; // per sector: the current visit
    int _n[SECTORS] = {};
    float _last[SECTORS] = {};   // s, time of the latest sample per sector
    float _time = 0.f;           // s since circling started
    float _rate = 0.f;
    float _counter = 0.f;
    bool _circling = false;
};

} // namespace airmass
