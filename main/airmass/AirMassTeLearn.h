#pragma once

// The TE compensation (the eAdjustment) learned in flight and carried over flights. A wrong setting leaves in eComp's
// energy height h + setting·V²/2g an error proportional to the change of the kinetic height: over 10 s windows, the
// change of the static altitude minus the change of the GNSS energy height (GNSS altitude and the GNSS velocity relative
// to the averaged wind), regressed on the change of V²/2g, gives the setting that cancels it, whatever setting is in
// use. A horizontal gust moves the measured V²/2g but neither altitude, and pulls the regression towards a lower
// setting: the window's ends are means over EDGE_MS around them (on real flights the plain ends read the setting 2–7 %
// low). On real flights within ~3 % of the swept optimum after one flight, ~2 % after two; one flight alone is not
// enough (RESULTS-2026-11-21-te-adjust.md).

#include "math/Units.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace airmass {

class TeLearn {
public:
    static constexpr uint32_t WINDOW_MS = 10000, EDGE_MS = 2500;
    // how much the ground velocity of a fix lags its altitude (0–0.64 s on the real flights, the browser replay's GNSS
    // reference measures it); the altitude's age at arrival is learned (0.2–1.05 s on the real flights, depending on the
    // receiver): where speed and climb change together (thermal entries, dolphin flight) a wrong one biases the setting
    static constexpr uint32_t SPEED_LAG_MS = 300;
    static constexpr float AGE_START = 0.85f, AGE_STEP = 0.05f; // s
    static constexpr int AGES = 41;                             // 0..2 s
    static constexpr float AGE_TAU = 3600.f;                    // s of fixes, the forgetting of the age's correlations
    static constexpr float FORGET_H = 20.f;      // h of windows: older flights fade, a changed installation is learned anew
    static constexpr float RECOMMEND_H = 1.5f;   // h of windows before the setting is recommended
    // and from two flights: a local error of one day (on a ridge near the terrain the setting read 7 % low through the
    // whole flight) is never recommended alone. A flight counts with FLIGHT_H of windows in one power-up: after less, the
    // first flight still carries the setting.
    static constexpr int RECOMMEND_FLIGHTS = 2;
    static constexpr float FLIGHT_H = 1.f;

    // what is carried over flights: the setting [%], its regression weight, the hours of windows behind it, the GNSS
    // altitude's age [s] and the flights
    struct State {
        float setting = NAN, weight = 0.f, hours = 0.f, age = AGE_START;
        int flights = 0;
    };
    const State& state() const { return _st; }
    void setState(const State& s) {
        _st = s;
        _flight_h = 0.f;
    }
    // the setting to recommend, NAN while too little is learned
    float learned() const {
        return _st.hours >= RECOMMEND_H && _st.flights >= RECOMMEND_FLIGHTS && std::isfinite(_st.setting) ? _st.setting : NAN;
    }
    void restart() { _na = _ne = _ng = 0; _start = 0; }

    // an air data cycle: the static altitude, the measured airspeed, usable (flying, the wind known)
    void air(uint32_t t, float alt, float tas, bool usable) {
        if (!usable) _unusable = t;
        if (_na && (int32_t)(t - _air[_ha].t) < (int32_t)AIR_STEP_MS) return;
        _ha = (_ha + 1) % AIR;
        _air[_ha] = {t, alt, tas * tas / (2.f * Units::g0)};
        _na = std::min(_na + 1, AIR);
        if (!_start && usable) _start = t;
    }
    // a GNSS altitude at its arrival, after the velocity of its fix: its 2 s change against the static altitude's at
    // each age, the age with the best correlation (the browser replay's gnssAltDelay); it waits for the next velocity
    void altitude(uint32_t arrival, float alt) {
        if (_ng && arrival - _fix[_hg].t < FIX_MS) return; // faster receivers at 1 Hz: the buffers' time spans
        const Fix* p = nullptr;
        for (int j = 0; j < _ng; ++j) {
            const Fix& f = _fix[(_hg - j + FIXES) % FIXES];
            if (arrival - f.t >= 1500 && arrival - f.t <= 2500) { p = &f; break; }
        }
        if (p && (int32_t)(_unusable - (p->t - (uint32_t)(AGES * AGE_STEP * 1000.f))) < 0) correlate(arrival, alt, *p);
        _hg = (_hg + 1) % FIXES;
        _fix[_hg] = {arrival, alt};
        _ng = std::min(_ng + 1, FIXES);
        if (_vel.t && arrival - _vel.t <= ALT_PAIR_MS) {
            _alt = {arrival, alt};
            _alt_wait = true;
        }
    }
    // a GNSS velocity and the averaged wind: the waiting altitude's energy height with the velocity of its fix,
    // SPEED_LAG_MS towards this one
    void velocity(uint32_t t, float vn, float ve, float wn, float we) {
        if (!std::isfinite(vn) || !std::isfinite(ve)) {
            _alt_wait = false;
            return;
        }
        if (_alt_wait && t - _vel.t > 0 && t - _vel.t <= 3000 && t - _alt.t <= 3000) {
            const float f = std::min((float)SPEED_LAG_MS / (float)(t - _vel.t), 1.f);
            const float an = _vel.vn + f * (vn - _vel.vn) - wn, ae = _vel.ve + f * (ve - _vel.ve) - we;
            gnss(_alt.t, _alt.alt + (an * an + ae * ae) / (2.f * Units::g0));
        }
        _alt_wait = false;
        _vel = {t, vn, ve};
    }

private:
    static constexpr int AIR = 220, ENERGY = 40, FIXES = 4; // 22 s at 10 Hz (a window and its edges behind the GNSS), 40 s at 1 Hz
    static constexpr uint32_t AIR_STEP_MS = 80, AIR_GAP_MS = 300, ENERGY_GAP_MS = 3000;
    static constexpr float OUTLIER_M = 8.f;
    static constexpr uint32_t ALT_PAIR_MS = 500; // a GGA after its RMC; later, its RMC is missing
    static constexpr uint32_t FIX_MS = 900;
    struct Air { uint32_t t; float alt, k; };
    struct Energy { uint32_t t; float e; };
    struct Fix { uint32_t t; float alt; };
    struct Velocity { uint32_t t; float vn, ve; };
    struct Corr { float sx = 0.f, sxx = 0.f, sxy = 0.f; };
    Fix _fix[FIXES];
    Corr _corr[AGES];
    float _corr_n = 0.f, _sy = 0.f, _syy = 0.f; // the GNSS side, the same at every age
    int _hg = 0, _ng = 0;
    Air _air[AIR];
    Energy _energy[ENERGY];
    int _ha = 0, _na = 0, _he = 0, _ne = 0;
    uint32_t _start = 0;    // the next window, 0: none yet
    uint32_t _unusable = 0; // the last air data cycle not usable
    Velocity _vel = {};
    Fix _alt = {};
    bool _alt_wait = false;
    float _flight_h = 0.f; // h of windows since power-up
    State _st;

    void correlate(uint32_t arrival, float alt, const Fix& p) {
        const float dt = (arrival - p.t) * 1e-3f, y = (alt - p.alt) / dt;
        float x[AGES];
        for (int i = 0; i < AGES; ++i) {
            const uint32_t l = (uint32_t)std::lround(i * AGE_STEP * 1000.f);
            float h0, h1, k;
            if (!airAt(arrival - l, h1, k) || !airAt(p.t - l, h0, k)) return;
            x[i] = (h1 - h0) / dt;
        }
        const float decay = std::exp(-dt / AGE_TAU);
        _corr_n = _corr_n * decay + 1.f;
        _sy = _sy * decay + y;
        _syy = _syy * decay + y * y;
        const float n = _corr_n, vy = n * _syy - _sy * _sy;
        int best = -1;
        float r_best = 0.8f;
        for (int i = 0; i < AGES; ++i) {
            Corr& c = _corr[i];
            c.sx = c.sx * decay + x[i];
            c.sxx = c.sxx * decay + x[i] * x[i];
            c.sxy = c.sxy * decay + x[i] * y;
            const float d = (n * c.sxx - c.sx * c.sx) * vy;
            const float r = d > 0.f ? (n * c.sxy - c.sx * _sy) / std::sqrt(d) : 0.f;
            if (r > r_best) { r_best = r; best = i; }
        }
        if (_corr_n >= 100.f && best >= 0) _st.age = best * AGE_STEP;
    }
    // a GNSS energy height at the altitude's arrival
    void gnss(uint32_t arrival, float energy) {
        const uint32_t t = arrival - (uint32_t)std::lround(_st.age * 1000.f);
        if (_ne && (int32_t)(t - _energy[_he].t) <= 0) return;
        _he = (_he + 1) % ENERGY;
        _energy[_he] = {t, energy};
        _ne = std::min(_ne + 1, ENERGY);
        while (_start && (int32_t)(t - (_start + WINDOW_MS + EDGE_MS)) >= (int32_t)ENERGY_GAP_MS) {
            window(_start, _start + WINDOW_MS);
            _start += WINDOW_MS;
        }
    }
    void window(uint32_t t0, uint32_t t1) {
        float h0, h1, k0, k1, e0, e1;
        if ((int32_t)(_unusable - (t0 - EDGE_MS)) >= 0 || !edge(t0, h0, k0, e0) || !edge(t1, h1, k1, e1)) return;
        const float x = k1 - k0, dh = h1 - h0, de = e1 - e0;
        const float prior = std::isfinite(_st.setting) ? _st.setting / 100.f : 1.f;
        const float err = dh + prior * x - de; // eComp's energy height at the setting learned so far against the GNSS
        if (!(std::fabs(err) <= 100.f && std::fabs(x) <= 300.f)) return; // also a NaN
        // a window far off what the setting learned so far expects (heavy turbulence, a GNSS altitude jump) is left out,
        // a few of them would otherwise carry the regression
        if (_st.hours > 0.25f && std::fabs(err) > OUTLIER_M) return;
        const float decay = std::exp(-(float)WINDOW_MS / 3.6e6f / FORGET_H), w = _st.weight * decay;
        _st.weight = w + x * x;
        if (_st.weight <= 0.f) return;
        _st.setting = 100.f * (w * prior + x * (de - dh)) / _st.weight;
        _st.hours = _st.hours * decay + WINDOW_MS / 3.6e6f;
        if (_flight_h < FLIGHT_H && (_flight_h += WINDOW_MS / 3.6e6f) >= FLIGHT_H) _st.flights = std::min(_st.flights + 1, 1000);
    }
    // the means over EDGE_MS around a window's end, at 0.5 s steps
    bool edge(uint32_t t, float& alt, float& k, float& e) const {
        alt = k = e = 0.f;
        int n = 0;
        for (int32_t d = -(int32_t)EDGE_MS; d <= (int32_t)EDGE_MS; d += 500, ++n) {
            float h, kk, ee;
            if (!airAt(t + d, h, kk) || !energyAt(t + d, ee)) return false;
            alt += h; k += kk; e += ee;
        }
        alt /= n; k /= n; e /= n;
        return true;
    }
    // the samples of a ring (newest at head) around time t, at most gap apart, and t's fraction between them
    template <typename T, int N>
    static bool around(const T (&ring)[N], int head, int n, uint32_t t, uint32_t gap, const T*& a, const T*& b, float& f) {
        for (int j = 1; j < n; ++j) {
            b = &ring[(head - j + 1 + N) % N];
            a = &ring[(head - j + N) % N];
            if ((int32_t)(t - a->t) >= 0 && (int32_t)(b->t - t) >= 0) {
                if (b->t - a->t > gap) return false;
                f = (float)(t - a->t) / (float)(b->t - a->t);
                return true;
            }
        }
        return false;
    }
    bool airAt(uint32_t t, float& alt, float& k) const {
        const Air *a, *b;
        float f;
        if (!around(_air, _ha, _na, t, AIR_GAP_MS, a, b, f)) return false;
        alt = a->alt + f * (b->alt - a->alt);
        k = a->k + f * (b->k - a->k);
        return true;
    }
    bool energyAt(uint32_t t, float& e) const {
        const Energy *a, *b;
        float f;
        if (!around(_energy, _he, _ne, t, ENERGY_GAP_MS, a, b, f)) return false;
        e = a->e + f * (b->e - a->e);
        return true;
    }
};

} // namespace airmass
