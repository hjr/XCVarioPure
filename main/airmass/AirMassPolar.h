/*
 * The glider polar as S2F fits it (S2F::recalculatePolar): w = a0 + a1 v + a2 v² at 1 g, for the actual wing load
 * and bugs. One model for the estimator on the device and in the host tools (tools/airmass_sim).
 * Plain C++, header only.
 */

#pragma once

#include <cmath>

namespace airmass {

struct Polar {
    float a0 = 0.f, a1 = 0.f, a2 = 0.f; // m/s, negative = sink, v in m/s
    float v_stall = 0.f;                 // m/s
    bool valid = false;

    // sink at the airspeed v [m/s] and load factor n, negative = sink, 0 below 90% of the stall speed
    float sink(float v, float n) const {
        float vs = v_stall * 0.9;
        if (v < vs || !valid) {
            return 0.0;
        }
        float sqn = std::sqrt(n);
        return a0 * n * sqn + a1 * v * n + a2 * v * v * sqn;
    }
    bool operator==(const Polar& o) const { return a0 == o.a0 && a1 == o.a1 && a2 == o.a2 && v_stall == o.v_stall && valid == o.valid; }
};

} // namespace airmass
