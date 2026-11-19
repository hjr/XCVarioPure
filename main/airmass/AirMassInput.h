#pragma once

// Conversion of the raw device measurements into the AirMass estimator input.
// Plain C++ like the estimator, verified on a host by tools/airmass_sim/input_test.cpp.

#include "AirMassEKF.h"
#include "math/Units.h"

#include <algorithm>
#include <cmath>

namespace airmass {

constexpr float G0 = Units::g0;
constexpr float RHO0 = Units::rho0;
constexpr float CL_ALPHA = 5.7f; // 1/rad, typical glider wing lift slope
constexpr float CY_BETA = 0.3f;  // 1/rad, side force slope of fuselage and fin, referenced to the wing area
constexpr float SLIP_MAX = 0.35f;
// Rolling swings the nose out of the turn (adverse yaw) more than the side force shows: up to ROLL_SLIP rad of
// sideslip per rad/s of roll rate, settling with the yaw dynamics (ROLL_SLIP_TAU)
constexpr float ROLL_SLIP = 0.3f;     // s
constexpr float ROLL_SLIP_TAU = 1.f;  // s
// The mean lateral specific force per g while not rolling (straight flight and steady turns, the same on real flights)
// is the IMU rolled against the wings, not sideslip: 3° make 0.05 g, read as 5-10° of sideslip. Learned within
// minutes, LAT_RATE_MAX/LAT_RATIO_MAX keep roll in/out (adverse yaw) and real sideslip out.
constexpr float LAT_TAU = 120.f;        // s
constexpr float LAT_RATE_MAX = 0.05f;   // rad/s roll rate
constexpr float LAT_RATIO_MAX = 0.15f;

struct Raw {
    float f_body[3] = {0.f, 0.f, -1.f}; // g, specific force at the IMU, body NED (level at rest: 0,0,-1)
    float down_body[3] = {0.f, 0.f, 1.f}; // nav down axis in body coordinates, from the AHRS
    float gyro[3] = {};                 // rad/s, body rates
    float gyro_dot[3] = {};             // rad/s², body angular acceleration
    float lever_arm = 0.f;              // m, distance of the IMU ahead of the CG
    float ias = 0.f, tas = 0.f;         // m/s, as measured
    float mass = 0.f;                   // kg, gross weight
    float wing_area = 0.f;              // m²
    float polar_sink_ias = NAN;         // m/s at the calibrated IAS and the lift load factor (liftLoadFactor), S2F convention (negative = sink)
};

// Specific force at the CG: the IMU ahead of the CG additionally senses w' x r + w x (w x r), r = (d, 0, 0)
inline void forceAtCg(const Raw& r, float f[3]) {
    const float p = r.gyro[0], q = r.gyro[1], w = r.gyro[2];
    const float d = r.lever_arm;
    const float a_rel[3] = {-d * (q * q + w * w), d * (r.gyro_dot[2] + p * q), d * (p * w - r.gyro_dot[1])};
    for (int i = 0; i < 3; ++i) {
        f[i] = r.f_body[i] - a_rel[i] / G0;
    }
}

// Angle of attack from the lift coefficient at the load factor, relative to the zero lift line
inline float aoaModel(float load, float ias, float mass, float wing_area) {
    if (ias < 10.f || wing_area < 1.f || mass <= 0.f) {
        return 0.f;
    }
    return 2.f * load * mass * G0 / (RHO0 * wing_area * ias * ias) / CL_ALPHA;
}

// Load factor of the lift, perpendicular to the air velocity: the body z specific force alone reads
// lift * cos(alpha) (+ drag * sin(alpha)), a few percent low at circling angles of attack. The body x axis is off
// the air velocity by the AoA model with the estimator's learned scale and offset (several degrees with flaps).
// f: specific force at the CG [g], ias: calibrated
inline float liftLoadFactor(const float f[3], float ias, float mass, float wing_area, const AirMassEKF& ekf) {
    float n = -f[2];
    for (int i = 0; i < 2; ++i) {
        const float a = ekf.alphaOf(aoaModel(n, ias, mass, wing_area));
        n = f[0] * std::sin(a) - f[2] * std::cos(a);
    }
    return n;
}

// Signed sideslip from the lateral specific force, side force = q * S * CY_BETA * beta.
// Positive beta means air from the right.
inline float slipBeta(float f_lat_g, float ias, float mass, float wing_area) {
    const float qs = 0.5f * RHO0 * ias * ias * wing_area * CY_BETA;
    if (ias < 10.f || qs < 1.f || mass <= 0.f) return 0.f;
    return std::clamp(-mass * G0 * f_lat_g / qs, -SLIP_MAX, SLIP_MAX);
}

// Air data from the raw measurements with the estimator's learned calibration, for the attitude and the input
struct AirData {
    float f_cg[3] = {};  // g, specific force at the CG
    float ias = 0.f;     // m/s, calibrated
    float n_lift = 1.f;  // g, lift load factor
    float aoa = 0.f;     // rad, AoA model, from the zero lift line
    float ca = 1.f, sa = 0.f; // cos(alpha), sin(alpha)
};

inline AirData airData(const Raw& r, const AirMassEKF& ekf) {
    AirData a;
    forceAtCg(r, a.f_cg);
    // the aerodynamic models need the calibrated airspeed, an airspeed scale error would otherwise
    // show up squared in the dynamic pressure (angle of attack, side force)
    a.ias = ekf.kTas() * r.ias;
    a.n_lift = liftLoadFactor(a.f_cg, a.ias, r.mass, r.wing_area, ekf);
    a.aoa = aoaModel(a.n_lift, a.ias, r.mass, r.wing_area);
    const float alpha = ekf.alphaOf(a.aoa); // body x axis off the air velocity
    a.ca = std::cos(alpha);
    a.sa = std::sin(alpha);
    return a;
}

// Fills the acceleration, load factor, AoA, polar sink and slip inputs; the attitude, time and
// baro fields are set by the caller. lat_ratio: the lateral specific force per g of normal force that is the
// installation's (an IMU rolled against the wings), not sideslip.
inline void fillInput(const Raw& r, const AirData& a, AirMassEKF::Input& in, float lat_ratio = 0.f) {
    const float* f = a.f_cg;
    const float* dn = r.down_body;
    // inertial acceleration = specific force + gravity, gravity in body coordinates is g * down_body
    in.acc_down = (f[0] * dn[0] + f[1] * dn[1] + f[2] * dn[2] + 1.f) * G0;
    in.load_factor = a.n_lift;
    in.tas = r.tas; // the estimator applies its airspeed scale itself
    in.aoa = a.aoa;
    in.polar_sink = NAN;
    in.slip_beta = slipBeta(f[1] - lat_ratio * std::fabs(f[2]), a.ias, r.mass, r.wing_area);
    in.slip_sigma = std::fabs(in.slip_beta);
    // the polar is defined on IAS: the true sink scales with TAS/IAS
    if (a.ias > 10.f && r.wing_area > 1.f && r.mass > 0.f && r.polar_sink_ias < 0.f) {
        in.polar_sink = -r.polar_sink_ias * r.tas / r.ias;
    }
    // along the air velocity u = (cos(alpha), 0, sin(alpha)), where it compares to the airspeed change: the body x
    // axis would pick up the centripetal acceleration w x (V u) in turns (wy V sin(alpha), ~1 m/s² at 40° bank)
    in.acc_long = ((f[0] + dn[0]) * a.ca + (f[2] + dn[2]) * a.sa) * G0;
    const float level = std::hypot(dn[1], dn[2]);
    in.acc_lat = level > 0.3f ? (f[1] * dn[2] - f[2] * dn[1]) / level * G0 : NAN;
}

} // namespace airmass
