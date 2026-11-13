// Synthetic flight simulation to verify the AirMass EKF on a host.
//
// A simple glider truth model flies scripted segments through a configurable air mass
// (horizontal wind incl. gusts, vertical air movement). Sensors are derived from the truth with
// noise, bias, lag and the 10Hz/GNSS sampling of the real device. The estimator output is compared
// to the truth and to a classic TE netto reference.
//
// The GNSS is the 1Hz NMEA velocity of a Flarm.
//
// usage: airmass_sim <scenario> [--lat-err ms] [--static-adj %] [--comp %] [--compass deg]
//                     [--slip-aid 0|1] [--max-wind-valid s] [--csv file]
//   --lat-err: the true GNSS latency differs from the nominal one the filter is told
//   --static-adj: static port error of the installation, as the TE adjustment that would cancel it
//   --comp: configured compensation of the air mass filter and the electronic TE reference
//   --compass: magnetic compass with this noise (plus a fixed deviation) aids the heading
//   --slip-aid: sideslip estimate from the lateral acceleration (default on)
//   --max-wind-valid: override the scenario's limit for the time to a valid wind
//   --max-wind-rms: override the scenario's limit for the wind rms
//   --max-climb-err: override the scenario's limit for the per thermal climb reading error
//   --seed: sensor noise realisation (the scenario's air mass stays the same)
//   --vario-agility: like the device setting, scales the vertical air movement process noise by its square
//   --set name=value: change an assumption (see Knobs), e.g. --set k_tas=0.95 --set wind=11
//   --warm k_tas,pitch_ofs,aoa_scale,latency: start with these learned values, like the device after a flight
//   --flap-warm o0,..,o4: learned pitch offsets per flap position [rad] of an earlier flight (--set flaps=1)
//   --no-learn: keep the calibration values as started
//   --record file: write the estimator inputs like the device's "NMEA Output" Recording, for tools/airmass_sim/replay

#include "AirMassEKF.h"
#include "AirMassInput.h"
#include "AirMassAttitude.h"
#include "AirMassFlaps.h"
#include "AirMassPipeline.h"
#include "AirMassRecord.h"
#include "replay_settings.h"
#include "ins_reference.h"
#include "stats.h"
#include "math/Quaternion.h"
#include "sensor/Filters.h"
#include "math/vector_3d.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <random>
#include <string>
#include <vector>

namespace {

constexpr float G = Units::g0;
constexpr float RHO = Units::rho0;
constexpr float D2R = Units::rad_per_deg;

// glider
constexpr float MASS = 400.f;
constexpr float AREA = 10.5f;
constexpr float CLA_MODEL = 5.7f;  // what the device assumes
constexpr float ALPHA0 = -0.03f;   // zero lift AoA of the real glider

// assumptions about the glider, its sensors and the day, changed with --set name=value
struct Knobs {
    float k_tas = 1.05f;      // true TAS = k_tas * indicated
    float mount = 0.02f;      // rad, IMU mounting pitch error
    float mount_roll = 0.f;   // deg, IMU mounting roll error (residual after the device's IMU calibration)
    float mount_yaw = 0.f;    // deg, IMU mounting yaw error
    float gyro_bias = 0.002f; // rad/s residual gyro bias (z), half of it on x and y
    float acc_bias = 0.005f;  // g, accelerometer z bias
    float cla_true = 5.5f;    // lift slope of the "real" glider
    float polar_err = 1.05f;  // configured polar sink against the real one
    float mass_err = 1.f;     // configured gross weight against the real one
    float tas_noise = 0.15f;  // m/s airspeed noise
    float wind = 7.f;         // m/s, Dutch day wind
    float turb = 1.f;         // Dutch day turbulence scale
    float lift = 1.f;         // Dutch day thermal strength scale
    float dolphin = 0.f;      // Dutch day: 1 flies speed to fly in cruise (slow in lift, fast in sink)
    float gps_out = -1.f;     // s, start of a GNSS outage
    float gps_out_len = 0.f;  // s, its length
    float flaps = 0.f;        // 1: an ASW 20 with flaps set by speed (and thermal flaps for circling)
    float flap_alpha0 = 0.45f;// rad of zero lift angle per rad of flap
    float flap_aid = 1.f;     // the estimator's flap position (a pitch offset per position): 1 flap sensor,
                              // 2 the device's flap recommendation (no sensor), 0 none
    float flap_pilot = 0.f;   // 0: the pilot's own flap schedule, 1: follows the flap recommendation, 2 s late
    float aoa_nonlin = 0.f;   // rad per CL², lift curve bending towards the stall (above CL 0.9, flaps shift that up)
    float gyro_gate = 0.f;    // °/s, experiment: the estimator on a gyro with the device's "Gyro Gating" dead zone (the device bypasses it, AirMassSensor's gyroRates)
    float predict_hz = 10.f;  // 10 or 20: prediction cadence with the same 10Hz physical observations
    // realism of the truth model and the air data (assumptions, to be calibrated with recorded flights)
    float gust_resp = 1.f;    // 1: a vertical gust first changes the angle of attack, the lift then accelerates the glider
                              // (plunge time constant 2m / (rho V S CLa), ~0.4 s); 0: the glider rides the air instantly
    float adverse_yaw = 0.25f;// s, sideslip per roll rate while rolling (rad per rad/s), the nose swings out of the turn
    float yaw_tau = 0.5f;     // s, the sideslip follows the pilot's slip and the adverse yaw with the yaw dynamics
    float lag_static = 0.15f; // s, pneumatic lag of the static line
    float lag_pitot = 0.3f;   // s, pneumatic lag of the pitot line (dynamic pressure)
    float static_cl = 0.01f;  // static port error per lift coefficient (pressure coefficient, around CL 0.6): changes
                              // with the angle of attack, so the pull-up compensation cannot cancel it
    float day = 0.f;          // Dutch day air mass realisation (thermal positions, turbulence), held-out days for the scoreboard
};
Knobs K;
const std::pair<const char*, float*> KNOBS[] = {
    {"predict_hz", &K.predict_hz},
    {"k_tas", &K.k_tas}, {"mount", &K.mount}, {"mount_roll", &K.mount_roll}, {"mount_yaw", &K.mount_yaw}, {"gyro_bias", &K.gyro_bias}, {"acc_bias", &K.acc_bias}, {"cla_true", &K.cla_true},
    {"polar_err", &K.polar_err}, {"mass_err", &K.mass_err}, {"tas_noise", &K.tas_noise}, {"wind", &K.wind}, {"turb", &K.turb},
    {"lift", &K.lift}, {"dolphin", &K.dolphin}, {"gps_out", &K.gps_out}, {"gps_out_len", &K.gps_out_len},
    {"flaps", &K.flaps}, {"flap_alpha0", &K.flap_alpha0}, {"flap_aid", &K.flap_aid}, {"flap_pilot", &K.flap_pilot}, {"aoa_nonlin", &K.aoa_nonlin}, {"gyro_gate", &K.gyro_gate},
    {"gust_resp", &K.gust_resp}, {"adverse_yaw", &K.adverse_yaw}, {"yaw_tau", &K.yaw_tau}, {"lag_static", &K.lag_static},
    {"lag_pitot", &K.lag_pitot}, {"static_cl", &K.static_cl}, {"day", &K.day},
};
constexpr float CY_TRUE = 0.25f;   // side force slope of the "real" glider, the device assumes airmass::CY_BETA
constexpr float CD_BETA = 0.4f;    // 1/rad², drag increase with sideslip (about +0.3 m/s sink at 8° and 100 km/h)
constexpr float AHRS_WARMUP = 20.f; // s, attitude filters running before the estimator starts
constexpr float COMPASS_DEV = 3.f * D2R; // residual compass deviation

// 1g polar, sink positive, V in m/s (roughly a 15m class glider at 38kg/m²)
float polar1g(float v) {
    return 0.62f + 0.0028f * (v - 24.f) * (v - 24.f) + 0.00003f * (v - 24.f) * (v - 24.f) * (v - 24.f);
}
float polar(float v, float n) {
    n = std::max(n, 0.3f);
    const float sq = std::sqrt(n);
    return n * sq * polar1g(v / sq);
}

// The polar the device works with: like S2F, the quadratic through three points of the glider's polar (here of the
// "real" glider, scaled by the polar error), sink negative.
airmass::Polar devicePolar() {
    const double v[3] = {22.0, 30.0, 42.0};
    double m[3][4];
    for (int i = 0; i < 3; ++i) {
        m[i][0] = 1.0;
        m[i][1] = v[i];
        m[i][2] = v[i] * v[i];
        m[i][3] = -K.polar_err * polar1g(v[i]);
    }
    for (int c = 0; c < 3; ++c) { // Gauss-Jordan, the Vandermonde system is well conditioned enough here
        for (int r = 0; r < 3; ++r) {
            if (r != c) {
                const double f = m[r][c] / m[c][c];
                for (int k = c; k < 4; ++k) m[r][k] -= f * m[c][k];
            }
        }
    }
    airmass::Polar p;
    p.a0 = m[0][3] / m[0][0];
    p.a1 = m[1][3] / m[1][1];
    p.a2 = m[2][3] / m[2][2];
    p.v_stall = 18.f;
    p.valid = true;
    return p;
}

struct Segment {
    float duration;  // s
    float bank_deg;  // target bank
    float v_target;  // m/s
};

struct AirMass {
    // horizontal wind (direction the air moves to), psi is the glider heading for flight relative gusts
    std::function<void(float t, float psi, float& wn, float& we)> horizontal = [](float, float, float& wn, float& we) {
        wn = -4.f;
        we = 3.f;
    };
    std::function<float(float t)> vertical = [](float) { return 0.f; }; // Wd, positive = sink
};

using SlipFn = std::function<float(float t)>; // sideslip of the pilot's flying [rad], positive: air from the right

struct Truth;
// reactive pilot instead of scripted segments: lift is the air mass rise felt at the glider [m/s]
using PilotFn = std::function<void(const Truth& tr, float lift, float dt, float& bank_deg, float& v_target)>;
// vertical air movement (Wd, positive = sink) at an air mass position [m], thermals drift with the air
using FieldFn = std::function<float(float t, float xa, float ya)>;

// what the reactive pilot is doing, for the evaluation
struct PilotState {
    int mode = 0;          // 0 cruise, 1 circling
    float mode_t = 0.f;    // s in the current mode
    int thermal = -1;      // index of the current climb
    int thermals = 0;
};
PilotState g_pilot;

struct Scenario {
    std::string name;
    std::vector<Segment> segments;
    float duration = 0.f; // s, pilot driven scenarios
    PilotFn pilot;
    FieldFn vertical_at;
    float mean_wind[2] = {NAN, NAN}; // north/east, the wind without turbulence, for the evaluation
    AirMass air;
    SlipFn slip = [](float) { return 0.f; };
    float v_tau = 2.f; // pilot speed control time constant
    // evaluation windows
    float eval_wind_after = 120.f;               // s, wind accuracy evaluated after this time
    std::vector<std::pair<float, float>> steady; // windows for steady vario accuracy
    std::vector<std::pair<float, float>> spikes; // windows for disturbance spikes
    // pass criteria
    float max_wind_rms = 1.0f;
    float max_vario_rms = 0.3f;
    float max_spike = 0.0f;   // 0: compare against TE netto only
    float te_margin = 0.0f;   // allowed excess over the (ideal) TE netto spike
    float max_time_to_wind_valid = 200.f;
    bool check_latency = false; // latency is only observable with changing turn rates (reversals)
    int max_latency_err = 150;  // ms, the estimate converges to the effective delay incl. integration lag
    // pilot driven scenarios: the vario as the pilot reads it
    float max_climb_err_rms = 0.f;  // per thermal average climb reading error (0: not checked)
    float max_mean_wind_rms = 0.f;  // 30s averaged wind against the same average of the truth (0: not checked)
};

struct Truth {
    float t = 0, h = 1500.f;
    float xa = 0, ya = 0; // position in the air mass (drifting with the mean wind)
    float gn = 0, ge = 0; // ground position north, east [m]
    float psi = 0, roll = 0, pitch = 0, v = 30.f, vdot = 0, load = 1.f;
    float gamma_up = 0, alpha = 0;
    float va[3] = {}, vg[3] = {};
    float wn = 0, we = 0, wd = 0, wdot[2] = {};
    float acc_down = 0, yaw_rate = 0;
};

struct Lpf {
    float a, y = 0;
    bool init = false;
    Lpf(float tau, float dt) : a(dt / (tau + dt)) {}
    float f(float x) {
        if (!init) {
            y = x;
            init = true;
        }
        y += (x - y) * a;
        return y;
    }
};

// true air velocity in NED: the body air velocity v (cos a cos b, sin b, sin a cos b) with the sideslip beta, rotated
// by roll, pitch and heading (the estimator's AirMassEKF::airVelocity is the same without sideslip)
void trueAirVelocity(float v, float roll, float pitch, float psi, float alpha, float beta, float out[3]) {
    const float sp = std::sin(roll), cp = std::cos(roll);
    const float st = std::sin(pitch), ct = std::cos(pitch);
    const float ss = std::sin(psi), cs = std::cos(psi);
    const float vx = v * std::cos(alpha) * std::cos(beta), vy = v * std::sin(beta), vz = v * std::sin(alpha) * std::cos(beta);
    const float zr = sp * vy + cp * vz; // after the roll
    const float ux = ct * vx + st * zr;
    const float uy = cp * vy - sp * vz;
    out[0] = cs * ux - ss * uy;
    out[1] = ss * ux + cs * uy;
    out[2] = -st * vx + ct * zr;
}

// solve pitch so that the body/air geometry yields the wanted air vertical speed
float solvePitch(float v, float roll, float alpha, float beta, float va_d) {
    float th = -std::asin(std::clamp(va_d / v, -0.9f, 0.9f)) + alpha * std::cos(roll);
    for (int i = 0; i < 5; ++i) {
        float out[3];
        trueAirVelocity(v, roll, th, 0.f, alpha, beta, out);
        float out2[3];
        trueAirVelocity(v, roll, th + 1e-4f, 0.f, alpha, beta, out2);
        const float d = (out2[2] - out[2]) / 1e-4f;
        th -= (out[2] - va_d) / d;
    }
    return th;
}

// The device AHRS (main/sensor/imu/FlightAHRS.cpp, postProcess) with its default tuning: gyro propagated
// down vector, corrected towards a gravity reference from the accelerometer pitch and the coordinated turn roll.
// The gyro is trusted more the further the load factor is away from 1g.
struct DeviceAhrs {
    Quaternion q;
    vector_f euler;
    float circle_omega = 0.f;
    static constexpr float MIN_GYRO_FACTOR = 20.f, GYRO_FACTOR = 100.f, DYNAMIC_FACTOR = 5.f;

    void update(const vector_f& accel, const vector_f& gyro, float tas) {
        constexpr float dt = 0.1f;
        const Quaternion d_gyro = Quaternion::fromGyro(gyro, dt);
        vector_f down = q.rotateInv({0, 0, 1});
        circle_omega = gyro.dot(down);
        const float load = std::clamp(accel.get_norm(), 0.f, 2.f);
        const float roll = std::atan(circle_omega * tas / G);
        const float pitch = std::atan2(accel.x, -accel.z);
        vector_f petal(-std::sin(pitch), std::sin(roll) * std::cos(pitch), std::cos(roll) * std::cos(pitch));
        const float gyro_trust = MIN_GYRO_FACTOR + GYRO_FACTOR * (std::pow(10.f, std::fabs(load - 1.f) * DYNAMIC_FACTOR) - 1.f);
        // update_fused_vector()
        vector_f virtual_gravity = d_gyro.rotateInv(down);
        virtual_gravity.normalize();
        virtual_gravity *= gyro_trust;
        petal.normalize();
        down = virtual_gravity + petal;
        down.normalize();
        q = Quaternion::fromDownVector(down);
        euler = q.toEulerRad();
    }
};

// nav (NED) vector into body coordinates for roll, pitch, yaw (ZYX)
void navToBody(float roll, float pitch, float yaw, const float n[3], float b[3]) {
    const float cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
    const float cy = std::cos(yaw), sy = std::sin(yaw);
    const float x = cy * n[0] + sy * n[1], y = -sy * n[0] + cy * n[1], z = n[2]; // yaw
    const float x2 = cp * x - sp * z, z2 = sp * x + cp * z;                       // pitch
    b[0] = x2;
    b[1] = cr * y + sr * z2;                                                      // roll
    b[2] = -sr * y + cr * z2;
}

// sensor axes misaligned against the body axes: by a pitch angle, then a roll and a yaw angle (small angles, any order)
vector_f mountRotate(const float b[3]) {
    const float c = std::cos(K.mount), s = std::sin(K.mount);
    float x = c * b[0] - s * b[2], y = b[1], z = s * b[0] + c * b[2];
    const float r = K.mount_roll * D2R, cr = std::cos(r), sr = std::sin(r);
    const float y1 = cr * y + sr * z, z1 = -sr * y + cr * z;
    const float yw = K.mount_yaw * D2R, cy = std::cos(yw), sy = std::sin(yw);
    return vector_f(cy * x + sy * y1, -sy * x + cy * y1, z1);
}

// ASW 20 flap positions (lever 0..4); a flap deflection shifts the zero lift angle of attack (Knobs::flap_alpha0).
// The positions are designed so the fuselage stays aligned with the air flow over the whole speed range.
constexpr float FLAP_DEG[5] = {-10.f, -5.f, 0.f, 8.f, 16.f};

// the pilot's flap choice: speed bands, thermal flaps while circling; with some hysteresis
int flapLever(int current, float v, float bank_deg, bool circling) {
    if (circling || std::fabs(bank_deg) > 20.f) {
        return 4;
    }
    constexpr float BANDS[3] = {38.f, 31.f, 26.f}; // m/s: above the first lever 0, ...
    int want = 3;
    for (int i = 0; i < 3; ++i) {
        if (v > BANDS[i]) {
            want = i;
            break;
        }
    }
    if (current >= 0 && current < 4 && std::abs(want - current) == 1) {
        // hold the current position unless the speed is clearly in the next band
        const float edge = BANDS[std::min(want, current)];
        if (std::fabs(v - edge) < 1.f) {
            return current;
        }
    }
    return want;
}

// the device's flap recommendation (Flap::getOptimum, the flap box): speed bands at the speed for 1g
int flapRecommendation(float ias, float load) {
    constexpr float MIN_SPEED[4] = {38.f, 31.f, 26.f, 21.f}; // m/s: lever 0 at and above the first, ...
    const float v = ias / std::sqrt(std::max(load, 0.3f));
    int i = 0;
    while (i < 4 && v <= MIN_SPEED[i]) {
        i++;
    }
    return i;
}

// the device's vario damping (VarioFilter): TE a low pass with tau = D/3, then y += (x - y) / D per cycle;
// the air mass vario a low pass with tau = 0.6 D/3
constexpr float DAMPING = 4.f; // vario_delay default (TE netto)

struct TeDamper {
    LowPassFilterT<float> lpf{LowPassFilterT<float>::alphaFromTau(DAMPING / 3.f, 0.1f)};
    float out = 0.f;
    float filter(float input) {
        out += (lpf.filter(input) - out) / DAMPING;
        return out;
    }
};

bool inWindows(float t, const std::vector<std::pair<float, float>>& w) {
    for (auto& p : w) {
        if (t >= p.first && t < p.second) {
            return true;
        }
    }
    return false;
}

// A typical Dutch soaring day: about 25 km/h WSW wind with convective turbulence, weak narrow thermals
// (core 1..3 m/s, radius 120..250 m) every few kilometres drifting with the wind, sink in between, cloud
// base about 1300 m. The pilot cruises at ~100 km/h, pulls up to ~83 km/h into lift, circles with simple
// centering and leaves at cloud base or when the climb gets weak.
struct NlDay {
    static constexpr float WIND_FROM = 240.f * D2R;              // direction it blows from, speed K.wind
    static constexpr float TURB_H = 1.0f, TURB_H_L = 500.f;     // m/s, m horizontal turbulence
    static constexpr float TURB_V = 0.4f, TURB_V_L = 200.f;     // m/s, m vertical turbulence
    static constexpr float SINK = 0.5f;                         // m/s between thermals
    static constexpr float CLOUD_BASE = 1300.f, V_CRUISE = 28.f, V_CLIMB = 23.f;
    struct Thermal { float x, y, r, w; };

    std::mt19937 rng{20240601};
    std::normal_distribution<float> nd{0.f, 1.f};
    std::uniform_real_distribution<float> ud{0.f, 1.f};
    std::vector<Thermal> thermals;
    float turb[3] = {};   // north, east, down: first order Gauss-Markov
    float gust[3] = {};   // smoothed once more (1s), the glider does not see sharper accelerations
    float to_next = 1500.f, travelled = 0.f;
    float lift_f = 0.f, lift_prev = 0.f, lift_avg = 0.f, dir = 1.f;

    static float wn() { return -K.wind * std::cos(WIND_FROM); } // the air moves to WIND_FROM + 180°
    static float we() { return -K.wind * std::sin(WIND_FROM); }

    void horizontal(float t, float psi, float& n, float& e) {
        constexpr float dt = 0.01f, tau = TURB_H_L / 26.f;
        for (int i = 0; i < 2; ++i) {
            turb[i] += -turb[i] * dt / tau + K.turb * TURB_H * std::sqrt(2.f * dt / tau) * nd(rng);
            gust[i] += (turb[i] - gust[i]) * dt / 1.f;
        }
        n = wn() + gust[0];
        e = we() + gust[1];
    }

    float vertical(float t, float xa, float ya) {
        constexpr float dt = 0.01f, tau = TURB_V_L / 26.f;
        turb[2] += -turb[2] * dt / tau + K.turb * TURB_V * std::sqrt(2.f * dt / tau) * nd(rng);
        gust[2] += (turb[2] - gust[2]) * dt / 1.f;
        float wd = SINK + gust[2];
        for (const Thermal& th : thermals) {
            const float d2 = ((xa - th.x) * (xa - th.x) + (ya - th.y) * (ya - th.y)) / (th.r * th.r);
            if (d2 < 9.f) {
                wd -= (th.w + SINK) * std::exp(-d2);
            }
        }
        return wd;
    }

    void pilot(const Truth& tr, float lift, float dt, float& bank_deg, float& v_target);
};
NlDay g_nl;

void NlDay::pilot(const Truth& tr, float lift, float dt, float& bank_deg, float& v_target) {
    lift_f += (lift - lift_f) * dt / 1.0f;           // the pilot's vario
    lift_avg += (lift - lift_avg) * dt / 20.f;       // the pilot's impression of the thermal
    const float lift_rate = (lift_f - lift_prev) / dt;
    lift_prev = lift_f;
    g_pilot.mode_t += dt;
    if (g_pilot.mode == 0) {
        // cruise legs of 15 minutes on three headings, the next thermal somewhere ahead
        static constexpr float LEGS[3] = {30.f, 150.f, 270.f};
        const float course = LEGS[int(tr.t / 900.f) % 3] * D2R;
        const float err = std::remainder(course - tr.psi, 6.2831853f) / D2R;
        bank_deg = std::clamp(1.5f * err, -30.f, 30.f);
        v_target = tr.h < 500.f ? V_CLIMB + 2.f : V_CRUISE;
        if (K.dolphin > 0.f) {
            v_target = std::clamp(v_target - 6.f * lift_f, V_CLIMB, V_CRUISE + 8.f); // slow in lift, fast in sink
        }
        travelled += tr.v * dt;
        if (travelled > to_next) {
            const float ahead = 350.f, side = (ud(rng) - 0.5f) * 240.f;
            thermals.push_back({tr.xa + ahead * std::cos(tr.psi) - side * std::sin(tr.psi), tr.ya + ahead * std::sin(tr.psi) + side * std::cos(tr.psi),
                                120.f + 130.f * ud(rng), K.lift * (1.f + 2.f * ud(rng))});
            travelled = 0.f;
            to_next = 1500.f + std::min(-2500.f * std::log(std::max(ud(rng), 1e-3f)), 3500.f);
        }
        const float enter = tr.h < 500.f ? 0.4f : 0.8f;
        if (lift_f > enter && tr.h < CLOUD_BASE - 100.f && g_pilot.mode_t > 20.f) {
            g_pilot.mode = 1;
            g_pilot.mode_t = 0.f;
            g_pilot.thermal = g_pilot.thermals++;
            dir = ud(rng) < 0.5f ? -1.f : 1.f;
            lift_avg = lift_f;
        }
    } else {
        // centering: straighten while the lift increases
        bank_deg = dir * (lift_rate > 0.05f ? 25.f : 42.f);
        v_target = V_CLIMB;
        const bool weak = g_pilot.mode_t > 120.f && lift_avg < 1.0f && tr.h > 450.f;
        if (tr.h > CLOUD_BASE || weak) {
            g_pilot.mode = 0;
            g_pilot.mode_t = 0.f;
            g_pilot.thermal = -1;
        }
    }
}

std::vector<Scenario> scenarios() {
    std::vector<Scenario> s;

    {   // straight flight with S-turns in a constant wind and calm air
        Scenario c;
        c.name = "straight";
        c.segments = {{60, 0, 30}, {20, 30, 30}, {20, -30, 30}, {20, 30, 30}, {20, -30, 30}, {120, 0, 35}, {20, 25, 35}, {20, -25, 35}, {100, 0, 28}};
        c.eval_wind_after = 180.f;
        c.steady = {{150, 190}, {260, 290}, {380, 400}};
        c.max_time_to_wind_valid = 160.f;
        c.check_latency = true;
        s.push_back(c);
    }
    {   // thermalling: glide in, circle in a thermal, glide out
        Scenario c;
        c.name = "thermal";
        c.segments = {{60, 0, 32}, {240, 40, 25}, {120, 0, 35}};
        c.air.vertical = [](float t) {
            // -2.5 m/s (lift) while circling, smooth 5s edges
            auto edge = [](float x) { return 0.5f * (1.f + std::tanh(x / 2.f)); };
            return -2.5f * edge(t - 62.f) * (1.f - edge(t - 300.f));
        };
        c.eval_wind_after = 150.f;
        c.steady = {{40, 58}, {120, 290}, {340, 420}};
        c.max_time_to_wind_valid = 150.f;
        s.push_back(c);
    }
    {   // horizontal gust in straight flight: the TE vario shows a false climb
        Scenario c;
        c.name = "gust";
        c.segments = {{90, 30, 28}, {90, -30, 28}, {200, 0, 32}};
        c.air.horizontal = [](float t, float psi, float& wn, float& we) {
            // 4 m/s head wind gust at t=250s in straight flight: 2s ramp up, 10s hold, 4s ramp down
            float g = 0.f;
            if (t > 250.f && t < 252.f) g = (t - 250.f) / 2.f;
            else if (t >= 252.f && t < 262.f) g = 1.f;
            else if (t >= 262.f && t < 266.f) g = 1.f - (t - 262.f) / 4.f;
            wn = -4.f - 4.f * g * std::cos(psi); // head wind: air moves against the flight direction
            we = 3.f - 4.f * g * std::sin(psi);
        };
        c.eval_wind_after = 200.f;
        c.steady = {{200, 248}, {290, 380}};
        c.spikes = {{249, 275}};
        // the energy balance takes the inertial acceleration in gusts (kinetic blend): 1.3 m/s against TE netto's 3.9; it
        // was 0.5 with the energy balance down weighted in gusts, which read climbs 0.17 m/s low on real flights
        c.max_spike = 2.2f; // 2.1 with sigma_pitch 0.03 and kinetic_tau 5, the real-flight defaults (TE netto 4.0)
        c.max_wind_rms = 1.4f; // the wind estimate cannot follow a 4 m/s step within 2s, the evaluation includes it
        s.push_back(c);
    }
    {   // stick thermal: speed reduction pull-up in calm air
        Scenario c;
        c.name = "pullup";
        c.segments = {{90, 30, 30}, {90, -30, 30}, {60, 0, 45}, {30, 0, 25}, {60, 0, 40}, {30, 0, 25}, {60, 0, 30}};
        c.v_tau = 1.2f;
        c.eval_wind_after = 200.f;
        c.steady = {{220, 238}, {375, 390}};
        c.spikes = {{240, 270}, {330, 360}};
        // the simulated TE is ideal (no probe lag); the gyro propagated attitude keeps the pitch through
        // the zoom (the device AHRS drifts by ~8°, 3.2 m/s spike), so the air mass vario beats it (0.6..0.7 against 0.8)
        c.max_spike = 0.8f;
        s.push_back(c);
    }
    {   // uncoordinated flight: alternating sideslip of 8° in straight flight and while turning
        Scenario c;
        c.name = "slip";
        c.segments = {{90, 30, 28}, {90, -30, 28}, {120, 0, 30}, {60, 25, 28}, {60, 0, 32}, {60, -25, 28}};
        c.slip = [](float t) {
            if (t < 200.f) {
                return 0.f;
            }
            const float phase = std::fmod(t - 200.f, 40.f);
            return phase < 20.f ? (std::fmod(t - 200.f, 80.f) < 40.f ? 8.f : -8.f) * D2R : 0.f;
        };
        c.eval_wind_after = 200.f;
        c.steady = {{200, 480}};
        c.max_wind_rms = 1.2f; // without the slip aid 1.3 m/s
        s.push_back(c);
    }
    {   // one hour cross country on a typical Dutch day, see NlDay
        Scenario c;
        c.name = "nl";
        c.duration = 3600.f;
        c.pilot = [](const Truth& tr, float lift, float dt, float& bank, float& v) { g_nl.pilot(tr, lift, dt, bank, v); };
        c.vertical_at = [](float t, float xa, float ya) { return g_nl.vertical(t, xa, ya); };
        c.air.horizontal = [](float t, float psi, float& wn, float& we) { g_nl.horizontal(t, psi, wn, we); };
        c.mean_wind[0] = NlDay::wn();
        c.mean_wind[1] = NlDay::we();
        c.v_tau = 1.5f;
        c.eval_wind_after = 300.f;
        c.max_wind_rms = 3.f;           // against the turbulent truth, informative only
        c.max_mean_wind_rms = 0.8f;     // the averaged wind for the navigation; 1Hz GNSS in 1 m/s turbulence
        c.max_climb_err_rms = 0.3f;
        c.max_time_to_wind_valid = 400.f;
        s.push_back(c);
    }
    {   // 3h endurance with mixed flight for numerical stability
        Scenario c;
        c.name = "endurance";
        for (int i = 0; i < 45; ++i) {
            c.segments.push_back({60, 0, 30.f + (i % 4) * 4.f});
            c.segments.push_back({120, (i % 2) ? 40.f : -40.f, 25});
            c.segments.push_back({60, (i % 3) ? 20.f : -20.f, 33});
        }
        c.air.horizontal = [](float t, float, float& wn, float& we) {
            wn = -4.f + 2.f * std::sin(t / 1800.f);
            we = 3.f + 2.f * std::cos(t / 2400.f);
        };
        // lift (Wd < 0) about the glider's sink: three hours at altitude, not 10 km below sea level (Plausibility::ALT_MIN)
        c.air.vertical = [](float t) { return -1.f + 1.2f * std::sin(t / 47.f) * std::sin(t / 13.f); };
        c.eval_wind_after = 300.f;
        c.max_wind_rms = 0.4f;   // 0.16 (0.25 with a latency error); 1.0 with a 2% clock error past one hour
        c.steady = {{600, 10800}};
        c.max_vario_rms = 0.25f; // 0.13
        c.check_latency = true;
        s.push_back(c);
    }
    return s;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <scenario> [--lat-err ms] [--static-adj %%] [--comp %%] [--compass deg] "
                             "[--slip-aid 0|1] [--max-wind-valid s] [--max-wind-rms m/s] [--csv file]\n", argv[0]);
        return 2;
    }
    const std::string which = argv[1];
    FILE* csv = nullptr;
    airmass::replay::Settings cfg_settings; // --set cfg.<name>=value: estimator settings by name
    const char* csv_name = nullptr;
    const char* record_name = nullptr;
    int lat_err = 0;
    float static_adj = 1.f, comp = 1.f;
    float compass_sigma = -1.f, max_wind_valid = -1.f, max_wind_rms = -1.f, max_climb_err = -1.f;
    bool slip_aid = true;
    unsigned seed = 4711;
    float vario_agility = 1.f;
    float warm[4] = {1.f, 0.f, 1.f, 0.75f};
    float flap_warm[airmass::FlapOffsets::N];
    std::fill(std::begin(flap_warm), std::end(flap_warm), airmass::FlapOffsets::UNKNOWN);
    bool learn = true;
    for (int i = 2; i < argc; ++i) {
        learn = learn && std::strcmp(argv[i], "--no-learn") != 0;
    }
    for (int i = 2; i + 1 < argc; i += 2) {
        if (std::strcmp(argv[i], "--lat-err") == 0) {
            lat_err = std::atoi(argv[i + 1]);
        } else if (std::strcmp(argv[i], "--static-adj") == 0) {
            static_adj = std::atof(argv[i + 1]) / 100.f;
        } else if (std::strcmp(argv[i], "--comp") == 0) {
            comp = std::atof(argv[i + 1]) / 100.f;
        } else if (std::strcmp(argv[i], "--compass") == 0) {
            compass_sigma = std::atof(argv[i + 1]) * D2R;
        } else if (std::strcmp(argv[i], "--slip-aid") == 0) {
            slip_aid = std::atoi(argv[i + 1]) != 0;
        } else if (std::strcmp(argv[i], "--max-wind-valid") == 0) {
            max_wind_valid = std::atof(argv[i + 1]);
        } else if (std::strcmp(argv[i], "--set") == 0 && std::strncmp(argv[i + 1], "cfg.", 4) == 0) {
            // an estimator setting by name (AirMassEKF::Config), e.g. --set cfg.gust_acc=1000
            std::string error;
            if (!airmass::replay::parseSettings(argv[i + 1] + 4, cfg_settings, &error)) {
                std::fprintf(stderr, "%s\n", error.c_str());
                return 2;
            }
        } else if (std::strcmp(argv[i], "--set") == 0) {
            const char* eq = std::strchr(argv[i + 1], '=');
            bool known = false;
            for (const auto& [name, ptr] : KNOBS) {
                if (eq && std::strncmp(argv[i + 1], name, eq - argv[i + 1]) == 0 && std::strlen(name) == size_t(eq - argv[i + 1])) {
                    *ptr = std::atof(eq + 1);
                    known = true;
                }
            }
            if (!known) {
                std::fprintf(stderr, "unknown setting %s\n", argv[i + 1]);
                return 2;
            }
        } else if (std::strcmp(argv[i], "--warm") == 0) {
            std::sscanf(argv[i + 1], "%f,%f,%f,%f", &warm[0], &warm[1], &warm[2], &warm[3]);
        } else if (std::strcmp(argv[i], "--flap-warm") == 0) {
            // learned pitch offsets per flap position of an earlier flight [rad]
            std::sscanf(argv[i + 1], "%f,%f,%f,%f,%f", &flap_warm[0], &flap_warm[1], &flap_warm[2], &flap_warm[3], &flap_warm[4]);
        } else if (std::strcmp(argv[i], "--no-learn") == 0) {
            learn = false;
            --i; // takes no value
        } else if (std::strcmp(argv[i], "--vario-agility") == 0) {
            vario_agility = std::atof(argv[i + 1]);
        } else if (std::strcmp(argv[i], "--seed") == 0) {
            seed = (unsigned)std::atoi(argv[i + 1]);
        } else if (std::strcmp(argv[i], "--max-climb-err") == 0) {
            max_climb_err = std::atof(argv[i + 1]);
        } else if (std::strcmp(argv[i], "--max-wind-rms") == 0) {
            max_wind_rms = std::atof(argv[i + 1]);
        } else if (std::strcmp(argv[i], "--record") == 0) {
            record_name = argv[i + 1];
        } else if (std::strcmp(argv[i], "--csv") == 0) {
            csv_name = argv[i + 1];
            csv = std::fopen(csv_name, "w");
        }
    }

    if (K.predict_hz != 10.f && K.predict_hz != 20.f) {
        std::fprintf(stderr, "predict_hz must be 10 or 20\n");
        return 2;
    }
    Scenario sc;
    bool found = false;
    for (auto& s : scenarios()) {
        if (s.name == which) {
            sc = s;
            found = true;
        }
    }
    if (!found) {
        std::fprintf(stderr, "unknown scenario %s\n", which.c_str());
        return 2;
    }
    if (max_wind_valid > 0.f) {
        sc.max_time_to_wind_valid = max_wind_valid;
    }
    if (max_wind_rms > 0.f) {
        sc.max_wind_rms = max_wind_rms;
    }
    if (max_climb_err > 0.f) {
        sc.max_climb_err_rms = max_climb_err;
    }

    std::mt19937 rng(seed);
    std::normal_distribution<float> nd(0.f, 1.f);

    const float dt_truth = 0.01f;
    const int sensor_div = 10;                      // 10Hz sensor loop
    const int gnss_div = 100;           // 1Hz NMEA from the Flarm
    const uint16_t gnss_latency = 750;  // ms
    const float gnss_sigma = 0.3f;

    g_nl.rng.seed(20240601u + static_cast<unsigned>(K.day));
    float total = sc.duration;
    for (auto& s : sc.segments) total += s.duration;

    Truth tr;
    tr.psi = 0.3f;
    if (sc.pilot) {
        tr.h = 900.f; // released from the winch / tow
        tr.v = NlDay::V_CRUISE;
    }
    // the device's processing (AirMassSensor): the same pipeline, setup and warm start
    airmass::Pipeline pipe;
    airmass::Setup setup;
    setup.mass = MASS * K.mass_err;
    setup.wing_area = AREA;
    setup.polar = devicePolar();
    setup.vario_resp = vario_agility;
    setup.te_adjust = comp * 100.f;
    pipe.setLearning(learn);
    pipe.configure(setup);
    airmass::Pipeline::Options opts;
    opts.slip_aid = slip_aid;
    pipe.setOptions(opts);
    std::string unknown;
    if (!airmass::replay::applySettings(pipe.ekf(), cfg_settings, &unknown)) {
        std::fprintf(stderr, "unknown estimator setting %s\n", unknown.c_str());
        return 2;
    }
    airmass::Warm warm_start;
    warm_start.k_tas = warm[0];
    warm_start.pitch_ofs = warm[1];
    warm_start.k_aoa = warm[2];
    warm_start.latency = warm[3];
    std::copy(std::begin(flap_warm), std::end(flap_warm), std::begin(warm_start.flap_ofs));
    AirMassEKF& ekf = pipe.ekf();
    FILE* rec = record_name ? std::fopen(record_name, "w") : nullptr;
    if (record_name && !rec) {
        std::fprintf(stderr, "cannot write %s\n", record_name);
        return 2;
    }
    unsigned rec_count = 0; // the 5Hz output, like AirMassSensor's 200 ms output timer

    // raw IMU into the device AHRS, whose attitude starts the estimator
    DeviceAhrs ahrs;
    bool ekf_started = false;
    airmass::Frame held_frame;
    float prev_vg[3] = {NAN, NAN, NAN};
    float prev_att[3] = {NAN, NAN, NAN};
    vector_f f_sum, w_sum;
    LowPassFilterT<float> am_display(LowPassFilterT<float>::alphaFromTau(DAMPING / 3.f * 0.6f, 0.1f));
    TeDamper te_damp;
    bool am_started = false;
    Lpf te_alt(0.5f, 0.1f);
    float prev_te_alt = NAN;
    struct GnssSample { uint32_t t_obs; float vn, ve; };
    std::vector<GnssSample> gnss_q;

    Stats wind_err, vario_err, te_err, spike_err, te_spike_err, avg_wind_err;
    // the GNSS/INS reference instrument (ins_reference.h) with its own 10 Hz GNSS and sensors; its own random numbers,
    // so it does not change the noise of ours. Its GNSS is not hit by the outage of the Flarm (--set gps_out).
    ref::InsReference ins;
    std::mt19937 rng_ref(seed + 1000003u);
    struct RefFix { uint32_t t; float v[3]; float alt; };
    std::vector<RefFix> ref_q;
    float ref_v[3] = {}, ref_alt = NAN, ref_att_err[3] = {}, ref_alt_err = 0.f;
    constexpr float REF_ATT_SIGMA[3] = {0.3f * D2R, 0.3f * D2R, 0.7f * D2R}; // roll, pitch, heading of its AHRS
    constexpr float REF_ATT_TAU = 30.f, REF_ALT_SIGMA = 1.5f, REF_ALT_TAU = 20.f;
    constexpr float REF_VEL_SIGMA = 0.08f, REF_VZ_SIGMA = 0.12f; // m/s, u-blox speed accuracy
    constexpr uint32_t REF_LATENCY_MS = 100;
    Stats ins_wind_err, ins_vario_err, ins_spike_err, ins_avg_wind_err;
    const airmass::WindAverage& avg_wind = pipe.avgWind();
    float t_wind_valid = -1.f;
    float prev_gamma = 0.f, prev_wn = NAN, prev_we = NAN, prev_wd = NAN;
    int flap_lever = -1;
    float rec_since = 0.f;
    float flap_defl = 0.f;
    airmass::FlapOffsets& flap_ofs = pipe.flaps();
    const bool flap_offsets = K.flaps > 0.f && K.flap_aid > 0.f;
    float vdot_l = 0.f;
    int restarts = 0;
    float w_eff = NAN;          // m/s, the vertical air movement the glider's inertial motion has taken up (gust response)
    float load_p = 1.f;         // the load factor the pilot flies (turn, path curvature), without the gust load
    float prev_gamma_ss = 0.f;  // its flight path angle
    float beta = 0.f;           // rad, sideslip: the pilot's slip and the adverse yaw, through the yaw dynamics
    float prev_roll = NAN;
    float static_alt = NAN, pitot_v2 = NAN; // the air data at the sensors, through the pneumatic lag

    // pilot driven scenarios: the vario as the pilot reads it
    struct FalseLift {
        int events = 0;
        float above = 0.f;
        bool counted = false;
        void add(float err, float dt) {
            if (err > 1.f) {
                above += dt;
                if (above >= 1.f && !counted) {
                    events++;
                    counted = true;
                }
            } else if (err < 0.5f) {
                above = 0.f;
                counted = false;
            }
        }
    };
    struct PilotEval {
        // [0] air mass, [1] TE netto, [2] the GNSS/INS reference
        Stats cruise[3], entry[3], circle[3], avg_wind, avg_mean[2], avg_truth_dev, inst_wind;
        Stats ins_inst_wind, ins_avg_wind, ins_avg_mean[2], ins_inst_all, ins_avg_all, inst_all;
        // the instant wind as shown (valid, or degraded), the reference on the same samples
        Stats wind_shown, ins_wind_shown, wind_degraded, last_valid_degraded, ins_degraded;
        float last_valid[2] = {NAN, NAN}; // the last valid wind, what holding it would show
        int wind_shown_n = 0;
        // AirMassSensor's averaged wind (navigation) against the 30s average of the truth and against the mean wind
        // (straight flight / circling), and the instant wind against the trackable part of the turbulent truth
        Lpf avg_tn{30.f, 0.1f}, avg_te{30.f, 0.1f}, true_n{5.f, 0.1f}, true_e{5.f, 0.1f};
        FalseLift false_lift[3];
        int valid_n = 0, total_n = 0, dropouts = 0;
        bool was_valid = true;
        std::vector<double> th_sum[3];
        std::vector<int> th_n;
        std::vector<float> truth, shown[3];
    } pe;

    int step = 0;
    size_t seg = 0;
    float seg_t = 0.f;
    for (; tr.t < total; ++step, tr.t += dt_truth, seg_t += dt_truth) {
        float bank_deg = 0.f, v_target = tr.v;
        if (sc.pilot) {
            sc.pilot(tr, -tr.wd, dt_truth, bank_deg, v_target);
        } else {
            if (seg_t >= sc.segments[seg].duration && seg + 1 < sc.segments.size()) {
                seg++;
                seg_t = 0.f;
            }
            bank_deg = sc.segments[seg].bank_deg;
            v_target = sc.segments[seg].v_target;
        }

        // pilot: roll rate limited, speed first order
        // sideslip: its side force (body y, opposing it) is balanced like in a real slip, the pilot lowers the wing into
        // the relative wind so the commanded turn stays (straight flight stays straight)
        // sideslip in body axes, like the angle of attack: the air velocity is v (cos a cos b, sin b, sin a cos b) in the body
        const float slip_pilot = sc.slip(tr.t);
        const float slip_hold_acc = -0.5f * RHO * tr.v * tr.v * AREA * CY_TRUE * slip_pilot / MASS;
        const float roll_target = bank_deg * D2R + std::asin(std::clamp(-slip_hold_acc / G, -0.5f, 0.5f));
        if (K.flaps > 0.f) {
            if (K.flap_pilot > 0.f) {
                // follows the flap box when it has shown another position for 2 s
                const int rec = flapRecommendation(tr.v, tr.load);
                rec_since = (rec == flap_lever) ? tr.t : rec_since;
                if (flap_lever < 0 || tr.t - rec_since > 2.f) {
                    flap_lever = rec;
                }
            } else {
                flap_lever = flapLever(flap_lever, tr.v, bank_deg, g_pilot.mode == 1 && sc.pilot);
            }
            // the flaps follow the lever at about 30°/s
            const float target = FLAP_DEG[flap_lever] * D2R;
            flap_defl += std::clamp(target - flap_defl, -30.f * D2R * dt_truth, 30.f * D2R * dt_truth);
        }
        tr.roll += std::clamp(roll_target - tr.roll, -15.f * D2R * dt_truth, 15.f * D2R * dt_truth);
        // adverse yaw: rolling into a turn swings the nose out of it (air from the side the glider rolls to); the
        // sideslip follows through the yaw dynamics, the nose turns against the flight path so that stays smooth
        const float roll_rate = std::isnan(prev_roll) ? 0.f : (tr.roll - prev_roll) / dt_truth;
        prev_roll = tr.roll;
        const float beta_prev = beta;
        beta += (slip_pilot + K.adverse_yaw * roll_rate - beta) * dt_truth / std::max(K.yaw_tau, dt_truth);
        tr.psi -= (beta - beta_prev) * std::cos(tr.roll);
        const float side_acc = -0.5f * RHO * tr.v * tr.v * AREA * CY_TRUE * beta / MASS;
        // speed changes ramp in through the pitch dynamics of the glider
        const float vdot_cmd = std::clamp((v_target - tr.v) / sc.v_tau, -3.f, 3.f);
        vdot_l += (vdot_cmd - vdot_l) * dt_truth / 0.8f;

        // air mass
        float wn, we;
        sc.air.horizontal(tr.t, tr.psi, wn, we);
        if (std::isnan(prev_wn)) {
            prev_wn = wn;
            prev_we = we;
        }
        tr.wdot[0] = (wn - prev_wn) / dt_truth;
        tr.wdot[1] = (we - prev_we) / dt_truth;
        prev_wn = wn;
        prev_we = we;
        tr.wn = wn;
        tr.we = we;
        tr.wd = sc.vertical_at ? sc.vertical_at(tr.t, tr.xa, tr.ya) : sc.air.vertical(tr.t);
        // riding a rising or sinking air mass takes lift: the vertical acceleration of the air
        const float wd_dot = std::isnan(prev_wd) ? 0.f : (tr.wd - prev_wd) / dt_truth;
        prev_wd = tr.wd;

        // vertical gust: the inertial velocity is continuous, so a change of the vertical air movement first turns the
        // relative wind (the angle of attack, in a bank partly the sideslip); the extra lift then accelerates the glider
        // with the plunge time constant. gust: the air relative descent not taken up yet (down positive)
        const float c_roll = std::cos(tr.roll), s_roll = std::sin(tr.roll);
        if (std::isnan(w_eff) || K.gust_resp <= 0.f) {
            w_eff = tr.wd;
        }
        const float gust = w_eff - tr.wd;
        const float tau_plunge = 2.f * MASS / (RHO * std::max(tr.v, 10.f) * AREA * K.cla_true);
        const float gust_load = gust * c_roll / (tau_plunge * G); // the lift of the angle of attack change
        w_eff += (tr.wd - w_eff) * c_roll * c_roll * dt_truth / tau_plunge;

        // a horizontal gust changes TAS directly (inertial speed is kept), the pilot then restores speed; the steeper
        // air relative path in a vertical gust (gust) speeds the glider up by gravity
        float va_dot_wdot = tr.va[0] * tr.wdot[0] + tr.va[1] * tr.wdot[1];
        if (K.gust_resp > 0.f) {
            va_dot_wdot += tr.va[2] * wd_dot;
        }
        tr.vdot = vdot_l - va_dot_wdot / std::max(tr.v, 1.f) + G * gust / std::max(tr.v, 1.f);
        tr.v += tr.vdot * dt_truth;

        // load factor from turn and path curvature, smoothed by the pitch dynamics of the glider, plus the gust load
        // (without the gust response the glider rides the vertical air acceleration directly)
        const float cosg = std::cos(prev_gamma_ss);
        const float ride = K.gust_resp > 0.f ? 0.f : wd_dot / G;
        const float load_cmd = std::clamp((cosg + tr.v * (prev_gamma_ss - prev_gamma) / dt_truth / G - ride) / c_roll, 0.3f, 3.f);
        load_p += (load_cmd - load_p) * dt_truth / 0.4f;
        tr.load = std::clamp(load_p + gust_load, 0.2f, 4.f);
        prev_gamma = prev_gamma_ss;

        // energy balance: air relative sink rate
        // sideslip drag (fuselage and fin side force, induced): delta CD = CD_BETA * beta², the sink rate increase is D V / (m g)
        const float slip_sink = 0.5f * RHO * tr.v * tr.v * AREA * CD_BETA * beta * beta * tr.v / (MASS * G);
        const float va_d = polar(tr.v, tr.load) + slip_sink + tr.v * tr.vdot / G + va_dot_wdot / G;
        tr.gamma_up = -std::asin(std::clamp(va_d / tr.v, -0.9f, 0.9f));
        prev_gamma_ss = -std::asin(std::clamp((va_d - gust) / tr.v, -0.9f, 0.9f)); // the path the pilot flies
        const float cl = 2.f * tr.load * MASS * G / (RHO * AREA * tr.v * tr.v);
        const float cl_bend = std::max(cl - (0.9f + 1.5f * flap_defl), 0.f);
        tr.alpha = cl / K.cla_true + ALPHA0 - K.flap_alpha0 * flap_defl + K.aoa_nonlin * cl_bend * cl_bend;
        // the angle of attack change of the gust comes with the gust load (CL), its side part is sideslip
        const float beta_air = beta + gust * s_roll / std::max(tr.v, 1.f);
        tr.pitch = solvePitch(tr.v, tr.roll, tr.alpha, beta_air, va_d);
        trueAirVelocity(tr.v, tr.roll, tr.pitch, tr.psi, tr.alpha, beta_air, tr.va);

        tr.vg[0] = tr.va[0] + tr.wn;
        tr.vg[1] = tr.va[1] + tr.we;
        tr.vg[2] = tr.va[2] + tr.wd;

        // raw IMU: specific force and body rates from the inertial acceleration and the attitude change
        if (!std::isnan(prev_vg[0])) {
            const float f_nav[3] = {(tr.vg[0] - prev_vg[0]) / dt_truth / G, (tr.vg[1] - prev_vg[1]) / dt_truth / G,
                                    (tr.vg[2] - prev_vg[2]) / dt_truth / G - 1.f};
            float f_b[3];
            navToBody(tr.roll, tr.pitch, tr.psi, f_nav, f_b);
            const float rd = (tr.roll - prev_att[0]) / dt_truth, pd = (tr.pitch - prev_att[1]) / dt_truth;
            const float yd = (tr.psi - prev_att[2]) / dt_truth;
            const float sr = std::sin(tr.roll), cr = std::cos(tr.roll), sp = std::sin(tr.pitch), cp = std::cos(tr.pitch);
            const float w_b[3] = {rd - yd * sp, pd * cr + yd * sr * cp, -pd * sr + yd * cr * cp};
            f_sum += mountRotate(f_b);
            w_sum += mountRotate(w_b);

            // the reference instrument at 100 Hz: its attitude errors (slowly varying), GNSS fixes with latency
            for (int i = 0; i < 3; ++i) {
                ref_att_err[i] += -ref_att_err[i] * dt_truth / REF_ATT_TAU + REF_ATT_SIGMA[i] * std::sqrt(2.f * dt_truth / REF_ATT_TAU) * nd(rng_ref);
            }
            ref_alt_err += -ref_alt_err * dt_truth / REF_ALT_TAU + REF_ALT_SIGMA * std::sqrt(2.f * dt_truth / REF_ALT_TAU) * nd(rng_ref);
            if (step % 10 == 0) {
                ref_q.push_back({(uint32_t)step * 10u,
                                 {tr.vg[0] + REF_VEL_SIGMA * nd(rng_ref), tr.vg[1] + REF_VEL_SIGMA * nd(rng_ref), tr.vg[2] + REF_VZ_SIGMA * nd(rng_ref)},
                                 tr.h + ref_alt_err + 0.3f * nd(rng_ref)});
            }
            while (!ref_q.empty() && ref_q.front().t + REF_LATENCY_MS <= (uint32_t)step * 10u) {
                std::copy(std::begin(ref_q.front().v), std::end(ref_q.front().v), ref_v);
                ref_alt = ref_q.front().alt;
                ref_q.erase(ref_q.begin());
            }
            if (std::isfinite(ref_alt)) {
                // specific force in NED, tilted by its attitude error
                const float cy = std::cos(tr.psi), sy = std::sin(tr.psi);
                const float e[3] = {ref_att_err[0] * cy - ref_att_err[1] * sy, ref_att_err[0] * sy + ref_att_err[1] * cy, ref_att_err[2]};
                float fn[3], acc[3];
                for (int i = 0; i < 3; ++i) fn[i] = f_nav[i] * G;
                acc[0] = fn[0] + e[1] * fn[2] - e[2] * fn[1] + 0.003f * G * nd(rng_ref);
                acc[1] = fn[1] + e[2] * fn[0] - e[0] * fn[2] + 0.003f * G * nd(rng_ref);
                acc[2] = fn[2] + e[0] * fn[1] - e[1] * fn[0] + 0.003f * G * nd(rng_ref);
                const float th = tr.pitch + ref_att_err[1], ps = tr.psi + ref_att_err[2];
                const float xb[3] = {std::cos(th) * std::cos(ps), std::cos(th) * std::sin(ps), -std::sin(th)};
                // its airspeed: the same installation error, the noise of a 100 Hz reading
                const float tas_ref = std::sqrt(pitot_v2) / K.k_tas + K.tas_noise * std::sqrt(10.f) * nd(rng_ref);
                ins.update(ref_v, ref_alt, acc, xb, tas_ref, std::remainder(ps, 6.2831853f), tr.yaw_rate);
            }
        } else {
            f_sum += vector_f(0.f, 0.f, -1.f);
        }
        for (int i = 0; i < 3; ++i) {
            prev_vg[i] = tr.vg[i];
        }
        prev_att[0] = tr.roll;
        prev_att[1] = tr.pitch;
        prev_att[2] = tr.psi;
        // turn rate from the body y force balance: the side force and gravity's part give the centripetal acceleration
        // (coordinated without sideslip: g tan(roll) / V); the IMU then reads the side force, consistent with the motion
        tr.yaw_rate = (side_acc + G * std::sin(tr.roll) * std::cos(tr.pitch)) / (tr.v * std::cos(tr.roll));
        tr.yaw_rate += gust_load * G * std::sin(tr.roll) / tr.v; // the gust lift tightens the turn
        // air data at the sensors: static port error (installation, with the angle of attack), pneumatic lags
        const float cp_static = (static_adj - 1.f) + K.static_cl * (cl - 0.6f);
        const float alt_port = tr.h - cp_static * tr.v * tr.v / (2.f * G);
        static_alt = std::isnan(static_alt) ? alt_port : static_alt + (alt_port - static_alt) * dt_truth / std::max(K.lag_static, dt_truth);
        pitot_v2 = std::isnan(pitot_v2) ? tr.v * tr.v : pitot_v2 + (tr.v * tr.v - pitot_v2) * dt_truth / std::max(K.lag_pitot, dt_truth);
        tr.psi += tr.yaw_rate * dt_truth;
        tr.h -= tr.vg[2] * dt_truth;
        tr.xa += tr.va[0] * dt_truth;
        tr.ya += tr.va[1] * dt_truth;
        tr.gn += tr.vg[0] * dt_truth;
        tr.ge += tr.vg[1] * dt_truth;

        // GNSS observation enters a latency queue
        const bool gps_out = K.gps_out >= 0.f && tr.t >= K.gps_out && tr.t < K.gps_out + K.gps_out_len;
        if (step % gnss_div == 0 && !gps_out) {
            gnss_q.push_back({(uint32_t)step * 10u, tr.vg[0] + gnss_sigma * nd(rng), tr.vg[1] + gnss_sigma * nd(rng)});
        }

        if (step % sensor_div != 0) {
            if (K.predict_hz == 20.f && ekf_started && ekf.isRunning() && step % (sensor_div / 2) == 0) {
                held_frame.time_ms = (uint32_t)step * 10u;
                pipe.step(held_frame);
                pipe.endCycle();
                if (rec) std::fputs(airmass::record::frame(held_frame).c_str(), rec);
            }
            continue;
        }

        // 10Hz sensor loop: averaged IMU samples (digital low pass of the sensor) with noise and bias
        // device milliseconds from the step count: the float time tr.t loses resolution in long flights
        const uint32_t now = (uint32_t)step * 10u;
        const vector_f accel = f_sum * (1.f / sensor_div) +
                               vector_f(0.01f * nd(rng), 0.01f * nd(rng), K.acc_bias + 0.01f * nd(rng));
        vector_f gyro = w_sum * (1.f / sensor_div) +
                        vector_f(0.5f * K.gyro_bias + 0.002f * nd(rng), 0.5f * K.gyro_bias + 0.002f * nd(rng), K.gyro_bias + 0.002f * nd(rng));
        for (int i = 0; i < 3; ++i) {
            if (std::fabs(gyro[i]) < K.gyro_gate * D2R) {
                gyro[i] = 0.f;
            }
        }
        f_sum = w_sum = vector_f();
        const float tas_m = std::sqrt(pitot_v2) / K.k_tas + K.tas_noise * nd(rng);
        ahrs.update(accel, gyro, tas_m);

        // static port: its error and lag (above), sensor noise
        const float baro_alt = static_alt + 0.25f * nd(rng);
        const vector_f ahrs_down = ahrs.q.rotateInv({0, 0, 1});
        const float device_down[3] = {ahrs_down.x, ahrs_down.y, ahrs_down.z};
        // the device AHRS has converged long before the takeoff starts the estimator
        if (tr.t < AHRS_WARMUP) {
            while (!gnss_q.empty() && gnss_q.front().t_obs + gnss_latency + lat_err <= now) {
                gnss_q.erase(gnss_q.begin());
            }
            continue;
        }

        // like AirMassSensor::postProcess: start (or restart) from the device AHRS, then one cycle
        if (!ekf_started || !ekf.isRunning()) {
            restarts += ekf_started;
            airmass::record::Start st;
            st.t = now;
            st.alt = baro_alt;
            for (int i = 0; i < 3; ++i) {
                st.down[i] = device_down[i];
                st.gyro[i] = gyro[i];
            }
            st.warm = warm_start;
            pipe.start(st.t, st.alt, st.down, st.gyro, st.warm);
            if (rec) {
                std::fputs(airmass::record::setup(pipe.setup()).c_str(), rec);
                std::fputs(airmass::record::start(st).c_str(), rec);
            }
            ekf_started = true;
        }
        airmass::Frame frame;
        frame.time_ms = now;
        if (K.predict_hz == 20.f) {
            frame.sample_times = true;
            for (auto& stamp : frame.sample_ms) stamp = now;
        }
        for (int i = 0; i < 3; ++i) {
            frame.f_body[i] = accel[i];
            frame.gyro[i] = gyro[i];
        }
        frame.ias = frame.tas = tas_m; // sea level
        frame.baro_alt = baro_alt;
        if (flap_offsets) {
            // the lever sensor reads the flaps' position, between the notches while they move
            float lever = 0.f;
            for (int i = 1; i < 5 && flap_defl > FLAP_DEG[i - 1] * D2R; ++i) {
                lever = i - 1 + std::min((flap_defl / D2R - FLAP_DEG[i - 1]) / (FLAP_DEG[i] - FLAP_DEG[i - 1]), 1.f);
            }
            if (K.flap_aid == 2.f) {
                // no sensor (AirMassSensor's flapLever): the recommendation at the turn's load
                lever = flapRecommendation(tas_m, 1.f / std::max(std::cos(pipe.attitude().roll()), 0.3f));
            }
            frame.flap_lever = lever;
        }
        pipe.step(frame);
        held_frame = frame;
        if (rec) {
            std::fputs(airmass::record::frame(frame).c_str(), rec);
        }
        const AirMassEKF::Input& in = pipe.input();
        if (compass_sigma > 0.f && step % (sensor_div * 100) == 0 && std::fabs(tr.roll) < 0.17f) {
            const airmass::record::Heading h{now, std::remainder(tr.psi + COMPASS_DEV + compass_sigma * nd(rng), 6.2831853f), 0.15f};
            pipe.heading(h.psi, h.sigma);
            if (rec) {
                std::fputs(airmass::record::heading(h).c_str(), rec);
            }
        }

        while (!gnss_q.empty() && gnss_q.front().t_obs + gnss_latency + lat_err <= now) {
            airmass::record::Gnss g;
            g.v.time_ms = gnss_q.front().t_obs + gnss_latency + lat_err;
            g.v.latency_ms = gnss_latency;
            g.v.vn = gnss_q.front().vn;
            g.v.ve = gnss_q.front().ve;
            g.v.sigma_h = 0.5f;
            pipe.gnss(g.v, g.prior, g.wn0, g.we0);
            if (rec) {
                std::fputs(airmass::record::gnss(g).c_str(), rec);
            }
            gnss_q.erase(gnss_q.begin());
        }
        pipe.endCycle();
        if (rec && !(++rec_count % 2)) {
            std::fputs(airmass::record::output(ekf).c_str(), rec);
        }

        // classic TE netto reference (electronic compensation, perfect polar knowledge)
        const float te_h = te_alt.f(baro_alt + comp * tas_m * tas_m / (2.f * G));
        const float te = std::isnan(prev_te_alt) ? 0.f : (te_h - prev_te_alt) / 0.1f;
        prev_te_alt = te_h;
        // the device netto uses its polar (S2F) at the raw IAS and the body z g load (S2F::getSink(ias))
        const float te_polar = -setup.polar.sink(tas_m, std::max(-accel.z, 0.3f));
        const float te_netto = te_damp.filter(te) + te_polar;
        // the reference's total energy vario as netto, with the same polar as TE netto
        const float ins_netto = ins.vario() + te_polar;

        // evaluation
        const uint8_t st = ekf.status();
        const float vario_true = -tr.wd;
        if (!am_started) { // the needle starts at the first estimate
            am_display.reset(ekf.airmassVario());
            am_started = true;
        }
        const float am_vario = am_display.filter(ekf.airmassVario()); // as displayed
        if ((st & AirMassEKF::ST_WIND_VALID) && t_wind_valid < 0.f) {
            t_wind_valid = tr.t;
        }
        if (tr.t > sc.eval_wind_after) {
            const float ew = std::hypot(ekf.windN() - tr.wn, ekf.windE() - tr.we);
            wind_err.add(ew);
            ins_wind_err.add(std::hypot(ins.windN() - tr.wn, ins.windE() - tr.we));
            if (!sc.pilot) {
                avg_wind_err.add(std::hypot(avg_wind.windN() - tr.wn, avg_wind.windE() - tr.we));
                ins_avg_wind_err.add(std::hypot(ins.avgWindN() - tr.wn, ins.avgWindE() - tr.we));
            }
        }
        if (inWindows(tr.t, sc.steady)) {
            vario_err.add(am_vario - vario_true);
            te_err.add(te_netto - vario_true);
            ins_vario_err.add(ins_netto - vario_true);
        }
        if (inWindows(tr.t, sc.spikes)) {
            spike_err.add(am_vario - vario_true);
            te_spike_err.add(te_netto - vario_true);
            ins_spike_err.add(ins_netto - vario_true);
        }
        if (sc.pilot) {
            const float shown[3] = {am_vario, te_netto, ins_netto};
            pe.truth.push_back(vario_true);
            for (int k = 0; k < 3; ++k) {
                const float e = shown[k] - vario_true;
                pe.shown[k].push_back(shown[k]);
                if (g_pilot.mode == 0 && g_pilot.mode_t > 8.f) {
                    pe.cruise[k].add(e);
                    pe.false_lift[k].add(e, 0.1f);
                } else if (g_pilot.mode == 1 && g_pilot.mode_t <= 15.f) {
                    pe.entry[k].add(e);
                } else if (g_pilot.mode == 1) {
                    pe.circle[k].add(e);
                    if ((int)pe.th_n.size() <= g_pilot.thermal) {
                        pe.th_n.resize(g_pilot.thermal + 1, 0);
                        for (auto& sum : pe.th_sum) {
                            sum.resize(g_pilot.thermal + 1, 0.);
                        }
                    }
                    pe.th_sum[k][g_pilot.thermal] += e;
                    if (k == 0) {
                        pe.th_n[g_pilot.thermal]++;
                    }
                }
            }
            const float an = avg_wind.windN(), ae = avg_wind.windE();
            const float atn = pe.avg_tn.f(tr.wn), ate = pe.avg_te.f(tr.we);
            const float tn = pe.true_n.f(tr.wn), te = pe.true_e.f(tr.we);
            if (t_wind_valid >= 0.f && tr.t > t_wind_valid + 60.f) {
                // what the consumers get: the outputs are invalid while the wind is not valid
                const bool valid = st & AirMassEKF::ST_WIND_VALID;
                pe.valid_n += valid;
                pe.total_n++;
                pe.dropouts += pe.was_valid && !valid;
                pe.was_valid = valid;
                // the reference over the whole flight, it has no validity
                pe.ins_inst_all.add(std::hypot(ins.windN() - tn, ins.windE() - te));
                pe.ins_avg_all.add(std::hypot(ins.avgWindN() - atn, ins.avgWindE() - ate));
                pe.inst_all.add(std::hypot(ekf.windN() - tn, ekf.windE() - te)); // ours, also while not valid
                if (st & (AirMassEKF::ST_WIND_VALID | AirMassEKF::ST_WIND_DEGRADED)) {
                    pe.wind_shown_n++;
                    pe.wind_shown.add(std::hypot(ekf.windN() - tn, ekf.windE() - te));
                    pe.ins_wind_shown.add(std::hypot(ins.windN() - tn, ins.windE() - te));
                }
                if (st & AirMassEKF::ST_WIND_VALID) {
                    pe.last_valid[0] = avg_wind.windN();
                    pe.last_valid[1] = avg_wind.windE();
                } else if (st & AirMassEKF::ST_WIND_DEGRADED) {
                    pe.wind_degraded.add(std::hypot(ekf.windN() - tn, ekf.windE() - te));
                    pe.last_valid_degraded.add(std::hypot(pe.last_valid[0] - tn, pe.last_valid[1] - te));
                    pe.ins_degraded.add(std::hypot(ins.windN() - tn, ins.windE() - te));
                }
            }
            if (t_wind_valid >= 0.f && tr.t > t_wind_valid + 60.f && (st & AirMassEKF::ST_WIND_VALID)) {
                pe.avg_wind.add(std::hypot(an - atn, ae - ate));
                pe.avg_mean[g_pilot.mode].add(std::hypot(an - sc.mean_wind[0], ae - sc.mean_wind[1]));
                pe.avg_truth_dev.add(std::hypot(atn - sc.mean_wind[0], ate - sc.mean_wind[1]));
                pe.inst_wind.add(std::hypot(ekf.windN() - tn, ekf.windE() - te));
                // the reference on the same samples
                pe.ins_inst_wind.add(std::hypot(ins.windN() - tn, ins.windE() - te));
                pe.ins_avg_wind.add(std::hypot(ins.avgWindN() - atn, ins.avgWindE() - ate));
                pe.ins_avg_mean[g_pilot.mode].add(std::hypot(ins.avgWindN() - sc.mean_wind[0], ins.avgWindE() - sc.mean_wind[1]));
            }
        }
        if (csv) {
            std::fprintf(csv, "%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.4f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.1f,%.1f,%d,%.3f,%.4f,%.5f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,%.4f\n", tr.t, tr.wn, tr.we, ekf.windN(),
                         ekf.windE(), vario_true, am_vario, te_netto, std::remainder(ekf.heading() - tr.psi, 6.2831853f),
                         ekf.kTas(), ekf.pitchOffset(), ekf.accBias(), st, std::sqrt(ekf.sigma(0) * ekf.sigma(0) + ekf.sigma(1) * ekf.sigma(1)),
                         tr.roll / D2R, tr.vg[2], ekf.vDown(), ekf.airmassVario(), (in.roll - tr.roll) / D2R,
                         (in.pitch - tr.pitch) / D2R, tr.v, in.load_factor, tr.h, tr.xa, tr.ya, g_pilot.mode, te + te_polar,
                         ekf.sigma(AirMassEKF::PSI), ekf.yawRateBias(), tr.gn, tr.ge, ins.windN(), ins.windE(), ins_netto,
                         tas_m, te_polar); // 34, 35: the measured TAS and the polar sink TE netto adds (positive), for references
        }
    }
    if (rec) {
        std::fputs(airmass::record::stop((uint32_t)step * 10u).c_str(), rec);
        std::fclose(rec);
    }
    if (csv) {
        std::fclose(csv);
        // the thermals of a pilot driven scenario, in air mass coordinates
        if (sc.pilot && csv_name) {
            if (FILE* f = std::fopen((std::string(csv_name) + ".thermals").c_str(), "w")) {
                for (const auto& th : g_nl.thermals) {
                    std::fprintf(f, "%.1f,%.1f,%.1f,%.2f\n", th.x, th.y, th.r, th.w);
                }
                std::fclose(f);
            }
        }
    }

    // expected learned calibration
    const float k_err = ekf.kTas() - K.k_tas;
    const int lat_est_err = (int)std::lround(ekf.gnssLatency() * 1000.f) - (gnss_latency + lat_err);
    std::printf("[%s] wind valid after %.0fs, wind rms %.2f m/s (max %.2f), vario rms %.2f m/s (TE netto %.2f), "
                "spike max %.2f m/s (TE netto %.2f), k_tas %.3f (err %.3f), pitch ofs %.3f rad, aoa scale %.3f (true %.3f), acc bias %.3f, "
                "gnss rejects %u, held out %u/%u/%u/%u, restarts %d\n",
                sc.name.c_str(), t_wind_valid, wind_err.rms(), wind_err.maxabs, vario_err.rms(), te_err.rms(),
                spike_err.maxabs, te_spike_err.maxabs, ekf.kTas(), k_err, ekf.pitchOffset(), ekf.aoaScale(), CLA_MODEL / K.cla_true, ekf.accBias(),
                ekf.rejectedGnss(), pipe.heldOut(0), pipe.heldOut(1), pipe.heldOut(2), pipe.heldOut(3), restarts);

    std::printf("  reference (GNSS/INS): wind rms %.2f m/s, averaged wind rms %.2f, vario rms %.2f, spike max %.2f m/s\n", ins_wind_err.rms(),
                ins_avg_wind_err.rms(), ins_vario_err.rms(), ins_spike_err.maxabs);

    bool ok = true;
    auto check = [&](bool cond, const char* what) {
        if (!cond) {
            std::printf("  FAIL: %s\n", what);
            ok = false;
        }
    };
    check(restarts == 0, "filter restarted (numerical health)");
    check(ekf.covarianceHealthy(), "covariance unhealthy at end");
    check(t_wind_valid >= 0.f && t_wind_valid <= sc.max_time_to_wind_valid, "wind not valid in time");
    check(wind_err.rms() <= sc.max_wind_rms, "wind rms too large");
    if (vario_err.n) {
        check(vario_err.rms() <= sc.max_vario_rms, "steady vario rms too large");
    }
    if (spike_err.n) {
        check(spike_err.maxabs < te_spike_err.maxabs + sc.te_margin, "disturbance spike larger than TE netto (+margin)");
        if (sc.max_spike > 0.f) {
            check(spike_err.maxabs <= sc.max_spike, "disturbance spike too large");
        }
    }
    check(std::fabs(k_err) < 0.03f, "k_tas did not converge");
    if (flap_offsets) {
        flap_ofs.keep(ekf);
        // the offsets differ by the zero lift angle shift of the flaps; their common part is the mounting and AoA model
        std::printf("  pitch offset per flap position, against the 0° position (true shift in brackets):");
        for (int i = 0; i < 5; ++i) {
            if (flap_ofs.known(i) && flap_ofs.known(2)) {
                std::printf(" %+.1f° σ%.1f", (flap_ofs.offset(i) - flap_ofs.offset(2)) / D2R, flap_ofs.sigma(i) / D2R);
            } else {
                std::printf(" -");
            }
            std::printf(" (%+.1f°)", K.flap_alpha0 * FLAP_DEG[i]);
        }
        std::printf("\n  --flap-warm ");
        for (int i = 0; i < 5; ++i) {
            std::printf("%s%.4f", i ? "," : "", flap_ofs.offset(i));
        }
        std::printf("\n");
    }
    if (sc.pilot) {
        // per thermal average climb reading error, and the lag of the display
        Stats climb[3];
        int used = 0;
        for (size_t i = 0; i < pe.th_n.size(); ++i) {
            if (pe.th_n[i] >= 100) {
                used++;
                for (int k = 0; k < 3; ++k) {
                    climb[k].add(pe.th_sum[k][i] / pe.th_n[i]);
                }
            }
        }
        int lag[3] = {};
        for (int k = 0; k < 3; ++k) {
            double best = 1e30;
            for (int l = 0; l <= 40; ++l) {
                double sum = 0.;
                for (size_t i = 0; i + l < pe.truth.size(); ++i) {
                    const double e = pe.shown[k][i + l] - pe.truth[i];
                    sum += e * e;
                }
                if (sum < best) {
                    best = sum;
                    lag[k] = l;
                }
            }
        }
        std::printf("  pilot view (air mass / TE netto): cruise rms %.2f / %.2f, false lift events %d / %d, thermal entry rms %.2f / %.2f "
                    "(max %.2f / %.2f), circling rms %.2f / %.2f\n",
                    pe.cruise[0].rms(), pe.cruise[1].rms(), pe.false_lift[0].events, pe.false_lift[1].events, pe.entry[0].rms(), pe.entry[1].rms(),
                    pe.entry[0].maxabs, pe.entry[1].maxabs, pe.circle[0].rms(), pe.circle[1].rms());
        std::printf("  climb reading per thermal (%d thermals): rms %.2f / %.2f, max %.2f / %.2f; display lag %.1f / %.1f s\n", used,
                    climb[0].rms(), climb[1].rms(), climb[0].maxabs, climb[1].maxabs, lag[0] * 0.1f, lag[1] * 0.1f);
        std::printf("  wind: 30s average %.2f m/s off the averaged truth (max %.2f; the averaged truth itself is %.2f off the mean wind), "
                    "averaged wind off the mean wind %.2f / %.2f m/s (straight / circling), "
                    "instant %.2f m/s off the 5s smoothed truth; final altitude %.0f m\n"
                    "  wind valid %.0f%% of the flight (after the first minute valid), %d dropouts; instant over the whole flight %.2f m/s\n",
                    pe.avg_wind.rms(), pe.avg_wind.maxabs, pe.avg_truth_dev.rms(), pe.avg_mean[0].rms(), pe.avg_mean[1].rms(),
                    pe.inst_wind.rms(), tr.h, 100.f * pe.valid_n / std::max(pe.total_n, 1), pe.dropouts, pe.inst_all.rms());
        std::printf("  reference (GNSS/INS TE netto, 10 Hz GNSS): cruise rms %.2f, false lift events %d, thermal entry rms %.2f (max %.2f), "
                    "circling rms %.2f, climb reading rms %.2f (max %.2f), display lag %.1f s\n",
                    pe.cruise[2].rms(), pe.false_lift[2].events, pe.entry[2].rms(), pe.entry[2].maxabs, pe.circle[2].rms(), climb[2].rms(),
                    climb[2].maxabs, lag[2] * 0.1f);
        std::printf("  reference wind (where ours is valid / whole flight): instant %.2f / %.2f, 30s average %.2f / %.2f m/s, "
                    "averaged off the mean wind %.2f / %.2f (straight / circling)\n",
                    pe.ins_inst_wind.rms(), pe.ins_inst_all.rms(), pe.ins_avg_wind.rms(), pe.ins_avg_all.rms(), pe.ins_avg_mean[0].rms(),
                    pe.ins_avg_mean[1].rms());
        std::printf("  shown wind (valid or degraded): %.0f%% of the flight, instant %.2f m/s (reference on the same samples %.2f), "
                    "degraded part %.2f m/s (%.0f%% of the flight; the last valid average held %.2f, the reference %.2f there)\n",
                    100.f * pe.wind_shown_n / std::max(pe.total_n, 1), pe.wind_shown.rms(), pe.ins_wind_shown.rms(), pe.wind_degraded.rms(),
                    100.f * pe.wind_degraded.n / std::max(pe.total_n, 1), pe.last_valid_degraded.rms(), pe.ins_degraded.rms());
        if (sc.max_climb_err_rms > 0.f) {
            check(used >= 3 && climb[0].rms() <= sc.max_climb_err_rms, "thermal climb reading off");
        }
        if (sc.max_mean_wind_rms > 0.f) {
            check(pe.avg_wind.rms() <= sc.max_mean_wind_rms, "averaged wind too far off");
        }
        check(pe.cruise[0].rms() <= pe.cruise[1].rms() + 0.05f, "cruise vario worse than TE netto");
        // against the GNSS/INS reference with its own 10 Hz GNSS (ins_reference.h)
        // cruise within 0.07 m/s (a flapped glider with a bending lift curve: 0.35 against 0.29 with the pitch path weighted
        // for the real attitude error, sigma_pitch 0.03; 0.31 with the simulator's 0.017), entry and circling better
        check(pe.cruise[0].rms() <= pe.cruise[2].rms() + 0.07f && pe.entry[0].rms() < pe.entry[2].rms() && pe.circle[0].rms() < pe.circle[2].rms(),
              "vario worse than the GNSS/INS reference");
        // the instant wind as shown (valid or degraded) against the reference on the same samples, which shows its wind
        // always; ours is shown at least 85% of the flight
        check(pe.wind_shown.rms() < pe.ins_wind_shown.rms() && pe.avg_wind.rms() < pe.ins_avg_wind.rms(), "wind worse than the GNSS/INS reference");
        check(pe.wind_shown_n >= 0.85f * pe.total_n, "instant wind shown too rarely");
        // the uncertainty grows in long straight glides, the wind goes invalid there, but must not flicker (17 per hour
        // without the hysteresis)
        check(pe.dropouts <= 12, "wind validity flickers");
        check(pe.false_lift[0].events <= pe.false_lift[1].events, "more false lift than TE netto");
    }
    if (avg_wind_err.n) {
        std::printf("  averaged wind rms %.2f m/s (max %.2f)\n", avg_wind_err.rms(), avg_wind_err.maxabs);
    }
    std::printf("  gnss latency: nominal %d ms, true %d ms, estimated %d ms\n", gnss_latency, gnss_latency + lat_err,
                (int)std::lround(ekf.gnssLatency() * 1000.f));
    if (sc.check_latency) {
        check(std::abs(lat_est_err) <= sc.max_latency_err, "gnss latency did not converge");
    }
    std::printf("  %s\n", ok ? "PASS" : "FAILED");
    return ok ? 0 : 1;
}
