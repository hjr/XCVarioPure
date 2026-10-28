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
//                     [--slip-aid 0|1] [--attitude own|device] [--max-wind-valid s] [--csv file]
//   --lat-err: the true GNSS latency differs from the nominal one the filter is told
//   --static-adj: static port error of the installation, as the TE adjustment that would cancel it
//   --comp: configured compensation of the air mass filter and the electronic TE reference
//   --compass: magnetic compass with this noise (plus a fixed deviation) aids the heading
//   --slip-aid: sideslip estimate from the lateral acceleration (default on)
//   --attitude: the estimator's own air data aided attitude (default), or the device AHRS
//   --max-wind-valid: override the scenario's limit for the time to a valid wind
//   --max-wind-rms: override the scenario's limit for the wind rms
//   --seed: sensor noise realisation (the scenario's air mass stays the same)
//   --vario-agility: like the device setting, scales the vertical air movement process noise by its square
//   --set name=value: change an assumption (see Knobs), e.g. --set k_tas=0.95 --set wind=11
//   --warm k_tas,pitch_ofs,aoa_scale,latency_ofs: start with these learned values, like the device after a flight
//   --flap-warm o0,..,o4: learned pitch offsets per flap position [rad] of an earlier flight (--set flaps=1)
//   --no-learn: keep the calibration values as started

#include "AirMassEKF.h"
#include "AirMassInput.h"
#include "AirMassAttitude.h"
#include "AirMassFlaps.h"
#include "math/Quaternion.h"
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
};
Knobs K;
const std::pair<const char*, float*> KNOBS[] = {
    {"k_tas", &K.k_tas}, {"mount", &K.mount}, {"gyro_bias", &K.gyro_bias}, {"acc_bias", &K.acc_bias}, {"cla_true", &K.cla_true},
    {"polar_err", &K.polar_err}, {"mass_err", &K.mass_err}, {"tas_noise", &K.tas_noise}, {"wind", &K.wind}, {"turb", &K.turb},
    {"lift", &K.lift}, {"dolphin", &K.dolphin}, {"gps_out", &K.gps_out}, {"gps_out_len", &K.gps_out_len},
    {"flaps", &K.flaps}, {"flap_alpha0", &K.flap_alpha0}, {"flap_aid", &K.flap_aid}, {"flap_pilot", &K.flap_pilot}, {"aoa_nonlin", &K.aoa_nonlin},
};
constexpr float CY_TRUE = 0.25f;   // side force slope of the "real" glider, the device assumes airmass::CY_BETA
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

// solve pitch so that the body/air geometry yields the wanted air vertical speed
float solvePitch(float v, float roll, float alpha, float va_d) {
    float th = -std::asin(std::clamp(va_d / v, -0.9f, 0.9f)) + alpha * std::cos(roll);
    for (int i = 0; i < 5; ++i) {
        float out[3];
        AirMassEKF::airVelocity(v, 1.f, roll, th, 0.f, alpha, out, nullptr);
        float out2[3];
        AirMassEKF::airVelocity(v, 1.f, roll, th + 1e-4f, 0.f, alpha, out2, nullptr);
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

// sensor axes misaligned by a pitch angle against the body axes
vector_f mountRotate(const float b[3]) {
    const float c = std::cos(K.mount), s = std::sin(K.mount);
    return vector_f(c * b[0] - s * b[2], b[1], s * b[0] + c * b[2]);
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

// the device's vario damping (VarioFilter): a low pass with tau = D/3, then y += (x - y) / D per 10 Hz cycle
struct DeviceDamp {
    float D, l = NAN, y = NAN;
    explicit DeviceDamp(float d) : D(d) {}
    float f(float x) {
        if (std::isnan(l)) {
            l = y = x;
        }
        l += (x - l) * 0.1f / (D / 3.f + 0.1f);
        y += (l - y) / std::max(D, 1.f);
        return y;
    }
};
constexpr float DAMPING = 4.f;         // vario_delay default (TE netto)
constexpr float AIRMASS_DAMPING = 2.f; // half of it for the air mass vario

struct Stats {
    double sum2 = 0;
    int n = 0;
    float maxabs = 0;
    void add(float e) {
        sum2 += e * e;
        n++;
        maxabs = std::max(maxabs, std::fabs(e));
    }
    float rms() const { return n ? std::sqrt(sum2 / n) : NAN; }
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
        c.max_spike = 1.2f;
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
        // the zoom (the device AHRS drifts by ~8°, 3.2 m/s spike), so the air mass vario beats it
        c.max_spike = 0.6f;
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
        c.air.vertical = [](float t) { return 1.2f * std::sin(t / 47.f) * std::sin(t / 13.f); };
        c.eval_wind_after = 300.f;
        c.max_wind_rms = 1.2f;
        c.steady = {{600, 10800}};
        c.max_vario_rms = 0.5f;
        c.check_latency = true;
        s.push_back(c);
    }
    return s;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <scenario> [--lat-err ms] [--static-adj %%] [--comp %%] [--compass deg] "
                             "[--slip-aid 0|1] [--attitude own|device] [--max-wind-valid s] [--max-wind-rms m/s] [--csv file]\n", argv[0]);
        return 2;
    }
    const std::string which = argv[1];
    FILE* csv = nullptr;
    const char* csv_name = nullptr;
    int lat_err = 0;
    float static_adj = 1.f, comp = 1.f;
    float compass_sigma = -1.f, max_wind_valid = -1.f, max_wind_rms = -1.f;
    bool slip_aid = true, own_attitude = true;
    unsigned seed = 4711;
    float vario_agility = 1.f;
    float warm[4] = {1.f, 0.f, 1.f, 0.f};
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
        } else if (std::strcmp(argv[i], "--attitude") == 0) {
            own_attitude = std::strcmp(argv[i + 1], "device") != 0;
        } else if (std::strcmp(argv[i], "--max-wind-valid") == 0) {
            max_wind_valid = std::atof(argv[i + 1]);
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
        } else if (std::strcmp(argv[i], "--max-wind-rms") == 0) {
            max_wind_rms = std::atof(argv[i + 1]);
        } else if (std::strcmp(argv[i], "--csv") == 0) {
            csv_name = argv[i + 1];
            csv = std::fopen(csv_name, "w");
        }
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

    std::mt19937 rng(seed);
    std::normal_distribution<float> nd(0.f, 1.f);

    const float dt_truth = 0.01f;
    const int sensor_div = 10;                      // 10Hz sensor loop
    const int gnss_div = 100;           // 1Hz NMEA from the Flarm
    const uint16_t gnss_latency = 750;  // ms
    const float gnss_sigma = 0.3f;

    float total = sc.duration;
    for (auto& s : sc.segments) total += s.duration;

    Truth tr;
    tr.psi = 0.3f;
    if (sc.pilot) {
        tr.h = 900.f; // released from the winch / tow
        tr.v = NlDay::V_CRUISE;
    }
    AirMassEKF ekf;
    AirMassEKF::Config cfg;
    cfg.static_comp = comp;
    cfg.q_wind_v *= vario_agility * vario_agility; // AirMassSensor::applyConfig
    cfg.learn_calibration = learn;
    ekf.setConfig(cfg);
    ekf.reset(tr.h, warm[0], warm[1], warm[3], warm[2]);

    // sensor processing state: raw IMU into the device AHRS
    DeviceAhrs ahrs;
    airmass::Attitude att;
    bool att_init = false, ekf_started = false;
    float prev_vg[3] = {NAN, NAN, NAN};
    float prev_att[3] = {NAN, NAN, NAN};
    vector_f f_sum, w_sum, gyro_prev, gyro_dot;
    DeviceDamp am_display(AIRMASS_DAMPING), te_lpf(DAMPING);
    Lpf te_alt(0.5f, 0.1f);
    float prev_te_alt = NAN;
    struct GnssSample { uint32_t t_obs; float vn, ve; };
    std::vector<GnssSample> gnss_q;

    Stats wind_err, vario_err, te_err, spike_err, te_spike_err;
    float t_wind_valid = -1.f;
    float prev_gamma = 0.f, prev_wn = NAN, prev_we = NAN, prev_wd = NAN;
    int flap_lever = -1;
    float rec_since = 0.f;
    float flap_defl = 0.f;
    airmass::FlapOffsets flap_ofs;
    flap_ofs.load(flap_warm); // first flight: nothing learned
    const bool flap_offsets = K.flaps > 0.f && K.flap_aid > 0.f;
    float vdot_l = 0.f;
    int restarts = 0;

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
        Stats cruise[2], entry[2], circle[2], avg_wind, avg_truth_dev, inst_wind;
        // the 30s average like AirMassSensor's averaged wind (navigation) against the same average of the truth,
        // and the instant wind against the trackable part of the turbulent truth
        Lpf avg_n{30.f, 0.1f}, avg_e{30.f, 0.1f}, avg_tn{30.f, 0.1f}, avg_te{30.f, 0.1f}, true_n{5.f, 0.1f}, true_e{5.f, 0.1f};
        FalseLift false_lift[2];
        std::vector<double> th_sum[2];
        std::vector<int> th_n;
        std::vector<float> truth, shown[2];
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
        const float roll_target = bank_deg * D2R;
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

        // a horizontal gust changes TAS directly (inertial speed is kept), the pilot then restores speed
        float va_dot_wdot = tr.va[0] * tr.wdot[0] + tr.va[1] * tr.wdot[1];
        tr.vdot = vdot_l - va_dot_wdot / std::max(tr.v, 1.f);
        tr.v += tr.vdot * dt_truth;

        // load factor from turn and path curvature, smoothed by the pitch dynamics of the glider
        const float cosg = std::cos(tr.gamma_up);
        const float load_cmd =
            std::clamp((cosg + tr.v * (tr.gamma_up - prev_gamma) / dt_truth / G - wd_dot / G) / std::cos(tr.roll), 0.3f, 3.f);
        tr.load += (load_cmd - tr.load) * dt_truth / 0.4f;
        prev_gamma = tr.gamma_up;

        // energy balance: air relative sink rate
        const float va_d = polar(tr.v, tr.load) + tr.v * tr.vdot / G + va_dot_wdot / G;
        tr.gamma_up = -std::asin(std::clamp(va_d / tr.v, -0.9f, 0.9f));
        const float cl = 2.f * tr.load * MASS * G / (RHO * AREA * tr.v * tr.v);
        const float cl_bend = std::max(cl - (0.9f + 1.5f * flap_defl), 0.f);
        tr.alpha = cl / K.cla_true + ALPHA0 - K.flap_alpha0 * flap_defl + K.aoa_nonlin * cl_bend * cl_bend;
        tr.pitch = solvePitch(tr.v, tr.roll, tr.alpha, va_d);
        // with sideslip the flight path is turned against the nose heading
        const float beta = sc.slip(tr.t);
        AirMassEKF::airVelocity(tr.v, 1.f, tr.roll, tr.pitch, tr.psi + beta, tr.alpha, tr.va, nullptr);

        tr.vg[0] = tr.va[0] + tr.wn;
        tr.vg[1] = tr.va[1] + tr.we;
        tr.vg[2] = tr.va[2] + tr.wd;

        // raw IMU: specific force and body rates from the inertial acceleration and the attitude change
        if (!std::isnan(prev_vg[0])) {
            const float f_nav[3] = {(tr.vg[0] - prev_vg[0]) / dt_truth / G, (tr.vg[1] - prev_vg[1]) / dt_truth / G,
                                    (tr.vg[2] - prev_vg[2]) / dt_truth / G - 1.f};
            float f_b[3];
            navToBody(tr.roll, tr.pitch, tr.psi, f_nav, f_b);
            // side force opposes the sideslip
            f_b[1] += -0.5f * RHO * tr.v * tr.v * AREA * CY_TRUE * beta / (MASS * G);
            const float rd = (tr.roll - prev_att[0]) / dt_truth, pd = (tr.pitch - prev_att[1]) / dt_truth;
            const float yd = (tr.psi - prev_att[2]) / dt_truth;
            const float sr = std::sin(tr.roll), cr = std::cos(tr.roll), sp = std::sin(tr.pitch), cp = std::cos(tr.pitch);
            const float w_b[3] = {rd - yd * sp, pd * cr + yd * sr * cp, -pd * sr + yd * cr * cp};
            f_sum += mountRotate(f_b);
            w_sum += mountRotate(w_b);
        } else {
            f_sum += vector_f(0.f, 0.f, -1.f);
        }
        for (int i = 0; i < 3; ++i) {
            prev_vg[i] = tr.vg[i];
        }
        prev_att[0] = tr.roll;
        prev_att[1] = tr.pitch;
        prev_att[2] = tr.psi;
        tr.yaw_rate = G * std::tan(tr.roll) / tr.v;
        tr.psi += tr.yaw_rate * dt_truth;
        tr.h -= tr.vg[2] * dt_truth;
        tr.xa += tr.va[0] * dt_truth;
        tr.ya += tr.va[1] * dt_truth;

        // GNSS observation enters a latency queue
        const bool gps_out = K.gps_out >= 0.f && tr.t >= K.gps_out && tr.t < K.gps_out + K.gps_out_len;
        if (step % gnss_div == 0 && !gps_out) {
            gnss_q.push_back({(uint32_t)(tr.t * 1000.f), tr.vg[0] + gnss_sigma * nd(rng), tr.vg[1] + gnss_sigma * nd(rng)});
        }

        if (step % sensor_div != 0) {
            continue;
        }

        // 10Hz sensor loop: averaged IMU samples (digital low pass of the sensor) with noise and bias
        const uint32_t now = (uint32_t)(tr.t * 1000.f);
        const vector_f accel = f_sum * (1.f / sensor_div) +
                               vector_f(0.01f * nd(rng), 0.01f * nd(rng), K.acc_bias + 0.01f * nd(rng));
        const vector_f gyro = w_sum * (1.f / sensor_div) +
                              vector_f(0.5f * K.gyro_bias + 0.002f * nd(rng), 0.5f * K.gyro_bias + 0.002f * nd(rng), K.gyro_bias + 0.002f * nd(rng));
        f_sum = w_sum = vector_f();
        const float tas_m = tr.v / K.k_tas + K.tas_noise * nd(rng);
        ahrs.update(accel, gyro, tas_m);

        // the same input conversion as the device (AirMassSensor::postProcess)
        AirMassEKF::Input in;
        in.time_ms = now;
        in.dt = 0.1f;
        in.roll = ahrs.euler.x;
        in.pitch = ahrs.euler.y;
        in.yaw_rate = ahrs.circle_omega;
        // static port error: the reading is off by (1 - static_adj) * V²/2g
        in.baro_alt = tr.h - (static_adj - 1.f) * tr.v * tr.v / (2.f * G) + 0.25f * nd(rng);
        airmass::Raw raw;
        const vector_f down = ahrs.q.rotateInv({0, 0, 1});
        gyro_dot = gyro_dot + ((gyro - gyro_prev) * (1.f / in.dt) - gyro_dot) * (0.1f / (0.2f + 0.1f));
        gyro_prev = gyro;
        for (int i = 0; i < 3; ++i) {
            raw.f_body[i] = accel[i];
            raw.down_body[i] = down[i];
            raw.gyro[i] = gyro[i];
            raw.gyro_dot[i] = gyro_dot[i];
        }
        raw.ias = raw.tas = tas_m; // sea level
        if (flap_offsets && ekf_started) { // like the device: before the inputs are converted
            // the lever sensor reads the flaps' position, between the notches while they move
            float lever = 0.f;
            for (int i = 1; i < 5 && flap_defl > FLAP_DEG[i - 1] * D2R; ++i) {
                lever = i - 1 + std::min((flap_defl / D2R - FLAP_DEG[i - 1]) / (FLAP_DEG[i] - FLAP_DEG[i - 1]), 1.f);
            }
            if (K.flap_aid == 2.f) {
                // no sensor (AirMassSensor's flapLever): the recommendation at the turn's load
                lever = flapRecommendation(tas_m, 1.f / std::max(std::cos(att.roll()), 0.3f));
            }
            flap_ofs.update(ekf, lever);
        }
        raw.mass = MASS * K.mass_err;
        raw.wing_area = AREA;
        const airmass::AirData air = airmass::airData(raw, ekf);
        raw.polar_sink_ias = -K.polar_err * polar(air.ias, std::max(air.n_lift, 0.3f)); // the configured polar
        // like the device (AirMassSensor::start): the attitude starts with the estimator, from the converged device AHRS
        if (!att_init && tr.t >= AHRS_WARMUP) {
            att.reset(raw.down_body);
            att_init = true;
        }
        // the centripetal reference needs the true airspeed, with the learned airspeed scale
        att.update(raw.gyro, air.f_cg, ekf.kTas() * tas_m, air.alpha, in.dt);
        if (own_attitude) {
            in.roll = att.roll();
            in.pitch = att.pitch();
            in.yaw_rate = att.yawRate(raw.gyro);
            for (int i = 0; i < 3; ++i) {
                raw.down_body[i] = att.down()[i];
            }
        }
        airmass::fillInput(raw, air, in);
        if (!slip_aid) {
            in.slip_sigma = 0.f;
        }
        // the device AHRS has converged long before the takeoff starts the estimator
        if (tr.t < AHRS_WARMUP) {
            while (!gnss_q.empty() && gnss_q.front().t_obs + gnss_latency + lat_err <= now) {
                gnss_q.erase(gnss_q.begin());
            }
            continue;
        }
        if (!ekf_started) {
            ekf.reset(in.baro_alt, warm[0], warm[1], warm[3], warm[2]);
            ekf_started = true;
        }
        ekf.update(in);
        if (compass_sigma > 0.f && step % (sensor_div * 100) == 0 && std::fabs(tr.roll) < 0.17f) {
            ekf.fuseHeading(std::remainder(tr.psi + COMPASS_DEV + compass_sigma * nd(rng), 6.2831853f), 0.15f);
        }

        while (!gnss_q.empty() && gnss_q.front().t_obs + gnss_latency + lat_err <= now) {
            AirMassEKF::GnssVelocity g;
            g.time_ms = gnss_q.front().t_obs + gnss_latency + lat_err;
            g.latency_ms = gnss_latency;
            g.vn = gnss_q.front().vn;
            g.ve = gnss_q.front().ve;
            g.sigma_h = 0.5f;
            ekf.fuseGnss(g);
            gnss_q.erase(gnss_q.begin());
        }
        if (!ekf.isRunning()) {
            restarts++;
            ekf.reset(in.baro_alt, warm[0], warm[1], warm[3], warm[2]);
            flap_ofs.load(flap_warm); // like the device: start() loads the saved offsets again
        }

        // classic TE netto reference (electronic compensation, perfect polar knowledge)
        const float te_h = te_alt.f(in.baro_alt + comp * tas_m * tas_m / (2.f * G));
        const float te = std::isnan(prev_te_alt) ? 0.f : (te_h - prev_te_alt) / 0.1f;
        prev_te_alt = te_h;
        // the device netto uses the polar at the raw IAS and the body z g load (S2F::getSink(ias))
        const float te_polar = K.polar_err * polar(tas_m, std::max(-accel.z, 0.3f));
        const float te_netto = te_lpf.f(te) + te_polar;

        // evaluation
        const uint8_t st = ekf.status();
        const float vario_true = -tr.wd;
        const float am_vario = am_display.f(ekf.airmassVario()); // as displayed
        if ((st & AirMassEKF::ST_WIND_VALID) && t_wind_valid < 0.f) {
            t_wind_valid = tr.t;
        }
        if (tr.t > sc.eval_wind_after) {
            const float ew = std::hypot(ekf.windN() - tr.wn, ekf.windE() - tr.we);
            wind_err.add(ew);
        }
        if (inWindows(tr.t, sc.steady)) {
            vario_err.add(am_vario - vario_true);
            te_err.add(te_netto - vario_true);
        }
        if (inWindows(tr.t, sc.spikes)) {
            spike_err.add(am_vario - vario_true);
            te_spike_err.add(te_netto - vario_true);
        }
        if (sc.pilot) {
            const float shown[2] = {am_vario, te_netto};
            pe.truth.push_back(vario_true);
            for (int k = 0; k < 2; ++k) {
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
                        pe.th_sum[0].resize(g_pilot.thermal + 1, 0.);
                        pe.th_sum[1].resize(g_pilot.thermal + 1, 0.);
                    }
                    pe.th_sum[k][g_pilot.thermal] += e;
                    if (k == 0) {
                        pe.th_n[g_pilot.thermal]++;
                    }
                }
            }
            const float an = pe.avg_n.f(ekf.windN()), ae = pe.avg_e.f(ekf.windE());
            const float atn = pe.avg_tn.f(tr.wn), ate = pe.avg_te.f(tr.we);
            const float tn = pe.true_n.f(tr.wn), te = pe.true_e.f(tr.we);
            if (t_wind_valid >= 0.f && tr.t > t_wind_valid + 60.f) {
                pe.avg_wind.add(std::hypot(an - atn, ae - ate));
                pe.avg_truth_dev.add(std::hypot(atn - sc.mean_wind[0], ate - sc.mean_wind[1]));
                pe.inst_wind.add(std::hypot(ekf.windN() - tn, ekf.windE() - te));
            }
        }
        if (csv) {
            std::fprintf(csv, "%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.4f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.1f,%.1f,%d,%.3f\n", tr.t, tr.wn, tr.we, ekf.windN(),
                         ekf.windE(), vario_true, am_vario, te_netto, std::remainder(ekf.heading() - tr.psi, 6.2831853f),
                         ekf.kTas(), ekf.pitchOffset(), ekf.accBias(), st, std::sqrt(ekf.sigma(0) * ekf.sigma(0) + ekf.sigma(1) * ekf.sigma(1)),
                         tr.roll / D2R, tr.vg[2], ekf.vDown(), ekf.airmassVario(), (in.roll - tr.roll) / D2R,
                         (in.pitch - tr.pitch) / D2R, tr.v, in.load_factor, tr.h, tr.xa, tr.ya, g_pilot.mode, te + te_polar);
        }
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
                "gnss rejects %u, restarts %d\n",
                sc.name.c_str(), t_wind_valid, wind_err.rms(), wind_err.maxabs, vario_err.rms(), te_err.rms(),
                spike_err.maxabs, te_spike_err.maxabs, ekf.kTas(), k_err, ekf.pitchOffset(), ekf.aoaScale(), CLA_MODEL / K.cla_true, ekf.accBias(),
                ekf.rejectedGnss(), restarts);

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
        Stats climb[2];
        int used = 0;
        for (size_t i = 0; i < pe.th_n.size(); ++i) {
            if (pe.th_n[i] >= 100) {
                used++;
                for (int k = 0; k < 2; ++k) {
                    climb[k].add(pe.th_sum[k][i] / pe.th_n[i]);
                }
            }
        }
        int lag[2] = {};
        for (int k = 0; k < 2; ++k) {
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
                    "instant %.2f m/s off the 5s smoothed truth; final altitude %.0f m\n",
                    pe.avg_wind.rms(), pe.avg_wind.maxabs, pe.avg_truth_dev.rms(), pe.inst_wind.rms(), tr.h);
        if (sc.max_climb_err_rms > 0.f) {
            check(used >= 3 && climb[0].rms() <= sc.max_climb_err_rms, "thermal climb reading off");
        }
        if (sc.max_mean_wind_rms > 0.f) {
            check(pe.avg_wind.rms() <= sc.max_mean_wind_rms, "averaged wind too far off");
        }
        check(pe.cruise[0].rms() <= pe.cruise[1].rms() + 0.05f, "cruise vario worse than TE netto");
        check(pe.false_lift[0].events <= pe.false_lift[1].events, "more false lift than TE netto");
    }
    std::printf("  gnss latency: nominal %d ms, true %d ms, estimated %d ms\n", gnss_latency, gnss_latency + lat_err,
                (int)std::lround(ekf.gnssLatency() * 1000.f));
    if (sc.check_latency) {
        check(std::abs(lat_est_err) <= sc.max_latency_err, "gnss latency did not converge");
    }
    std::printf("  %s\n", ok ? "PASS" : "FAILED");
    return ok ? 0 : 1;
}
