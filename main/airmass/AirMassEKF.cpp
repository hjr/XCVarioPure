#include "AirMassEKF.h"
#include "math/Trigonometry.h"

#include <algorithm>

namespace {
constexpr float G0 = Units::g0;
// plausible ranges of the learned calibration states
constexpr float KTAS_MIN = 0.8f, KTAS_MAX = 1.25f;
constexpr float THB_MAX = 0.25f; // rad
constexpr float TAU_MIN = 0.25f, TAU_MAX = 1.75f; // s, GNSS latency
constexpr float TAU_SIGMA0 = 0.3f; // s, initial uncertainty of the latency
constexpr float SLIP_ALIGN_MAX = 0.1f; // rad, slipping samples do not go into the turn alignment
constexpr uint32_t HEADING_AID_AGE_MS = 3000;

// Gate the complete horizontal observation before changing any state. A bad east component must not
// partially update the north/wind/calibration states. Joseph covariance update also supports frozen gains.
bool velocityUpdate(Matrix<AirMassEKF::NX, AirMassEKF::NX>& p, float (&x)[AirMassEKF::NX],
                    const float h[2][AirMassEKF::NX], const float y[2], const float r[2], float gate, bool freeze) {
    constexpr int N = AirMassEKF::NX;
    float ph[N][2] = {}, k[N][2];
    for (int i = 0; i < N; ++i)
        for (int a = 0; a < 2; ++a)
            for (int j = 0; j < N; ++j) ph[i][a] += p(i, j) * h[a][j];
    float s00 = r[0], s11 = r[1], s01 = 0.f;
    for (int i = 0; i < N; ++i) {
        s00 += h[0][i] * ph[i][0];
        s11 += h[1][i] * ph[i][1];
        s01 += h[0][i] * ph[i][1];
    }
    const float det = s00 * s11 - s01 * s01;
    if (!(det > 0.f) || !std::isfinite(det)) return false;
    const float distance = (s11 * y[0] * y[0] - 2.f * s01 * y[0] * y[1] + s00 * y[1] * y[1]) / det;
    if (!std::isfinite(distance) || (gate > 0.f && distance > gate * gate)) return false;
    for (int i = 0; i < N; ++i) {
        const bool frozen = freeze && (i == AirMassEKF::KTAS || i == AirMassEKF::THB || i == AirMassEKF::KCL ||
                                       i == AirMassEKF::TAU || i == AirMassEKF::BGZ);
        k[i][0] = frozen ? 0.f : (ph[i][0] * s11 - ph[i][1] * s01) / det;
        k[i][1] = frozen ? 0.f : (ph[i][1] * s00 - ph[i][0] * s01) / det;
        x[i] += k[i][0] * y[0] + k[i][1] * y[1];
    }
    for (int i = 0; i < N; ++i) {
        for (int j = i; j < N; ++j) {
            const float delta = -k[i][0] * ph[j][0] - k[i][1] * ph[j][1] - ph[i][0] * k[j][0] - ph[i][1] * k[j][1] +
                k[i][0] * (s00 * k[j][0] + s01 * k[j][1]) + k[i][1] * (s01 * k[j][0] + s11 * k[j][1]);
            p(i, j) += delta;
            p(j, i) = p(i, j);
        }
    }
    return true;
}

inline float wrapPi(float a) {
    while (a >= My_PIf) a -= 2.f * My_PIf;
    while (a < -My_PIf) a += 2.f * My_PIf;
    return a;
}
} // namespace

AirMassEKF::AirMassEKF() {
    reset(0.f);
    _running = false;
}

void AirMassEKF::reset(float baro_alt, float k_tas, float pitch_ofs, float latency, float k_aoa) {
    for (float& v : _x) {
        v = 0.f;
    }
    _x[KTAS] = k_tas;
    _x[THB] = pitch_ofs;
    _x[TAU] = latency;
    _x[KCL] = k_aoa;
    clampCalibration();
    _x[ALT] = baro_alt;
    _cov = Matrix<NX, NX>();
    _cov(WN, WN) = _cov(WE, WE) = 100.f; // unknown until aligned with GNSS
    _cov(WD, WD) = 1.f;
    _cov(PSI, PSI) = 10.f;
    _cov(KTAS, KTAS) = 0.05f * 0.05f;
    _cov(THB, THB) = 0.035f * 0.035f;
    _cov(ALT, ALT) = 1.f;
    _cov(VD, VD) = 1.f;
    _cov(BAZ, BAZ) = 0.04f;
    _cov(BGZ, BGZ) = 0.01f * 0.01f;
    _cov(TAU, TAU) = TAU_SIGMA0 * TAU_SIGMA0;
    _cov(KCL, KCL) = 0.05f * 0.05f;
    if (!_cfg.learn_calibration) {
        for (int i : {KTAS, THB, TAU, KCL}) {
            _cov(i, i) = 0.f; // sanitize() keeps them at their floor
        }
    }
    _aid = {};
    _hist_head = -1;
    _hist_level = 0;
    _psi_gyro = 0.f;
    _heading_rate = 0.f;
    _wind_prev[0] = _wind_prev[1] = 0.f;
    _wind_rate[0] = _wind_rate[1] = 0.f;
    _tas_f = NAN;
    _tas_dot_f = 0.f;
    _vd_f = _vd_p = 0.f;
    _acc_long_f = _acc_long_p = NAN;
    _sink_f = _sink_p = NAN;
    _acc_corr = _acc_corr_c = 0.f;
    _acc_kin = 0.f;
    _a_air = 0.f;
    _anchor = 0.f;
    _vario_out = NAN;
    _wd_energy = _wd_energy_plain = NAN;
    _ww_lp = _blend_lp = 0.f;
    _acc_mem = 0.f;
    _gnss_rejects = 0;
    _gnss_consecutive_rejects = 0;
    _baro_reject_ms = 0;
    _baro_clock = _kinematic_clock = _energy_clock = {};
    _vertical_settle_ms = 0;
    _last_gnss_ms = _last_time_ms;
    _have_gnss = _have_received_gnss = _have_input = false;
    _gnss_timing.reset();
    _inertial_trust = 1.f;
    _gnss_trust = 0.f;
    _aligned = false;
    _wind_valid = false;
    _wind_degraded = false;
    _wind_valid_ms = 0;
    restartAlignment();
    _running = true;
}

void AirMassEKF::restartAlignment() {
    _al = {};
}

namespace {
// solve a symmetric 4x4 system by Gaussian elimination with partial pivoting
bool solve4(float a[4][4], float b[4], float x[4]) {
    for (int c = 0; c < 4; ++c) {
        int p = c;
        for (int r = c + 1; r < 4; ++r) {
            if (std::fabs(a[r][c]) > std::fabs(a[p][c])) p = r;
        }
        if (std::fabs(a[p][c]) < 1e-6f) return false;
        if (p != c) {
            for (int k = 0; k < 4; ++k) std::swap(a[p][k], a[c][k]);
            std::swap(b[p], b[c]);
        }
        for (int r = c + 1; r < 4; ++r) {
            const float f = a[r][c] / a[c][c];
            for (int k = c; k < 4; ++k) a[r][k] -= f * a[c][k];
            b[r] -= f * b[c];
        }
    }
    for (int r = 3; r >= 0; --r) {
        float acc = b[r];
        for (int k = r + 1; k < 4; ++k) acc -= a[r][k] * x[k];
        x[r] = acc / a[r][r];
    }
    return true;
}
} // namespace

void AirMassEKF::airVelocity(float tas, float k, float roll, float pitch, float psi, float alpha, float out[3], float dalpha[3]) {
    const float sp = std::sin(roll), cp = std::cos(roll);
    const float st = std::sin(pitch), ct = std::cos(pitch);
    const float ss = std::sin(psi), cs = std::cos(psi);
    const float sa = std::sin(alpha), ca = std::cos(alpha);
    const float v = k * tas;

    // body air velocity (coordinated flight, no sideslip) v * (cos a, 0, sin a), rotated by roll and pitch
    const float vx = v * ca, vz = v * sa;
    const float ux = ct * vx + st * cp * vz;
    const float uy = -sp * vz;
    const float uz = -st * vx + ct * cp * vz;
    out[0] = cs * ux - ss * uy;
    out[1] = ss * ux + cs * uy;
    out[2] = uz;
    if (dalpha) {
        const float dvx = -v * sa, dvz = v * ca;
        const float dux = ct * dvx + st * cp * dvz;
        const float duy = -sp * dvz;
        dalpha[0] = cs * dux - ss * duy;
        dalpha[1] = ss * dux + cs * duy;
        dalpha[2] = -st * dvx + ct * cp * dvz;
    }
}

void AirMassEKF::predict(const Input& in) {
    const float dt = in.dt;
    const float acc = in.acc_down - _x[BAZ];

    // BGZ is the body z gyro bias, it enters the heading rate with cos(roll) / cos(pitch): a single yaw rate bias
    // learned while circling would be off in straight flight
    const float cb = std::cos(in.roll) / std::max(std::cos(in.pitch), 0.1f);
    _heading_rate = in.yaw_rate - cb * _x[BGZ];
    _x[PSI] += _heading_rate * dt;
    wrapPsi();
    _psi_gyro = wrapPi(_psi_gyro + _heading_rate * dt);
    _x[ALT] += -_x[VD] * dt - 0.5f * acc * dt * dt;
    _x[VD] += acc * dt;

    // P = F P F' + Q, F is identity except the heading and altitude/vertical speed bias chains.
    // Applied in place (rows, then columns) to keep the sensor task stack small. ALT goes before VD,
    // because it needs the VD entries before their own update.
    const float hdt2 = 0.5f * dt * dt;
    for (int j = 0; j < NX; ++j) {
        _cov(ALT, j) += -dt * _cov(VD, j) + hdt2 * _cov(BAZ, j);
        _cov(VD, j) += -dt * _cov(BAZ, j);
        _cov(PSI, j) += -dt * cb * _cov(BGZ, j);
    }
    for (int i = 0; i < NX; ++i) {
        _cov(i, ALT) += -dt * _cov(i, VD) + hdt2 * _cov(i, BAZ);
        _cov(i, VD) += -dt * _cov(i, BAZ);
        _cov(i, PSI) += -dt * cb * _cov(i, BGZ);
    }
    _cov(WN, WN) += _cfg.q_wind_h * dt;
    _cov(WE, WE) += _cfg.q_wind_h * dt;
    _cov(WD, WD) += _cfg.q_wind_v * dt;
    const float held_noise = 1.f + std::clamp(in.imu_age, 0.f, 10.f) / 0.1f;
    _cov(PSI, PSI) += _cfg.q_psi * dt * held_noise;
    if (_cfg.learn_calibration) {
        _cov(KTAS, KTAS) += _cfg.q_ktas * dt;
        _cov(THB, THB) += _cfg.q_thb * dt;
        _cov(TAU, TAU) += _cfg.q_tau * dt;
        _cov(KCL, KCL) += _cfg.q_kcl * dt;
    }
    _cov(ALT, ALT) += _cfg.q_alt * dt;
    _cov(VD, VD) += _cfg.q_vd * dt * held_noise;
    _cov(BAZ, BAZ) += _cfg.q_baz * dt;
    _cov(BGZ, BGZ) += _cfg.q_bgz * dt;

    // the energy balance uses the same low pass for TAS and vertical speed, like a TE vario does,
    // to keep the large speed and climb rate terms consistent during pull-ups
    const float ae = dt / (_cfg.energy_tau + dt);
    if (std::isnan(_tas_f)) {
        _tas_f = in.tas;
        _vd_f = _vd_p = _x[VD];
        _acc_long_f = _acc_long_p = in.acc_long;
    }

    const float tas_prev = _tas_f;
    _tas_f += (in.tas - _tas_f) * ae;
    _tas_dot_f = (_tas_f - tas_prev) / dt;
    // the measured airspeed lags by the pitot line: the inertial terms get the same delay
    const float ap = dt / (_cfg.lag_pitot + dt);
    _vd_p += (_x[VD] - _vd_p) * ap;
    _acc_long_p += (in.acc_long - _acc_long_p) * ap;
    _vd_f += (_vd_p - _vd_f) * ae;
    _acc_long_f += (_acc_long_p - _acc_long_f) * ae;
    // a complementary filter AHRS recovers from acceleration errors with seconds of delay
    _acc_mem = std::max(std::fabs(in.acc_long), _acc_mem * (1.f - dt / 2.f));

    // horizontal wind change rate for the energy constraint, low pass ~2s
    const float a = dt / (2.f + dt);
    for (int i = 0; i < 2; ++i) {
        const float rate = (_x[WN + i] - _wind_prev[i]) / dt;
        _wind_rate[i] += (rate - _wind_rate[i]) * a;
        _wind_prev[i] = _x[WN + i];
    }
}

void AirMassEKF::update(const Input& in) {
    if (!_running || !(in.dt > 0.f)) {
        return;
    }
    const bool continuous = in.continuous && (!_have_input || (uint32_t)(in.time_ms - _last_time_ms) <= 300);
    _have_input = true;
    _last_time_ms = in.time_ms;
    if (!continuous) {
        _vertical_settle_ms = 0;
        _inertial_trust = 0.f;
        _gnss_trust = 0.f;
        _wind_rate[0] = _wind_rate[1] = 0.f;
        _ww_lp = _blend_lp = 0.f;
        // Unobserved motion is not a quiet-flight measurement. Do not differentiate the endpoint jump as
        // if it happened in one sensor cycle, and allow the barometer to recover the vertical state.
        _tas_f = NAN;
        _cov(PSI, PSI) += 0.01f * in.dt;
        _cov(VD, VD) += 4.f * in.dt;
        _cov(WN, WN) += 0.25f * in.dt;
        _cov(WE, WE) += 0.25f * in.dt;
    } else {
        _inertial_trust = std::min(1.f, _inertial_trust + in.dt / 10.f);
    }
    const bool flying_speed = in.flying_speed && std::isfinite(in.tas) && in.tas > 10.f;
    if (!flying_speed) {
        _vertical_settle_ms = 0;
        // Do not carry ground braking or a frozen air-mass output into the next takeoff.
        _tas_f = NAN;
        _sink_f = _sink_p = NAN;
        _acc_corr = _acc_corr_c = _acc_kin = _a_air = 0.f;
        _anchor = _ww_lp = _blend_lp = 0.f;
        _vario_out = NAN;
        _wd_energy = _wd_energy_plain = NAN;
    }
    predict(in);

    // remember the raw inputs for delayed GNSS fusion
    _hist_head = (_hist_head + 1) % HIST;
    if (_hist_level < HIST) {
        _hist_level++;
    }
    Snapshot& s = _hist[_hist_head];
    s.time_ms = in.time_ms;
    s.tas = in.tas;
    s.roll = in.roll;
    s.pitch = in.pitch;
    s.aoa = in.aoa;
    s.alpha_shift = 0.f;
    s.psi_gyro = _psi_gyro;
    s.slip_sigma = in.slip_sigma;
    s.continuous = continuous;
    s.flying_speed = flying_speed;

    if (in.baro_fresh && std::isfinite(in.baro_alt) && _baro_clock.observe(in.time_ms, in.time_ms, true)) {
        // the static pressure error grows with dynamic pressure and fakes climb/sink on speed changes
        const float sample_dt = _baro_clock.interval_ms ? _baro_clock.interval_ms * 1e-3f : in.dt;
        fuseBaro(in.baro_alt + (_cfg.static_comp - 1.f) * in.tas * in.tas / (2.f * G0), sample_dt);
    }

    _straight = blendBankWeight(in.roll);
    if (flying_speed && continuous) {
        float va[3], dalpha[3];
        airVelocity(in.tas, _x[KTAS], in.roll, in.pitch, _x[PSI], alphaOf(in.aoa), va, dalpha);
        if (in.kinematic_fresh && _kinematic_clock.observe(in.time_ms, in.time_ms, true)) fuseKinematic(in, va, dalpha);
        updateKinetic(in, va);
        if (std::isfinite(in.polar_sink)) {
            fuseEnergy(in);
        }
    }

    sanitize();
    // the vario output, smoothed while circling: without the blend the airspeed derivative reaches the needle
    const float vario = rawVario();
    const float tau = _cfg.circling_tau * (1.f - _straight);
    _vario_out = std::isnan(_vario_out) ? vario : _vario_out + (vario - _vario_out) * in.dt / (tau + in.dt);
    updateWindValid();
    if (_running && flying_speed && continuous) {
        _vertical_settle_ms = std::min<uint32_t>(5000, _vertical_settle_ms + (uint32_t)(in.dt * 1000.f + 0.5f));
    }
}

void AirMassEKF::fuseBaro(float alt, float sample_dt) {
    float h[NX] = {};
    h[ALT] = 1.f;
    // the static line delays the reading: it shows the altitude of lag_static ago, alt + lag * vd
    h[VD] = _cfg.lag_static;
    if (ekfScalarUpdate(_cov, _x, h, alt - (_x[ALT] + _cfg.lag_static * _x[VD]), _cfg.r_baro, _cfg.gate * 2.f)) {
        _baro_reject_ms = 0;
    } else if ((_baro_reject_ms += (uint32_t)(std::clamp(sample_dt, 0.001f, 0.3f) * 1000.f + 0.5f)) > 2000) {
        // disruptive altitude change (e.g. simulator jump), restart the vertical channel
        _x[ALT] = alt;
        _x[VD] = 0.f;
        _cov(ALT, ALT) = 1.f;
        _cov(VD, VD) = 1.f;
        _baro_reject_ms = 0;
        _vertical_settle_ms = 0;
    }
}

// Pitch based air mass vertical velocity: Wd = vd - va_d
void AirMassEKF::fuseKinematic(const Input& in, const float va[3], const float dalpha[3]) {
    if (_inertial_trust < 1.f) return; // attitude and its pitch/AoA calibration need continuous motion history
    float h[NX] = {};
    h[WD] = -1.f;
    h[VD] = 1.f;
    // k_tas is left to the GNSS fusion, its small vertical effect is indistinguishable from Wd
    h[THB] = dalpha[2]; // d(-va_d)/d(thb), alpha = aoa * kcl - thb
    h[KCL] = -dalpha[2] * in.aoa;
    const float y = 0.f - (_x[VD] - va[2] - _x[WD]);
    // attitude and angle of attack are less certain in turns and after sustained longitudinal acceleration;
    // the gyro propagated attitude (AirMassAttitude.h) needs no extra margin for the load factor
    const float sig = _cfg.sigma_pitch * (1.f + 2.f * std::fabs(std::sin(in.roll)) + 0.7f * _acc_mem);
    const float sv = in.tas * sig;
    // sideslip in a bank tilts the air vector vertically by V * beta * sin(roll)
    const float ss = in.tas * in.slip_sigma * std::fabs(std::sin(in.roll));
    // These model constraints have correlated noise. Keep their information per second stable across rates.
    const float sample_dt = _kinematic_clock.interval_ms ? _kinematic_clock.interval_ms * 1e-3f : in.dt;
    const float rate_scale = 0.1f / std::clamp(sample_dt, 0.001f, 0.3f);
    ekfScalarUpdate(_cov, _x, h, y, (sv * sv + ss * ss + 0.01f) * rate_scale, _cfg.gate);
}

float AirMassEKF::blendBankWeight(float roll) const {
    if (_cfg.blend_bank <= 0.f) {
        return 1.f;
    }
    const float bank = Units::rad_to_deg(std::fabs(roll));
    return std::clamp((_cfg.blend_bank + 0.5f * _cfg.blend_bank_width - bank) / std::max(_cfg.blend_bank_width, 0.1f), 0.f, 1.f);
}

// Acceleration along the air velocity for the energy balance. Relative to the air the kinetic energy changes
// with V * dV/dt + va . dW/dt, which is V times the inertial acceleration along the air velocity: a horizontal
// gust changes the airspeed, not the inertial acceleration. The airspeed derivative plus the estimated wind
// change is free of attitude errors, but misses gusts (the wind estimate is slow). The inertial acceleration
// follows gusts, but carries g times the pitch error. Their disagreement splits into a slow part (attitude error,
// tracked with kinetic_tau) and a transient part (gust): the inertial acceleration, cleared from the slow part,
// takes over as the transient part grows towards gust_acc.
void AirMassEKF::updateKinetic(const Input& in, const float va[3]) {
    const float k = _x[KTAS];
    float work = _cfg.wind_work * (va[0] * _wind_rate[0] + va[1] * _wind_rate[1]) / std::max(k * in.tas, 1.f);
    if (_cfg.wind_work_hp > 0.f) {
        _ww_lp += (work - _ww_lp) * in.dt / (_cfg.wind_work_hp + in.dt);
        work -= _ww_lp;
    }
    const float fresh = _have_gnss ? std::clamp((10000.f - gnssAgeMs()) / 5000.f, 0.f, 1.f) : 0.f;
    work *= _inertial_trust * _gnss_trust * fresh;
    const float a_air = k * _tas_dot_f + work;
    const float d = a_air - _acc_long_f;
    _acc_corr += (d - _acc_corr) * in.dt / (_cfg.kinetic_tau + in.dt);
    const float gust = d - _acc_corr;
    const float w = std::clamp(std::fabs(gust) / _cfg.gust_acc, 0.f, 1.f) * _straight;
    float corr = -w * gust;
    if (_cfg.circling_blend_tau > 0.f) {
        _acc_corr_c += (d - _acc_corr_c) * in.dt / (_cfg.circling_blend_tau + in.dt);
        corr -= (1.f - _straight) * (d - _acc_corr_c);
    }
    if (_cfg.blend_hp > 0.f) {
        _blend_lp += (corr - _blend_lp) * in.dt / (_cfg.blend_hp + in.dt);
        corr -= _blend_lp;
    }
    _acc_kin = a_air + _inertial_trust * corr;
    _a_air = a_air;
}

// Pitch independent energy balance: Wd = vd - s(V,n) - V * a/g, a the acceleration along the air velocity
// (updateKinetic), on low passed vertical speed and acceleration
void AirMassEKF::fuseEnergy(const Input& in) {
    const float k = _x[KTAS];
    const float v = k * _tas_f;
    // the polar sink follows the load factor, it gets the same low pass as the vertical speed it is compared with:
    // a roll-in raises the sink within a second or two
    const float ae = in.dt / (_cfg.energy_tau + in.dt);
    // and the delay of the airspeed (lag_pitot)
    const float ap = in.dt / (_cfg.lag_pitot + in.dt);
    _sink_p = std::isnan(_sink_p) ? in.polar_sink : _sink_p + (in.polar_sink - _sink_p) * ap;
    _sink_f = std::isnan(_sink_f) ? _sink_p : _sink_f + (_sink_p - _sink_f) * ae;
    const float z = -(_sink_f + v * _acc_kin / G0);
    _wd_energy = z + _vd_f;
    _wd_energy_plain = -(_sink_f + v * _a_air / G0) + _vd_f;
    float h[NX] = {};
    h[WD] = 1.f;
    h[VD] = -1.f; // approximately, the low passed vertical speed follows the state
    const float y = z - (_x[WD] - _vd_f);
    // airspeed changes not explained by inertial acceleration indicate horizontal gusts
    const float gust = std::fabs(k * _tas_dot_f - in.acc_long);
    const float sig = _cfg.sigma_energy * (1.f + _cfg.energy_load_weight * std::fabs(in.load_factor - 1.f) + _cfg.energy_gust_weight * gust);
    if (in.energy_fresh && _energy_clock.observe(in.time_ms, in.time_ms, true)) {
        const float sample_dt = _energy_clock.interval_ms ? _energy_clock.interval_ms * 1e-3f : in.dt;
        const float rate_scale = 0.1f / std::clamp(sample_dt, 0.001f, 0.3f);
        ekfScalarUpdate(_cov, _x, h, y, sig * sig * rate_scale, _cfg.energy_gate);
    }

    // The energy balance is down weighted in gusts and at high load factors, which come with the lift in thermals: on
    // its own the estimate then reads low over minutes. The output follows the plain energy balance (the airspeed
    // derivative, equal weight) over anchor_tau, the estimator keeps the fast part (gust rejection, pull-ups).
    if (_cfg.anchor_tau > 0.f) {
        _anchor += (_wd_energy_plain - _x[WD] - _anchor) * in.dt / (_cfg.anchor_tau + in.dt);
    }
}

const AirMassEKF::Snapshot* AirMassEKF::findSnapshot(uint32_t t) const {
    if (_hist_level == 0) {
        return nullptr;
    }
    const Snapshot* best = &_hist[_hist_head];
    int32_t best_d = INT32_MAX;
    for (int i = 0; i < _hist_level; ++i) {
        const Snapshot& s = _hist[(_hist_head - i + HIST) % HIST];
        int32_t d = (int32_t)(s.time_ms - t);
        d = d < 0 ? -d : d;
        if (d < best_d) {
            best_d = d;
            best = &s;
        }
    }
    return best;
}

// Interpolate the air vector at time t between the two adjacent snapshots, re-evaluated with the
// current calibration states. The heading at that time is the current heading minus the gyro increment
// since then. The rates between the snapshots give the derivatives by the GNSS latency.
bool AirMassEKF::delayedState(uint32_t t, Delayed& d) const {
    if ((uint32_t)(_last_time_ms - t) > HISTORY_MS) return false;
    if (_hist_level < 2) {
        return false;
    }
    const int oldest = (_hist_head - _hist_level + 1 + HIST) % HIST;
    if ((int32_t)(t - _hist[oldest].time_ms) < 0 || (int32_t)(t - _hist[_hist_head].time_ms) > 0) return false;
    // newest snapshot not later than t (or the oldest), paired with its successor;
    // beyond the newest snapshot the last pair is used (no extrapolation)
    int i0 = _hist_head;
    for (int i = 0; i < _hist_level - 1 && (int32_t)(t - _hist[i0].time_ms) < 0; ++i) {
        i0 = (i0 - 1 + HIST) % HIST;
    }
    if (i0 == _hist_head) {
        i0 = (i0 - 1 + HIST) % HIST;
    }
    const Snapshot& s0 = _hist[i0];
    const Snapshot& s1 = _hist[(i0 + 1) % HIST];
    const float span = (s1.time_ms - s0.time_ms) * 1e-3f;
    if (!(span > 0.f) || span > 0.3f || !s1.continuous || !s0.flying_speed || !s1.flying_speed) {
        return false;
    }
    const float f = std::clamp((int32_t)(t - s0.time_ms) * 1e-3f / span, 0.f, 1.f);

    float va0[3], va1[3], da0[3], da1[3];
    const Snapshot* sn[2] = {&s0, &s1};
    float* va[2] = {va0, va1};
    float* da[2] = {da0, da1};
    for (int k = 0; k < 2; ++k) {
        const float psi_then = wrapPi(_x[PSI] - wrapPi(_psi_gyro - sn[k]->psi_gyro));
        airVelocity(sn[k]->tas, _x[KTAS], sn[k]->roll, sn[k]->pitch, psi_then, alphaOf(sn[k]->aoa) + sn[k]->alpha_shift, va[k], da[k]);
    }
    for (int i = 0; i < 3; ++i) {
        d.va[i] = va0[i] + f * (va1[i] - va0[i]);
        d.va_dot[i] = (va1[i] - va0[i]) / span;
        d.dalpha[i] = da0[i] + f * (da1[i] - da0[i]);
    }
    d.tas = s0.tas + f * (s1.tas - s0.tas);
    d.aoa = s0.aoa + f * (s1.aoa - s0.aoa);
    d.slip_sigma = std::max(s0.slip_sigma, s1.slip_sigma);
    return true;
}

// heading from the air vector, given a trusted prior wind
void AirMassEKF::alignFromPrior(const GnssVelocity& g, float wn0, float we0) {
    const float an = g.vn - wn0, ae = g.ve - we0;
    if (an * an + ae * ae < 100.f) {
        return; // too slow for a reliable track
    }
    commitAlignment(wn0, we0, std::atan2(ae, an), 4.f, 0.05f);
}

// heading from an absolute heading source (compass), the wind follows from the GNSS velocity
void AirMassEKF::alignFromHeading(const GnssVelocity& g, const Snapshot& s) {
    // heading at the observation time and now, through the gyro increments since the aid was taken
    const float psi_then = wrapPi(_aid.psi - wrapPi(_aid.psi_gyro - s.psi_gyro));
    const float psi_now = wrapPi(_aid.psi + wrapPi(_psi_gyro - _aid.psi_gyro));
    float va[3];
    airVelocity(s.tas, _x[KTAS], s.roll, s.pitch, psi_then, alphaOf(s.aoa) + s.alpha_shift, va, nullptr);
    const float v = _x[KTAS] * s.tas;
    const float sair = _cfg.sigma_air + 0.01f * s.tas;
    const float var_psi = _aid.sigma * _aid.sigma;
    const float var_wind = std::max(g.sigma_h * g.sigma_h, 0.01f) + sair * sair + v * v * var_psi;
    commitAlignment(g.vn - va[0], g.ve - va[1], psi_now, var_wind, var_psi);
}

// take over wind and heading from an alignment, decorrelated from the other states
void AirMassEKF::commitAlignment(float wn, float we, float psi, float var_wind, float var_psi) {
    _x[WN] = wn;
    _x[WE] = we;
    _x[PSI] = wrapPi(psi);
    for (int i = 0; i < NX; ++i) {
        _cov(WN, i) = _cov(i, WN) = 0.f;
        _cov(WE, i) = _cov(i, WE) = 0.f;
        _cov(PSI, i) = _cov(i, PSI) = 0.f;
    }
    _cov(WN, WN) = _cov(WE, WE) = var_wind;
    _cov(PSI, PSI) = var_psi;
    _wind_prev[0] = wn;
    _wind_prev[1] = we;
    _wind_rate[0] = _wind_rate[1] = 0.f;
    _gnss_consecutive_rejects = 0;
    _aligned = true;
    _aid.valid = false;
    restartAlignment();
}

// Collect GNSS velocities while turning. With the horizontal air vector u in the gyro heading frame,
// v = W + Rz(psi_gyro) * [c -s; s c] * u is linear in (Wn, We, c, s), with c + i*s = k * exp(i*delta),
// delta the offset of the true heading to the gyro heading integral.
void AirMassEKF::accumulateAlignment(const GnssVelocity& g, const Snapshot& s) {
    if (s.slip_sigma > SLIP_ALIGN_MAX) {
        return; // the air vector direction is off by the sideslip
    }
    if (_al.n > 0 && (g.time_ms - _al.start_ms) > 180000) {
        restartAlignment(); // turn took too long, the wind might have changed
    }
    if (_al.n == 0) {
        _al.start_ms = g.time_ms;
        _al.psi_last = s.psi_gyro;
        _al.psi_unwrapped = _al.psi_min = _al.psi_max = 0.f;
    }
    _al.psi_unwrapped += wrapPi(s.psi_gyro - _al.psi_last);
    _al.psi_last = s.psi_gyro;
    _al.psi_min = std::min(_al.psi_min, _al.psi_unwrapped);
    _al.psi_max = std::max(_al.psi_max, _al.psi_unwrapped);

    float u[3];
    airVelocity(s.tas, _x[KTAS], s.roll, s.pitch, s.psi_gyro, alphaOf(s.aoa) + s.alpha_shift, u, nullptr);
    const float rows[2][4] = {{1.f, 0.f, u[0], -u[1]}, {0.f, 1.f, u[1], u[0]}};
    const float z[2] = {g.vn, g.ve};
    for (int r = 0; r < 2; ++r) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                _al.ata[i][j] += rows[r][i] * rows[r][j];
            }
            _al.atz[i] += rows[r][i] * z[r];
        }
        _al.zz += z[r] * z[r];
    }
    _al.n++;

    if (_al.psi_max - _al.psi_min < My_PIf || _al.n < 10) {
        return; // not yet enough heading diversity
    }
    float a[4][4], b[4], p[4];
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) a[i][j] = _al.ata[i][j];
        b[i] = _al.atz[i];
    }
    if (!solve4(a, b, p)) {
        restartAlignment();
        return;
    }
    // residual: z'z - 2 p'A'z + p'A'Ap
    float res = _al.zz;
    for (int i = 0; i < 4; ++i) {
        res -= 2.f * p[i] * _al.atz[i];
        for (int j = 0; j < 4; ++j) res += p[i] * _al.ata[i][j] * p[j];
    }
    const float rms = std::sqrt(std::max(res, 0.f) / (2.f * _al.n));
    const float k_rel = std::sqrt(p[2] * p[2] + p[3] * p[3]);
    if (rms > 2.f || k_rel < 0.85f || k_rel > 1.15f) {
        restartAlignment();
        return;
    }
    if (_cfg.learn_calibration) {
        _x[KTAS] = std::clamp(_x[KTAS] * k_rel, KTAS_MIN, KTAS_MAX);
    }
    commitAlignment(p[0], p[1], _psi_gyro + std::atan2(p[3], p[2]), 1.f, 0.01f);
}

void AirMassEKF::fuseGnss(const GnssVelocity& g, bool has_prior, float wn0, float we0) {
    if (!_running) {
        return;
    }
    _last_received_gnss_ms = _last_time_ms;
    _have_received_gnss = true;
    uint32_t transport_delay = 0;
    if (!std::isfinite(g.vn) || !std::isfinite(g.ve) || !std::isfinite(g.sigma_h) || !(g.sigma_h > 0.f) ||
        std::hypot(g.vn, g.ve) > 150.f || (int32_t)(_last_time_ms - g.time_ms) < 0 ||
        !_gnss_timing.observe(g.time_ms, g.fix_time_ms, transport_delay)) {
        ++_gnss_rejects;
        return;
    }
    const uint32_t t_obs = g.time_ms - transport_delay - (uint32_t)(gnssLatency() * 1000.f + 0.5f);
    Delayed d;
    if (!delayedState(t_obs, d)) {
        ++_gnss_rejects;
        _gnss_trust = 0.f;
        return;
    }
    const Snapshot* s = findSnapshot(t_obs);
    if (!s || s->tas < 10.f) {
        return;
    }
    if (!_aligned) {
        if (_inertial_trust < 1.f) return;
        has_prior = has_prior && std::isfinite(wn0) && std::isfinite(we0);
        if (has_prior) {
            alignFromPrior(g, wn0, we0);
        } else if (_aid.valid && (uint32_t)(_last_time_ms - _aid.time_ms) <= HEADING_AID_AGE_MS) {
            alignFromHeading(g, *s);
        } else {
            accumulateAlignment(g, *s);
        }
        if (_aligned) {
            _last_gnss_ms = t_obs;
            _have_gnss = true;
            _gnss_trust = 0.f;
        }
        return;
    }

    // GNSS noise plus the error of the modelled air vector (airspeed noise, attitude, sideslip)
    const float sair = _cfg.sigma_air + 0.01f * d.tas + (1.f - _inertial_trust) * 0.1f * d.tas;
    const float sslip = d.tas * d.slip_sigma;
    const float r = std::max(g.sigma_h * g.sigma_h, 0.01f) + sair * sair + sslip * sslip;
    const float meas[2] = {g.vn, g.ve};
    float h[2][NX] = {}, y[2], variance[2];
    // Source timestamps remove transport jitter. Without them its unknown part must not be fitted as wind
    // or calibration, especially while turning. Irregular cadence alone does not make a fix invalid.
    const float time_sigma = g.fix_time_ms == airmass::GnssTiming::UNKNOWN ? 0.1f : 0.05f;
    for (int i = 0; i < 2; ++i) {
        h[i][WN + i] = 1.f;
        h[i][PSI] = i ? d.va[0] : -d.va[1]; // d(va)/d(psi) = (-va_e, va_n)
        h[i][KTAS] = d.va[i] / _x[KTAS];
        h[i][THB] = -d.dalpha[i];
        h[i][KCL] = d.dalpha[i] * d.aoa;
        h[i][TAU] = -d.va_dot[i]; // a larger latency looks further back
        y[i] = meas[i] - (d.va[i] + _x[WN + i]);
        variance[i] = r + time_sigma * time_sigma * d.va_dot[i] * d.va_dot[i];
    }
    const bool rejected = !velocityUpdate(_cov, _x, h, y, variance, _cfg.gate, _inertial_trust < 1.f || _gnss_trust < 0.5f);
    wrapPsi();

    if (rejected) {
        _gnss_rejects++;
        _gnss_trust *= 0.5f;
        if (++_gnss_consecutive_rejects > 10) {
            // the filter lost track, align again
            _aligned = false;
            restartAlignment();
        }
    } else {
        _gnss_consecutive_rejects = 0;
        if (gnssAgeMs() > 5000) {
            _gnss_trust = 0.f;
            _wind_rate[0] = _wind_rate[1] = 0.f;
            _ww_lp = 0.f;
        }
        _last_gnss_ms = t_obs;
        _have_gnss = true;
        _gnss_trust = std::min(1.f, _gnss_trust + 0.25f);
    }
    sanitize();
}

void AirMassEKF::clampCalibration() {
    _x[KTAS] = std::clamp(_x[KTAS], KTAS_MIN, KTAS_MAX);
    _x[THB] = std::clamp(_x[THB], -THB_MAX, THB_MAX);
    _x[TAU] = std::clamp(_x[TAU], TAU_MIN, TAU_MAX);
    _x[KCL] = std::clamp(_x[KCL], _cfg.kcl_min, _cfg.kcl_max);
}

void AirMassEKF::setPitchOffset(float thb, float sigma) {
    thb = std::clamp(thb, -THB_MAX, THB_MAX);
    // the snapshots for the delayed GNSS fusion were taken with the old offset (flaps): keep their angle of attack
    const float shift = thb - _x[THB];
    for (int i = 0; i < _hist_level; ++i) {
        _hist[i].alpha_shift += shift; // retain the original AoA and its scale derivative
    }
    _x[THB] = thb;
    for (int i = 0; i < NX; ++i) {
        _cov(THB, i) = _cov(i, THB) = 0.f;
    }
    _cov(THB, THB) = sigma * sigma;
}

void AirMassEKF::fuseHeading(float psi, float sig) {
    if (!_running) {
        return;
    }
    if (!_aligned) {
        // kept for the alignment with the next GNSS velocity
        _aid = {wrapPi(psi), sig, _psi_gyro, _last_time_ms, true};
        return;
    }
    if (sigma(PSI) < sig) {
        return; // the GNSS turns give a better heading, the source's bias (compass deviation) would only harm
    }
    float h[NX] = {};
    h[PSI] = 1.f;
    ekfScalarUpdate(_cov, _x, h, wrapPi(psi - _x[PSI]), sig * sig, _cfg.gate);
    wrapPsi();
}

uint8_t AirMassEKF::status() const {
    if (!_running) {
        return 0;
    }
    uint8_t st = 0;
    if (_vertical_settle_ms >= 5000) {
        st |= ST_VARIO_VALID;
    }
    if (_aligned) {
        st |= ST_ALIGNED;
        const uint32_t age = gnssAgeMs();
        if (age < 5000) {
            st |= ST_GNSS_FRESH;
        }
        if (_wind_valid) {
            st |= ST_WIND_VALID;
        } else if (_wind_degraded) {
            st |= ST_WIND_DEGRADED;
        }
    }
    return st;
}

// wind validity with hysteresis: the uncertainty grows slowly in straight flight and saw-tooths with every GNSS
// fix, valid again only clearly below the threshold, so the outputs do not flicker
void AirMassEKF::updateWindValid() {
    constexpr float RECOVER = 0.9f;
    const float u = windUncertainty();
    const bool ok = _running && _aligned && gnssAgeMs() < 30000;
    _wind_valid = ok && u < (_wind_valid ? 1.f : RECOVER);
    if (_wind_valid) {
        _wind_valid_ms = _last_time_ms;
    }
    // after the wind got invalid (long straight glides: the heading is only checked in turns) the estimate stays
    // better than the last valid wind for a while
    _wind_degraded = !_wind_valid && ok && _wind_valid_ms != 0 && (_last_time_ms - _wind_valid_ms) * 1e-3f <= _cfg.degraded_time &&
                     u < _cfg.degraded_factor;
}

bool AirMassEKF::covarianceHealthy() const {
    if (!_cov.allFinite()) {
        return false;
    }
    for (int i = 0; i < NX; ++i) {
        if (!std::isfinite(_x[i])) {
            return false;
        }
    }
    return true;
}

void AirMassEKF::wrapPsi() {
    _x[PSI] = wrapPi(_x[PSI]);
}

void AirMassEKF::sanitize() {
    if (!covarianceHealthy()) {
        _running = false; // owner has to restart the filter
        return;
    }
    clampCalibration();
    // bounded uncertainties keep the filter responsive and numerically sane
    constexpr float pmax[NX] = {400.f, 400.f, 100.f, 10.f, 0.01f, 0.01f, 1e4f, 100.f, 1.f, 1e-3f, 0.25f, 0.04f};
    constexpr float pmin[NX] = {1e-4f, 1e-4f, 1e-4f, 1e-6f, 1e-8f, 1e-8f, 1e-4f, 1e-4f, 1e-8f, 1e-10f, 1e-6f, 1e-8f};
    for (int i = 0; i < NX; ++i) {
        const float pii = _cov(i, i);
        if (pii > pmax[i]) {
            // scale row and column to keep the correlations and positive definiteness
            const float s = std::sqrt(pmax[i] / pii);
            for (int j = 0; j < NX; ++j) {
                _cov(i, j) *= s;
                _cov(j, i) *= s;
            }
        } else if (pii < pmin[i]) {
            // numerically collapsed, decorrelate this state
            for (int j = 0; j < NX; ++j) {
                _cov(i, j) = _cov(j, i) = 0.f;
            }
            _cov(i, i) = pmin[i];
        }
    }
}
