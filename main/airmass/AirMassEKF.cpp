#include "AirMassEKF.h"
#include "math/Trigonometry.h"

#include <algorithm>

namespace {
constexpr float G0 = Units::g0;
// plausible ranges of the learned calibration states
constexpr float KTAS_MIN = 0.8f, KTAS_MAX = 1.25f;
constexpr float THB_MAX = 0.25f; // rad
constexpr float TAU_MIN = -0.5f, TAU_MAX = 1.f; // s, GNSS latency correction
constexpr float TAU_SIGMA0 = 0.3f; // s, initial uncertainty of the latency correction
constexpr float KCL_MIN = 0.8f, KCL_MAX = 1.25f;
constexpr float SLIP_ALIGN_MAX = 0.1f; // rad, slipping samples do not go into the turn alignment
constexpr uint32_t HEADING_AID_AGE_MS = 3000;

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

void AirMassEKF::reset(float baro_alt, float k_tas, float pitch_ofs, float latency_ofs, float k_aoa) {
    for (float& v : _x) {
        v = 0.f;
    }
    _x[KTAS] = k_tas;
    _x[THB] = pitch_ofs;
    _x[TAU] = latency_ofs;
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
    _wind_prev[0] = _wind_prev[1] = 0.f;
    _wind_rate[0] = _wind_rate[1] = 0.f;
    _tas_f = NAN;
    _tas_dot_f = 0.f;
    _vd_f = 0.f;
    _acc_mem = 0.f;
    _gnss_rejects = 0;
    _gnss_consecutive_rejects = 0;
    _baro_consecutive_rejects = 0;
    _vertical_settle = 0;
    _last_gnss_ms = _last_time_ms;
    _aligned = false;
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

    const float yaw_rate = in.yaw_rate - _x[BGZ];
    _x[PSI] += yaw_rate * dt;
    wrapPsi();
    _psi_gyro = wrapPi(_psi_gyro + yaw_rate * dt);
    _x[ALT] += -_x[VD] * dt - 0.5f * acc * dt * dt;
    _x[VD] += acc * dt;

    // P = F P F' + Q, F is identity except the heading and altitude/vertical speed bias chains.
    // Applied in place (rows, then columns) to keep the sensor task stack small. ALT goes before VD,
    // because it needs the VD entries before their own update.
    const float hdt2 = 0.5f * dt * dt;
    for (int j = 0; j < NX; ++j) {
        _cov(ALT, j) += -dt * _cov(VD, j) + hdt2 * _cov(BAZ, j);
        _cov(VD, j) += -dt * _cov(BAZ, j);
        _cov(PSI, j) += -dt * _cov(BGZ, j);
    }
    for (int i = 0; i < NX; ++i) {
        _cov(i, ALT) += -dt * _cov(i, VD) + hdt2 * _cov(i, BAZ);
        _cov(i, VD) += -dt * _cov(i, BAZ);
        _cov(i, PSI) += -dt * _cov(i, BGZ);
    }
    _cov(WN, WN) += _cfg.q_wind_h * dt;
    _cov(WE, WE) += _cfg.q_wind_h * dt;
    _cov(WD, WD) += _cfg.q_wind_v * dt;
    _cov(PSI, PSI) += _cfg.q_psi * dt;
    if (_cfg.learn_calibration) {
        _cov(KTAS, KTAS) += _cfg.q_ktas * dt;
        _cov(THB, THB) += _cfg.q_thb * dt;
        _cov(TAU, TAU) += _cfg.q_tau * dt;
        _cov(KCL, KCL) += _cfg.q_kcl * dt;
    }
    _cov(ALT, ALT) += _cfg.q_alt * dt;
    _cov(VD, VD) += _cfg.q_vd * dt;
    _cov(BAZ, BAZ) += _cfg.q_baz * dt;
    _cov(BGZ, BGZ) += _cfg.q_bgz * dt;

    // the energy balance uses the same low pass for TAS and vertical speed, like a TE vario does,
    // to keep the large speed and climb rate terms consistent during pull-ups
    const float ae = dt / (_cfg.energy_tau + dt);
    if (std::isnan(_tas_f)) {
        _tas_f = in.tas;
        _vd_f = _x[VD];
    }
    const float tas_prev = _tas_f;
    _tas_f += (in.tas - _tas_f) * ae;
    _tas_dot_f = (_tas_f - tas_prev) / dt;
    _vd_f += (_x[VD] - _vd_f) * ae;
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
    _last_time_ms = in.time_ms;
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
    s.psi_gyro = _psi_gyro;
    s.slip_sigma = in.slip_sigma;

    if (std::isfinite(in.baro_alt)) {
        // the static pressure error grows with dynamic pressure and fakes climb/sink on speed changes
        fuseBaro(in.baro_alt + (_cfg.static_comp - 1.f) * in.tas * in.tas / (2.f * G0));
    }

    if (in.tas > 10.f) {
        float va[3], dalpha[3];
        airVelocity(in.tas, _x[KTAS], in.roll, in.pitch, _x[PSI], alphaOf(in.aoa), va, dalpha);
        fuseKinematic(in, va, dalpha);
        if (std::isfinite(in.polar_sink)) {
            fuseEnergy(in, va);
        }
    }

    sanitize();
    if (_running && _vertical_settle < 0xffff) {
        _vertical_settle++;
    }
}

void AirMassEKF::fuseBaro(float alt) {
    float h[NX] = {};
    h[ALT] = 1.f;
    if (ekfScalarUpdate(_cov, _x, h, alt - _x[ALT], _cfg.r_baro, _cfg.gate * 2.f)) {
        _baro_consecutive_rejects = 0;
    } else if (++_baro_consecutive_rejects > 20) {
        // disruptive altitude change (e.g. simulator jump), restart the vertical channel
        _x[ALT] = alt;
        _x[VD] = 0.f;
        _cov(ALT, ALT) = 1.f;
        _cov(VD, VD) = 1.f;
        _baro_consecutive_rejects = 0;
        _vertical_settle = 0;
    }
}

// Pitch based air mass vertical velocity: Wd = vd - va_d
void AirMassEKF::fuseKinematic(const Input& in, const float va[3], const float dalpha[3]) {
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
    ekfScalarUpdate(_cov, _x, h, y, sv * sv + 0.01f, _cfg.gate);
}

// Pitch independent energy balance: Wd = vd - s(V,n) - V*dV/dt/g - va.dW/dt/g
// on low passed vertical speed and TAS (the wind is assumed to change slower than energy_tau)
void AirMassEKF::fuseEnergy(const Input& in, const float va[3]) {
    const float k = _x[KTAS];
    const float v = k * _tas_f;
    const float vdot = k * _tas_dot_f;
    const float z = -(in.polar_sink + v * vdot / G0 + (va[0] * _wind_rate[0] + va[1] * _wind_rate[1]) / G0);
    float h[NX] = {};
    h[WD] = 1.f;
    h[VD] = -1.f; // approximately, the low passed vertical speed follows the state
    const float y = z - (_x[WD] - _vd_f);
    // airspeed changes not explained by inertial acceleration indicate horizontal gusts, which the
    // energy balance can only follow through the slow GNSS wind estimate
    const float gust = std::fabs(vdot - in.acc_long);
    const float sig = _cfg.sigma_energy * (1.f + 2.f * std::fabs(in.load_factor - 1.f) + 1.5f * gust);
    ekfScalarUpdate(_cov, _x, h, y, sig * sig, _cfg.gate);
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
    if (_hist_level < 2) {
        return false;
    }
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
    if (!(span > 0.f)) {
        return false;
    }
    const float f = std::clamp((int32_t)(t - s0.time_ms) * 1e-3f / span, 0.f, 1.f);

    float va0[3], va1[3], da0[3], da1[3];
    const Snapshot* sn[2] = {&s0, &s1};
    float* va[2] = {va0, va1};
    float* da[2] = {da0, da1};
    for (int k = 0; k < 2; ++k) {
        const float psi_then = wrapPi(_x[PSI] - wrapPi(_psi_gyro - sn[k]->psi_gyro));
        airVelocity(sn[k]->tas, _x[KTAS], sn[k]->roll, sn[k]->pitch, psi_then, alphaOf(sn[k]->aoa), va[k], da[k]);
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
    airVelocity(s.tas, _x[KTAS], s.roll, s.pitch, psi_then, alphaOf(s.aoa), va, nullptr);
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
    airVelocity(s.tas, _x[KTAS], s.roll, s.pitch, s.psi_gyro, alphaOf(s.aoa), u, nullptr);
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
    _gnss_nominal_ms = g.latency_ms;
    const uint32_t t_obs = g.time_ms - (uint32_t)(gnssLatency() * 1000.f + 0.5f);
    const Snapshot* s = findSnapshot(t_obs);
    if (!s || s->tas < 10.f) {
        return;
    }
    _last_gnss_ms = g.time_ms;
    if (!_aligned) {
        if (has_prior) {
            alignFromPrior(g, wn0, we0);
        } else if (_aid.valid && (uint32_t)(_last_time_ms - _aid.time_ms) <= HEADING_AID_AGE_MS) {
            alignFromHeading(g, *s);
        } else {
            accumulateAlignment(g, *s);
        }
        return;
    }

    Delayed d;
    if (!delayedState(t_obs, d)) {
        return;
    }

    // GNSS noise plus the error of the modelled air vector (airspeed noise, attitude, sideslip)
    const float sair = _cfg.sigma_air + 0.01f * d.tas;
    const float sslip = d.tas * d.slip_sigma;
    const float r = std::max(g.sigma_h * g.sigma_h, 0.01f) + sair * sair + sslip * sslip;
    const float meas[2] = {g.vn, g.ve};
    bool rejected = false;
    for (int i = 0; i < 2; ++i) {
        float h[NX] = {};
        h[WN + i] = 1.f;
        h[PSI] = i ? d.va[0] : -d.va[1]; // d(va)/d(psi) = (-va_e, va_n)
        h[KTAS] = d.va[i] / _x[KTAS];
        h[THB] = -d.dalpha[i];
        h[KCL] = d.dalpha[i] * d.aoa;
        h[TAU] = -d.va_dot[i]; // a larger latency looks further back
        const float y = meas[i] - (d.va[i] + _x[WN + i]);
        if (!ekfScalarUpdate(_cov, _x, h, y, r, _cfg.gate)) {
            rejected = true;
        }
    }
    wrapPsi();

    if (rejected) {
        _gnss_rejects++;
        if (++_gnss_consecutive_rejects > 10) {
            // the filter lost track, align again
            _aligned = false;
            restartAlignment();
        }
    } else {
        _gnss_consecutive_rejects = 0;
    }
    sanitize();
}

void AirMassEKF::clampCalibration() {
    _x[KTAS] = std::clamp(_x[KTAS], KTAS_MIN, KTAS_MAX);
    _x[THB] = std::clamp(_x[THB], -THB_MAX, THB_MAX);
    _x[TAU] = std::clamp(_x[TAU], TAU_MIN, TAU_MAX);
    _x[KCL] = std::clamp(_x[KCL], KCL_MIN, KCL_MAX);
}

void AirMassEKF::setPitchOffset(float thb, float sigma) {
    thb = std::clamp(thb, -THB_MAX, THB_MAX);
    // the snapshots for the delayed GNSS fusion were taken with the old offset (flaps): keep their angle of attack
    const float shift = (thb - _x[THB]) / _x[KCL];
    for (int i = 0; i < _hist_level; ++i) {
        _hist[i].aoa += shift; // the ring fills from index 0
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
    if (_vertical_settle >= 50) {
        st |= ST_VARIO_VALID;
    }
    if (_aligned) {
        st |= ST_ALIGNED;
        const uint32_t age = gnssAgeMs();
        if (age < 5000) {
            st |= ST_GNSS_FRESH;
        }
        const float sw = std::sqrt(_cov(WN, WN) + _cov(WE, WE));
        if (age < 30000 && sw < _cfg.sigma_wind_valid && sigma(PSI) < _cfg.sigma_psi_valid) {
            st |= ST_WIND_VALID;
        }
    }
    return st;
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
