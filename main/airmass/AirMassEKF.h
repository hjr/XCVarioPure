#pragma once

// AirMass estimator: an extended Kalman filter fusing IMU, attitude, airspeed, baro altitude
// and GNSS velocity into the 3D wind vector (horizontal instant wind and vertical air mass movement).
//
// The class is plain C++ without ESP-IDF dependencies, so it can be verified on a host with
// synthetic flight data (see tools/airmass_sim).
//
// Frames: NED navigation frame, heading psi clockwise from north, wind as the air mass velocity
// vector (the direction the air moves to), Wd > 0 for sinking air.

#include "math/Matrix.h"
#include "AirMassGnssTiming.h"
#include "AirMassSampling.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

class AirMassEKF {
   public:
    // state vector layout
    // KCL scales the modelled angle of attack (lift slope error), THB is the offset of the body x axis
    enum : uint8_t { WN = 0, WE, WD, PSI, KTAS, THB, ALT, VD, BAZ, BGZ, TAU, KCL, NX };

    // per prediction cycle input (10Hz on target; recordings can have 20Hz)
    struct Input {
        uint32_t time_ms = 0;
        float dt = 0.1f;        // s
        float roll = 0.f;       // rad, right wing down positive
        float pitch = 0.f;      // rad, nose up positive
        float yaw_rate = 0.f;   // rad/s, heading rate (clockwise positive), with the body z gyro bias left in
        float acc_down = 0.f;   // m/s², inertial acceleration NED down component (gravity removed)
        float acc_long = 0.f;   // m/s², inertial acceleration along the air velocity (gravity removed)
        float tas = 0.f;        // m/s, unfiltered
        float load_factor = 1.f;// g
        float aoa = 0.f;        // rad, model angle of attack from lift coefficient (CL/CLalpha)
        float polar_sink = NAN; // m/s, positive down, polar sink at current TAS and load factor; NAN to skip
        float baro_alt = NAN;   // m, NAN when invalid
        float slip_sigma = 0.f; // rad, uncertainty of the unmodelled sideslip (the air vector assumes coordinated flight)
        float slip_beta = 0.f;  // rad, signed sideslip from lateral specific force; positive air from the right
        bool continuous = true; // false: an inferred cycle or the first measured cycle after a sensor gap
        bool flying_speed = true; // false below the IAS flying-speed cutoff (set by Pipeline)
        bool baro_fresh = true;
        bool kinematic_fresh = true;
        bool energy_fresh = true;
        float imu_age = 0.f; // seconds since the older of the latest acceleration/gyro samples
    };

    // GNSS velocity observation
    struct GnssVelocity {
        uint32_t time_ms = 0;   // time of arrival
        uint16_t latency_ms = 0;// nominal age of the observation at arrival, for the recording; the filter learns the latency
        float vn = 0.f, ve = 0.f;
        float sigma_h = 0.5f;   // m/s, horizontal speed accuracy
        uint32_t fix_time_ms = airmass::GnssTiming::UNKNOWN; // source UTC milliseconds of day, when available
    };

    // tuning
    struct Config {
        float q_wind_h = 0.04f;     // (m/s)²/s horizontal wind random walk
        float q_wind_v = 1.125f;    // (m/s)²/s vertical wind random walk; thermal entry within ~0.5s at the lighter damping of its needle
        float q_psi = 3e-5f;        // rad²/s gyro heading drift
        float q_ktas = 1e-8f;       // 1/s airspeed scale random walk
        float q_thb = 1e-7f;        // rad²/s pitch offset random walk
        float q_alt = 0.01f;        // m²/s
        float q_vd = 0.25f;         // (m/s)²/s accelerometer noise
        float q_baz = 1e-5f;        // (m/s²)²/s accel bias random walk
        float q_bgz = 1e-8f;        // (rad/s)²/s body z gyro bias random walk (temperature drift)
        float q_tau = 1e-5f;        // s²/s GNSS latency random walk: a receiver's latency is learned within half an hour
                                    // (1e-6: 2.5 h); 1e-4 jitters by 0.15 s
        float q_kcl = 1e-8f;        // 1/s angle of attack scale random walk
        float r_baro = 0.09f;       // m² baro altitude noise
        float sigma_pitch = 0.03f;  // rad, the attitude's pitch error in calm straight flight: the weight of the pitch path
                                    // (Wd = vd - va_d) against the energy balance. 0.017 rad fits the simulator's attitude,
                                    // the real one (gated gyros, flaps, the AoA model) is worse: on 22 real flights 0.03
                                    // takes the vario's own error (three-cornered hat, 10 s) from 0.25 to 0.22 m/s straight
                                    // and 0.26 to 0.25 circling, over 30 s from 0.19 to 0.15 m/s; simulator cruise rms
                                    // 0.28 to 0.33 (TE netto 0.44), false lift 1.3 to 2.2 /h
        float sigma_energy = 0.5f;  // m/s energy constraint noise (polar error, TAS derivative)
        float energy_tau = 0.5f;    // s low pass for the energy balance terms (acceleration and vertical speed alike)
        float kinetic_tau = 5.f;    // s, low pass of the airspeed derivative minus the inertial acceleration: what changes
                                    // slower is not a gust but an error of the inertial acceleration (attitude, AoA model).
                                    // Faster follows that error better on real flights (30 s straight: 0.297 m/s rms off
                                    // the GNSS energy at 10 s, 0.286 at 8 s, 0.270 at 5 s; with sigma_pitch 0.03 the own
                                    // error over 30 s 0.152 at 8 s, 0.115 at 5 s), but also takes part of a long gust for
                                    // it (simulator gust spike 1.28, 1.40, 1.73 m/s; 2.1 with sigma_pitch 0.03)
        float gust_acc = 1.5f;      // m/s², transient disagreement at which the inertial acceleration takes over (gust)
        float wind_work = 1.f;      // weight of the work of the estimated wind change in the energy balance (va . dW/dt)
        float wind_work_hp = 10.f;  // s, high pass of that term (0: none): only its fast part (gusts); slower, it is mostly
                                    // the wobble of the estimated wind, which drifted the vario by 0.1 m/s over minutes
        float polar_calibrated = 1.f; // 1: the polar at the calibrated IAS (k_tas), 0: at the measured IAS like S2F
        float blend_hp = 120.f;     // s, high pass of the kinetic blend's gust correction (0: none): a gust is short, what the
                                    // blend keeps correcting for minutes is attitude error
        float blend_bank = 30.f;    // deg, the kinetic blend fades out around this bank angle (0: at any bank): on real
                                    // flights it rejects gusts in straight flight, but in circling the inertial acceleration
                                    // is not good enough for it, AirMass read 0.79 m/s rms off the GNSS energy over 10 s with
                                    // it at full weight against 0.63 without (the simulator does not show this, its cause is
                                    // not known yet). Without it the airspeed derivative reaches the needle: faded out
                                    // between 15 and 25° the needle moved 1.63 m/s within 1 s while circling, TE netto 0.85.
                                    // Faded from 10 to 50° (25 % at 40°) it moves 1.22, as accurate (0.644 m/s, TE 0.642)
        float blend_bank_width = 40.f; // deg, over which it fades (blend_bank -+ width / 2)
        float circling_blend_tau = 0.5f; // s, as the blend fades out: the inertial acceleration for what changes faster
                                    // than this, the airspeed derivative for the rest (complementary; 0: off). Too fast for
                                    // the attitude error around the circle (period ~20 s) to pass, fast enough to keep the
                                    // pitot noise off the needle and the lift early: on real flights the needle shows the
                                    // lift 1.4 s late while circling (TE netto 2.6 s; without it 1.8 s, 2.2 s with the
                                    // circling_tau that took for a steady needle), 0.660 m/s rms over 10 s (TE 0.642);
                                    // 1 s: 0.0 s late, 0.676 m/s
        float circling_tau = 0.5f;  // s, low pass of the vario output, faded in as the blend fades out (0: none): with
                                    // the above the needle moves 0.86 m/s within 1 s while circling, as TE netto (0.85)
        float energy_load_weight = 0.f; // energy balance noise growth with the load factor deviation |n - 1|
        float energy_gust_weight = 0.f; // energy balance noise growth with the airspeed/inertia disagreement [per m/s²]
        float kcl_min = 1.f, kcl_max = 1.f; // range of the angle of attack scale; equal: not learned, the wing's lift slope
                                    // (CL_ALPHA) is taken as it is. With the pitch offset learned per flap position the scale is
                                    // not observable: within one flap setting the modelled angle of attack hardly varies, and
                                    // between settings the offsets take the difference. Learned, it ran to its 0.8 limit on most
                                    // of 25 real flights (all flapped gliders), freed it wandered from 0.2 to 1.1 with the offset
                                    // following it to -9°, and the vario did not change with it pinned at 1 (own error 0.112
                                    // against 0.114 m/s over 10 s on 11 flights)
        float anchor_tau = 0.f;     // s, the vario output follows the plain energy balance over this time (0: off)
        float sigma_air = 0.25f;    // m/s air vector model error in the GNSS fusion (plus 1% of TAS)
        // pneumatic lags (lines and sensors), not measured yet: 0.2 s each has the smallest worst case pull-up error
        // over 0..0.3 s static and 0..0.6 s pitot lag in tools/airmass_sim (an overestimate costs less than none)
        float lag_static = 0.2f;    // s, static pressure: the baro altitude model
        float lag_pitot = 0.2f;     // s, dynamic pressure: the energy balance delays the inertial terms by it, to
                                    // compare them with the airspeed change at the same time
        float static_comp = 1.f;    // static port error compensation, like the TE adjustment (1 = lossless):
                                    // baro altitude corrected by (static_comp - 1) * V²/2g
        float gate = 5.f;           // innovation gate in sigma
        float energy_gate = 100.f;  // innovation gate of the energy balance in sigma (effectively none, see above)
        bool learn_calibration = true; // false: airspeed scale, pitch/AoA offset, AoA scale and latency stay as started
        float sigma_wind_valid = 1.5f; // m/s horizontal wind validity threshold
        float sigma_psi_valid = 0.17f; // rad heading validity threshold
        float degraded_factor = 2.f;   // degraded wind: up to this times the validity thresholds,
        float degraded_time = 1200.f;  // s, and this long after the wind was valid (about 40 km of glide)
    };

    // status bits
    // ST_WIND_DEGRADED: not valid any more, but still more accurate than holding the last valid wind (shown marked)
    enum : uint8_t { ST_VARIO_VALID = 1, ST_WIND_VALID = 2, ST_GNSS_FRESH = 4, ST_ALIGNED = 8, ST_WIND_DEGRADED = 16 };

    AirMassEKF();

    void setConfig(const Config& c) { _cfg = c; }
    const Config& config() const { return _cfg; }

    // Start the vertical part, warm start with learned calibration values
    static constexpr float START_LATENCY = 0.75f; // s, the GNSS latency before anything was learned (classical Flarm)
    void reset(float baro_alt, float k_tas = 1.f, float pitch_ofs = 0.f, float latency = START_LATENCY, float k_aoa = 1.f);
    void stop() { _running = false; }
    bool isRunning() const { return _running; }
    bool isAligned() const { return _aligned; }
    bool learnsAoaScale() const { return _cfg.learn_calibration && _cfg.kcl_min < _cfg.kcl_max; }

    // Main cycle: prediction plus baro, kinematic and energy fusion
    void update(const Input& in);
    // Horizontal alignment and GNSS fusion (delayed observation).
    // Without a prior wind the filter aligns itself after a turn of about 180°.
    void fuseGnss(const GnssVelocity& g, bool has_prior = false, float wn0 = 0.f, float we0 = 0.f);
    // Optional absolute true heading (e.g. compass), also aligns the filter without a turn
    void fuseHeading(float psi, float sig);

    // Outputs
    float windN() const { return _x[WN]; }
    float windE() const { return _x[WE]; }
    float windD() const { return _x[WD]; }
    // up positive, anchored to the plain energy balance, smoothed while circling (circling_tau)
    float airmassVario() const { return std::isnan(_vario_out) ? rawVario() : _vario_out; }
    // diagnostics: the energy balance's vertical air movement (Wd, down positive) with the kinetic blend, and plain
    // (the airspeed derivative only), of the last cycle
    float energyWd() const { return _wd_energy; }
    float energyWdPlain() const { return _wd_energy_plain; }
    // diagnostics: the pitch path's vertical air movement (Wd = vd - va_d, down positive) of the last cycle
    float kinematicWd() const { return _wd_kinematic; }
    float heading() const { return _x[PSI]; }
    float kTas() const { return _x[KTAS]; }
    float pitchOffset() const { return _x[THB]; }
    float aoaScale() const { return _x[KCL]; }
    // replace the pitch/AoA offset, e.g. for another flap setting, with its uncertainty [rad]
    void setPitchOffset(float thb, float sigma);
    // angle of attack of the body x axis for a modelled angle of attack, with the learned scale and offset
    float alphaOf(float aoa_model) const { return aoa_model * _x[KCL] - _x[THB]; }
    float vDown() const { return _x[VD]; }
    float accBias() const { return _x[BAZ]; }
    float headingRate() const { return _heading_rate; } // rad/s, the heading rate of the last cycle, without the gyro bias
    float yawRateBias() const { return _x[BGZ]; } // rad/s, body z gyro bias (yaw_rate input minus the true heading rate in level flight)
    float gnssLatency() const { return _x[TAU]; } // s, age of a GNSS fix at its arrival on the monotone device clock
    float sigma(int i) const { return std::sqrt(_cov(i, i)); }
    // the wind's uncertainty relative to its validity limits, the larger of wind and heading: valid below 1
    float windUncertainty() const {
        return std::max(std::sqrt(_cov(WN, WN) + _cov(WE, WE)) / _cfg.sigma_wind_valid, sigma(PSI) / _cfg.sigma_psi_valid);
    }
    uint8_t status() const;
    uint32_t gnssAgeMs() const { return _have_gnss ? _last_time_ms - _last_gnss_ms : UINT32_MAX; }
    uint32_t gnssReceivedAgeMs() const { return _have_received_gnss ? _last_time_ms - _last_received_gnss_ms : UINT32_MAX; }
    uint32_t rejectedGnss() const { return _gnss_rejects; }
    float inertialTrust() const { return _inertial_trust; }
    bool covarianceHealthy() const;

    // Air velocity in NED (no wind), as used by the filter
    // dalpha (optional) receives the derivative by alpha; the derivative by psi is (-out[1], out[0], 0)
    static void airVelocity(float tas, float k, float roll, float pitch, float psi, float alpha, float out[3], float dalpha[3]) {
        airVelocity(tas, k, Tilt(roll, pitch), psi, alpha, out, dalpha);
    }

   private:
    // sine and cosine of roll (sp, cp) and pitch (st, ct), taken once per cycle
    struct Tilt {
        float sp, cp, st, ct;
        Tilt(float roll, float pitch) : sp(std::sin(roll)), cp(std::cos(roll)), st(std::sin(pitch)), ct(std::cos(pitch)) {}
    };
    static void airVelocity(float tas, float k, const Tilt& tilt, float psi, float alpha, float out[3], float dalpha[3]);

    friend struct AirMassFlapHistoryTest; // host regression access to delayed flap geometry
    // raw inputs of one cycle; the delayed air vector gets re-evaluated with the current
    // calibration states, the heading through the uncorrected gyro integral
    struct Snapshot {
        uint32_t time_ms;
        float tas, roll, pitch, aoa;
        float alpha_shift; // flap offset changes since this snapshot, independent of the lift slope
        float psi_gyro; // gyro heading integral at that time
        float slip_sigma;
        bool continuous;
        bool flying_speed;
    };
    static constexpr uint32_t HISTORY_MS = 6400; // the oldest GNSS fix the delayed fusion takes
    // supported flight until the vario is valid; started in flight the attitude has to converge first: without a
    // converged AHRS the vario is off by 0.5 to 1 m/s for half a minute (RESULTS-2026-11-13-start.md)
    static constexpr uint32_t SETTLE_MS = 5000, SETTLE_IN_FLIGHT_MS = 30000;
    // A snapshot every cycle, but faster ticks (20Hz) share one: the newest replaces the head until it is this far from
    // the one before, so the history keeps 6.4s in less memory; the delayed fusion interpolates between them anyway
    static constexpr uint32_t HIST_STEP_MS = 90;
    static constexpr int HIST = HISTORY_MS / HIST_STEP_MS + 3; // and the interpolation endpoints

    // air vector and vertical speed at a past time, interpolated between snapshots
    struct Delayed {
        float va[3], va_dot[3], dalpha[3];
        float tas;
        float aoa;
        float slip_sigma;
    };

    void predict(const Input& in, const Tilt& tilt);
    void fuseBaro(float alt, float sample_dt);
    void fuseKinematic(const Input& in, const Tilt& tilt, const float va[3], const float dalpha[3]);
    void updateKinetic(const Input& in, const float va[3]);
    // the weight of the kinetic blend at this roll angle: 1 in straight flight, fading to 0 around blend_bank
    float blendBankWeight(float roll) const;
    // the vario output before the circling low pass (circling_tau): up positive, anchored to the plain energy balance
    float rawVario() const { return -(_x[WD] + _anchor); }
    void fuseEnergy(const Input& in);
    void alignFromPrior(const GnssVelocity& g, const Delayed& d, float wn0, float we0);
    void alignFromHeading(const GnssVelocity& g, const Snapshot& s);
    void accumulateAlignment(const GnssVelocity& g, const Snapshot& s);
    void commitAlignment(float wn, float we, float psi, float var_wind, float var_psi);
    void restartAlignment();
    const Snapshot* findSnapshot(uint32_t t) const;
    bool delayedState(uint32_t t, Delayed& d) const;
    void wrapPsi();
    void clampCalibration();
    void updateWindValid();
    void sanitize();

    Config _cfg;
    float _x[NX] = {};
    Matrix<NX, NX> _cov;
    Snapshot _hist[HIST] = {};
    int _hist_head = -1;
    int _hist_level = 0;
    float _psi_gyro = 0.f;
    float _heading_rate = 0.f;
    float _tas_f = NAN;  // energy balance low pass of TAS
    float _tas_dot_f = 0.f;
    float _vd_f = 0.f;   // matched low pass of the vertical speed
    float _vd_p = 0.f, _acc_long_p = NAN, _sink_p = NAN; // the inertial terms delayed like the airspeed (lag_pitot)
    float _sink_f = NAN; // matched low pass of the polar sink
    float _acc_long_f = NAN; // matched low pass of the inertial acceleration along the air velocity
    float _acc_corr = 0.f;   // low passed airspeed derivative minus inertial acceleration (attitude error)
    float _acc_corr_c = 0.f; // the same with circling_blend_tau
    float _acc_kin = 0.f;    // acceleration along the air velocity for the energy balance (updateKinetic)
    float _a_air = 0.f;      // its airspeed derivative part, without the gust blend
    float _anchor = 0.f;     // m/s, low pass of the plain energy balance minus the estimate (anchor_tau)
    float _vario_out = NAN;  // m/s, the vario output (airmassVario)
    float _straight = 1.f;   // the kinetic blend's weight at this cycle's bank (blendBankWeight)
    float _wd_energy = NAN, _wd_energy_plain = NAN, _wd_kinematic = NAN; // diagnostics, energyWd(), kinematicWd()
    float _ww_lp = 0.f, _blend_lp = 0.f; // low passes of the wind work and the blend correction (wind_work_hp, blend_hp)
    float _acc_mem = 0.f;// decaying peak of the longitudinal acceleration (AHRS disturbance memory)
    float _wind_prev[2] = {};
    float _wind_rate[2] = {};
    uint32_t _last_time_ms = 0;
    uint32_t _last_gnss_ms = 0;
    uint32_t _last_received_gnss_ms = 0;
    bool _have_gnss = false, _have_received_gnss = false, _have_input = false;
    airmass::GnssTiming _gnss_timing;
    float _inertial_trust = 1.f, _gnss_trust = 0.f;
    uint32_t _gnss_rejects = 0;
    uint16_t _gnss_consecutive_rejects = 0;
    uint32_t _baro_reject_ms = 0;
    uint32_t _energy_reject_ms = 0; // the energy balance off by more than WD_INNOVATION_MAX for this long
    airmass::SampleClock _baro_clock, _kinematic_clock, _energy_clock;
    // least squares alignment: v_gnss = W + Rz(psi_gyro) * [c -s; s c] * u_air
    struct Alignment {
        float ata[4][4];
        float atz[4];
        float zz;
        int n;
        float psi_min, psi_max, psi_last, psi_unwrapped;
        uint32_t start_ms;
    } _al = {};
    // absolute heading for the alignment, taken at the gyro heading integral psi_gyro
    struct HeadingAid {
        float psi, sigma, psi_gyro;
        uint32_t time_ms;
        bool valid;
    } _aid = {};
    uint32_t _vertical_settle_ms = 0;
    uint32_t _settle_ms = SETTLE_MS; // what the vario needs now to be valid
    bool _running = false;
    bool _aligned = false;
    bool _wind_valid = false;
    bool _wind_degraded = false;
    uint32_t _wind_valid_ms = 0; // the last cycle with a valid wind
};
