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

#include <algorithm>
#include <cmath>
#include <cstdint>

class AirMassEKF {
   public:
    // state vector layout
    // KCL scales the modelled angle of attack (lift slope error), THB is the offset of the body x axis
    enum : uint8_t { WN = 0, WE, WD, PSI, KTAS, THB, ALT, VD, BAZ, BGZ, TAU, KCL, NX };

    // per cycle input (10Hz typical)
    struct Input {
        uint32_t time_ms = 0;
        float dt = 0.1f;        // s
        float roll = 0.f;       // rad, right wing down positive
        float pitch = 0.f;      // rad, nose up positive
        float yaw_rate = 0.f;   // rad/s, rotation around the nav down axis (clockwise positive)
        float acc_down = 0.f;   // m/s², inertial acceleration NED down component (gravity removed)
        float acc_long = 0.f;   // m/s², inertial acceleration along the air velocity (gravity removed)
        float tas = 0.f;        // m/s, unfiltered
        float load_factor = 1.f;// g
        float aoa = 0.f;        // rad, model angle of attack from lift coefficient (CL/CLalpha)
        float polar_sink = NAN; // m/s, positive down, polar sink at current TAS and load factor; NAN to skip
        float baro_alt = NAN;   // m, NAN when invalid
        float slip_sigma = 0.f; // rad, uncertainty of the unmodelled sideslip (the air vector assumes coordinated flight)
    };

    // GNSS velocity observation
    struct GnssVelocity {
        uint32_t time_ms = 0;   // time of arrival
        uint16_t latency_ms = 0;// nominal age of the observation at arrival, the filter estimates a correction
        float vn = 0.f, ve = 0.f;
        float sigma_h = 0.5f;   // m/s, horizontal speed accuracy
    };

    // tuning
    struct Config {
        float q_wind_h = 0.04f;     // (m/s)²/s horizontal wind random walk
        float q_wind_v = 1.125f;    // (m/s)²/s vertical wind random walk; thermal entry within ~0.5s at half the TE damping
        float q_psi = 3e-5f;        // rad²/s gyro heading drift
        float q_ktas = 1e-8f;       // 1/s airspeed scale random walk
        float q_thb = 1e-7f;        // rad²/s pitch offset random walk
        float q_alt = 0.01f;        // m²/s
        float q_vd = 0.25f;         // (m/s)²/s accelerometer noise
        float q_baz = 1e-5f;        // (m/s²)²/s accel bias random walk
        float q_bgz = 1e-8f;        // (rad/s)²/s yaw rate bias random walk (temperature drift)
        float q_tau = 1e-6f;        // s²/s GNSS latency correction random walk
        float q_kcl = 1e-8f;        // 1/s angle of attack scale random walk
        float r_baro = 0.09f;       // m² baro altitude noise
        float sigma_pitch = 0.017f; // rad AHRS pitch noise in calm straight flight
        float sigma_energy = 0.5f;  // m/s energy constraint noise (polar error, TAS derivative)
        float energy_tau = 0.5f;    // s low pass for the energy balance terms (TAS derivative and vertical speed alike)
        float sigma_air = 0.25f;    // m/s air vector model error in the GNSS fusion (plus 1% of TAS)
        float static_comp = 1.f;    // static port error compensation, like the TE adjustment (1 = lossless):
                                    // baro altitude corrected by (static_comp - 1) * V²/2g
        float gate = 5.f;           // innovation gate in sigma
        bool learn_calibration = true; // false: airspeed scale, pitch/AoA offset, AoA scale and latency stay as started
        float sigma_wind_valid = 1.5f; // m/s horizontal wind validity threshold
        float sigma_psi_valid = 0.17f; // rad heading validity threshold
    };

    // status bits
    enum : uint8_t { ST_VARIO_VALID = 1, ST_WIND_VALID = 2, ST_GNSS_FRESH = 4, ST_ALIGNED = 8 };

    AirMassEKF();

    void setConfig(const Config& c) { _cfg = c; }

    // Start the vertical part, warm start with learned calibration values
    void reset(float baro_alt, float k_tas = 1.f, float pitch_ofs = 0.f, float latency_ofs = 0.f, float k_aoa = 1.f);
    void stop() { _running = false; }
    bool isRunning() const { return _running; }
    bool isAligned() const { return _aligned; }

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
    float airmassVario() const { return -_x[WD]; } // up positive
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
    float yawRateBias() const { return _x[BGZ]; }
    float latencyOffset() const { return _x[TAU]; } // s, learned correction to the nominal GNSS latency
    float gnssLatency() const { return std::max(_gnss_nominal_ms * 1e-3f + _x[TAU], 0.f); } // s, effective
    float sigma(int i) const { return std::sqrt(_cov(i, i)); }
    uint8_t status() const;
    uint32_t gnssAgeMs() const { return _last_time_ms - _last_gnss_ms; }
    uint32_t rejectedGnss() const { return _gnss_rejects; }
    bool covarianceHealthy() const;

    // Air velocity in NED (no wind), as used by the filter
    // dalpha (optional) receives the derivative by alpha; the derivative by psi is (-out[1], out[0], 0)
    static void airVelocity(float tas, float k, float roll, float pitch, float psi, float alpha, float out[3], float dalpha[3]);

   private:
    // raw inputs of one cycle; the delayed air vector gets re-evaluated with the current
    // calibration states, the heading through the uncorrected gyro integral
    struct Snapshot {
        uint32_t time_ms;
        float tas, roll, pitch, aoa;
        float psi_gyro; // gyro heading integral at that time
        float slip_sigma;
    };
    static constexpr int HIST = 24; // 2.4s at 10Hz, covers the NMEA GNSS latency plus correction

    // air vector and vertical speed at a past time, interpolated between snapshots
    struct Delayed {
        float va[3], va_dot[3], dalpha[3];
        float tas;
        float aoa;
        float slip_sigma;
    };

    void predict(const Input& in);
    void fuseBaro(float alt);
    void fuseKinematic(const Input& in, const float va[3], const float dalpha[3]);
    void fuseEnergy(const Input& in, const float va[3]);
    void alignFromPrior(const GnssVelocity& g, float wn0, float we0);
    void alignFromHeading(const GnssVelocity& g, const Snapshot& s);
    void accumulateAlignment(const GnssVelocity& g, const Snapshot& s);
    void commitAlignment(float wn, float we, float psi, float var_wind, float var_psi);
    void restartAlignment();
    const Snapshot* findSnapshot(uint32_t t) const;
    bool delayedState(uint32_t t, Delayed& d) const;
    void wrapPsi();
    void clampCalibration();
    void sanitize();

    Config _cfg;
    float _x[NX] = {};
    Matrix<NX, NX> _cov;
    Snapshot _hist[HIST] = {};
    int _hist_head = -1;
    int _hist_level = 0;
    float _psi_gyro = 0.f;
    float _tas_f = NAN;  // energy balance low pass of TAS
    float _tas_dot_f = 0.f;
    float _vd_f = 0.f;   // matched low pass of the vertical speed
    float _acc_mem = 0.f;// decaying peak of the longitudinal acceleration (AHRS disturbance memory)
    float _wind_prev[2] = {};
    float _wind_rate[2] = {};
    uint32_t _last_time_ms = 0;
    uint32_t _last_gnss_ms = 0;
    uint16_t _gnss_nominal_ms = 0;
    uint32_t _gnss_rejects = 0;
    uint16_t _gnss_consecutive_rejects = 0;
    uint16_t _baro_consecutive_rejects = 0;
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
    uint16_t _vertical_settle = 0;
    bool _running = false;
    bool _aligned = false;
};
