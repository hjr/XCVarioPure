#pragma once

// Comparison reference for the simulation: a GNSS/INS flight sensor without an air mass estimator. Its vario is a
// total energy vario from the GNSS vertical speed and a GNSS/INS speed compensation, its wind the GNSS velocity minus
// the airspeed along the body axis. Reimplemented after the open-source larus-breeze sw_algorithms_lib (GPL-3.0;
// variometer, KalmanVario_PVA, Kalman_V_A_Aoff_observer, wind_observer, soaring_flight_averager, pt2, HP_LP_fusion),
// with its published steady state Kalman gains and default time constants. It runs at 100 Hz on its own 10 Hz GNSS
// receiver (3D velocity and altitude). Its GNSS acceleration aided AHRS is modelled as the true attitude with small
// slowly varying errors, which if anything flatters it.

#include <cmath>

namespace ref {

constexpr float G = 9.81f;
constexpr float TS = 0.01f; // s, 100 Hz

// second order Butterworth low pass, cutoff as fraction of the sampling rate (pt2)
struct Pt2 {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, old = 0, very_old = 0, out = 0;
    explicit Pt2(float fc) {
        constexpr double B0 = 0.292893218813452, B1 = 0.585786437626905, B2 = 0.292893218813452, A1 = 0.0, A2 = 0.171572875253810, DF = 0.25;
        double d = std::sin(M_PI * (DF - fc)) / std::sin(M_PI * (fc + DF));
        const double a0x = A2 * d * d - A1 + 1.0;
        const double a1x = -2.0 * d * A2 + (d * d + 1.0) * A1 - 2.0 * d;
        const double a2x = A2 - d * A1 + d * d;
        const double b0x = B2 * d * d - B1 * d + B0;
        const double b1x = -2.0 * d * B2 + (d * d + 1.0) * B1 - 2.0 * d * B0;
        const double b2x = B2 - d * B1 + d * d * B0;
        a1 = a1x / a0x;
        a2 = a2x / a0x;
        double nb0 = b0x / a0x, nb1 = b1x / a0x, nb2 = b2x / a0x;
        d = (1.0 + a1 + a2) / (nb0 + nb1 + nb2);
        b0 = nb0 * d;
        b1 = nb1 * d;
        b2 = nb2 * d;
    }
    void settle(float x) {
        very_old = old = x / (1.f + a1 + a2);
        out = x;
    }
    float respond(float in) {
        const float x = in - old * a1 - very_old * a2;
        out = x * b0 + old * b1 + very_old * b2;
        very_old = old;
        old = x;
        return out;
    }
};

// altitude, vertical speed, vertical acceleration and its offset from GNSS altitude, GNSS vertical speed and the INS
// vertical specific force (KalmanVario_PVA, NED: altitude and speed negative up)
struct KalmanPVA {
    float x[4] = {0, 0, 0, -G};
    void update(float neg_alt, float v_down, float acc_down) {
        static constexpr float K[4][3] = {{0.014624427f, 0.008213519f, -0.000020792f},
                                          {0.018480417f, 0.033924392f, 0.006461500f},
                                          {0.014990634f, 0.067156008f, 0.612880096f},
                                          {-0.015011427f, -0.064284231f, 0.006830750f}};
        const float e0 = x[0] + TS * x[1] + 0.5f * TS * TS * x[2], e1 = x[1] + TS * x[2], e2 = x[2], e3 = x[3];
        const float ix = neg_alt - e0, iv = v_down - e1, ia = acc_down - e2 - e3;
        const float e[4] = {e0, e1, e2, e3};
        for (int i = 0; i < 4; ++i) {
            x[i] = e[i] + K[i][0] * ix + K[i][1] * iv + K[i][2] * ia;
        }
    }
};

// horizontal velocity and acceleration (Kalman_V_A_Aoff_observer)
struct KalmanVA {
    float x[3] = {};
    void update(float v, float a) {
        static constexpr float K[3][2] = {{0.045525746f, 0.005179336f}, {0.102816403f, 0.906791781f}, {-0.097637066f, 0.001591462f}};
        const float e0 = x[0] + TS * x[1], e1 = x[1], e2 = x[2];
        const float iv = v - e0, ia = a - e1 - e2;
        x[0] = e0 + K[0][0] * iv + K[0][1] * ia;
        x[1] = e1 + K[1][0] * iv + K[1][1] * ia;
        x[2] = e2 + K[2][0] * iv + K[2][1] * ia;
    }
};

// high pass of the first input plus low pass of the second (HP_LP_fusion)
struct HpLp {
    float a1, out = 0, old_hp = 0;
    explicit HpLp(float feedback) : a1(-feedback) {}
    float respond(float hp, float lp) {
        out = lp * (1.f + a1) - hp * a1 + old_hp * a1 - out * a1;
        old_hp = hp;
        return out;
    }
};

// low pass in straight flight, mean over 16 heading sectors while circling (soaring_flight_averager, soft transitions)
struct SectorAverager {
    static constexpr int N = 16;
    Pt2 lpv[3]; // the straight flight low pass, per component
    bool circling = false;
    float out[3] = {}, sum[N][3] = {};
    int count[N] = {}, old_sector = 0;
    explicit SectorAverager(float fc) : lpv{Pt2(fc), Pt2(fc), Pt2(fc)} {}
    void update(const float v[3], float heading, bool now_circling) {
        if (heading < 0.f) heading += 2.f * float(M_PI);
        for (int k = 0; k < 3; ++k) lpv[k].respond(v[k]);
        if (!circling) {
            if (now_circling) {
                circling = true;
                for (int i = 0; i < N; ++i) {
                    for (int k = 0; k < 3; ++k) sum[i][k] = lpv[k].out;
                    count[i] = 1;
                }
                record(v, heading);
                boxcar();
            } else {
                for (int k = 0; k < 3; ++k) out[k] = lpv[k].out;
            }
        } else {
            if (!now_circling) {
                circling = false;
                for (int k = 0; k < 3; ++k) lpv[k].settle(out[k]);
                for (int i = 0; i < N; ++i) {
                    count[i] = 0;
                    for (int k = 0; k < 3; ++k) sum[i][k] = 0.f;
                }
            }
            record(v, heading);
            bool complete = true;
            for (int i = 0; i < N; ++i) complete = complete && count[i] > 0;
            if (complete) boxcar();
        }
    }
    void record(const float v[3], float heading) {
        int s = static_cast<int>(heading / (2.f * float(M_PI)) * N);
        s = s >= N ? N - 1 : s;
        if (s != old_sector) {
            old_sector = s;
            if (count[s] > 1) {
                count[s] = 0;
                for (int k = 0; k < 3; ++k) sum[s][k] = 0.f;
            }
        }
        for (int k = 0; k < 3; ++k) sum[s][k] += v[k];
        count[s]++;
    }
    void boxcar() {
        int used = 0;
        float a[3] = {};
        for (int i = 0; i < N; ++i) {
            if (count[i] > 0) {
                for (int k = 0; k < 3; ++k) a[k] += sum[i][k] / count[i];
                used++;
            }
        }
        for (int k = 0; k < 3; ++k) out[k] = used ? a[k] / used : 0.f;
    }
};

// the reference instrument, updated at 100 Hz
class InsReference {
public:
    // gnss: the latest GNSS velocity (NED) and altitude (held between fixes); acc_nav: INS specific force in NED [m/s²]
    // (with the instrument's attitude error); x_body: its body x axis in NED; tas: its airspeed; yaw: its heading;
    // turn_rate: around the nav down axis
    void update(const float gnss_v[3], float gnss_alt, const float acc_nav[3], const float x_body[3], float tas, float yaw, float turn_rate) {
        if (!_init) {
            _kv.x[0] = -gnss_alt;
            _init = true;
            _e_diff_old = NAN;
        }
        // circling state (AHRS: turn rate low pass 0.5 s, 10 s in or out)
        _turn.respond(turn_rate);
        const float r = std::fabs(_turn.out);
        if (_counter < 1000 && r > 4.f * float(M_PI) / 180.f) _counter++;
        if (_counter > 0 && r < 0.5f * float(M_PI) / 180.f) _counter--;
        if (_counter == 0) _circling = false;
        else if (_counter == 1000) _circling = true;

        // wind: instant measurement at 100 Hz, its averages at 10 Hz
        float w[3];
        for (int k = 0; k < 3; ++k) {
            w[k] = gnss_v[k] - x_body[k] * tas;
            _resample[k].respond(w[k]);
            _inst[k].respond(w[k]);
        }
        if (++_div == 10) {
            _div = 0;
            const float m[3] = {_resample[0].out, _resample[1].out, _resample[2].out};
            _avg.update(m, yaw, _circling);
        }

        // vario
        _kv.update(-gnss_alt, gnss_v[2], acc_nav[2]);
        float acc[3] = {acc_nav[0], acc_nav[1], _kv.x[2]};
        const float* wa = _avg.out; // speed compensator wind
        _kn.update(gnss_v[0] - wa[0], acc_nav[0]);
        _ke.update(gnss_v[1] - wa[1], acc_nav[1]);
        const float va[3] = {gnss_v[0] - wa[0], gnss_v[1] - wa[1], gnss_v[2] - wa[2]};
        const float e = (va[0] * va[0] + va[1] * va[1] + gnss_v[2] * gnss_v[2]) / (2.f * G);
        const float sc1 = (va[0] * acc[0] + va[1] * acc[1] + va[2] * acc[2]) / G;
        const float sc2 = (_kn.x[0] * _kn.x[1] + _ke.x[0] * _ke.x[1] + _kv.x[1] * _kv.x[2]) / G;
        const float sc3 = std::isnan(_e_diff_old) ? 0.f : (e - _e_diff_old) / TS;
        _e_diff_old = e;
        const float kav[3] = {_kn.x[0], _ke.x[0], _kv.x[1]};
        const float kac[3] = {_kn.x[1], _ke.x[1], _kv.x[2]};
        const float vp = kav[0] * x_body[0] + kav[1] * x_body[1] + kav[2] * x_body[2];
        const float ap = kac[0] * x_body[0] + kac[1] * x_body[1] + kac[2] * x_body[2];
        const float sc4 = vp * ap / G;
        _fusion.respond((sc1 + sc2 + sc4) / 3.f, sc3);
        _vario.respond(_fusion.out - _kv.x[1]);
    }

    float vario() const { return _vario.out; } // total energy vario as displayed, up positive
    // the wind shown: the last circle mean while circling, else the short term average (north, east)
    float windN() const { return _circling ? _avg.out[0] : _inst[0].out; }
    float windE() const { return _circling ? _avg.out[1] : _inst[1].out; }
    float avgWindN() const { return _avg.out[0]; }
    float avgWindE() const { return _avg.out[1]; }

private:
    bool _init = false;
    KalmanPVA _kv;
    KalmanVA _kn, _ke;
    HpLp _fusion{0.9995f};
    Pt2 _vario{TS / 2.f};          // vario time constant 2 s
    Pt2 _turn{1.f / 0.5f / 100.f}; // AHRS turn rate filter
    Pt2 _resample[3] = {Pt2(0.04f), Pt2(0.04f), Pt2(0.04f)};
    Pt2 _inst[3] = {Pt2(TS / 5.f), Pt2(TS / 5.f), Pt2(TS / 5.f)}; // wind time constant 5 s
    SectorAverager _avg{0.1f / 30.f}; // the speed compensator wind and the reported average: both 30 s, the same
    float _e_diff_old = NAN;
    int _counter = 0, _div = 0;
    bool _circling = false;
};

} // namespace ref
