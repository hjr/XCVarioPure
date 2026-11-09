#pragma once
// The replay of an AirMass recording (main/airmass/AirMassRecord.h) through airmass::Pipeline, line by line: shared by
// airmass_replay (replay.cpp) and the browser replay (web/airmass_wasm.cpp).

#include "AirMassPipeline.h"
#include "AirMassRecord.h"
#include "replay_settings.h"
#include "stats.h"
#ifndef AIRMASS_NO_CONFIG_SETTINGS
#include "math/MahonyAHRS.h"
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace airmass::replay {

class Replayer {
public:
    // called when a cycle ends (with the next one, a restart, the stop or finish()): the pipeline then has the cycle's
    // GNSS and compass fusion and the averaged wind; device_vario is the device's own $PXCVA output where recorded
    std::function<void(const Pipeline& pipe, float device_vario)> on_cycle;

    Settings settings;
    bool override_warm = false;
    float warm[4] = {1.f, 0.f, 1.f, 0.75f}; // k_tas, pitch offset [rad], aoa scale, GNSS latency [s]

    long lines = 0, frames = 0, gnss = 0, headings = 0, starts = 0, skipped = 0, bad = 0;
    uint32_t t_first = 0, t_last = 0;
    Stats vario_diff;
    long status_diff = 0, outputs = 0;
    std::string error; // set when a setting is unknown, the replay then stops

    void setLearning(bool learn) { _pipe.setLearning(learn); }
    bool setMahony(bool enabled) {
#ifdef AIRMASS_NO_CONFIG_SETTINGS
        if (enabled) {
            error = "Mahony AHRS is not available in this older estimator";
            return false;
        }
#endif
        _use_mahony = enabled;
        return true;
    }
    bool usingMahony() const { return _use_mahony; }
    Pipeline& pipeline() { return _pipe; }

    // one log line (any NMEA, other sentences are skipped); false after an error
    bool line(const char* text) {
        if (!error.empty()) {
            return false;
        }
        lines++;
        namespace rec = airmass::record;
        if (!rec::parse(text, _s)) {
            if (std::strncmp(text, "$PXCV", 5) == 0) {
                bad++; // ours, but damaged (checksum)
            }
            return true;
        }
        if (rec::decode(_s, _setup)) {
            _pipe.configure(_setup);
            std::string unknown;
            if (!applySettings(_pipe.ekf(), settings, &unknown)) {
                error = "unknown setting " + unknown;
                return false;
            }
            _configured = true;
        } else if (rec::decode(_s, _st)) {
            endCycle();
#ifdef AIRMASS_NO_CONFIG_SETTINGS
            // an estimator before the absolute latency (analysis/build_old.sh): it starts from an offset to 0.75 s
            if (_s.n > 12 + FlapOffsets::N) {
                _st.warm.lat_ofs -= 0.75f;
            }
#endif
            if (override_warm) {
                _st.warm.k_tas = warm[0];
                _st.warm.pitch_ofs = warm[1];
                _st.warm.k_aoa = warm[2];
#ifdef AIRMASS_NO_CONFIG_SETTINGS
                _st.warm.lat_ofs = warm[3] - 0.75f;
#else
                _st.warm.latency = warm[3];
#endif
                std::fill(std::begin(_st.warm.flap_ofs), std::end(_st.warm.flap_ofs), FlapOffsets::UNKNOWN); // a new calibration
            }
            _pipe.start(_st.t, _st.alt, _st.down, _st.gyro, _st.warm);
#ifndef AIRMASS_NO_CONFIG_SETTINGS
            if (_use_mahony) {
                _mahony_q = Quaternion::fromDownVector({_st.down[0], _st.down[1], _st.down[2]});
                _mahony_omega = 0.f;
                _mahony_calibrating = 7; // like FlightAHRS::setup() when airborne
                _mahony_time = _st.t;
            }
#endif
            _running = _configured;
            starts++;
            if (!t_first) {
                t_first = _st.t;
            }
        } else if (!_running) {
            skipped += !std::strcmp(_s.name, "PXCVR"); // the log started in flight: wait for the next start
        } else if (rec::decode(_s, _fr)) {
            endCycle();
#ifndef AIRMASS_NO_CONFIG_SETTINGS
            if (_use_mahony) {
                const float dt = std::clamp((_fr.time_ms - _mahony_time) / 1000.f, 0.05f, 0.3f);
                _mahony_time = _fr.time_ms;
                const vector_f gyro{_fr.gyro[0], _fr.gyro[1], _fr.gyro[2]};
                MahonyAHRS::update(_mahony_q, _mahony_omega, _mahony_calibrating, {},
                                  {_fr.f_body[0], _fr.f_body[1], _fr.f_body[2]}, gyro, _fr.tas, dt);
                const vector_f down = _mahony_q.rotateInv({0.f, 0.f, 1.f});
                ExternalAttitude att;
                for (int i = 0; i < 3; ++i) att.down[i] = down[i];
                att.roll = std::atan2(down.y, down.z);
                att.pitch = MahonyAHRS::pitchRad(_mahony_q);
                // Euler heading rate, as expected by AirMass (not gyro dot down).
                const float cp2 = down.y * down.y + down.z * down.z;
                att.yaw_rate = cp2 < 0.01f ? 0.f : (gyro.y * down.y + gyro.z * down.z) / cp2;
                _pipe.step(_fr, &att);
            } else
#endif
            {
                _pipe.step(_fr);
            }
            _open = true;
            _device_vario = NAN;
            frames++;
            t_last = _fr.time_ms;
        } else if (rec::decode(_s, _g)) {
            _pipe.gnss(_g.v, _g.prior, _g.wn0, _g.we0);
            gnss++;
        } else if (rec::decode(_s, _h)) {
            _pipe.heading(_h.psi, _h.sigma);
            headings++;
        } else if (!std::strcmp(_s.name, "PXCVX")) {
            endCycle();
            _running = false;
        } else if (rec::decode(_s, _out)) {
            // the device's output for the cycle before
            const AirMassEKF& ekf = _pipe.ekf();
            vario_diff.add(ekf.airmassVario() - _out.vario);
            status_diff += _out.status != ekf.status();
            outputs++;
            _device_vario = _out.vario;
        }
        return true;
    }

    void finish() { endCycle(); }

private:
    void endCycle() {
        if (!_open) {
            return;
        }
        _open = false;
        _pipe.endCycle();
        if (on_cycle) {
            on_cycle(_pipe, _device_vario);
        }
    }

    Pipeline _pipe;
    bool _use_mahony = false;
#ifndef AIRMASS_NO_CONFIG_SETTINGS
    Quaternion _mahony_q;
    float _mahony_omega = 0.f;
    int8_t _mahony_calibrating = 0;
    uint32_t _mahony_time = 0;
#endif
    bool _running = false, _configured = false, _open = false;
    float _device_vario = NAN;
    airmass::record::Sentence _s;
    Setup _setup;
    airmass::record::Start _st;
    Frame _fr;
    airmass::record::Gnss _g;
    airmass::record::Heading _h;
    airmass::record::Output _out;
};

} // namespace airmass::replay
