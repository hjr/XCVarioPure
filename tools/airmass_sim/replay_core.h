#pragma once
// The replay of an AirMass recording (main/airmass/AirMassRecord.h) through airmass::Pipeline, line by line: shared by
// airmass_replay (replay.cpp) and the browser replay (web/airmass_wasm.cpp).

#include "AirMassPipeline.h"
#include "AirMassRecord.h"
#include "mahony_source.h"
#include "replay_settings.h"
#include "stats.h"

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

    long lines = 0, frames = 0, gnss = 0, headings = 0, rotations = 0, starts = 0, skipped = 0, bad = 0;
    long recovered_starts = 0, estimated_attitude_starts = 0;
    uint32_t t_first = 0, t_last = 0;
    double seconds = 0.;
    Stats vario_diff;
    long status_diff = 0, outputs = 0;
    std::string error; // set when a setting is unknown, the replay then stops

    void setLearning(bool learn) { _pipe.setLearning(learn); }
    void setPredictionPeriod(uint32_t ms) { _prediction_ms = ms; }
    uint32_t predictionPeriod() const { return _prediction_ms; }
    void setMahony(bool enabled) { _use_mahony = enabled; }
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
            if (_pending_start) begin(_st);
        } else if (rec::decode(_s, _st)) {
            // A logger can catch the start before the next repeated configuration telegram.
            _pending_start = true;
            if (_configured) begin(_st);
        } else if (_s.is("PXCV", 11)) {
            const float roll = Units::deg_to_rad(_s.f[9]), pitch = Units::deg_to_rad(_s.f[10]);
            _have_logged_attitude = std::isfinite(roll) && std::isfinite(pitch) &&
                                    std::fabs(_s.f[9]) <= 180.f && std::fabs(_s.f[10]) <= 90.f;
            if (_have_logged_attitude) {
                _logged_down[0] = -std::sin(pitch);
                _logged_down[1] = std::sin(roll) * std::cos(pitch);
                _logged_down[2] = std::cos(roll) * std::cos(pitch);
                _attitude_time = _last_frame_time;
                _attitude_timed = _have_frame_time;
            }
        } else if (!std::strcmp(_s.name, "PXCVX")) {
            endCycle();
            _running = _pending_start = _have_logged_attitude = false;
            _allow_recovery = false; // an explicit stop is not a missing start
        } else if (rec::decode(_s, _fr)) {
            endCycle();
            if (_have_logged_attitude && !_attitude_timed) {
                _attitude_time = _fr.time_ms;
                _attitude_timed = true;
            }
            _last_frame_time = _fr.time_ms;
            _have_frame_time = true;
            if (!_running && !recover(_fr)) {
                skipped++;
                return true;
            }
            stepFrame();
        } else if (!_running) {
            return true;
        } else if (rec::decode(_s, _g)) {
            _pipe.gnss(_g.v, _g.prior, _g.wn0, _g.we0);
            gnss++;
        } else if (rec::decode(_s, _h)) {
            _pipe.heading(_h.psi, _h.sigma);
            headings++;
        } else if (rec::decode(_s, _rot)) {
            _pipe.rotateBody(_rot.rv);
            rotations++;
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
    void overrideWarm(Warm& w) const {
        if (!override_warm) return;
        w.k_tas = warm[0];
        w.pitch_ofs = warm[1];
        w.latency = warm[3];
        w.k_aoa = warm[2];
        w.latency_sigma = AirMassEKF::START_LATENCY_SIGMA;
        std::fill(std::begin(w.flap_ofs), std::end(w.flap_ofs), FlapOffsets::UNKNOWN);
    }

    void begin(record::Start st) {
        endCycle();
        overrideWarm(st.warm);
        _pipe.start(st.t, st.alt, st.down, st.gyro, st.warm);
        if (_use_mahony) _mahony.start(st.t, st.down, st.gyro);
        _running = true;
        _pending_start = false;
        _allow_recovery = true;
        _cycle_time = st.t;
        _have_cycle_frame = false;
        if (starts++ == 0) t_first = st.t;
    }

    bool recover(const Frame& f) {
        if (!_configured || !_allow_recovery || _pending_start) return false;
        if (f.valid != Frame::ALL) return false;
        if (f.sample_times) for (uint32_t stamp : f.sample_ms) {
            if ((uint32_t)(f.time_ms - stamp) > 300) return false;
        }
        if (!std::isfinite(f.baro_alt) || !std::isfinite(f.ias) || !std::isfinite(f.tas) ||
            f.ias < std::max(10.f, _setup.polar.v_stall * 0.8f) || f.tas <= 10.f) return false;
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(f.f_body[i]) || !std::isfinite(f.gyro[i])) return false;
        }
        record::Start st;
        st.t = f.time_ms;
        st.alt = f.baro_alt;
        std::copy(std::begin(f.gyro), std::end(f.gyro), st.gyro);
        const bool logged = _have_logged_attitude && (uint32_t)(f.time_ms - _attitude_time) <= 2000;
        if (logged) {
            std::copy(std::begin(_logged_down), std::end(_logged_down), st.down);
        } else {
            Warm w;
            overrideWarm(w);
            // An unmodelled manoeuvre cannot supply a useful initial gravity reference. Wait for another frame.
            if (!downFromMotion(_setup, f, w, st.down)) return false;
        }
        begin(st);
        recovered_starts++;
        estimated_attitude_starts += !logged;
        return true;
    }

    void stepFrame() {
        const int32_t elapsed = (int32_t)(_fr.time_ms - _cycle_time);
        if (elapsed < 0 || (_have_cycle_frame && elapsed == 0)) {
            skipped++;
            return;
        }
        if (_prediction_ms && _have_cycle_frame && elapsed <= 10000) {
            const Frame next = _fr;
            _fr = _held_frame;
            // Hold only past readings. Their source clocks must not advance with the prediction clock.
            // a late frame is still one cycle of the device, not a missed one
            while ((uint32_t)(next.time_ms - _fr.time_ms) > _prediction_ms * 3 / 2) {
                _fr.time_ms += _prediction_ms;
                stepOnce();
                endCycle();
            }
            _fr = next;
        }
        if (_prediction_ms && !_fr.sample_times) {
            for (auto& stamp : _fr.sample_ms) stamp = _fr.time_ms;
            _fr.sample_times = true;
        }
        stepOnce();
        _held_frame = _fr;
        _have_cycle_frame = true;
    }

    void stepOnce() {
        if ((int32_t)(_fr.time_ms - _cycle_time) > 0) {
            seconds += (_fr.time_ms - _cycle_time) * 1e-3;
            _cycle_time = _fr.time_ms;
        }
        if (_use_mahony) {
            const ExternalAttitude att = _mahony.step(_fr);
            _pipe.step(_fr, &att);
        } else {
            _pipe.step(_fr);
        }
        _open = true;
        _device_vario = NAN;
        frames++;
        t_last = _fr.time_ms;
    }
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
    MahonySource _mahony;
    bool _running = false, _configured = false, _open = false;
    bool _pending_start = false, _allow_recovery = true;
    bool _have_logged_attitude = false, _attitude_timed = false, _have_frame_time = false;
    float _logged_down[3] = {};
    uint32_t _attitude_time = 0, _last_frame_time = 0;
    uint32_t _cycle_time = 0;
    uint32_t _prediction_ms = 0;
    Frame _held_frame;
    bool _have_cycle_frame = false;
    float _device_vario = NAN;
    airmass::record::Sentence _s;
    Setup _setup;
    airmass::record::Start _st;
    Frame _fr;
    airmass::record::Gnss _g;
    airmass::record::Heading _h;
    airmass::record::Rotation _rot;
    airmass::record::Output _out;
};

} // namespace airmass::replay
