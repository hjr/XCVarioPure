// The browser replay (index.html): the AirMass recording replayed through airmass::Pipeline, compiled to WebAssembly
// (build.sh). The page feeds the recording text in chunks and reads one row per cycle.

#include "math/Units.h"
#include "replay_core.h"

#include <emscripten/emscripten.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

// per cycle, see am_columns()
enum Col { VARIO, WIND_N, WIND_E, WIND_D, STATUS, HEADING, AVG_N, AVG_E, ROLL, PITCH, K_TAS, PITCH_OFS, LATENCY, POLAR_SINK, WIND_FADE,
           GNSS_AGE, GNSS_RECEIVED_AGE, GNSS_REJECTS, INERTIAL_TRUST, SLIP_BETA, ENERGY_WD, ENERGY_WD_PLAIN, KINEMATIC_WD, VD, SIGMA_WD, GUST, GUST_EVENT, NCOLS };
const char* COLUMNS = "vario,wind_n,wind_e,wind_d,status,heading_deg,avg_wind_n,avg_wind_e,roll_deg,pitch_deg,k_tas,pitch_ofs_deg,latency_ms,polar_sink,wind_fade,gnss_age_ms,gnss_received_age_ms,gnss_rejects,inertial_trust,slip_beta_deg,energy_wd,energy_wd_plain,kinematic_wd,vd,sigma_wd,gust,gust_event";

airmass::replay::Replayer* g_replay = nullptr;
std::vector<float> g_rows;
std::vector<uint32_t> g_times;
std::string g_text;

} // namespace

extern "C" {

// start a replay. settings: "name=value" pairs separated by ',' or ';' (AirMassEKF::Config); warm: use the given
// calibration instead of the recorded one. Returns 0 and the reason in am_error() for an unknown setting.
EMSCRIPTEN_KEEPALIVE int am_begin(const char* settings, int learn, int warm, float k_tas, float pitch_ofs_deg, float aoa_scale, float latency_s, int mahony) {
    delete g_replay;
    g_replay = new airmass::replay::Replayer();
    g_replay->setPredictionPeriod(100);
    g_rows.clear();
    g_times.clear();
    g_replay->setMahony(mahony != 0);
    g_replay->setLearning(learn != 0);
    g_replay->override_warm = warm != 0;
    g_replay->warm[0] = k_tas;
    g_replay->warm[1] = Units::deg_to_rad(pitch_ofs_deg);
    g_replay->warm[2] = aoa_scale;
    g_replay->warm[3] = latency_s;
    if (!airmass::replay::parseSettings(settings ? settings : "", g_replay->settings, &g_replay->error)) {
        return 0;
    }
    g_replay->on_cycle = [](const airmass::Pipeline& pipe, float) {
        const AirMassEKF& ekf = pipe.ekf();
        const AirMassEKF::Input& i = pipe.input();
        // the wind's uncertainty from the validity limit (0) to where it is no longer shown (1)
        const float fade = std::clamp((ekf.windUncertainty() - 1.f) / std::max(ekf.config().degraded_factor - 1.f, 0.01f), 0.f, 1.f);
        const float row[NCOLS] = {ekf.airmassVario(), ekf.windN(), ekf.windE(), ekf.windD(), static_cast<float>(ekf.status()),
                                  Units::rad_to_deg(ekf.heading()), pipe.avgWind().windN(), pipe.avgWind().windE(), Units::rad_to_deg(i.roll),
                                  Units::rad_to_deg(i.pitch), ekf.kTas(), Units::rad_to_deg(ekf.pitchOffset()), ekf.gnssLatency() * 1000.f,
                                  i.polar_sink, fade, static_cast<float>(ekf.gnssAgeMs()), static_cast<float>(ekf.gnssReceivedAgeMs()),
                                  static_cast<float>(ekf.rejectedGnss()), ekf.inertialTrust(), Units::rad_to_deg(i.slip_beta),
                                  ekf.energyWd(), ekf.energyWdPlain(), ekf.kinematicWd(), ekf.vDown(), ekf.sigma(AirMassEKF::WD),
                                  pipe.gust().value(), static_cast<float>(pipe.gust().event())};
        g_rows.insert(g_rows.end(), row, row + NCOLS);
        g_times.push_back(i.time_ms);
    };
    return 1;
}

// a chunk of log lines ('\n' separated); 0 after an error (am_error())
EMSCRIPTEN_KEEPALIVE int am_feed(char* text) {
    if (!g_replay) {
        return 0;
    }
    char* p = text;
    while (*p) {
        char* nl = std::strchr(p, '\n');
        if (nl) {
            *nl = 0;
        }
        if (!g_replay->line(p)) {
            return 0;
        }
        if (!nl) {
            break;
        }
        p = nl + 1;
    }
    return 1;
}

EMSCRIPTEN_KEEPALIVE void am_finish() {
    if (g_replay) {
        g_replay->finish();
    }
}

EMSCRIPTEN_KEEPALIVE int am_nrows() { return static_cast<int>(g_times.size()); }
EMSCRIPTEN_KEEPALIVE int am_ncols() { return NCOLS; }
EMSCRIPTEN_KEEPALIVE const char* am_columns() { return COLUMNS; }
EMSCRIPTEN_KEEPALIVE const float* am_rows() { return g_rows.data(); }
EMSCRIPTEN_KEEPALIVE const uint32_t* am_times() { return g_times.data(); } // device ms per row
EMSCRIPTEN_KEEPALIVE const char* am_error() { return g_replay ? g_replay->error.c_str() : "not started"; }

// counters and the learned calibration as JSON
EMSCRIPTEN_KEEPALIVE const char* am_summary() {
    if (!g_replay) {
        return "{}";
    }
    const airmass::replay::Replayer& r = *g_replay;
    const AirMassEKF& ekf = g_replay->pipeline().ekf();
    char buf[768];
    std::snprintf(buf, sizeof(buf),
                  "{\"lines\":%ld,\"cycles\":%ld,\"gnss\":%ld,\"compass\":%ld,\"rotations\":%ld,\"starts\":%ld,\"skipped\":%ld,\"damaged\":%ld,"
                  "\"recovered_starts\":%ld,\"estimated_attitude_starts\":%ld,\"seconds\":%.3f,\"prediction_ms\":%lu,"
                  "\"k_tas\":%.4f,\"pitch_ofs_deg\":%.3f,\"aoa_scale\":%.4f,\"latency_ms\":%.0f,\"gyro_z_bias_dps\":%.4f,\"mahony\":%s}",
                  r.lines, r.frames, r.gnss, r.headings, r.rotations, r.starts, r.skipped, r.bad, r.recovered_starts, r.estimated_attitude_starts,
                  r.seconds, static_cast<unsigned long>(r.predictionPeriod()),
                  ekf.kTas(), Units::rad_to_deg(ekf.pitchOffset()),
                  ekf.aoaScale(), ekf.gnssLatency() * 1000.f, Units::rad_to_deg(ekf.yawRateBias()), r.usingMahony() ? "true" : "false");
    g_text = buf;
    return g_text.c_str();
}

} // extern "C"
