// Replay of a recorded flight through the AirMass estimator (the same airmass::Pipeline the device runs).
//
// The log is any NMEA log with the AirMass recording ("NMEA Output" Recording, see main/airmass/AirMassRecord.h),
// e.g. the navigation computer's NMEA log of a flight; other sentences are skipped. Where the log has the device's
// own $PXCVA output, the replay compares itself with it: with the same firmware it reproduces the device.
//
// usage: airmass_replay <log> [--csv file] [--check m/s] [--warm k_tas,pitch_ofs,aoa_scale,latency_ofs] [--no-learn]
//   --csv: per cycle t, air mass vario, wind N/E/D, status, heading, k_tas, pitch offset, latency, averaged wind
//          N/E, roll, pitch, the device's vario where recorded, the estimator's polar sink (down positive), load
//          factor, vertical speed (down), acceleration along the air velocity, vertical acceleration (down)
//   --check: fail when the replayed vario differs from the device's by more than this, or a status differs
//   --warm: start with these learned values instead of the recorded ones (what a new calibration would do)
//   --no-learn: keep the calibration values as started
//   --set name=value: experiment with an estimator setting (AirMassEKF::Config), e.g. --set sigma_pitch=0.05

#include "AirMassPipeline.h"
#include "AirMassRecord.h"
#include "math/Units.h"
#include "stats.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

using Units::rad_to_deg;

namespace {
// estimator settings for experiments (--set), applied after every setup
bool applySettings(AirMassEKF& ekf, const std::vector<std::pair<std::string, float>>& settings) {
    AirMassEKF::Config c = ekf.config();
    const std::pair<const char*, float*> known[] = {
        {"sigma_pitch", &c.sigma_pitch}, {"sigma_energy", &c.sigma_energy}, {"energy_tau", &c.energy_tau}, {"q_wind_v", &c.q_wind_v},
        {"q_wind_h", &c.q_wind_h}, {"q_thb", &c.q_thb}, {"q_kcl", &c.q_kcl}, {"lag_static", &c.lag_static}, {"lag_pitot", &c.lag_pitot},
        {"sigma_air", &c.sigma_air}, {"gust_acc", &c.gust_acc}, {"kinetic_tau", &c.kinetic_tau}, {"r_baro", &c.r_baro},
        {"wind_work", &c.wind_work}, {"gate", &c.gate}, {"energy_load_weight", &c.energy_load_weight},
        {"energy_gust_weight", &c.energy_gust_weight}, {"anchor_tau", &c.anchor_tau},
        {"kcl_min", &c.kcl_min}, {"kcl_max", &c.kcl_max}, {"energy_gate", &c.energy_gate},
        {"wind_work_hp", &c.wind_work_hp}, {"blend_hp", &c.blend_hp}, {"polar_calibrated", &c.polar_calibrated},
    };
    for (const auto& s : settings) {
        bool found = false;
        for (const auto& k : known) {
            if (s.first == k.first) {
                *k.second = s.second;
                found = true;
            }
        }
        if (!found) {
            std::fprintf(stderr, "unknown setting %s\n", s.first.c_str());
            return false;
        }
    }
    ekf.setConfig(c);
    return true;
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <log> [--csv file] [--check m/s] [--warm k_tas,pitch_ofs,aoa_scale,latency_ofs] [--no-learn]\n", argv[0]);
        return 2;
    }
    const char* csv_name = nullptr;
    float check = -1.f;
    bool override_warm = false, learn = true;
    float warm_override[4] = {1.f, 0.f, 1.f, 0.f};
    std::vector<std::pair<std::string, float>> settings;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--no-learn") == 0) {
            learn = false;
        } else if (i + 1 < argc && std::strcmp(argv[i], "--csv") == 0) {
            csv_name = argv[++i];
        } else if (i + 1 < argc && std::strcmp(argv[i], "--check") == 0) {
            check = std::atof(argv[++i]);
        } else if (i + 1 < argc && std::strcmp(argv[i], "--set") == 0) {
            const char* eq = std::strchr(argv[++i], '=');
            if (!eq) {
                std::fprintf(stderr, "--set needs name=value\n");
                return 2;
            }
            settings.emplace_back(std::string(argv[i], eq - argv[i]), std::atof(eq + 1));
        } else if (i + 1 < argc && std::strcmp(argv[i], "--warm") == 0) {
            override_warm = std::sscanf(argv[++i], "%f,%f,%f,%f", &warm_override[0], &warm_override[1], &warm_override[2], &warm_override[3]) == 4;
        } else {
            std::fprintf(stderr, "unknown option %s\n", argv[i]);
            return 2;
        }
    }
    FILE* in = std::fopen(argv[1], "r");
    if (!in) {
        std::fprintf(stderr, "cannot open %s\n", argv[1]);
        return 2;
    }
    FILE* csv = csv_name ? std::fopen(csv_name, "w") : nullptr;
    if (csv_name && !csv) {
        std::fprintf(stderr, "cannot write %s\n", csv_name);
        return 2;
    }
    if (csv) {
        std::fprintf(csv, "t_s,vario,wind_n,wind_e,wind_d,status,heading_deg,k_tas,pitch_ofs_deg,latency_ms,avg_wind_n,avg_wind_e,roll_deg,pitch_deg,device_vario,"
                          "polar_sink,load_factor,vd,acc_long,acc_down,wd_energy,wd_energy_plain\n");
    }

    airmass::Pipeline pipe;
    pipe.setLearning(learn);
    bool running = false, configured = false;
    long lines = 0, frames = 0, gnss = 0, headings = 0, starts = 0, skipped = 0, bad = 0;
    uint32_t t_first = 0, t_last = 0;
    Stats vario_diff;
    long status_diff = 0, outputs = 0;

    // a cycle ends with the next one (or a restart, the stop, the end of the log): its CSV row then has the cycle's
    // GNSS and compass fusion, the averaged wind and the device's output
    bool open = false;
    float device_vario = NAN;
    auto endCycle = [&]() {
        if (!open) {
            return;
        }
        open = false;
        pipe.endCycle();
        if (!csv) {
            return;
        }
        const AirMassEKF& ekf = pipe.ekf();
        const AirMassEKF::Input& i = pipe.input();
        std::fprintf(csv, "%.1f,%.3f,%.3f,%.3f,%.3f,%u,%.1f,%.4f,%.3f,%.0f,%.3f,%.3f,%.2f,%.2f,", i.time_ms * 1e-3, ekf.airmassVario(), ekf.windN(),
                     ekf.windE(), ekf.windD(), static_cast<unsigned>(ekf.status()), rad_to_deg(ekf.heading()), ekf.kTas(),
                     rad_to_deg(ekf.pitchOffset()), ekf.gnssLatency() * 1000.f, pipe.avgWind().windN(), pipe.avgWind().windE(), rad_to_deg(i.roll),
                     rad_to_deg(i.pitch));
        if (std::isfinite(device_vario)) {
            std::fprintf(csv, "%.2f", device_vario);
        }
        // the estimator's inputs and vertical speed, for diagnostics
        std::fprintf(csv, ",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", i.polar_sink, i.load_factor, ekf.vDown(), i.acc_long, i.acc_down, ekf.energyWd(),
                     ekf.energyWdPlain());
    };

    char line[512];
    namespace rec = airmass::record;
    rec::Sentence s;
    airmass::Setup setup;
    rec::Start st;
    airmass::Frame fr;
    rec::Gnss g;
    rec::Heading h;
    rec::Output out;
    while (std::fgets(line, sizeof(line), in)) {
        lines++;
        if (!rec::parse(line, s)) {
            if (std::strncmp(line, "$PXCV", 5) == 0) {
                bad++; // ours, but damaged (checksum)
            }
            continue;
        }
        if (rec::decode(s, setup)) {
            pipe.configure(setup);
            if (!applySettings(pipe.ekf(), settings)) {
                return 2;
            }
            configured = true;
        } else if (rec::decode(s, st)) {
            endCycle();
            if (override_warm) {
                st.warm.k_tas = warm_override[0];
                st.warm.pitch_ofs = warm_override[1];
                st.warm.k_aoa = warm_override[2];
                st.warm.lat_ofs = warm_override[3];
                std::fill(std::begin(st.warm.flap_ofs), std::end(st.warm.flap_ofs), airmass::FlapOffsets::UNKNOWN); // a new calibration
            }
            pipe.start(st.t, st.alt, st.down, st.gyro, st.warm);
            running = configured;
            starts++;
            if (!t_first) {
                t_first = st.t;
            }
        } else if (!running) {
            skipped += !std::strcmp(s.name, "PXCVR"); // the log started in flight: wait for the next start
        } else if (rec::decode(s, fr)) {
            endCycle();
            pipe.step(fr);
            open = true;
            device_vario = NAN;
            frames++;
            t_last = fr.time_ms;
        } else if (rec::decode(s, g)) {
            pipe.gnss(g.v, g.prior, g.wn0, g.we0);
            gnss++;
        } else if (rec::decode(s, h)) {
            pipe.heading(h.psi, h.sigma);
            headings++;
        } else if (!std::strcmp(s.name, "PXCVX")) {
            endCycle();
            running = false;
        } else if (rec::decode(s, out)) {
            // the device's output for the cycle before
            const AirMassEKF& ekf = pipe.ekf();
            vario_diff.add(ekf.airmassVario() - out.vario);
            status_diff += out.status != ekf.status();
            outputs++;
            device_vario = out.vario;
        }
    }
    endCycle();
    std::fclose(in);
    if (csv) {
        std::fclose(csv);
    }

    const AirMassEKF& ekf = pipe.ekf();
    std::printf("%ld lines: %ld cycles (%.0f s), %ld GNSS, %ld compass, %ld starts; %ld cycles before the first start, %ld damaged\n", lines, frames,
                (t_last - t_first) * 1e-3, gnss, headings, starts, skipped, bad);
    std::printf("learned: k_tas %.3f, pitch ofs %.2f°, aoa scale %.3f, gnss latency %.0f ms, gyro z bias %.3f°/s\n", ekf.kTas(),
                rad_to_deg(ekf.pitchOffset()), ekf.aoaScale(), ekf.gnssLatency() * 1000.f, rad_to_deg(ekf.yawRateBias()));
    bool ok = frames > 0;
    if (outputs) {
        std::printf("against the device's $PXCVA (%ld): vario rms %.3f m/s, max %.3f m/s, status differs %ld times\n", outputs, vario_diff.rms(),
                    vario_diff.maxabs, status_diff);
        if (check >= 0.f) {
            ok = ok && vario_diff.maxabs <= check && status_diff == 0;
        }
    } else if (check >= 0.f) {
        std::printf("no $PXCVA in the log, nothing to check against\n");
        ok = false;
    }
    std::printf("%s\n", ok ? "PASS" : "FAILED");
    return ok ? 0 : 1;
}
