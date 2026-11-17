// Replay of a recorded flight through the AirMass estimator (the same airmass::Pipeline the device runs).
//
// The log is any NMEA log with the AirMass recording (System > Logging "All Sensor Data", see main/airmass/AirMassRecord.h),
// e.g. the navigation computer's NMEA log of a flight; other sentences are skipped. Where the log has the device's
// own $PXCVA output, the replay compares itself with it: with the same firmware it reproduces the device.
//
// usage: airmass_replay <log> [--csv file] [--check m/s] [--warm k_tas,pitch_ofs,aoa_scale,latency] [--no-learn] [--mahony]
//   --csv: per cycle t, air mass vario, wind N/E/D, status, heading, k_tas, pitch offset, latency, averaged wind, ..., gust detector
//          N/E, roll, pitch, the device's vario where recorded, the estimator's polar sink (down positive), load
//          factor, vertical speed (down), acceleration along the air velocity, vertical acceleration (down)
//   --check: fail when the replayed vario differs from the device's by more than this, or a status differs
//   --warm: start with these learned values instead of the recorded ones (what a new calibration would do)
//   --no-learn: keep the calibration values as started
//   --set name=value: experiment with an estimator setting (AirMassEKF::Config), e.g. --set sigma_pitch=0.05

#include "math/Units.h"
#include "replay_core.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using Units::rad_to_deg;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <log> [--csv file] [--check m/s] [--warm k_tas,pitch_ofs,aoa_scale,latency] [--no-learn] [--mahony]\n", argv[0]);
        return 2;
    }
    const char* csv_name = nullptr;
    float check = -1.f;
    bool override_warm = false, learn = true, mahony = false;
    float warm_override[4] = {1.f, 0.f, 1.f, 0.75f};
    airmass::replay::Settings settings;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--no-learn") == 0) {
            learn = false;
        } else if (std::strcmp(argv[i], "--mahony") == 0) {
            mahony = true;
        } else if (i + 1 < argc && std::strcmp(argv[i], "--csv") == 0) {
            csv_name = argv[++i];
        } else if (i + 1 < argc && std::strcmp(argv[i], "--check") == 0) {
            check = std::atof(argv[++i]);
        } else if (i + 1 < argc && std::strcmp(argv[i], "--set") == 0) {
            std::string error;
            if (!airmass::replay::parseSettings(argv[++i], settings, &error)) {
                std::fprintf(stderr, "--set: %s\n", error.c_str());
                return 2;
            }
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
                          "polar_sink,load_factor,vd,acc_long,acc_down,wd_energy,wd_energy_plain,gnss_age_ms,gnss_received_age_ms,gnss_rejects,inertial_trust,gust,gust_event\n");
    }

    airmass::replay::Replayer r;
    r.setLearning(learn);
    r.setMahony(mahony);
    r.settings = settings;
    r.override_warm = override_warm;
    std::copy(std::begin(warm_override), std::end(warm_override), r.warm);
    long gusts_up = 0, gusts_down = 0;
    r.on_cycle = [csv, &gusts_up, &gusts_down](const airmass::Pipeline& pipe, float device_vario) {
        if (pipe.gust().event()) {
            ++(pipe.gust().event() > 0 ? gusts_up : gusts_down);
        }
        if (!csv) {
            return;
        }
        const AirMassEKF& ekf = pipe.ekf();
        const AirMassEKF::Input& i = pipe.input();
        std::fprintf(csv, "%.3f,%.3f,%.3f,%.3f,%.3f,%u,%.1f,%.4f,%.3f,%.0f,%.3f,%.3f,%.2f,%.2f,", i.time_ms * 1e-3, ekf.airmassVario(), ekf.windN(),
                     ekf.windE(), ekf.windD(), static_cast<unsigned>(ekf.status()), rad_to_deg(ekf.heading()), ekf.kTas(),
                     rad_to_deg(ekf.pitchOffset()), ekf.gnssLatency() * 1000.f, pipe.avgWind().windN(), pipe.avgWind().windE(), rad_to_deg(i.roll),
                     rad_to_deg(i.pitch));
        if (std::isfinite(device_vario)) {
            std::fprintf(csv, "%.2f", device_vario);
        }
        // the estimator's inputs and vertical speed, for diagnostics
        std::fprintf(csv, ",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%.3f,%.2f,%d\n", i.polar_sink, i.load_factor, ekf.vDown(), i.acc_long, i.acc_down,
                     ekf.energyWd(), ekf.energyWdPlain(), ekf.gnssAgeMs(), ekf.gnssReceivedAgeMs(), ekf.rejectedGnss(), ekf.inertialTrust(),
                     pipe.gust().value(), pipe.gust().event());
    };

    char line[512];
    while (std::fgets(line, sizeof(line), in) && r.line(line)) {
    }
    r.finish();
    std::fclose(in);
    if (csv) {
        std::fclose(csv);
    }
    if (!r.error.empty()) {
        std::fprintf(stderr, "%s\n", r.error.c_str());
        return 2;
    }

    const AirMassEKF& ekf = r.pipeline().ekf();
    std::printf("%ld lines: %ld cycles (%.0f s), %ld GNSS, %ld compass, %ld starts; %ld skipped cycles, %ld damaged\n", r.lines, r.frames,
                r.seconds, r.gnss, r.headings, r.starts, r.skipped, r.bad);
    if (r.recovered_starts) {
        std::printf("recovered %ld missing starts (%ld with estimated attitude); initialization is approximate, allow settling\n",
                    r.recovered_starts, r.estimated_attitude_starts);
    }
    std::printf("learned: k_tas %.3f, pitch ofs %.2f°, aoa scale %.3f, gnss latency %.0f ms, gyro z bias %.3f°/s\n", ekf.kTas(),
                rad_to_deg(ekf.pitchOffset()), ekf.aoaScale(), ekf.gnssLatency() * 1000.f, rad_to_deg(ekf.yawRateBias()));
    std::printf("gusts: %ld head wind increase, %ld head wind drop\n", gusts_up, gusts_down);
    bool ok = r.frames > 0;
    if (r.outputs) {
        std::printf("against the device's $PXCVA (%ld): vario rms %.3f m/s, max %.3f m/s, status differs %ld times\n", r.outputs, r.vario_diff.rms(),
                    r.vario_diff.maxabs, r.status_diff);
        if (check >= 0.f) {
            ok = ok && r.vario_diff.maxabs <= check && r.status_diff == 0;
        }
    } else if (check >= 0.f) {
        std::printf("no $PXCVA in the log, nothing to check against\n");
        ok = false;
    }
    std::printf("%s\n", ok ? "PASS" : "FAILED");
    return ok ? 0 : 1;
}
