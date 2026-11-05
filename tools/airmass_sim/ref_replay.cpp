// The GNSS/INS reference (ins_reference.h, after the Larus design) on a replayed XCVario $SENS flight: the sensors of
// that installation (1 Hz Flarm GNSS velocity and altitude, the vertical speed from the altitude differences), the device
// AHRS roll and pitch, and the heading of AirMass once aligned (the log has no compass, the reference needs a heading to
// rotate the IMU into north/east, so its wind is not independent of AirMass). It runs at 100 Hz, the 10 Hz samples
// interpolated.
//
// usage: ref_replay <name>.aux.csv <name>.out.csv <out.csv>   (sens2rec.py --aux, airmass_replay --csv)
//   out.csv: t_s, vario (TE, as shown), wind_n, wind_e (as shown), avg_wind_n, avg_wind_e

#include "ins_reference.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr float D2R = 0.017453293f;

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string f;
    while (std::getline(ss, f, ',')) {
        out.push_back(f);
    }
    return out;
}

float num(const std::string& s) { return s.empty() ? NAN : std::strtof(s.c_str(), nullptr); }

struct Sample {
    long t_ds = 0;    // deciseconds of the device clock
    float utc = NAN;  // s, of the last fix
    float gs = NAN, gt = NAN, galt = NAN;
    float roll = 0.f, pitch = 0.f, tas = 0.f, f[3] = {0.f, 0.f, -1.f};
    float heading = NAN;
};

} // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "usage: %s <aux.csv> <out.csv of airmass_replay> <reference.csv>\n", argv[0]);
        return 2;
    }
    // AirMass heading by decisecond
    std::map<long, float> heading;
    {
        FILE* in = std::fopen(argv[2], "r");
        if (!in) {
            std::fprintf(stderr, "cannot open %s\n", argv[2]);
            return 2;
        }
        char buf[512];
        bool header = true;
        while (std::fgets(buf, sizeof(buf), in)) {
            const auto f = split(buf);
            if (header) {
                header = false;
            } else if (f.size() > 6 && (std::atoi(f[5].c_str()) & 8)) { // only once AirMass's heading is aligned
                heading[std::lround(num(f[0]) * 10.f)] = num(f[6]) * D2R;
            }
        }
        std::fclose(in);
    }
    std::vector<Sample> s;
    {
        FILE* in = std::fopen(argv[1], "r");
        if (!in) {
            std::fprintf(stderr, "cannot open %s\n", argv[1]);
            return 2;
        }
        char buf[512];
        bool header = true;
        while (std::fgets(buf, sizeof(buf), in)) {
            const auto f = split(buf);
            if (header) {
                header = false;
                continue;
            }
            if (f.size() < 19 || f[15] != "1") {
                continue; // not flying
            }
            Sample x;
            x.t_ds = std::lround(std::atol(f[0].c_str()) / 100.0);
            auto h = heading.find(x.t_ds);
            if (h == heading.end()) {
                continue;
            }
            x.heading = h->second;
            x.utc = num(f[1]);
            x.galt = num(f[4]);
            x.gs = num(f[5]);
            x.gt = num(f[6]) * D2R;
            x.roll = num(f[8]) * D2R;
            x.pitch = num(f[9]) * D2R;
            x.tas = num(f[11]);
            for (int i = 0; i < 3; ++i) {
                x.f[i] = num(f[16 + i]);
            }
            s.push_back(x);
        }
        std::fclose(in);
    }
    FILE* out = std::fopen(argv[3], "w");
    if (!out) {
        std::fprintf(stderr, "cannot write %s\n", argv[3]);
        return 2;
    }
    std::fprintf(out, "t_s,vario,wind_n,wind_e,avg_wind_n,avg_wind_e\n");

    ref::InsReference ins;
    float gv[3] = {0.f, 0.f, 0.f}, galt = NAN, prev_alt = NAN, prev_utc = NAN;
    size_t written = 0;
    for (size_t k = 0; k + 1 < s.size(); ++k) {
        const Sample& a = s[k];
        const Sample& b = s[k + 1];
        if (b.t_ds - a.t_ds > 5) {
            ins = ref::InsReference(); // gap (a restart of the estimator): start over
            prev_alt = prev_utc = NAN;
            continue;
        }
        // a new fix: horizontal velocity from speed and track, vertical from the altitude difference
        if (std::isfinite(a.utc) && a.utc != prev_utc && std::isfinite(a.gs) && std::isfinite(a.gt) && std::isfinite(a.galt)) {
            gv[0] = a.gs * std::cos(a.gt);
            gv[1] = a.gs * std::sin(a.gt);
            if (std::isfinite(prev_alt) && std::isfinite(prev_utc) && a.utc - prev_utc > 0.5f && a.utc - prev_utc < 3.f) {
                gv[2] = -(a.galt - prev_alt) / (a.utc - prev_utc);
            }
            galt = a.galt;
            prev_alt = a.galt;
            prev_utc = a.utc;
        }
        if (!std::isfinite(galt)) {
            continue;
        }
        for (int j = 0; j < 10; ++j) {
            const float u = j / 10.f;
            auto lerp = [u](float x, float y) { return x + (y - x) * u; };
            const float roll = lerp(a.roll, b.roll), pitch = lerp(a.pitch, b.pitch);
            const float dpsi = std::remainder(b.heading - a.heading, 2.f * float(M_PI));
            const float psi = a.heading + dpsi * u;
            const float tas = lerp(a.tas, b.tas);
            float fb[3];
            for (int i = 0; i < 3; ++i) {
                fb[i] = lerp(a.f[i], b.f[i]) * ref::G;
            }
            // body to NED (yaw, pitch, roll)
            const float cr = std::cos(roll), sr = std::sin(roll), cp = std::cos(pitch), sp = std::sin(pitch);
            const float cy = std::cos(psi), sy = std::sin(psi);
            const float R[3][3] = {{cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy},
                                   {cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy},
                                   {-sp, sr * cp, cr * cp}};
            float acc[3];
            for (int i = 0; i < 3; ++i) {
                acc[i] = R[i][0] * fb[0] + R[i][1] * fb[1] + R[i][2] * fb[2];
            }
            const float xb[3] = {R[0][0], R[1][0], R[2][0]};
            ins.update(gv, galt, acc, xb, tas, std::remainder(psi, 2.f * float(M_PI)), dpsi / 0.1f);
        }
        std::fprintf(out, "%.1f,%.3f,%.3f,%.3f,%.3f,%.3f\n", a.t_ds / 10.0, ins.vario(), ins.windN(), ins.windE(), ins.avgWindN(),
                     ins.avgWindE());
        written++;
    }
    std::fclose(out);
    std::printf("%s: %zu cycles\n", argv[3], written);
    return written ? 0 : 1;
}
