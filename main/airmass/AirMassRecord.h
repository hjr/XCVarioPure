#pragma once

// Flight recording of the AirMass estimator inputs as NMEA sentences, so a flight can be replayed on a host
// (tools/airmass_sim/replay) through the same airmass::Pipeline the device runs. Any NMEA logger records them
// (e.g. the navigation computer's NMEA log). Plain C++, the device and the host tools share it.
//
//   $PXCVC,lever,mass,area,a0,a1,a2,vstall,valid,wresp,vresp,teadj   setup (at start, on change, every 10s)
//   $PXCVS,t,alt,d0,d1,d2,g0,g1,g2,ktas,pitch,lat,kaoa,f0,..,f6,fmt    estimator start: AHRS down axis, gyro, warm start
//                                                                     fmt 2: lat is the latency; absent: an offset to 0.75 s
//   $PXCVR,t,fx,fy,fz,gx,gy,gz,ias,tas,alt,lever                      one cycle (10Hz): specific force [g], gyro [rad/s]
//   $PXCVG,t,vn,ve,sigma,latency,prior,wn0,we0                        GNSS velocity fused (arrival time, nominal latency)
//   $PXCVH,t,psi,sigma                                                compass heading fused
//   $PXCVX,t                                                          estimator stopped
// t: device milliseconds. An empty field is NAN (no baro, no flap lever).
//
// $PXCVA,vario,wdir,wspeed,wd,k_tas,pitch_ofs,latency,status: the estimator output, see AirMassOutput.cpp. With the
// recording it follows the cycle it belongs to, so the replay can check itself against the device.

#include "AirMassEKF.h"
#include "AirMassPipeline.h"
#include "math/Units.h"
#include "protocol/nmea_util.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace airmass::record {

inline std::string begin(const char* name) {
    std::string s;
    s.reserve(200); // the longest sentence ($PXCVS) has about 190 characters
    s = name;
    return s;
}

// "*hh\r\n" for the sentence in s (starting with '$')
inline void finish(std::string& s) {
    s += '*';
    s += NMEA::CheckSum(s.c_str());
    s += "\r\n";
}

inline void appendf(std::string& s, const char* fmt, float v) {
    s += ',';
    if (std::isfinite(v)) {
        char buf[24];
        std::snprintf(buf, sizeof(buf), fmt, v);
        s += buf;
    }
}
inline void appendu(std::string& s, uint32_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), ",%lu", static_cast<unsigned long>(v));
    s += buf;
}

inline std::string setup(const Setup& c) {
    std::string s = begin("$PXCVC");
    appendf(s, "%.4f", c.lever_arm);
    appendf(s, "%.2f", c.mass);
    appendf(s, "%.4f", c.wing_area);
    appendf(s, "%.8g", c.polar.a0);
    appendf(s, "%.8g", c.polar.a1);
    appendf(s, "%.8g", c.polar.a2);
    appendf(s, "%.4f", c.polar.v_stall);
    s += c.polar.valid ? ",1" : ",0";
    appendf(s, "%.3f", c.wind_resp);
    appendf(s, "%.3f", c.vario_resp);
    appendf(s, "%.3f", c.te_adjust);
    finish(s);
    return s;
}

// the estimator start: time, baro altitude, the device AHRS down axis, the gyro and the warm start values
struct Start {
    uint32_t t = 0;
    float alt = 0.f;
    float down[3] = {0.f, 0.f, 1.f};
    float gyro[3] = {};
    Warm warm;
};

inline std::string start(const Start& st) {
    std::string s = begin("$PXCVS");
    appendu(s, st.t);
    appendf(s, "%.2f", st.alt);
    for (float d : st.down) appendf(s, "%.6f", d);
    for (float g : st.gyro) appendf(s, "%.6f", g);
    appendf(s, "%.5f", st.warm.k_tas);
    appendf(s, "%.6f", st.warm.pitch_ofs);
    appendf(s, "%.4f", st.warm.latency);
    appendf(s, "%.5f", st.warm.k_aoa);
    for (float o : st.warm.flap_ofs) appendf(s, "%.6f", o);
    appendu(s, 2); // format: the absolute latency
    finish(s);
    return s;
}

inline std::string frame(const Frame& f) {
    std::string s = begin("$PXCVR");
    appendu(s, f.time_ms);
    for (float v : f.f_body) appendf(s, "%.5f", v);
    for (float v : f.gyro) appendf(s, "%.6f", v);
    appendf(s, "%.3f", f.ias);
    appendf(s, "%.3f", f.tas);
    appendf(s, "%.2f", f.baro_alt);
    appendf(s, "%.3f", f.flap_lever);
    finish(s);
    return s;
}

// a GNSS velocity fused, with the wind prior for the alignment
struct Gnss {
    AirMassEKF::GnssVelocity v;
    bool prior = false;
    float wn0 = 0.f, we0 = 0.f;
};

inline std::string gnss(const Gnss& g) {
    std::string s = begin("$PXCVG");
    appendu(s, g.v.time_ms);
    appendf(s, "%.3f", g.v.vn);
    appendf(s, "%.3f", g.v.ve);
    appendf(s, "%.3f", g.v.sigma_h);
    appendu(s, g.v.latency_ms);
    s += g.prior ? ",1" : ",0";
    appendf(s, "%.3f", g.wn0);
    appendf(s, "%.3f", g.we0);
    finish(s);
    return s;
}

// a compass heading fused
struct Heading {
    uint32_t t = 0;
    float psi = 0.f, sigma = 0.f;
};

inline std::string heading(const Heading& h) {
    std::string s = begin("$PXCVH");
    appendu(s, h.t);
    appendf(s, "%.5f", h.psi);
    appendf(s, "%.4f", h.sigma);
    finish(s);
    return s;
}

inline std::string stop(uint32_t t) {
    std::string s = begin("$PXCVX");
    appendu(s, t);
    finish(s);
    return s;
}

// the estimator output $PXCVA (vario up positive, wind direction it blows from [deg], speed [km/h], ...)
inline std::string output(const AirMassEKF& ekf) {
    const float wn = ekf.windN(), we = ekf.windE();
    int wdir = static_cast<int>(std::lround(Units::rad_to_deg(std::atan2(-we, -wn))));
    wdir = (wdir % 360 + 360) % 360;
    char str[96];
    std::snprintf(str, sizeof(str), "$PXCVA,%.2f,%d,%.1f,%.2f,%.3f,%.2f,%.0f,%X", ekf.airmassVario(), wdir, Units::mps_to_kmh(std::hypot(wn, we)),
                  ekf.windD(), ekf.kTas(), Units::rad_to_deg(ekf.pitchOffset()), ekf.gnssLatency() * 1000.f, static_cast<unsigned>(ekf.status()));
    std::string s = str;
    finish(s);
    return s;
}

// a parsed sentence: the name without '$' and the fields after it
struct Sentence {
    char name[8] = {};
    int n = 0;
    double f[24] = {}; // NAN for empty fields
    bool is(const char* nm, int min_fields) const { return std::strcmp(name, nm) == 0 && n >= min_fields; }
};

// parses one line, false for other sentences or a bad checksum
inline bool parse(const char* line, Sentence& out) {
    if (line[0] != '$' || std::strncmp(line + 1, "PXCV", 4) != 0) {
        return false;
    }
    const char* star = std::strchr(line, '*');
    if (!star || std::strtoul(star + 1, nullptr, 16) != std::strtoul(NMEA::CheckSum(line).c_str(), nullptr, 16)) {
        return false;
    }
    const char* p = line + 1;
    const char* comma = std::strchr(p, ',');
    if (!comma || comma > star || comma - p >= static_cast<long>(sizeof(out.name))) {
        return false;
    }
    std::memcpy(out.name, p, comma - p);
    out.name[comma - p] = 0;
    out.n = 0;
    p = comma + 1;
    while (p <= star && out.n < 24) {
        const char* end = p;
        while (end < star && *end != ',') ++end;
        if (end == p) {
            out.f[out.n++] = NAN;
        } else if (std::strcmp(out.name, "PXCVA") == 0 && out.n == 7) {
            out.f[out.n++] = static_cast<double>(std::strtoul(p, nullptr, 16)); // status in hex
        } else {
            out.f[out.n++] = std::strtod(p, nullptr);
        }
        if (end >= star) break;
        p = end + 1;
    }
    return true;
}

// the inverses of the writers above, false for another sentence
inline bool decode(const Sentence& s, Setup& c) {
    if (!s.is("PXCVC", 11)) return false;
    const double* f = s.f;
    c.lever_arm = f[0];
    c.mass = f[1];
    c.wing_area = f[2];
    c.polar.a0 = f[3];
    c.polar.a1 = f[4];
    c.polar.a2 = f[5];
    c.polar.v_stall = f[6];
    c.polar.valid = f[7] != 0.;
    c.wind_resp = f[8];
    c.vario_resp = f[9];
    c.te_adjust = f[10];
    return true;
}

inline bool decode(const Sentence& s, Start& st) {
    if (!s.is("PXCVS", 12 + FlapOffsets::N)) return false;
    const double* f = s.f;
    st.t = static_cast<uint32_t>(f[0]);
    st.alt = f[1];
    for (int i = 0; i < 3; ++i) {
        st.down[i] = f[2 + i];
        st.gyro[i] = f[5 + i];
    }
    st.warm.k_tas = f[8];
    st.warm.pitch_ofs = f[9];
    st.warm.latency = s.n > 12 + FlapOffsets::N ? f[10] : f[10] + AirMassEKF::START_LATENCY;
    st.warm.k_aoa = f[11];
    for (int i = 0; i < FlapOffsets::N; ++i) {
        st.warm.flap_ofs[i] = f[12 + i];
    }
    return true;
}

inline bool decode(const Sentence& s, Frame& fr) {
    if (!s.is("PXCVR", 11)) return false;
    const double* f = s.f;
    fr.time_ms = static_cast<uint32_t>(f[0]);
    for (int i = 0; i < 3; ++i) {
        fr.f_body[i] = f[1 + i];
        fr.gyro[i] = f[4 + i];
    }
    fr.ias = f[7];
    fr.tas = f[8];
    fr.baro_alt = f[9];
    fr.flap_lever = f[10];
    return true;
}

inline bool decode(const Sentence& s, Gnss& g) {
    if (!s.is("PXCVG", 8)) return false;
    const double* f = s.f;
    g.v.time_ms = static_cast<uint32_t>(f[0]);
    g.v.vn = f[1];
    g.v.ve = f[2];
    g.v.sigma_h = f[3];
    g.v.latency_ms = static_cast<uint16_t>(f[4]);
    g.prior = f[5] != 0.;
    g.wn0 = f[6];
    g.we0 = f[7];
    return true;
}

inline bool decode(const Sentence& s, Heading& h) {
    if (!s.is("PXCVH", 3)) return false;
    h.t = static_cast<uint32_t>(s.f[0]);
    h.psi = s.f[1];
    h.sigma = s.f[2];
    return true;
}

// $PXCVA: the device's vario and status
struct Output {
    float vario = NAN;
    unsigned status = 0;
};

inline bool decode(const Sentence& s, Output& o) {
    if (!s.is("PXCVA", 8)) return false;
    o.vario = s.f[0];
    o.status = static_cast<unsigned>(s.f[7]);
    return true;
}

} // namespace airmass::record
