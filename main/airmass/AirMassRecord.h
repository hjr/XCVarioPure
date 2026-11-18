#pragma once

// Flight recording of the AirMass estimator inputs as NMEA sentences, so a flight can be replayed on a host
// (tools/airmass_sim/replay) through the same airmass::Pipeline the device runs. Any NMEA logger records them
// (e.g. the navigation computer's NMEA log). Plain C++, the device and the host tools share it. The device leaves
// out the input frames ($PXCVR): its $SENS sentences of the same logging carry the sensor readings, the converters
// (sens2rec.py, web/convert.js) make the frames from them.
//
//   $PXCVC,lever,mass,area,a0,a1,a2,vstall,valid,wresp,vresp,teadj   setup (at start, on change, every 10s)
//   $PXCVS,t,alt,d0,d1,d2,g0,g1,g2,ktas,pitch,lat,kaoa,f0,..,f6,fmt[,lat_sigma,lat_ratio]  estimator start: AHRS down
//                                                                     axis, gyro, warm start; fmt 2: lat is the latency;
//                                                                     absent: an offset to 0.75 s; 3: lat_sigma its
//                                                                     uncertainty (else 0.3 s); 4: lat_ratio the lateral
//                                                                     specific force per g (else the replay's own)
//   $PXCVR,t,fx,fy,fz,gx,gy,gz,ias,tas,alt,lever[,ta,tg,tb,tv,valid]  one poll: optional source times and valid mask
//   $PXCVG,t,vn,ve,sigma,latency,prior,wn0,we0[,fix_utc_ms]           GNSS velocity (arrival, nominal latency, optional source epoch)
//   $PXCVH,t,psi,sigma                                                compass heading fused
//   $PXCVO,t,rx,ry,rz                                                 body axes turned (IMU reference changed), rotation vector [rad]
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

// One sentence, formatted in place without heap allocations: the device queues the buffer as it is. A sentence that
// does not fit (absurd values only) comes out empty, it is dropped.
struct Line {
    static constexpr int SIZE = 224; // the longest sentence ($PXCVS) has about 180 characters
    char buf[SIZE];
    int len; // SIZE: did not fit

    explicit Line(const char* name) : len(static_cast<int>(std::strlen(name))) { std::memcpy(buf, name, len + 1); }
    const char* c_str() const { return buf; }
    int size() const { return len; }
    bool empty() const { return len == 0; }

    // room for n more characters and the terminating zero, else the sentence is lost
    bool fits(int n) {
        if (len + n < SIZE) return true;
        len = SIZE;
        return false;
    }
    // printf at the end; when it does not fit the sentence is lost
    template <typename... Args>
    void printf(const char* fmt, Args... args) {
        if (len >= SIZE) return;
        const int n = std::snprintf(buf + len, SIZE - len, fmt, args...);
        if (n >= 0 && fits(n)) {
            len += n;
        } else {
            len = SIZE;
        }
    }
    void append(const char* text) {
        const int n = static_cast<int>(std::strlen(text));
        if (fits(n)) {
            std::memcpy(buf + len, text, n);
            len += n;
        }
    }
};

// "*hh\r\n" for the sentence in s (starting with '$'), the checksum as NMEA::CheckSum
inline void finish(Line& s) {
    if (!s.fits(5)) {
        s.len = 0;
        s.buf[0] = 0;
        return;
    }
    unsigned crc = 0;
    for (int i = 0; i < s.len; ++i) {
        const char c = s.buf[i];
        if (c != '$' && c != '!') crc ^= static_cast<unsigned char>(c);
    }
    char* p = s.buf + s.len;
    *p++ = '*';
    *p++ = NMEA::hex[(crc >> 4) & 15];
    *p++ = NMEA::hex[crc & 15];
    *p++ = '\r';
    *p++ = '\n';
    *p = 0;
    s.len += 5;
}

inline void appendf(Line& s, const char* fmt, float v) {
    if (!s.fits(1)) return;
    s.buf[s.len++] = ',';
    if (std::isfinite(v)) {
        s.printf(fmt, v);
    }
}
inline void appendu(Line& s, uint32_t v) {
    char digits[10];
    int n = 0;
    do {
        digits[n++] = static_cast<char>('0' + v % 10);
        v /= 10;
    } while (v);
    if (!s.fits(n + 1)) return;
    s.buf[s.len++] = ',';
    while (n) s.buf[s.len++] = digits[--n];
}

inline Line setup(const Setup& c) {
    Line s("$PXCVC");
    appendf(s, "%.4f", c.lever_arm);
    appendf(s, "%.2f", c.mass);
    appendf(s, "%.4f", c.wing_area);
    appendf(s, "%.8g", c.polar.a0);
    appendf(s, "%.8g", c.polar.a1);
    appendf(s, "%.8g", c.polar.a2);
    appendf(s, "%.4f", c.polar.v_stall);
    s.append(c.polar.valid ? ",1" : ",0");
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

inline Line start(const Start& st) {
    Line s("$PXCVS");
    appendu(s, st.t);
    appendf(s, "%.2f", st.alt);
    for (float d : st.down) appendf(s, "%.6f", d);
    for (float g : st.gyro) appendf(s, "%.6f", g);
    appendf(s, "%.5f", st.warm.k_tas);
    appendf(s, "%.6f", st.warm.pitch_ofs);
    appendf(s, "%.4f", st.warm.latency);
    appendf(s, "%.5f", st.warm.k_aoa);
    for (float o : st.warm.flap_ofs) appendf(s, "%.6f", o);
    appendu(s, 4);
    appendf(s, "%.4f", st.warm.latency_sigma);
    appendf(s, "%.4f", st.warm.lat_ratio);
    finish(s);
    return s;
}

// A new recording can join a running estimator. Capture its current calibration/attitude without resetting it;
// this seeds a new replay, rather than restoring the EKF's complete dynamic state and covariance.
inline Start snapshot(Pipeline& pipe, const Frame& next) {
    Start st;
    st.t = next.time_ms;
    st.alt = next.baro_alt;
    for (int i = 0; i < 3; ++i) {
        st.down[i] = pipe.attitude().down()[i];
        st.gyro[i] = next.gyro[i];
    }
    st.warm = pipe.learned();
    return st;
}

inline Line frame(const Frame& f) {
    Line s("$PXCVR");
    appendu(s, f.time_ms);
    for (float v : f.f_body) appendf(s, "%.5f", v);
    for (float v : f.gyro) appendf(s, "%.6f", v);
    appendf(s, "%.3f", f.ias);
    appendf(s, "%.3f", f.tas);
    appendf(s, "%.2f", f.baro_alt);
    appendf(s, "%.3f", f.flap_lever);
    if (f.sample_times) {
        for (uint32_t stamp : f.sample_ms) appendu(s, stamp);
        appendu(s, f.valid);
    }
    finish(s);
    return s;
}

// a GNSS velocity fused, with the wind prior for the alignment
struct Gnss {
    AirMassEKF::GnssVelocity v;
    bool prior = false;
    float wn0 = 0.f, we0 = 0.f;
};

inline Line gnss(const Gnss& g) {
    Line s("$PXCVG");
    appendu(s, g.v.time_ms);
    appendf(s, "%.3f", g.v.vn);
    appendf(s, "%.3f", g.v.ve);
    appendf(s, "%.3f", g.v.sigma_h);
    appendu(s, g.v.latency_ms);
    s.append(g.prior ? ",1" : ",0");
    appendf(s, "%.3f", g.wn0);
    appendf(s, "%.3f", g.we0);
    if (g.v.fix_time_ms != GnssTiming::UNKNOWN) appendu(s, g.v.fix_time_ms);
    finish(s);
    return s;
}

// a compass heading fused
struct Heading {
    uint32_t t = 0;
    float psi = 0.f, sigma = 0.f;
};

inline Line heading(const Heading& h) {
    Line s("$PXCVH");
    appendu(s, h.t);
    appendf(s, "%.5f", h.psi);
    appendf(s, "%.4f", h.sigma);
    finish(s);
    return s;
}

// the body axes turned in flight: v_new = R(rv) v_old (Pipeline::rotateBody)
struct Rotation {
    uint32_t t = 0;
    float rv[3] = {};
};

inline Line rotation(const Rotation& r) {
    Line s("$PXCVO");
    appendu(s, r.t);
    for (float v : r.rv) appendf(s, "%.6f", v);
    finish(s);
    return s;
}

inline Line stop(uint32_t t) {
    Line s("$PXCVX");
    appendu(s, t);
    finish(s);
    return s;
}

// the estimator output $PXCVA (vario up positive, wind direction it blows from [deg], speed [km/h], ...)
inline Line output(const AirMassEKF& ekf) {
    const float wn = ekf.windN(), we = ekf.windE();
    int wdir = static_cast<int>(std::lround(Units::rad_to_deg(std::atan2(-we, -wn))));
    wdir = (wdir % 360 + 360) % 360;
    Line s("");
    s.printf("$PXCVA,%.2f,%d,%.1f,%.2f,%.3f,%.2f,%.0f,%X", ekf.airmassVario(), wdir,
                                Units::mps_to_kmh(std::hypot(wn, we)), ekf.windD(), ekf.kTas(), Units::rad_to_deg(ekf.pitchOffset()),
                                ekf.gnssLatency() * 1000.f, static_cast<unsigned>(ekf.status()));
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
    const double fmt = s.n > 12 + FlapOffsets::N ? f[12 + FlapOffsets::N] : 0.;
    st.warm.latency_sigma = s.n > 13 + FlapOffsets::N && fmt >= 3. && std::isfinite(f[13 + FlapOffsets::N])
                                ? f[13 + FlapOffsets::N] : AirMassEKF::START_LATENCY_SIGMA;
    st.warm.lat_ratio = s.n > 14 + FlapOffsets::N && fmt >= 4. ? f[14 + FlapOffsets::N] : NAN;
    return true;
}

inline bool decode(const Sentence& s, Frame& fr) {
    if (!s.is("PXCVR", 11)) return false;
    if (s.n > 11 && s.n < 15) return false;
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
    fr.sample_times = s.n >= 15;
    fr.valid = Frame::ALL;
    if (fr.sample_times) {
        for (int i = 0; i < 4; ++i) {
            if (!std::isfinite(f[11 + i]) || f[11 + i] < 0 || f[11 + i] > UINT32_MAX) return false;
            fr.sample_ms[i] = static_cast<uint32_t>(f[11 + i]);
        }
        if (s.n >= 16) {
            if (!std::isfinite(f[15]) || f[15] < 0 || f[15] > static_cast<unsigned>(Frame::ALL)) return false;
            fr.valid = static_cast<uint8_t>(f[15]);
        }
    }
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
    g.v.fix_time_ms = s.n < 9 ? GnssTiming::UNKNOWN :
        std::isfinite(f[8]) && f[8] >= 0. && f[8] < GnssTiming::DAY_MS ? static_cast<uint32_t>(f[8]) : GnssTiming::DAY_MS;
    return true;
}

inline bool decode(const Sentence& s, Heading& h) {
    if (!s.is("PXCVH", 3)) return false;
    h.t = static_cast<uint32_t>(s.f[0]);
    h.psi = s.f[1];
    h.sigma = s.f[2];
    return true;
}

inline bool decode(const Sentence& s, Rotation& r) {
    if (!s.is("PXCVO", 4)) return false;
    r.t = static_cast<uint32_t>(s.f[0]);
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(s.f[1 + i]) || std::fabs(s.f[1 + i]) > 3.) return false;
        r.rv[i] = s.f[1 + i];
    }
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
