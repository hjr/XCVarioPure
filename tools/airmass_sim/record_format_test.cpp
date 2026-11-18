// The in place recording formatter (AirMassRecord.h) against the former std::string one: byte identical sentences,
// and a sentence too long for the device queue item is dropped, as the device did.
#include "AirMassPipeline.h"
#include "AirMassRecord.h"

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>

namespace legacy {
using namespace airmass;

// the writers as they were, with a heap string per sentence and NMEA::CheckSum
std::string begin(const char* name) { return name; }
void finish(std::string& s) {
    s += '*';
    s += NMEA::CheckSum(s.c_str());
    s += "\r\n";
}
void appendf(std::string& s, const char* fmt, float v) {
    s += ',';
    if (std::isfinite(v)) {
        char buf[64]; // was 24, it cut numbers beyond 1e16 (which now come out complete or the sentence is dropped)
        std::snprintf(buf, sizeof(buf), fmt, v);
        s += buf;
    }
}
void appendu(std::string& s, uint32_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), ",%lu", static_cast<unsigned long>(v));
    s += buf;
}

std::string setup(const Setup& c) {
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

std::string start(const record::Start& st) {
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
    appendu(s, 2);
    finish(s);
    return s;
}

std::string frame(const Frame& f) {
    std::string s = begin("$PXCVR");
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

std::string gnss(const record::Gnss& g) {
    std::string s = begin("$PXCVG");
    appendu(s, g.v.time_ms);
    appendf(s, "%.3f", g.v.vn);
    appendf(s, "%.3f", g.v.ve);
    appendf(s, "%.3f", g.v.sigma_h);
    appendu(s, g.v.latency_ms);
    s += g.prior ? ",1" : ",0";
    appendf(s, "%.3f", g.wn0);
    appendf(s, "%.3f", g.we0);
    if (g.v.fix_time_ms != GnssTiming::UNKNOWN) appendu(s, g.v.fix_time_ms);
    finish(s);
    return s;
}

std::string heading(const record::Heading& h) {
    std::string s = begin("$PXCVH");
    appendu(s, h.t);
    appendf(s, "%.5f", h.psi);
    appendf(s, "%.4f", h.sigma);
    finish(s);
    return s;
}

std::string stop(uint32_t t) {
    std::string s = begin("$PXCVX");
    appendu(s, t);
    finish(s);
    return s;
}

std::string output(const AirMassEKF& ekf) {
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
} // namespace legacy

namespace {
using namespace airmass;

int failures = 0, same = 0, dropped = 0;

// identical bytes, or dropped where the device dropped the former sentence (too long for the queue item)
void compare(const std::string& old, const record::Line& now) {
    const bool fits = old.size() < static_cast<size_t>(record::Line::SIZE);
    const bool ok = fits ? old == now.c_str() && static_cast<size_t>(now.size()) == old.size() : now.empty();
    if (!ok) {
        if (++failures <= 10) std::printf("  FAIL\n    was %s    now %s\n", old.c_str(), now.c_str());
        return;
    }
    ++(fits ? same : dropped);
}

std::mt19937 rng(451);

const float SPECIAL[] = {0.f, -0.f, 1e-7f, -1e-7f, 0.5f, -0.00049999f, 0.0000049999f, 9.80665f, -123.456f, 1234.5678f, 65535.5f,
                         1e6f, -1e7f, 3.4e10f, -2.5e15f, 1e30f, FLT_MAX, -FLT_MAX, FLT_MIN, NAN, INFINITY, -INFINITY};

// mostly plausible values of any sign and magnitude, sometimes a special one
float value() {
    std::uniform_int_distribution<int> pick(0, 9);
    if (pick(rng) == 0) {
        std::uniform_int_distribution<int> i(0, sizeof(SPECIAL) / sizeof(SPECIAL[0]) - 1);
        return SPECIAL[i(rng)];
    }
    std::uniform_real_distribution<float> exp10(-6.f, 6.f), sign(-1.f, 1.f);
    return std::copysign(std::pow(10.f, exp10(rng)), sign(rng));
}

uint32_t stamp() {
    static const uint32_t SPECIAL_U[] = {0u, 1u, 9u, 10u, 99u, 100u, 999999999u, 1000000000u, 4294967295u};
    std::uniform_int_distribution<int> pick(0, 3);
    if (pick(rng) == 0) {
        std::uniform_int_distribution<int> i(0, sizeof(SPECIAL_U) / sizeof(SPECIAL_U[0]) - 1);
        return SPECIAL_U[i(rng)];
    }
    return std::uniform_int_distribution<uint32_t>()(rng);
}

Frame randomFrame() {
    Frame f;
    f.time_ms = stamp();
    for (float& v : f.f_body) v = value();
    for (float& v : f.gyro) v = value();
    f.ias = value();
    f.tas = value();
    f.baro_alt = value();
    f.flap_lever = value();
    f.sample_times = rng() & 1;
    for (uint32_t& t : f.sample_ms) t = stamp();
    f.valid = rng() & Frame::ALL;
    return f;
}

void sentences() {
    // a typical device frame, as recorded in flight
    Frame typical;
    typical.time_ms = 3456789;
    const float fb[3] = {0.01234f, -0.00321f, -1.02345f}, gy[3] = {-0.001234f, 0.0456f, -0.123456f};
    for (int i = 0; i < 3; ++i) {
        typical.f_body[i] = fb[i];
        typical.gyro[i] = gy[i];
    }
    typical.ias = 27.345f;
    typical.tas = 30.123f;
    typical.baro_alt = 1234.56f;
    typical.sample_times = true;
    typical.sample_ms[0] = typical.sample_ms[1] = 3456780;
    typical.sample_ms[2] = 3456760;
    typical.sample_ms[3] = 3456770;
    compare(legacy::frame(typical), record::frame(typical));
    typical.flap_lever = 2.5f;
    typical.baro_alt = NAN;
    typical.valid = Frame::ALL & ~Frame::BARO;
    compare(legacy::frame(typical), record::frame(typical));

    for (int i = 0; i < 20000; ++i) {
        const Frame f = randomFrame();
        compare(legacy::frame(f), record::frame(f));

        Setup c;
        c.lever_arm = value();
        c.mass = value();
        c.wing_area = value();
        c.polar = {value(), value(), value(), value(), (rng() & 1) != 0};
        c.wind_resp = value();
        c.vario_resp = value();
        c.te_adjust = value();
        compare(legacy::setup(c), record::setup(c));

        record::Start st;
        st.t = stamp();
        st.alt = value();
        for (int k = 0; k < 3; ++k) {
            st.down[k] = value();
            st.gyro[k] = value();
        }
        st.warm.k_tas = value();
        st.warm.pitch_ofs = value();
        st.warm.latency = value();
        st.warm.k_aoa = value();
        for (float& o : st.warm.flap_ofs) o = (rng() & 1) ? FlapOffsets::UNKNOWN : value();
        compare(legacy::start(st), record::start(st));

        record::Gnss g;
        g.v.time_ms = stamp();
        g.v.vn = value();
        g.v.ve = value();
        g.v.sigma_h = value();
        g.v.latency_ms = static_cast<uint16_t>(stamp());
        g.v.fix_time_ms = (rng() & 1) ? GnssTiming::UNKNOWN : stamp();
        g.prior = rng() & 1;
        g.wn0 = value();
        g.we0 = value();
        compare(legacy::gnss(g), record::gnss(g));

        const record::Heading h{stamp(), value(), value()};
        compare(legacy::heading(h), record::heading(h));
        const uint32_t t = stamp();
        compare(legacy::stop(t), record::stop(t));
    }
    compare(legacy::start(record::Start()), record::start(record::Start()));
    compare(legacy::setup(Setup()), record::setup(Setup()));
}

// the estimator output along a flight, with the wind turning through all directions
void outputs() {
    AirMassEKF idle;
    compare(legacy::output(idle), record::output(idle));
    Pipeline pipe;
    Setup setup;
    setup.mass = 460.f;
    setup.wing_area = 11.81f;
    setup.polar = {-0.8f, 0.f, 0.f, 21.636f, true};
    pipe.configure(setup);
    const float down[3] = {0.f, 0.f, 1.f}, gyro[3] = {};
    pipe.start(1000, 1000.f, down, gyro, {});
    Frame f;
    f.ias = 30.f;
    f.tas = 35.f;
    ExternalAttitude att;
    for (f.time_ms = 1050; f.time_ms <= 120000; f.time_ms += 50) {
        f.baro_alt = 1000.f + 0.5f * std::sin(f.time_ms * 0.0003f);
        pipe.step(f, &att);
        if (f.time_ms % 1000 == 0) {
            const float course = f.time_ms * 0.0001f;
            pipe.gnss({f.time_ms, 750, 35.f * std::cos(course) - 4.f, 35.f * std::sin(course) + 3.f, 0.5f}, true, -4.f, 3.f);
        }
        pipe.endCycle();
        compare(legacy::output(pipe.ekf()), record::output(pipe.ekf()));
    }
}
// $PXCVO back from its sentence, and the rotation vector back from its matrix
void rotations() {
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-0.6f, 0.6f);
    for (int i = 0; i < 1000; ++i) {
        record::Rotation r{static_cast<uint32_t>(rng()), {u(rng), u(rng), u(rng)}};
        record::Sentence sen;
        record::Rotation back;
        const bool ok = record::parse(record::rotation(r).c_str(), sen) && record::decode(sen, back) && back.t == r.t;
        float m[3][3], rv[3];
        rotationMatrix(r.rv, m);
        rotationVector(m, rv);
        float err = 0.f;
        for (int k = 0; k < 3; ++k) err = std::fmax(err, std::fmax(std::fabs(back.rv[k] - r.rv[k]), std::fabs(rv[k] - r.rv[k])));
        if (!ok || err > 1e-5f) {
            std::printf("rotation %d: %.6f %.6f %.6f off by %g\n", i, r.rv[0], r.rv[1], r.rv[2], err);
            failures++;
        }
    }
}
} // namespace

int main() {
    sentences();
    outputs();
    rotations();
    std::printf("record format: %d identical, %d too long and dropped, %d different\n", same, dropped, failures);
    const bool ok = failures == 0 && same > 100000 && dropped > 0;
    std::printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
