// Mixed sensor rates, prediction-only cycles and elapsed-time guarantees for the target's 20Hz poll.
#include "replay_core.h"

#include <cmath>
#include <cstdio>

namespace {
int failures = 0;
void check(bool ok, const char* name) {
    std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
    failures += !ok;
}
bool near(float a, float b, float tol = 1e-5f) { return std::isfinite(a) && std::fabs(a - b) <= tol; }
airmass::Frame frame(uint32_t t) {
    airmass::Frame f;
    f.time_ms = t;
    f.ias = f.tas = 30.f;
    f.baro_alt = 1000.f;
    f.sample_times = true;
    for (auto& stamp : f.sample_ms) stamp = t;
    return f;
}
void start(airmass::Pipeline& p) {
    p.configure({});
    const float d[3] = {0.f, 0.f, 1.f}, gyro[3] = {};
    p.start(0, 1000.f, d, gyro, {});
}

void clocks() {
    airmass::SampleClock c;
    check(c.observe(0, 0, true), "zero is a valid first source timestamp");
    check(!c.observe(50, 0, true) && c.age(50) == 50, "a new poll does not refresh the source clock");
    check(!c.observe(50, 51, true) && !c.observe(50, 25, false), "future and invalid samples do not advance freshness");
    check(c.observe(100, 100, true) && c.interval_ms == 100, "a new source timestamp advances its own clock");
    check(!c.observe(150, 50, true), "backward source timestamps are ignored");
    c = {};
    check(c.observe(UINT32_MAX - 49, UINT32_MAX - 49, true) && c.observe(50, 50, true) && c.interval_ms == 100,
          "source clocks work through millis rollover");
    for (uint32_t period : {200u, 500u, 1000u, 5000u, 10000u, 300000u}) {
        int counts[2] = {};
        for (int j = 0; j < 2; ++j) {
            airmass::Periodic timer;
            for (uint32_t t = 0; t <= 600000; t += j ? 50 : 100) counts[j] += timer.due(t, period);
        }
        check(counts[0] == counts[1], "output/compass/setup/save schedules have the same period at 10 and 20Hz");
    }
    airmass::Periodic timer;
    check(timer.due(UINT32_MAX - 99, 200) && !timer.due(0, 200) && timer.due(100, 200), "periodic work survives rollover");
    check(timer.due(10000, 200) && !timer.due(10000, 200), "missed timer periods do not produce a catch-up burst");
}

void freshMeasurements() {
    airmass::Pipeline p;
    start(p);
    auto f = frame(100);
    p.step(f);
    const float alt_sigma = p.ekf().sigma(AirMassEKF::ALT), wind_sigma = p.ekf().sigma(AirMassEKF::WD);
    f.time_ms = 150; // no source changed
    f.baro_alt = 2000.f;
    f.ias = f.tas = 90.f;
    f.gyro[0] = 2.f;
    f.f_body[1] = 5.f;
    p.step(f);
    check(!p.input().baro_fresh && !p.input().kinematic_fresh && !p.input().energy_fresh,
          "held timestamps produce a prediction-only cycle");
    check(near(p.input().tas, 30.f) && near(p.input().roll, 0.f), "changed values with held stamps cannot leak into prediction");
    check(p.ekf().sigma(AirMassEKF::ALT) > alt_sigma && p.ekf().sigma(AirMassEKF::WD) > wind_sigma,
          "prediction grows covariance instead of gaining confidence in repeated measurements");
    const float before_baro = p.ekf().sigma(AirMassEKF::ALT);
    f = frame(200);
    f.sample_ms[0] = f.sample_ms[1] = f.sample_ms[3] = 100;
    f.baro_alt = 1001.f;
    p.step(f);
    check(p.input().baro_fresh && !p.input().kinematic_fresh && !p.input().energy_fresh && p.ekf().sigma(AirMassEKF::ALT) < before_baro,
          "a new barometer corrects independently of held IMU, pitot and GNSS");
    f = frame(250);
    f.sample_ms[0] = f.sample_ms[1] = 100;
    f.sample_ms[2] = 200;
    p.step(f);
    check(!p.input().baro_fresh && p.input().kinematic_fresh && p.input().energy_fresh,
          "new airspeed permits aerodynamic corrections without repeating the barometer");
    f = frame(300);
    f.valid = 0;
    f.baro_alt = f.tas = f.gyro[0] = f.f_body[2] = NAN;
    p.step(f);
    check(p.ekf().covarianceHealthy() && std::isfinite(p.ekf().airmassVario()) && !(p.ekf().status() & AirMassEKF::ST_VARIO_VALID),
          "invalid sensor readouts preserve finite prediction and suppress vario validity");
}

void mixedRates() {
    airmass::Pipeline p;
    start(p);
    int baro = 0, kinetic = 0, energy = 0;
    for (uint32_t t = 50; t <= 6000; t += 50) {
        auto f = frame(t);
        f.sample_ms[0] = f.sample_ms[3] = t / 100 * 100;
        f.sample_ms[2] = t / 200 * 200;
        p.step(f);
        baro += p.input().baro_fresh;
        kinetic += p.input().kinematic_fresh;
        energy += p.input().energy_fresh;
        if (t == 2500 || t == 4950) check(!(p.ekf().status() & AirMassEKF::ST_VARIO_VALID), "20Hz cannot shorten the five-second settling period");
        if (t == 5000) check(p.ekf().status() & AirMassEKF::ST_VARIO_VALID, "five seconds of supported data settles at 20Hz");
    }
    check(baro == 31 && energy == 61 && kinetic == 120, "5Hz barometer, 10Hz pitot/accel and 20Hz gyro correct independently");
    auto f = frame(6000);
    for (uint32_t t = 6050; t <= 6500; t += 50) { f.time_ms = t; p.step(f); }
    check(!p.input().continuous && p.ekf().inertialTrust() == 0.f && !(p.ekf().status() & AirMassEKF::ST_VARIO_VALID),
          "polling held data cannot hide an IMU/air-data outage");
    for (uint32_t t = 6550; t <= 11550; t += 50) p.step(frame(t));
    check(p.ekf().status() & AirMassEKF::ST_VARIO_VALID, "fresh data restores validity after another five-second settling period");
}

void jitter() {
    airmass::Pipeline p;
    start(p);
    airmass::ExternalAttitude attitude;
    attitude.yaw_rate = 0.1f;
    uint32_t now = 0;
    for (int i = 0; i < 200; ++i) {
        now += i % 2 ? 60 : 40;
        auto f = frame(now);
        f.gyro[2] = 0.1f;
        p.step(f, &attitude);
        if (now == 4900) check(!(p.ekf().status() & AirMassEKF::ST_VARIO_VALID), "scheduler jitter cannot shorten five-second settling");
    }
    std::printf("jitter heading %.6f, expected 1.000000 rad after %u ms\n", p.ekf().heading(), now);
    check(near(p.ekf().heading(), 1.f, 1e-4f), "40/60ms polling integrates actual elapsed time");
    airmass::replay::Replayer replay;
    replay.setMahony(true);
    replay.line(airmass::record::setup(p.setup()).c_str());
    replay.line(airmass::record::start({}).c_str());
    now = 0;
    for (int i = 0; i < 200; ++i) {
        now += i % 2 ? 60 : 40;
        auto f = frame(now);
        f.valid = airmass::Frame::GYRO;
        f.gyro[0] = 0.1f;
        replay.line(airmass::record::frame(f).c_str());
    }
    check(near(replay.pipeline().input().roll, 1.f, 1e-4f), "Mahony propagation uses actual time through scheduler jitter");
}

void invalidStartup() {
    for (bool mahony : {false, true}) {
        airmass::replay::Replayer replay;
        replay.setMahony(mahony);
        replay.line(airmass::record::setup({}).c_str());
        airmass::record::Start st;
        st.alt = st.gyro[0] = NAN;
        replay.line(airmass::record::start(st).c_str());
        auto f = frame(50);
        f.valid = 0;
        f.baro_alt = f.ias = f.tas = f.gyro[0] = f.f_body[2] = NAN;
        replay.line(airmass::record::frame(f).c_str());
        bool finite = std::isfinite(replay.pipeline().input().roll) && std::isfinite(replay.pipeline().ekf().airmassVario());
        for (uint32_t t = 100; t <= 10000; t += 50) replay.line(airmass::record::frame(frame(t)).c_str());
        finite = finite && std::isfinite(replay.pipeline().input().roll) && std::isfinite(replay.pipeline().ekf().airmassVario());
        check(finite && (replay.pipeline().ekf().status() & AirMassEKF::ST_VARIO_VALID),
              mahony ? "Mahony recovers from invalid initial gyro and missing first readings" : "default filter recovers from invalid initial readings");
    }
}

void history() {
    for (uint32_t dt : {50u, 100u}) for (uint32_t age : {6200u, 6700u}) {
        airmass::Pipeline p;
        start(p);
        for (uint32_t t = dt; t <= 20000; t += dt) p.step(frame(t));
        p.gnss({20000 - age + 750, 750, 30.f, 0.f, 0.5f}, true, 0.f, 0.f);
        check(p.ekf().isAligned() == (age == 6200), "GNSS delayed-history support stays 6.4 seconds at both 10 and 20Hz");
    }
}

void longSourceOutage() {
    airmass::Pipeline p;
    start(p);
    for (uint32_t t = 50; t <= 20000; t += 50) {
        auto f = frame(t);
        f.gyro[0] = 0.02f;
        p.step(f);
    }
    p.gnss({20000, 750, 30.f, 0.f, 0.5f}, true, 0.f, 0.f);
    check(p.ekf().isAligned(), "source-outage fixture starts with aligned GNSS");
    float supported_roll = 0.f;
    for (uint32_t t = 20050; t <= 31500; t += 50) {
        auto f = frame(t);
        f.gyro[0] = 0.02f;
        f.sample_ms[0] = f.sample_ms[1] = 20000;
        p.step(f);
        if (t == 20300) supported_roll = p.input().roll;
    }
    const float ktas = p.ekf().kTas();
    auto resumed = frame(31550);
    resumed.gyro[0] = 0.02f;
    p.step(resumed);
    check(!p.ekf().isAligned() && !(p.ekf().status() & AirMassEKF::ST_VARIO_VALID) &&
          near(p.ekf().kTas(), ktas) && near(p.input().roll, supported_roll, 0.01f),
          "long source outage resets dynamics and restores supported attitude while polling continued");
}

void recording() {
    namespace rec = airmass::record;
    airmass::Pipeline live;
    start(live);
    airmass::replay::Replayer replay;
    replay.line(rec::setup(live.setup()).c_str());
    rec::Start st;
    st.alt = 1000.f;
    replay.line(rec::start(st).c_str());
    for (uint32_t t = 50; t <= 12000; t += 50) {
        auto f = frame(t);
        for (auto& stamp : f.sample_ms) stamp = t / 100 * 100;
        f.baro_alt = 1000.f - f.sample_ms[2] * 0.001f;
        rec::Sentence parsed;
        airmass::Frame decoded;
        const auto line = rec::frame(f);
        if (!rec::parse(line.c_str(), parsed) || !rec::decode(parsed, decoded) || !decoded.sample_times ||
            decoded.sample_ms[2] != f.sample_ms[2] || decoded.valid != f.valid) {
            check(false, "new frame metadata round trip"); return;
        }
        live.step(f);
        replay.line(line.c_str());
        if (t % 200 == 0) replay.line(rec::output(live.ekf()).c_str());
    }
    replay.finish();
    check(replay.frames == 240 && near(replay.seconds, 12.f) && replay.vario_diff.maxabs < 0.02f && replay.status_diff == 0,
          "recording/replay reproduces 20Hz predictions and 10Hz measurements with actual elapsed duration");
    auto f = frame(UINT32_MAX);
    const auto line = rec::frame(f);
    check(line.size() < 224, "extended input frame fits the device NMEA queue");
    f.sample_times = false;
    rec::Sentence s;
    airmass::Frame decoded;
    decoded.sample_times = true;
    decoded.valid = 0;
    rec::parse(rec::frame(f).c_str(), s);
    check(rec::decode(s, decoded) && !decoded.sample_times && decoded.valid == airmass::Frame::ALL,
          "legacy frames remain fresh and do not inherit newer optional metadata");
    f = frame(100);
    rec::parse(rec::frame(f).c_str(), s);
    bool rejected = true;
    for (int n = 12; n < 15; ++n) {
        s.n = n;
        rejected = rejected && !rec::decode(s, decoded);
    }
    check(rejected, "incomplete source metadata cannot masquerade as fresh legacy measurements");
}

void predictionReplay() {
    namespace rec = airmass::record;
    for (bool mahony : {false, true}) for (bool legacy : {false, true}) {
        airmass::replay::Replayer sparse, dense;
        sparse.setPredictionPeriod(50);
        for (auto* replay : {&sparse, &dense}) {
            replay->setMahony(mahony);
            replay->line(rec::setup({}).c_str());
            rec::Start st;
            st.alt = 1000.f;
            replay->line(rec::start(st).c_str());
        }
        int ticks = 0, barometers = 0, energies = 0;
        bool cadence = true, equal = true;
        sparse.on_cycle = [&](const airmass::Pipeline& p, float) {
            cadence = cadence && p.input().time_ms == static_cast<uint32_t>(ticks * 50);
            ++ticks;
            barometers += p.input().baro_fresh;
            energies += p.input().energy_fresh;
        };
        airmass::Frame held;
        for (uint32_t t = 0; t <= 12000; t += 50) {
            if (t % 100 == 0) {
                held = frame(t);
                held.baro_alt -= t * 0.0005f;
                held.ias = held.tas = 30.f + std::sin(t * 0.001f);
                held.gyro[2] = 0.02f;
                auto source = held;
                source.sample_times = !legacy;
                sparse.line(rec::frame(source).c_str());
            }
            auto f = held;
            f.time_ms = t;
            dense.line(rec::frame(f).c_str());
            if (t % 1000 == 0 && t >= 1000) {
                rec::Gnss g;
                g.v = {t, 750, 30.f, 0.f, 0.5f};
                const auto line = rec::gnss(g);
                sparse.line(line.c_str());
                dense.line(line.c_str());
            }
            if (t % 100 == 0) {
                const auto& a = sparse.pipeline().ekf();
                const auto& b = dense.pipeline().ekf();
                equal = equal && near(a.airmassVario(), b.airmassVario()) && near(a.heading(), b.heading()) && a.status() == b.status();
                for (int i = 0; i < AirMassEKF::NX; ++i) equal = equal && near(a.sigma(i), b.sigma(i));
            }
        }
        sparse.finish();
        dense.finish();
        check(cadence && ticks == 241 && barometers == 121 && energies == 121 && sparse.gnss == 12,
              "web cadence adds 50ms predictions without repeating sensor or GNSS corrections");
        check(equal && sparse.frames == dense.frames && near(sparse.seconds, dense.seconds),
              "upsampled legacy/new logs match a recorded 20Hz flight with either attitude filter");
        sparse.line(rec::frame(frame(23000)).c_str());
        sparse.finish();
        check(sparse.frames == 242 && !(sparse.pipeline().ekf().status() & AirMassEKF::ST_VARIO_VALID),
              "a long outage resets instead of inventing prediction history");
        rec::Start st;
        st.t = 30000;
        st.alt = 1000.f;
        sparse.line(rec::start(st).c_str());
        sparse.line(rec::frame(frame(30100)).c_str());
        sparse.finish();
        check(sparse.frames == 243, "a new flight waits for its first actual sensor frame");
    }
}
} // namespace
int main() {
    clocks();
    freshMeasurements();
    mixedRates();
    jitter();
    invalidStartup();
    history();
    longSourceOutage();
    recording();
    predictionReplay();
    return failures ? 1 : 0;
}
