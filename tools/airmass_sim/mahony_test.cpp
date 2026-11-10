// Physical tilt/turn cases and replay selection/restart checks for the shared device AHRS.
#include "replay_core.h"
#include "test_util.h"

#include <cmath>
#include <cstdio>

namespace {
float roll(const Quaternion& q) {
    const vector_f d = q.rotateInv({0.f, 0.f, 1.f});
    return std::atan2(d.y, d.z);
}
}

int main() {
    constexpr float bank = 0.3f, dt = 0.1f;
    {
        Quaternion q = Quaternion::fromDownVector({0.f, std::sin(bank), std::cos(bank)});
        float omega = 0.f;
        int8_t calibrating = 0;
        for (int i = 0; i < 3000; ++i) MahonyAHRS::update(q, omega, calibrating, {}, {0.f, 0.f, -1.f}, {}, 0.f, dt);
        expect("stationary tilt converges towards gravity", roll(q), 0.f, 1e-4f);
        expect("stationary pitch stays level", MahonyAHRS::pitchRad(q), 0.f, 1e-6f);
    }
    {
        constexpr float turnBank = 30.f * Units::rad_per_deg, tas = 30.f;
        const float rate = Units::g0 * std::tan(turnBank) / tas;
        const vector_f gyro{0.f, rate * std::sin(turnBank), rate * std::cos(turnBank)};
        Quaternion q = Quaternion::fromDownVector({0.f, std::sin(turnBank), std::cos(turnBank)});
        float omega = 0.f;
        int8_t calibrating = 0;
        for (int i = 0; i < 1800; ++i) {
            MahonyAHRS::update(q, omega, calibrating, {}, {0.f, 0.f, -1.f / std::cos(turnBank)}, gyro, tas, dt);
        }
        expect("coordinated turn keeps the bank, despite tilted apparent gravity", roll(q), turnBank, 0.002f);
        expect("coordinated turn keeps pitch", MahonyAHRS::pitchRad(q), 0.f, 0.002f);
    }
    {
        Quaternion q = Quaternion::fromDownVector({0.f, std::sin(bank), std::cos(bank)});
        float omega = 0.f;
        int8_t calibrating = 0;
        for (int i = 0; i < 100; ++i) MahonyAHRS::update(q, omega, calibrating, {}, {0.f, 0.f, -1.5f}, {}, 30.f, dt);
        expect("dynamic acceleration gates out a false tilt correction", roll(q), bank, 1e-5f);
        const Quaternion before = q;
        const bool valid = MahonyAHRS::update(q, omega, calibrating, {}, {}, {1.f, 0.f, 0.f}, 30.f, dt);
        expect("invalid acceleration skips the update", valid ? 1.f : 0.f, 0.f, 0.f);
        expect("invalid acceleration preserves attitude", roll(q), roll(before), 0.f);
    }
    {
        namespace rec = airmass::record;
        airmass::Setup setup;
        setup.mass = 400.f;
        setup.wing_area = 10.5f;
        rec::Start start;
        start.t = 1000;
        airmass::replay::Replayer normal, mahony;
        mahony.setMahony(true);
        normal.line(rec::setup(setup).c_str());
        mahony.line(rec::setup(setup).c_str());
        normal.line(rec::start(start).c_str());
        mahony.line(rec::start(start).c_str());
        airmass::Frame f;
        f.ias = f.tas = 30.f;
        f.baro_alt = 0.f;
        // A persistent longitudinal acceleration: the two attitude references should give different tilt.
        f.f_body[0] = 0.15f;
        for (int i = 1; i <= 100; ++i) {
            f.time_ms = start.t + i * 100;
            const auto line = rec::frame(f);
            normal.line(line.c_str());
            mahony.line(line.c_str());
        }
        normal.finish();
        mahony.finish();
        expect("default replay keeps the air-data attitude", normal.usingMahony(), 0.f, 0.f);
        expect("selected replay uses Mahony", mahony.usingMahony(), 1.f, 0.f);
        expect("selection changes estimator attitude", std::fabs(normal.pipeline().input().pitch - mahony.pipeline().input().pitch) > 0.01f, 1.f, 0.f);
        expect("selected replay output stays finite", std::isfinite(mahony.pipeline().ekf().airmassVario()), 1.f, 0.f);
        // A new flight must reset quaternion and fast-calibration state instead of inheriting the last flight's tilt.
        start.t = 20000;
        mahony.line(rec::start(start).c_str());
        f.time_ms = 20100;
        f.f_body[0] = 0.f;
        mahony.line(rec::frame(f).c_str());
        mahony.finish();
        expect("new flight resets Mahony attitude", mahony.pipeline().input().pitch, 0.f, 1e-6f);
        expect("new flight reset keeps all frames", mahony.frames, 101.f, 0.f);
    }
    return failures ? 1 : 0;
}
