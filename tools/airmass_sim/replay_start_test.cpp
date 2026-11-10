// Mid-flight recording joins, including a banked start without a logged initialization or attitude.
#include "replay_core.h"
#include "test_util.h"

#include <cmath>
#include <cstdio>

namespace {
namespace rec = airmass::record;
using airmass::replay::Replayer;

airmass::Setup setup() {
    airmass::Setup s;
    s.mass = 400.f;
    s.wing_area = 10.5f;
    s.polar = {-1.f, 0.f, 0.f, 20.f, true};
    return s;
}

struct Turn {
    airmass::Frame f;
    float down[3];
    float pitch;
    explicit Turn(float bank) {
        f.time_ms = 300000;
        f.ias = 30.f;
        f.tas = 36.f;
        f.baro_alt = 1000.f;
        float n = 1.f / std::cos(bank), alpha = 0.f;
        for (int i = 0; i < 10; ++i) {
            alpha = airmass::aoaModel(n, f.ias, setup().mass, setup().wing_area);
            pitch = std::atan(std::cos(bank) * std::tan(alpha));
            n = 1.f / (std::sin(alpha) * std::sin(pitch) + std::cos(alpha) * std::cos(bank) * std::cos(pitch));
        }
        down[0] = -std::sin(pitch);
        down[1] = std::sin(bank) * std::cos(pitch);
        down[2] = std::cos(bank) * std::cos(pitch);
        const float rate = airmass::G0 * std::tan(bank) / f.tas;
        for (int i = 0; i < 3; ++i) f.gyro[i] = rate * down[i];
        const float vx = f.tas * std::cos(alpha), vz = f.tas * std::sin(alpha);
        f.f_body[0] = f.gyro[1] * vz / airmass::G0 - down[0];
        f.f_body[1] = (f.gyro[2] * vx - f.gyro[0] * vz) / airmass::G0 - down[1];
        f.f_body[2] = -f.gyro[1] * vx / airmass::G0 - down[2];
    }
};

std::string attitude(float roll, float pitch) {
    rec::Line s("$PXCV,0,1,0,1,1,15,1013,1000,0");
    s.printf(",%.6f,%.6f", Units::rad_to_deg(roll), Units::rad_to_deg(pitch));
    rec::finish(s);
    return s.c_str();
}

void snapshot() {
    airmass::Pipeline live;
    live.configure(setup());
    Turn turn(0.5f);
    airmass::Warm warm;
    warm.k_tas = 1.03f;
    warm.pitch_ofs = 0.02f;
    warm.latency = 0.45f;
    warm.k_aoa = 0.95f;
    warm.flap_ofs[2] = 0.04f;
    live.start(turn.f.time_ms - 100, turn.f.baro_alt, turn.down, turn.f.gyro, warm);
    for (int i = 0; i < 100; ++i, turn.f.time_ms += 100) live.step(turn.f);
    live.endCycle();
    const auto status = live.ekf().status();
    const float vario = live.ekf().airmassVario(), heading = live.ekf().heading();
    const auto snap = rec::snapshot(live, turn.f);
    check(live.ekf().status() == status && live.ekf().airmassVario() == vario && live.ekf().heading() == heading,
          "starting a recording leaves the live estimator untouched");
    check(snap.t == turn.f.time_ms && near(snap.alt, turn.f.baro_alt) && near(snap.down[1], live.attitude().down()[1]) &&
          near(snap.gyro[2], turn.f.gyro[2]), "snapshot carries current attitude, time, air data and gyro");
    check(near(snap.warm.k_tas, live.ekf().kTas()) && near(snap.warm.pitch_ofs, live.ekf().pitchOffset()) &&
          near(snap.warm.latency, live.ekf().gnssLatency()) && near(snap.warm.k_aoa, live.ekf().aoaScale()) &&
          near(snap.warm.flap_ofs[2], 0.04f), "snapshot carries learned calibration and flap offsets");
    Replayer replay;
    replay.line(rec::setup(setup()).c_str());
    replay.line(rec::start(snap).c_str());
    replay.line(rec::frame(turn.f).c_str());
    replay.finish();
    check(replay.frames == 1 && replay.recovered_starts == 0 && near(replay.pipeline().ekf().kTas(), snap.warm.k_tas, 0.001f),
          "mid-flight recording snapshot initializes replay with the live calibration");
}

void recovery(bool mahony, bool logged) {
    constexpr float bank = 0.6f;
    Turn turn(bank);
    Replayer replay;
    replay.setMahony(mahony);
    replay.line(rec::setup(setup()).c_str());
    if (logged) replay.line(attitude(bank, turn.pitch).c_str());
    replay.line(rec::frame(turn.f).c_str());
    check(replay.starts == 1 && replay.recovered_starts == 1 && replay.estimated_attitude_starts == !logged,
          "missing initialization recovers with the selected attitude source/filter");
    check(near(replay.pipeline().input().roll, bank, 0.01f) && near(replay.pipeline().input().pitch, turn.pitch, 0.01f),
          "banked recovery separates gravity from turn acceleration");
    check(!(replay.pipeline().ekf().status() & AirMassEKF::ST_VARIO_VALID) && !replay.pipeline().ekf().isAligned(),
          "recovered output still waits for vertical settling and heading alignment");
    for (int i = 0; i < 100; ++i) {
        turn.f.time_ms += 100;
        replay.line(rec::frame(turn.f).c_str());
    }
    replay.finish();
    check(replay.frames == 101 && replay.pipeline().ekf().covarianceHealthy() &&
          (replay.pipeline().ekf().status() & AirMassEKF::ST_VARIO_VALID), "recovered flight reaches valid vario after settling");
    replay.line(rec::stop(turn.f.time_ms).c_str());
    turn.f.time_ms += 100;
    replay.line(rec::frame(turn.f).c_str());
    check(replay.frames == 101 && replay.skipped == 1, "explicit stop prevents automatic airborne restart");
    rec::Start next;
    next.t = turn.f.time_ms;
    next.alt = turn.f.baro_alt;
    next.warm.k_tas = 1.07f;
    replay.line(rec::start(next).c_str());
    replay.line(rec::frame(turn.f).c_str());
    check(replay.frames == 102 && replay.starts == 2 && near(replay.pipeline().ekf().kTas(), 1.07f, 0.001f),
          "a later explicit start replaces recovered state and calibration");
}

void prerequisites() {
    Turn turn(0.f);
    Replayer r;
    r.line(rec::frame(turn.f).c_str());
    check(r.frames == 0 && r.skipped == 1, "missing configuration waits without inventing a polar");
    r.line(rec::setup(setup()).c_str());
    const auto good = turn.f;
    turn.f.baro_alt = NAN;
    r.line(rec::frame(turn.f).c_str());
    turn.f = good;
    turn.f.ias = 15.f;
    r.line(rec::frame(turn.f).c_str());
    turn.f = good;
    turn.f.f_body[2] = -3.f;
    r.line(rec::frame(turn.f).c_str());
    turn.f = good;
    turn.f.gyro[0] = NAN;
    r.line(rec::frame(turn.f).c_str());
    check(r.frames == 0 && r.skipped == 5, "invalid data, low IAS and implausible gravity cannot seed replay");
    r.line(rec::frame(good).c_str());
    check(r.frames == 1 && r.recovered_starts == 1, "usable data recovers after unusable frames");

    Replayer pending;
    rec::Start start;
    start.t = good.time_ms;
    start.alt = good.baro_alt;
    start.warm.k_tas = 1.06f;
    pending.line(rec::start(start).c_str());
    pending.line(rec::frame(good).c_str());
    pending.line(rec::setup(setup()).c_str());
    pending.line(rec::frame(good).c_str());
    check(pending.frames == 1 && pending.starts == 1 && pending.recovered_starts == 0 &&
          near(pending.pipeline().ekf().kTas(), 1.06f, 0.001f), "start before configuration is retained until configuration arrives");

    Replayer stale;
    stale.line(attitude(0.7f, 0.1f).c_str());
    stale.line(rec::frame(good).c_str());
    stale.line(rec::setup(setup()).c_str());
    turn.f = good;
    turn.f.time_ms += 3000;
    stale.line(rec::frame(turn.f).c_str());
    check(stale.estimated_attitude_starts == 1 && near(stale.pipeline().input().roll, 0.f, 0.01f),
          "stale logged attitude is replaced by a current motion reference");

    Replayer overridden;
    overridden.override_warm = true;
    overridden.warm[0] = 1.04f;
    overridden.warm[3] = 0.35f;
    overridden.line(rec::setup(setup()).c_str());
    overridden.line(rec::frame(good).c_str());
    check(near(overridden.pipeline().ekf().kTas(), 1.04f) && near(overridden.pipeline().ekf().gnssLatency(), 0.35f),
          "recovered starts honor explicit calibration overrides");
}
} // namespace

int main() {
    snapshot();
    for (bool mahony : {false, true}) for (bool logged : {false, true}) recovery(mahony, logged);
    prerequisites();
    return failures ? 1 : 0;
}
