// The gust detector (AirMassGust.h) on synthetic airspeed/inertia disagreements at 20 Hz.
#include "AirMassGust.h"
#include "test_util.h"

#include <functional>

using airmass::GustDetector;


struct Run {
    int up = 0, down = 0;
    float first_s = -1.f;
};

// acc(t): the disagreement in m/s², slow: the estimator's low pass of it, active(t); the airspeed follows the
// disagreement by tas_share (a gust: all of it, an error of the inertial acceleration: none)
Run run(float seconds, const std::function<float(float)>& acc, float slow = 0.f,
        const std::function<bool(float)>& active = [](float) { return true; }, float tas_share = 1.f, float dt = 0.05f) {
    GustDetector g;
    Run r;
    float tas = 30.f;
    for (float t = 0.f; t < seconds; t += dt) {
        const float a = acc(t);
        if (std::isfinite(a)) tas += tas_share * (a - slow) * dt;
        g.update(a, slow, tas, dt, active(t));
        if (g.event()) {
            g.event() > 0 ? ++r.up : ++r.down;
            if (r.first_s < 0.f) r.first_s = t;
        }
    }
    return r;
}

// a head wind change of dv m/s over ramp_s starting at t0
std::function<float(float)> ramp(float t0, float ramp_s, float dv) {
    return [=](float t) { return t >= t0 && t < t0 + ramp_s ? dv / ramp_s : 0.f; };
}

int main() {
    Run r = run(30.f, ramp(10.f, 2.f, 6.f));
    check(r.up == 1 && r.down == 0, "6 m/s head wind increase within 2 s: one event");
    check(r.first_s > 10.f && r.first_s < 12.f, "detected during the ramp");

    r = run(30.f, ramp(10.f, 2.f, -6.f));
    check(r.up == 0 && r.down == 1, "6 m/s head wind drop: one event of the other sign");

    r = run(30.f, ramp(10.f, 2.f, 4.f));
    check(r.up + r.down == 0, "4 m/s is below the threshold");

    r = run(60.f, ramp(10.f, 20.f, 8.f));
    check(r.up + r.down == 0, "8 m/s over 20 s is no gust");

    for (float dt : {0.05f, 0.099f, 0.25f}) {
        r = run(60.f, ramp(10.f, 20.f, 18.f), 0.f, [](float) { return true; }, 1.f, dt);
        check(r.up + r.down == 0, "0.9 m/s² over 20 s (4.5 m/s within 5 s) at any cycle time: none");
    }

    r = run(60.f, [](float t) { return 1.5f * std::sin(6.2831853f * t / 4.f); });
    check(r.up + r.down == 0, "oscillation of +-1 m/s: none");

    r = run(60.f, [](float) { return 0.3f; }, 0.3f);
    check(r.up + r.down == 0, "a steady disagreement (attitude error) the estimator already tracks: none");

    r = run(30.f, [](float t) { return ramp(10.f, 2.f, 6.f)(t) + ramp(13.f, 2.f, 6.f)(t); });
    check(r.up == 1, "a second gust within the hold time: no second event");

    r = run(40.f, [](float t) { return ramp(10.f, 2.f, 6.f)(t) + ramp(20.f, 2.f, 6.f)(t); });
    check(r.up == 2, "a second gust after the hold time: a second event");

    r = run(30.f, ramp(10.f, 2.f, 6.f), 0.f, [](float t) { return t < 9.f || t > 11.f; });
    check(r.up + r.down == 0, "inactive (turning, a gap) during the ramp: none");

    r = run(30.f, ramp(10.f, 2.f, 6.f), 0.f, [](float) { return true; }, 0.f);
    check(r.up + r.down == 0, "disagreement without an airspeed change (inertial acceleration error): none");

    r = run(30.f, ramp(10.f, 2.f, 6.f), 0.f, [](float) { return true; }, 0.6f);
    check(r.up == 1, "the pilot holding part of the airspeed: still a gust");

    r = run(30.f, [](float t) { return t > 10.f && t < 10.5f ? NAN : 0.f; });
    check(r.up + r.down == 0, "no disagreement (not flying): none");

    return failures;
}
