// Flap offsets must preserve delayed air vectors and their AoA scale sensitivity.
#include "AirMassEKF.h"
#include <cstdio>

struct AirMassFlapHistoryTest {
    static bool check(float scale, float offset, uint32_t time_ms) {
        AirMassEKF e;
        e.reset(1000.f, 1.f, 0.f, AirMassEKF::START_LATENCY, scale);
        for (int k = 1; k <= 40; ++k) {
            AirMassEKF::Input in;
            in.time_ms = k * 100;
            in.tas = 30.f;
            in.aoa = .06f;
            in.pitch = .03f;
            in.kinematic_fresh = in.energy_fresh = in.baro_fresh = false;
            e.update(in);
        }
        AirMassEKF::Delayed before, after, plus, minus;
        if (!e.delayedState(time_ms, before)) return false;
        e.setPitchOffset(offset, .01f);
        if (!e.delayedState(time_ms, after)) return false;
        for (int i = 0; i < 3; ++i)
            if (std::fabs(before.va[i] - after.va[i]) > 1e-5f) return false;
        if (std::fabs(after.aoa - .06f) > 1e-6f) return false;

        const float original_scale = e.aoaScale(), eps = .001f;
        e._x[AirMassEKF::KCL] = original_scale + eps;
        if (!e.delayedState(time_ms, plus)) return false;
        e._x[AirMassEKF::KCL] = original_scale - eps;
        if (!e.delayedState(time_ms, minus)) return false;
        for (int i = 0; i < 3; ++i) {
            const float derivative = (plus.va[i] - minus.va[i]) / (2.f * eps);
            if (std::fabs(derivative - after.dalpha[i] * .06f) > .003f) return false;
        }
        return true;
    }
};

int main() {
    int failures = 0;
    for (float scale : {.8f, 1.f, 1.2f}) {
        for (float offset : {-.1f, .1f}) {
            for (uint32_t time_ms : {3000u, 3050u}) {
                const bool ok = AirMassFlapHistoryTest::check(scale, offset, time_ms);
                std::printf("flap history scale=%.1f offset=%.1f time=%u: %s\n",
                            scale, offset, time_ms, ok ? "PASS" : "FAIL");
                failures += !ok;
            }
        }
    }
    return failures ? 1 : 0;
}
