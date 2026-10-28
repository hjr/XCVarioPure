// Unit checks of the raw measurement conversion (main/airmass/AirMassInput.h):
// frame conventions, signs and units against hand calculated flight states.

#include "AirMassInput.h"
#include "AirMassAttitude.h"
#include "AirMassFlaps.h"

#include <cmath>
#include <cstdio>

namespace {

int failures = 0;

void expect(const char* what, float got, float want, float tol) {
    const bool ok = std::fabs(got - want) <= tol;
    std::printf("  %-48s %9.4f (expected %9.4f) %s\n", what, got, want, ok ? "ok" : "FAIL");
    if (!ok) {
        failures++;
    }
}

constexpr float D2R = Units::rad_per_deg;
constexpr float G = airmass::G0;

// fresh calibration: airspeed and AoA scale 1, no pitch offset
AirMassEKF fresh() {
    AirMassEKF ekf;
    ekf.reset(0.f);
    return ekf;
}
const AirMassEKF EKF = fresh();

void fill(const airmass::Raw& r, AirMassEKF::Input& in, const AirMassEKF& ekf = EKF) {
    airmass::fillInput(r, airmass::airData(r, ekf), in);
}

// a glider of 400kg on 10.5m², IMU 1m ahead of the CG
airmass::Raw glider(float ias) {
    airmass::Raw r;
    r.ias = ias;
    r.tas = ias * 1.1f;
    r.mass = 400.f;
    r.wing_area = 10.5f;
    r.lever_arm = 1.f;
    r.polar_sink_ias = -0.7f;
    return r;
}

// body coordinates of the nav down axis for a roll/pitch attitude
void downAxis(float roll, float pitch, float d[3]) {
    d[0] = -std::sin(pitch);
    d[1] = std::sin(roll) * std::cos(pitch);
    d[2] = std::cos(roll) * std::cos(pitch);
}

} // namespace

int main() {
    AirMassEKF::Input in;

    std::printf("steady glide, 6° nose down (gravity only)\n");
    {
        airmass::Raw r = glider(25.f);
        downAxis(0.f, -6.f * D2R, r.down_body);
        for (int i = 0; i < 3; ++i) r.f_body[i] = -r.down_body[i];
        fill(r, in);
        expect("acc_down [m/s²]", in.acc_down, 0.f, 1e-4f);
        expect("acc_long [m/s²]", in.acc_long, 0.f, 1e-4f);
        // lift: the specific force perpendicular to the air velocity, which is AoA off the body x axis
        expect("lift load factor [g]", in.load_factor,
               r.f_body[0] * std::sin(in.aoa) - r.f_body[2] * std::cos(in.aoa), 1e-3f);
        expect("slip sigma [rad]", in.slip_sigma, 0.f, 1e-6f);
    }

    std::printf("coordinated 30° bank turn\n");
    {
        airmass::Raw r = glider(25.f);
        const float roll = 30.f * D2R;
        downAxis(roll, 0.f, r.down_body);
        // lift of 1/cos(roll) perpendicular to the air velocity: the body z axis reads it with cos(alpha)
        const float n = 1.f / std::cos(roll);
        const float a = airmass::aoaModel(n, r.ias, r.mass, r.wing_area);
        r.f_body[0] = n * std::sin(a);
        r.f_body[1] = 0.f;
        r.f_body[2] = -n * std::cos(a);
        // level turn with the nose up by alpha
        downAxis(roll, a, r.down_body);
        fill(r, in);
        expect("lift load factor [g]", in.load_factor, n, 2e-3f);
        expect("angle of attack [rad]", in.aoa, a, 2e-4f);
        expect("body z specific force reads low [g]", -r.f_body[2], n * std::cos(a), 1e-6f);
    }

    std::printf("level, accelerating upwards with 2 m/s² and forward with 1 m/s²\n");
    {
        airmass::Raw r = glider(25.f);
        r.f_body[0] = 1.f / G;
        r.f_body[2] = -1.f - 2.f / G;
        fill(r, in);
        expect("acc_down [m/s²] (down positive)", in.acc_down, -2.f, 1e-4f);
        // along the air velocity, which is the angle of attack off the body x axis
        expect("acc along the air velocity [m/s²]", in.acc_long, std::cos(in.aoa) - 2.f * std::sin(in.aoa), 1e-4f);
    }

    std::printf("pull-up: pitch acceleration 2 rad/s², pitch rate 0.3 rad/s at the CG level flight\n");
    {
        airmass::Raw r = glider(25.f);
        const float q = 0.3f, qdot = 2.f;
        r.gyro[1] = q;
        r.gyro_dot[1] = qdot;
        // the IMU 1m ahead senses the tangential (up) and centripetal (backwards) acceleration
        r.f_body[0] = -r.lever_arm * q * q / G;
        r.f_body[2] = -1.f - r.lever_arm * qdot / G;
        fill(r, in);
        expect("acc_down at the CG [m/s²]", in.acc_down, 0.f, 1e-4f);
        expect("acc_long at the CG [m/s²]", in.acc_long, 0.f, 1e-4f);
        expect("lift load factor at the CG [g] (1g along body z)", in.load_factor, std::cos(in.aoa), 1e-3f);
    }

    std::printf("coordinated 40° bank turn with thermal flaps: fuselage 6° below the AoA model\n");
    {
        airmass::Raw r = glider(25.f);
        AirMassEKF flaps = fresh();
        flaps.setPitchOffset(6.f * D2R, 0.01f);
        const float roll = 40.f * D2R;
        // level turn: the air velocity u = (cos a, 0, sin a) is horizontal, the lift of n g perpendicular to it
        // holds the weight; so the turn's acceleration has nothing along u (the zero lift line is 6° off u)
        float n = 1.f / std::cos(roll), a = 0.f, pitch = 0.f;
        for (int i = 0; i < 3; ++i) {
            a = flaps.alphaOf(airmass::aoaModel(n, r.ias, r.mass, r.wing_area));
            pitch = std::atan(std::cos(roll) * std::tan(a));
            n = 1.f / (std::sin(a) * std::sin(pitch) + std::cos(a) * std::cos(roll) * std::cos(pitch));
        }
        downAxis(roll, pitch, r.down_body);
        r.f_body[0] = n * std::sin(a);
        r.f_body[1] = 0.f;
        r.f_body[2] = -n * std::cos(a);
        fill(r, in, flaps);
        expect("lift load factor [g]", in.load_factor, n, 2e-3f);
        expect("acc along the air velocity [m/s²]", in.acc_long, 0.f, 1e-3f);
        expect("AoA model, without the offset [rad]", in.aoa, a + flaps.pitchOffset(), 2e-4f);
    }

    std::printf("air data model at 25 m/s IAS, 1g\n");
    {
        airmass::Raw r = glider(25.f);
        fill(r, in);
        const float cl = 2.f * in.load_factor * 400.f * G / (airmass::RHO0 * 10.5f * 25.f * 25.f);
        expect("angle of attack from the lift coefficient [rad]", in.aoa, cl / airmass::CL_ALPHA, 1e-5f);
        expect("polar sink, positive down, TAS scaled [m/s]", in.polar_sink, 0.7f * 1.1f, 1e-5f);
        expect("tas [m/s]", in.tas, 27.5f, 1e-5f);
        r.polar_sink_ias = 0.f; // below stall the polar returns 0
        fill(r, in);
        expect("no polar below stall (NaN -> 1)", std::isnan(in.polar_sink) ? 1.f : 0.f, 1.f, 0.f);
    }

    std::printf("sideslip from the lateral force\n");
    {
        airmass::Raw r = glider(25.f);
        r.f_body[1] = -0.05f;
        fill(r, in);
        const float qs = 0.5f * airmass::RHO0 * 25.f * 25.f * 10.5f * airmass::CY_BETA;
        expect("slip sigma [rad]", in.slip_sigma, 400.f * G * 0.05f / qs, 1e-5f);
        r.f_body[1] = 0.5f;
        fill(r, in);
        expect("slip sigma clamped [rad]", in.slip_sigma, airmass::SLIP_MAX, 1e-6f);
        r.ias = 5.f;
        fill(r, in);
        expect("no slip estimate on ground [rad]", in.slip_sigma, 0.f, 1e-6f);
    }

    std::printf("attitude through a 10s zoom: 10° nose up, slowing down with 3 m/s²\n");
    {
        const float pitch = 10.f * D2R;
        float down[3];
        downAxis(0.f, pitch, down);
        airmass::Attitude att;
        att.reset(down);
        float tas = 40.f, f[3] = {};
        const float gyro[3] = {};
        for (int i = 0; i < 100; ++i) {
            tas -= 0.3f;
            // specific force = acceleration (along body x) - gravity
            f[0] = -3.f / G - down[0];
            f[1] = -down[1];
            f[2] = -down[2];
            att.update(gyro, f, tas, 0.f, 0.1f);
        }
        expect("pitch of the air data aided attitude [deg]", att.pitch() / D2R, 10.f, 0.3f);
        expect("pitch of a plain accelerometer reference [deg]", std::atan2(f[0], -f[2]) / D2R, 10.f - 17.f, 1.5f);
    }

    std::printf("pitch offset per flap position\n");
    {
        AirMassEKF ekf;
        ekf.reset(0.f, 1.f, 0.01f);
        airmass::FlapOffsets flaps;
        float ofs[airmass::FlapOffsets::N];
        for (float& o : ofs) o = airmass::FlapOffsets::UNKNOWN;
        ofs[1] = 0.02f;
        ofs[3] = 0.10f;
        flaps.load(ofs);
        flaps.update(ekf, NAN);
        expect("no flap sensor: offset untouched [rad]", ekf.pitchOffset(), 0.01f, 1e-6f);
        flaps.update(ekf, 1.1f);
        expect("at a learned notch: its offset [rad]", ekf.pitchOffset(), 0.02f, 1e-6f);
        flaps.update(ekf, 2.f);
        expect("never flown notch: the nearest learned one [rad]", ekf.pitchOffset(), 0.02f, 1e-6f);
        expect("never flown notch: loose [rad]", ekf.sigma(AirMassEKF::THB), airmass::FlapOffsets::SIGMA_NEW, 1e-6f);
        ekf.setPitchOffset(0.05f, 0.002f); // learned in flight
        flaps.update(ekf, 2.5f);
        expect("between two notches: interpolated [rad]", ekf.pitchOffset(), 0.075f, 1e-6f);
        expect("the notch left kept what was learned [rad]", flaps.offset(2), 0.05f, 1e-6f);
        flaps.update(ekf, 2.9f);
        expect("at the next notch: its offset [rad]", ekf.pitchOffset(), 0.10f, 1e-6f);
        expect("learned notch: certain [rad]", flaps.sigma(2), 0.002f, 1e-6f);
        expect("never flown notch: unknown", flaps.known(5) ? 1.f : 0.f, 0.f, 0.f);
    }

    std::printf("%s\n", failures ? "FAILED" : "PASS");
    return failures ? 1 : 0;
}
