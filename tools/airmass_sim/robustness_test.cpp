// Deterministic timing faults against known flight/wind, independent of the GNSS-derived replay reference.
#include "AirMassPipeline.h"
#include "AirMassRecord.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {
int failures = 0;
void check(bool ok, const char* what) {
    std::printf("  %s: %s\n", what, ok ? "PASS" : "FAIL");
    failures += !ok;
}

AirMassEKF::Input input(uint32_t now, float& psi) {
    AirMassEKF::Input in;
    in.time_ms = now;
    in.roll = 0.5f * std::sin(now * 0.00015f);
    in.aoa = 0.06f;
    in.tas = 30.f;
    const float a = std::cos(in.aoa), b = std::sin(in.aoa) * std::cos(in.roll);
    in.pitch = std::atan2(b, a) - std::asin(0.8f / (in.tas * std::hypot(a, b)));
    in.yaw_rate = Units::g0 * std::tan(in.roll) / in.tas;
    psi += in.yaw_rate * in.dt;
    in.baro_alt = 1000.f - now * 0.0008f;
    in.polar_sink = 0.8f;
    in.load_factor = 1.f / std::cos(in.roll);
    return in;
}

struct Fix { uint32_t due; AirMassEKF::GnssVelocity g; };
void flight(const char* name, bool irregular, bool jitter, bool epochs, bool corrupt, bool outage) {
    std::printf("%s\n", name);
    AirMassEKF ekf;
    ekf.reset(1000.f, 1.f, 0.f, 0.8f);
    float psi = 0.f;
    std::vector<Fix> queue;
    uint32_t next = 0;
    int count = 0, scored = 0;
    double sum = 0.;
    float max_vario = 0.f;
    for (uint32_t now = 100; now <= 180000; now += 100) {
        const auto in = input(now, psi);
        ekf.update(in);
        if (now >= next) {
            ++count;
            next = now + (irregular ? (count % 7 == 0 ? 4000 : 200 + 200 * (count % 5)) : 1000);
            AirMassEKF::GnssVelocity g;
            float va[3];
            AirMassEKF::airVelocity(in.tas, 1.f, in.roll, in.pitch, psi, in.aoa, va, nullptr);
            g.vn = va[0] + 2.f;
            g.ve = va[1] - 3.f;
            g.latency_ms = 800;
            g.fix_time_ms = epochs ? (86390000u + now) % airmass::GnssTiming::DAY_MS : airmass::GnssTiming::UNKNOWN;
            g.time_ms = now + 800 + (jitter ? (count % 9 == 0 ? 2500 : 200 * (count % 4)) : 0);
            if (corrupt && count % 13 == 0) { g.vn += 8.f; g.ve += 70.f; }
            if (!(outage && now > 60000 && now < 85000)) queue.push_back({g.time_ms, g});
        }
        std::stable_sort(queue.begin(), queue.end(), [](const Fix& a, const Fix& b) { return a.due < b.due; });
        while (!queue.empty() && queue.front().due <= now) {
            ekf.fuseGnss(queue.front().g, true, 2.f, -3.f);
            queue.erase(queue.begin());
        }
        if (!ekf.covarianceHealthy() || !std::isfinite(ekf.airmassVario())) {
            check(false, "finite state/covariance throughout");
            return;
        }
        if (now > 30000) {
            const float err = std::hypot(ekf.windN() - 2.f, ekf.windE() + 3.f);
            sum += err * err;
            ++scored;
            max_vario = std::max(max_vario, std::fabs(ekf.airmassVario()));
        }
    }
    const float rms = std::sqrt(sum / scored);
    std::printf("  wind RMS %.3f m/s, max false vario %.3f m/s, rejected %u\n", rms, max_vario, ekf.rejectedGnss());
    check(ekf.isAligned() && rms < (epochs ? 0.4f : 0.8f), "wind remains accurate through timing faults");
    check(max_vario < 0.5f, "no large false lift/sink from timing faults");
    check(!corrupt || ekf.rejectedGnss() > 0, "velocity outliers rejected");
    check(ekf.gnssAgeMs() < 6000, "good fixes recover after outage");
}

void timing() {
    std::printf("source clock, delay and ordering\n");
    airmass::GnssTiming t;
    uint32_t delay;
    check(t.observe(1000, 86399000, delay) && delay == 0, "first epoch establishes offset");
    check(t.observe(2000, 0, delay) && delay == 0, "UTC midnight is continuous");
    check(!t.observe(2100, 0, delay), "retransmitted source epoch rejected");
    check(!t.observe(2200, 86399000, delay), "late/out-of-order epoch rejected");
    check(t.observe(4000, 1000, delay) && delay == 1000, "extra transport delay measured");
    check(t.observe(4000, 2000, delay) && delay == 0, "distinct fixes in one arrival burst supported");
    check(t.observe(9000, 7000, delay) && delay == 0, "missing epochs do not imply transport delay");
    t.reset();
    check(t.observe(UINT32_MAX - 99, 1000, delay) && t.observe(900, 2000, delay) && delay == 0,
          "sensor-clock wrap supported");
}

void rejection() {
    std::printf("whole-fix rejection and freshness\n");
    AirMassEKF ekf;
    ekf.reset(1000.f);
    float psi = 0.f;
    for (uint32_t now = 100; now <= 10000; now += 100) {
        const auto in = input(now, psi);
        ekf.update(in);
        if (now == 2000) {
            AirMassEKF::GnssVelocity g{now, 750, 32.f, -3.f, 0.5f, 10000};
            ekf.fuseGnss(g, true, 2.f, -3.f);
        }
    }
    const float wn = ekf.windN(), we = ekf.windE(), k = ekf.kTas(), latency = ekf.gnssLatency();
    const uint32_t age = ekf.gnssAgeMs();
    AirMassEKF::GnssVelocity bad{10000, 750, wn + 29.f, we + 90.f, 0.5f, 18000};
    ekf.fuseGnss(bad);
    check(ekf.windN() == wn && ekf.windE() == we && ekf.kTas() == k && ekf.gnssLatency() == latency,
          "rejection leaves both axes and calibration untouched");
    check(ekf.gnssAgeMs() == age && ekf.gnssReceivedAgeMs() == 0, "received and accepted freshness separated");
    check(!(ekf.status() & AirMassEKF::ST_GNSS_FRESH), "rejected stream cannot keep GNSS fresh");
    bad.time_ms = 10001;
    ekf.fuseGnss(bad);
    check(ekf.gnssAgeMs() == age, "future arrival rejected");
    bad.time_ms = 10000;
    bad.vn = NAN;
    ekf.fuseGnss(bad);
    check(ekf.covarianceHealthy(), "NaN velocity rejected without poisoning covariance");
}

void gaps() {
    std::printf("sensor continuity and delayed-history support\n");
    airmass::Pipeline p;
    airmass::Setup setup;
    p.configure(setup);
    const float down[3] = {0.f, 0.f, 1.f}, gyro[3] = {};
    p.start(1000, 1000.f, down, gyro, {});
    airmass::Frame f;
    f.ias = f.tas = 30.f;
    f.baro_alt = 1000.f;
    f.f_body[2] = -1.f;
    for (f.time_ms = 1100; f.time_ms <= 12000; f.time_ms += 100) p.step(f);
    f.time_ms = 14500;
    p.step(f);
    check(p.ekf().inertialTrust() == 0.f, "gap immediately suppresses inertial gust correction");
    check(!(p.ekf().status() & AirMassEKF::ST_VARIO_VALID), "missing motion clears vertical validity during recovery");
    const float heading = p.ekf().heading();
    p.step(f);
    check(p.ekf().heading() == heading && p.ekf().inertialTrust() == 0.f, "duplicate sensor cycle ignored");
    const uint32_t rejected = p.ekf().rejectedGnss();
    p.gnss({14500, 750, 30.f, 0.f, 0.5f}, true, 0.f, 0.f);
    check(p.ekf().rejectedGnss() == rejected + 1 && !p.ekf().isAligned(), "GNSS cannot interpolate across inferred motion");
    for (f.time_ms = 14600; f.time_ms <= 26000; f.time_ms += 100) p.step(f);
    check(p.ekf().inertialTrust() == 1.f && p.ekf().covarianceHealthy(), "continuous samples restore trust gradually");
    const auto before = p.ekf().rejectedGnss();
    p.gnss({16000, 750, 30.f, 0.f, 0.5f}, true, 0.f, 0.f);
    check(p.ekf().rejectedGnss() == before + 1, "observation older than retained history rejected");
    f.time_ms = 40000;
    p.step(f);
    check(p.ekf().isRunning() && p.ekf().covarianceHealthy() && p.ekf().inertialTrust() == 0.f,
          "long sensor outage restarts dynamics without integrating invented motion");

    airmass::Pipeline rotating;
    rotating.configure(setup);
    rotating.start(1000, 1000.f, down, gyro, {});
    airmass::ExternalAttitude att;
    att.down[2] = 1.f;
    att.yaw_rate = 0.1f;
    f.time_ms = 1100;
    rotating.step(f, &att);
    const float before_gap = rotating.ekf().heading();
    f.time_ms = 3600;
    rotating.step(f, &att);
    check(std::fabs(rotating.ekf().heading() - before_gap - 0.25f) < 1e-4f,
          "held gyro propagates full elapsed time, rather than clipping the gap to 0.3s");
}

void recording() {
    airmass::record::Sentence s;
    airmass::record::Gnss g, decoded;
    g.v = {12000, 750, 30.f, -2.f, 0.5f, 29361000};
    check(airmass::record::parse(airmass::record::gnss(g).c_str(), s) && airmass::record::decode(s, decoded) &&
          decoded.v.fix_time_ms == g.v.fix_time_ms, "recording retains source epoch");
    g.v.fix_time_ms = airmass::GnssTiming::UNKNOWN;
    check(airmass::record::parse(airmass::record::gnss(g).c_str(), s) && airmass::record::decode(s, decoded) &&
          decoded.v.fix_time_ms == airmass::GnssTiming::UNKNOWN, "legacy recording remains supported");
}

void landing() {
    std::printf("low-speed landing validity and recovery\n");
    airmass::Pipeline pipe;
    airmass::Setup setup;
    setup.mass = 460.f;
    setup.wing_area = 11.81f;
    setup.polar = {-0.8f, 0.f, 0.f, 21.636f, true};
    pipe.configure(setup);
    const float down[3] = {0.f, 0.f, 1.f}, gyro[3] = {};
    pipe.start(1000, 1000.f, down, gyro, {});
    airmass::Frame f;
    f.ias = 30.f;
    f.tas = 35.f;
    f.baro_alt = 1000.f;
    airmass::ExternalAttitude att;
    for (f.time_ms = 1100; f.time_ms <= 12000; f.time_ms += 100) pipe.step(f, &att);
    check(pipe.ekf().status() & AirMassEKF::ST_VARIO_VALID, "valid before landing at flying speed");

    // IAS 17 m/s is below 80% of the 21.636 m/s polar reference, while TAS exceeds its separate floor.
    f.ias = 17.f;
    f.tas = 25.f;
    f.time_ms = 12100;
    pipe.step(f, &att);
    const float pitch_ofs = pipe.ekf().pitchOffset(), aoa_scale = pipe.ekf().aoaScale();
    bool invalid = !(pipe.ekf().status() & AirMassEKF::ST_VARIO_VALID);
    for (f.time_ms = 12200; f.time_ms <= 16000; f.time_ms += 100) {
        pipe.step(f, &att);
        invalid &= !(pipe.ekf().status() & AirMassEKF::ST_VARIO_VALID);
    }
    check(!pipe.input().flying_speed && invalid, "ground roll immediately and continuously selects TE fallback");
    check(pipe.ekf().pitchOffset() == pitch_ofs && pipe.ekf().aoaScale() == aoa_scale,
          "wheel support cannot train pitch or AoA calibration");
    const auto rejects = pipe.ekf().rejectedGnss();
    pipe.gnss({16000, 750, 25.f, 0.f, 0.5f}, true, 0.f, 0.f);
    check(pipe.ekf().rejectedGnss() == rejects + 1 && !pipe.ekf().isAligned(),
          "low-speed air-vector history cannot align wind from wheel support");
    f.ias = f.tas = 0.f;
    for (f.time_ms = 16100; f.time_ms <= 50000; f.time_ms += 100) {
        pipe.step(f, &att);
        invalid &= !(pipe.ekf().status() & AirMassEKF::ST_VARIO_VALID);
    }
    check(invalid && pipe.ekf().covarianceHealthy(), "stopped glider never reports stale AirMass as valid");

    // IAS 18 m/s is above the cutoff; AirMass must wait for settling before becoming valid again.
    f.ias = 18.f;
    f.tas = 27.f;
    bool premature = false;
    for (f.time_ms = 50100; f.time_ms <= 54900; f.time_ms += 100) {
        pipe.step(f, &att);
        premature |= pipe.ekf().status() & AirMassEKF::ST_VARIO_VALID;
    }
    check(pipe.input().flying_speed && !premature, "return to flying speed waits for settling before leaving TE");
    f.time_ms = 55000;
    pipe.step(f, &att);
    check(pipe.ekf().status() & AirMassEKF::ST_VARIO_VALID, "sustained flying speed restores AirMass after settling");

    AirMassEKF ekf;
    ekf.reset(1000.f);
    AirMassEKF::Input in;
    in.tas = 0.f;
    in.baro_alt = 1000.f;
    bool direct_invalid = true;
    for (in.time_ms = 100; in.time_ms <= 10000; in.time_ms += 100) {
        ekf.update(in);
        direct_invalid &= !(ekf.status() & AirMassEKF::ST_VARIO_VALID);
    }
    check(direct_invalid, "direct EKF callers also cannot mark zero TAS valid");
}
} // namespace

int main() {
    timing();
    rejection();
    gaps();
    recording();
    landing();
    flight("regular delayed fixes", false, false, true, false, false);
    flight("irregular fix cadence", true, false, true, false, false);
    flight("jitter, late and out-of-order fixes", false, true, true, false, false);
    flight("legacy fixes without source epochs", false, true, false, false, false);
    flight("combined irregularity, jitter, outliers and outage", true, true, true, true, true);
    return failures ? 1 : 0;
}
