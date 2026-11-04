#include "AirMassSensor.h"
#include "AirMass.h"
#include "AirMassInput.h"
#include "AirMassRecord.h"

#include "sensor/imu/Accelerometer.h"
#include "sensor/imu/Gyroscope.h"
#include "sensor/imu/ImuSensor.h"
#include "sensor/imu/FlightAHRS.h"
#include "sensor/pressure/PressureSensor.h"
#include "sensor/press_diff/AirspeedSensor.h"
#include "sensor/SensorMgr.h"
#include "wind/Wind.h"
#include "S2F.h"
#include "Flap.h"
#include "setup/SetupNG.h"
#include "math/Trigonometry.h"
#include "math/Units.h"
#include "logdefnone.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <initializer_list>

AirMassSensor *airmassSensor = nullptr;

constexpr int SENSOR_HISTORY_DURATION_MS = 500; // nobody evaluates the wind history so far
constexpr int DUTY_CYCLE_MS = 100;              // 10Hz
constexpr size_t HSIZE = SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) vector_f airmass_buffer[HSIZE + 1];

constexpr float NMEA_SPEED_SIGMA = 0.5f;      // m/s, speed accuracy of the NMEA (Flarm) ground velocity
constexpr float COMPASS_SIGMA = 0.15f;        // rad, compass heading incl. deviation, correlated over time
constexpr int COMPASS_CYCLES = 100;           // fuse the compass every 10s, its errors are not white
constexpr float COMPASS_MAX_ROLL = 0.17f;     // rad, the compass is only trusted in straight flight
constexpr uint16_t PERSIST_CYCLES = 3000;     // save the learned calibration every 5 minutes in flight
constexpr uint16_t SETUP_CYCLES = 100;        // repeat the recorded setup every 10s, for logs started in flight

// "NMEA Output" (airmass_nmea)
enum : int { NMEA_OFF = 0, NMEA_OUTPUT = 1, NMEA_RECORD = 2 };

// lines for the NMEA output, from the sensor loop to the NMEA feed (5Hz), created with the first line
constexpr int QUEUE_LINES = 20;
constexpr int QUEUE_LEN = 224; // the longest sentence ($PXCVS) has about 190 characters
static QueueHandle_t nmea_queue = nullptr;

static void queueLine(const std::string& line)
{
    if (!nmea_queue) {
        nmea_queue = xQueueCreate(QUEUE_LINES, QUEUE_LEN);
    }
    if (nmea_queue && line.size() < QUEUE_LEN) {
        char buf[QUEUE_LEN]; // the queue copies whole items
        std::memcpy(buf, line.c_str(), line.size() + 1);
        xQueueSend(nmea_queue, buf, 0); // full: dropped, the replay sees a gap
    }
}

bool airmassNmeaPop(std::string& line)
{
    char buf[QUEUE_LEN];
    if (!nmea_queue || xQueueReceive(nmea_queue, buf, 0) != pdTRUE) {
        return false;
    }
    line = buf;
    return true;
}

static bool recording()
{
    return airmass_nmea.get() == NMEA_RECORD;
}

static_assert(airmass::FlapOffsets::N == Flap::MAX_NR_POS, "a pitch offset per flap position");
static SetupNG<float>* const FLAP_OFS[airmass::FlapOffsets::N] = {&airmass_flap_ofs_0, &airmass_flap_ofs_1, &airmass_flap_ofs_2,
    &airmass_flap_ofs_3, &airmass_flap_ofs_4, &airmass_flap_ofs_5, &airmass_flap_ofs_6};

// The flap lever position from the flap sensor. Without one the flap recommendation (also without the flap box
// shown), which pilots follow: at the load of the turn rather than the accelerometer's, a roll-out or pull-up does
// not move the flaps. NAN without flaps.
static float flapLever(float roll)
{
    // flap levels left from an earlier glider do not count; a flapped glider keeps FLAP (Flap::needed)
    const bool sensor = Flap::sensAvailable();
    if (!(sensor || Speed2Fly.hasFlaps()) || !FLAP || FLAP->getNrPositions() < 2) {
        return NAN;
    }
    if (sensor) {
        return flap_pos.getValid() ? flap_pos.get() : NAN;
    }
    return std::round(FLAP->getOptimum(ias.get(), 1.f / std::max(std::cos(roll), 0.3f)));
}

// FNV-1a over the raw values of setup items, identifies the installation the calibration was learned with
static int signatureOf(std::initializer_list<SetupCommon*> items)
{
    uint32_t h = 2166136261u;
    for (SetupCommon* item : items) {
        const uint8_t* p = static_cast<const uint8_t*>(item->getPtr());
        for (int i = 0; i < item->getSize(); ++i) {
            h = (h ^ p[i]) * 16777619u;
        }
    }
    return static_cast<int>(h);
}

// the pitch offset depends on the IMU mounting and the polar (zero lift angle of attack)
static int pitchSignature()
{
    return signatureOf({&imu_reference, &imu_facref, &glider_ground_aa, &glider_type});
}

// the airspeed scale depends on the airspeed calibration and the pitot/static installation
static int speedSignature()
{
    return signatureOf({&speedcal, &glider_type});
}

// Body rates without the "Gyro Gating" of the processed gyro: its dead zone (0.5°/s) drops the slow rotations, the
// gentle pitch changes in cruise, small heading corrections, the roll axis rate in a steady turn. The recorded
// reading has the bias estimate subtracted once, the processed one twice (the estimator learns on the recorded
// readings and settles at half the bias), so this is the processed gyro before the gating.
static vector_f gyroRates()
{
    return gyroSensor->getHead() - gyroSensor->getBias();
}

// latest GNSS ground velocity, written by the protocol task, read by the sensor loop
static AirMassEKF::GnssVelocity gnss_vel;
static uint16_t gnss_epoch = 0; // counts the velocities, 0: none yet
static portMUX_TYPE gnss_mux = portMUX_INITIALIZER_UNLOCKED;

void airmass::gnssVelocity(mps_t speed, rad_t course, uint32_t time_ms, uint16_t latency_ms)
{
    taskENTER_CRITICAL(&gnss_mux);
    gnss_vel = {time_ms, latency_ms, speed * fast_cos_rad(course), speed * fast_sin_rad(course), NMEA_SPEED_SIGMA};
    if (++gnss_epoch == 0) {
        gnss_epoch = 1;
    }
    taskEXIT_CRITICAL(&gnss_mux);
}

static uint16_t latestGnss(AirMassEKF::GnssVelocity& v)
{
    taskENTER_CRITICAL(&gnss_mux);
    v = gnss_vel;
    const uint16_t epoch = gnss_epoch;
    taskEXIT_CRITICAL(&gnss_mux);
    return epoch;
}

bool airmass::heading(rad_t track)
{
    if (!airmassSensor || !(airmass_status.get() & AirMassEKF::ST_WIND_VALID)) {
        return false;
    }
    const rad_t hdg = Vector::normalizePI2(airmassSensor->getEKF().heading());
    heading_tru.set(hdg);
    heading_wca.set(Vector::angleDiff(hdg, track));
    return true;
}

void airmass::startup()
{
    // master only, needs the local IMU and air data
    if (SetupCommon::isMaster() && ahrsSensor && imuSensor && baroSensor && asSensor) {
        airmassSensor = new AirMassSensor();
        airmassSensor->setup();
        SensorRegistry::registerSensor(airmassSensor);
    }
}

AirMassSensor::AirMassSensor() :
    SensorTP<vector_f>(airmass_buffer, HSIZE, DUTY_CYCLE_MS, 1)
{
    // after AHRS and airspeed, right before the vario filter
    _id = SensorId(SensorType::AIRMASS, 9);
}

AirMassSensor::~AirMassSensor()
{
    airmassSensor = nullptr;
}

bool AirMassSensor::setup()
{
    applyConfig();
    return true;
}

// installation, polar (with ballast) and settings, they can change in flight
airmass::Setup AirMassSensor::currentSetup() const
{
    airmass::Setup s;
    s.lever_arm = imuSensor ? imuSensor->getLeverArm() : 0.f;
    s.mass = gross_weight.get();
    s.wing_area = polar_wingarea.get();
    Speed2Fly.getPolar(s.polar.a0, s.polar.a1, s.polar.a2, s.polar.v_stall, s.polar.valid);
    s.wind_resp = airmass_wind_resp.get();
    s.vario_resp = airmass_vario_resp.get();
    s.te_adjust = airmass_te_adjust.get();
    return s;
}

void AirMassSensor::applyConfig()
{
    _pipe.configure(currentSetup());
    if (recording()) {
        queueLine(airmass::record::setup(_pipe.setup()));
    }
}

// static port altitude, independent of the altimeter source setting (TE or external would corrupt the estimator)
bool AirMassSensor::readBaroAltitude(float& alt) const
{
    bool ok = false;
    if (baroSensor) {
        alt = baroSensor->readAltitudeISA(ok);
    }
    return ok && std::isfinite(alt);
}

void AirMassSensor::start()
{
    // learned values only apply to the installation they were learned with
    const bool pitch_ok = airmass_sig_pitch.get() == pitchSignature();
    const bool speed_ok = airmass_sig_speed.get() == speedSignature();
    const float k_tas = speed_ok ? airmass_k_tas.get() : 1.f;
    const float pitch_ofs = pitch_ok ? airmass_pitch_ofs.get() : 0.f;
    const float k_aoa = pitch_ok ? airmass_k_aoa.get() : 1.f;
    ESP_LOGI(FNAME, "AirMass start, k_tas %.3f%s, pitch ofs %.2f°%s, gnss latency ofs %.2fs", k_tas, speed_ok ? "" : " (reset)",
             Units::rad_to_deg(pitch_ofs), pitch_ok ? "" : " (reset)", airmass_lat_ofs.get());
    float alt = 0.f;
    readBaroAltitude(alt);
    airmass::Warm warm;
    warm.k_tas = k_tas;
    warm.pitch_ofs = pitch_ofs;
    warm.lat_ofs = airmass_lat_ofs.get();
    warm.k_aoa = k_aoa;
    for (int i = 0; i < airmass::FlapOffsets::N; ++i) {
        warm.flap_ofs[i] = pitch_ok ? FLAP_OFS[i]->get() : airmass::FlapOffsets::UNKNOWN;
    }
    // the own attitude starts from the converged AHRS
    airmass::record::Start st;
    st.t = Clock::getMillis();
    st.alt = alt;
    const vector_f down = ahrsSensor->getBody2Nav().rotateInv(vector_f(0.f, 0.f, 1.f));
    const vector_f gyro = gyroSensor ? gyroRates() : vector_f();
    for (int i = 0; i < 3; ++i) {
        st.down[i] = down[i];
        st.gyro[i] = gyro[i];
    }
    st.warm = warm;
    applyConfig();
    _pipe.start(st.t, st.alt, st.down, st.gyro, st.warm);
    if (recording()) {
        queueLine(airmass::record::start(st));
    }
    AirMassEKF::GnssVelocity g;
    _gnss_epoch = latestGnss(g); // only fresh epochs
}

// NVS write only when the value moved: learned values change in the last digits all the time, and every
// write stalls the sensor loop
static void store(SetupNG<float>& item, float value, float resolution)
{
    if (std::fabs(value - item.get()) > resolution) {
        item.set(value, false);
    }
}
static void store(SetupNG<int>& item, int value)
{
    if (value != item.get()) {
        item.set(value, false);
    }
}

// keep the learned calibration as warm start for the next flight
void AirMassSensor::persistLearned()
{
    AirMassEKF& ekf = _pipe.ekf();
    airmass::FlapOffsets& flaps = _pipe.flaps();
    if (!ekf.isRunning() || !ekf.isAligned()) {
        return;
    }
    constexpr float PITCH_SIGMA = 0.01f; // rad, a pitch offset this certain is kept
    if (ekf.sigma(AirMassEKF::KTAS) < 0.02f) {
        store(airmass_k_tas, ekf.kTas(), 1e-3f);
        store(airmass_sig_speed, speedSignature());
    }
    if (flaps.active()) {
        // flapped glider: an offset per lever position
        flaps.keep(ekf);
        for (int i = 0; i < airmass::FlapOffsets::N; ++i) {
            if (flaps.sigma(i) < PITCH_SIGMA) {
                store(*FLAP_OFS[i], flaps.offset(i), 1e-3f);
            }
        }
        store(airmass_k_aoa, ekf.aoaScale(), 1e-3f);
        store(airmass_sig_pitch, pitchSignature());
    } else if (ekf.sigma(AirMassEKF::THB) < PITCH_SIGMA) {
        store(airmass_pitch_ofs, ekf.pitchOffset(), 1e-3f);
        store(airmass_k_aoa, ekf.aoaScale(), 1e-3f);
        store(airmass_sig_pitch, pitchSignature());
    }
    if (ekf.sigma(AirMassEKF::TAU) < 0.1f) {
        store(airmass_lat_ofs, ekf.latencyOffset(), 0.01f);
    }
    ESP_LOGI(FNAME, "AirMass learned k_tas %.3f, pitch ofs %.2f°, gnss latency ofs %.2fs", airmass_k_tas.get(),
             Units::rad_to_deg(airmass_pitch_ofs.get()), airmass_lat_ofs.get());
}

void AirMassSensor::stop()
{
    persistLearned();
    _pipe.ekf().stop();
    if (recording()) {
        queueLine(airmass::record::stop(Clock::getMillis()));
    }
    airmass_status.set(0);
    airmass_inst_wind.set(WindData().data());
    airmass_avg_wind.set(WindData().data());
}

void AirMassSensor::postProcess()
{
    const bool active = airmass_enable.get() && airborne.get() && ahrsSensor && accSensor && gyroSensor;
    if (active != _active) {
        _active = active;
        if (active) {
            start();
        } else {
            stop();
        }
    }
    if (!active) {
        return;
    }
    float baro_alt = NAN;
    if (!tas.getValid() || !readBaroAltitude(baro_alt)) {
        // no air data: invalidate the outputs, the vario falls back to netto
        if (airmass_status.get() != 0) {
            airmass_status.set(0);
            airmass_inst_wind.set(WindData().data());
        }
        return;
    }
    if (!_pipe.ekf().isRunning()) {
        ESP_LOGW(FNAME, "AirMass estimator restart");
        start();
    }
    // settings, ballast and the polar can change in flight
    const bool rec = recording();
    if (currentSetup() != _pipe.setup()) {
        applyConfig();
    } else if (rec && !(_count % SETUP_CYCLES)) {
        queueLine(airmass::record::setup(_pipe.setup())); // for logs started in flight
    }

    // the raw measurements of this cycle
    airmass::Frame f;
    f.time_ms = Clock::getMillis();
    const vector_f& f_body = accSensor->getHead(); // specific force in g, body NED
    const vector_f gyro = gyroRates();             // rad/s, body
    for (int i = 0; i < 3; ++i) {
        f.f_body[i] = f_body[i];
        f.gyro[i] = gyro[i];
    }
    f.ias = ias.get();
    f.tas = tas.get();
    f.baro_alt = baro_alt;
    f.flap_lever = flapLever(_pipe.attitude().roll());
    _pipe.step(f);
    if (rec) {
        queueLine(airmass::record::frame(f));
    }

    // compass: absolute heading in straight flight, aligns the wind before the first turn
    if (!(_count % COMPASS_CYCLES) && heading_mag.getValid() && std::fabs(_pipe.input().roll) < COMPASS_MAX_ROLL) {
        const airmass::record::Heading h{f.time_ms, Vector::normalizePI(heading_mag.get() + Units::deg_to_rad(compass_declination.get())),
                                         COMPASS_SIGMA};
        _pipe.heading(h.psi, h.sigma);
        if (rec) {
            queueLine(airmass::record::heading(h));
        }
    }

    // GNSS velocity epochs
    airmass::record::Gnss g;
    const uint16_t epoch = latestGnss(g.v);
    if (epoch && epoch != _gnss_epoch) {
        _gnss_epoch = epoch;
        // a valid synoptic wind (circling/straight wind) serves as prior for the alignment
        g.prior = synoptic_wind.getValid();
        if (g.prior) {
            const vector_f w = static_cast<WindData>(synoptic_wind.get()).getVector();
            g.wn0 = w.x;
            g.we0 = w.y;
        }
        _pipe.gnss(g.v, g.prior, g.wn0, g.we0);
        if (rec) {
            queueLine(airmass::record::gnss(g));
        }
    }

    _pipe.endCycle();
    publish();
    // the estimator output at 5Hz, after the cycle it belongs to
    if (airmass_nmea.get() != NMEA_OFF && !(_count % 2)) {
        queueLine(airmass::record::output(_pipe.ekf()));
    }

    // a power cut before the landing detection must not lose the whole flight
    if (!(_count % PERSIST_CYCLES)) {
        persistLearned();
    }
}

void AirMassSensor::publish()
{
    const AirMassEKF& ekf = _pipe.ekf();
    const uint32_t now = Clock::getMillis();
    const vector_f wind(ekf.windN(), ekf.windE(), ekf.windD());
    pushAndPublish(wind, now);
    const uint8_t st = ekf.status();
    _count++;

    // vertical air mass movement at 5Hz to the second seat, the vario filter reads it locally every cycle
    airmass_vario.set(ekf.airmassVario(), !(_count % 2));
    // volatile entries stay dirty once set, so every set() would sync: only set on change
    auto setChanged = [](SetupNG<int>& var, int val) {
        if (val != var.get()) {
            var.set(val);
        }
    };
    setChanged(airmass_status, st);

    if (st & AirMassEKF::ST_WIND_VALID) {
        const vector_f avg(_pipe.avgWind().windN(), _pipe.avgWind().windE(), 0.f);
        if (!(_count % 5)) {
            setChanged(airmass_inst_wind, windFromVector(wind).data());
        }
        if (!(_count % 10)) {
            setChanged(airmass_avg_wind, windFromVector(avg).data());
        }
    } else if (st & AirMassEKF::ST_WIND_DEGRADED) {
        // not valid any more, the estimate is still the best instant wind there is: shown marked, no average
        if (!(_count % 5)) {
            setChanged(airmass_inst_wind, windFromVector(wind).data());
        }
        setChanged(airmass_avg_wind, WindData().data());
    } else {
        // no stale wind for consumers that do not check the status
        setChanged(airmass_inst_wind, WindData().data());
        setChanged(airmass_avg_wind, WindData().data());
    }
    if (!(_count % 50)) {
        ESP_LOGI(FNAME, "AirMass st %x wind %.1f/%.1f/%.2f psi %.1f° k %.3f thb %.2f° baz %.3f bgz %.3f°/s lat %.0fms", st, wind.x, wind.y,
                 wind.z, Units::rad_to_deg(ekf.heading()), ekf.kTas(), Units::rad_to_deg(ekf.pitchOffset()), ekf.accBias(),
                 Units::rad_to_deg(ekf.yawRateBias()), ekf.gnssLatency() * 1000.f);
    }
}
