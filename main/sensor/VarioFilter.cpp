/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "VarioFilter.h"
#include "SensorMgr.h"

#include "pressure/PressureSensor.h"
#include "press_diff/AirspeedSensor.h"
#include "math/Floats.h"
#include "S2F.h"
#include "AverageVario.h"
#include "setup/CruiseMode.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"


#include <cmath>
#include <algorithm>

TEcompFilter *tecompSensor = nullptr;
VarioFilter *varioSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
constexpr size_t HSIZE = MAX_SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) mps_t vario_buffer[ HSIZE + 1 ]; // history buffer for vario indicator

constexpr size_t TEALTSIZE = 20000 / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) meter_t tealt_buffer[ TEALTSIZE + 1 ]; // internal te altitude trace

//
// The doRead portion of the variometer (-> only on master).
// On a client do the postproc only instead and fetch the te_alt from the master.
//
TEcompFilter::TEcompFilter() :
    SensorTP<meter_t>(tealt_buffer, TEALTSIZE, DUTY_CYCLE_MS, 0),
    _tealt_lpf(0.25f)
{
    _id = SensorId(SensorType::VIRTUAL, 9);
    _id.flags |= SensorId::SENSOR_LOCAL;
    // mark as essential sensor to be able to simulate
    _id.flags |= SensorId::SENSOR_ESSENTIAL;
    setNVSVar(&te_alt);
    setFilter(&_tealt_lpf);
    meter_t alt = altitude_isa.get();
    _tealt_lpf.reset(alt);
}

// extract from real existing sensors according to the selected TE compensation method
// a total energy compensated altitude.
bool TEcompFilter::doRead(meter_t& val) {
    meter_t curr_altitude;
    if (te_comp_enable.get()) {
        // electronic compensation
        // method 1
        curr_altitude = altitude_isa.get();  // already read
        if (!altitude_isa.getValid() || std::isnan(curr_altitude)) {
            curr_altitude = getHead();  // ignore readout when failed
        }
        mps_t ta_speed = tas.get();  // m/s
        curr_altitude += ((ta_speed * ta_speed) / (2.f * Units::g0)) * te_comp_adjust.get() / 100.0f; // Ekin ~ h = v²/2g  * adjust
        // method 2
        pascal_t barP = baroSensor->getHead();
        pascal_t dynP = asSensor->getHead();
        curr_altitude += Units::calcAltitudeISA(barP - (dynP * te_comp_adjust.get() / 100.0f));  // subtract PI pressure like TEK probe does
        curr_altitude /= 2.f; // simple average of both methods
    }
    else {
        // TEK probe
        bool success;
        curr_altitude = teSensor->readAltitudeISA(success);
    }

    val = curr_altitude; // meter
    return true;
}

//
// Variometer Kalman Filter
//
struct VarioKF {
    // State
    meter_t h;  // altitude [m]
    mps_t   v;  // vertical speed [m/s]

    // Covariance
    float P00, P01, P10, P11;

    // Noise
    float R;     // measurement noise
    float sigma_a;

    void reset(meter_t h0) {
        h = h0;
        v = 0.0f;

        P00 = 0.01f; // trust the initial set altitude
        P11 = 0.001f;
        P01 = P10 = 0.0f;

        R = 0.3f * 0.3f;    // baro ~30cm RMS
        setTau(vario_delay.get());
   }
    void setTau(float tau) {
        sigma_a = sqrtf(2.0f / tau);
    }

    void predict(second_t dt) {
        // State prediction
        h += v * dt;

        // Process noise
        float dt2 = dt * dt;
        float dt3 = dt2 * dt;
        float dt4 = dt2 * dt2;
        float q = sigma_a * sigma_a;

        float Q00 = q * dt4 * 0.25f;
        float Q01 = q * dt3 * 0.5f;
        float Q11 = q * dt2;

        // Covariance prediction
        float P00_ = P00 + dt*(P10 + P01) + dt2*P11 + Q00;
        float P01_ = P01 + dt*P11 + Q01;
        float P10_ = P10 + dt*P11 + Q01;
        float P11_ = P11 + Q11;

        P00 = P00_;
        P01 = P01_;
        P10 = P10_;
        P11 = P11_;
    }

    void update(meter_t z) {
        // Innovation
        meter_t y = z - h;
        float S = P00 + R;

        // Kalman gain
        float K0 = P00 / S;
        float K1 = P10 / S;

        // State update
        h += K0 * y;
        v += K1 * y;

        // Covariance update
        float P00_ = P00 - K0 * P00;
        float P01_ = P01 - K0 * P01;
        float P10_ = P10 - K1 * P00;
        float P11_ = P11 - K1 * P01;

        P00 = P00_;
        P01 = P10 = 0.5f * (P01_ + P10_); // enforce symmetry
        P11 = P11_;
        // ESP_LOGI(FNAME, "VKF(%.3f/%.3f/%.3f/%.3f): K(%.3f,%.3f) pre: %.3f err:%f R:%.3f up: %.3f", P00, P01, P10, P11, K0, K1, h, y, R, v);
    }
};

static VarioKF vkf;


//
// the post processing portion of the variometer
//
VarioFilter::VarioFilter() :
    SensorTP<float>(vario_buffer, HSIZE, DUTY_CYCLE_MS, 1),
    _Gact(15.f, DUTY_CYCLE_MS / 1000.f),
    _Goptimal(15.f, DUTY_CYCLE_MS / 1000.f),
    _avg_vario(LowPassFilterT<float>::alphaFromTau(1.f, 0.1f))
{
    _id = SensorId(SensorType::VARIOMETER, 10);
    assert(tecompSensor == nullptr);
    tecompSensor = new TEcompFilter();
    SensorRegistry::registerSensor(tecompSensor);
    _prepare_sim_jump = 40; // preparation for a disruptive jump to the ground level
}

bool VarioFilter::setup() {
    ESP_LOGI(FNAME, "VarioFilter setup as %s sensor with alt %f", _id.isLocalSensor() ? "local" : "remote", altitude.get());
    // vario needle damping
    vkf.setTau(vario_delay.get()); // KF
    _prev_time = Clock::getMillis();
    startRunningAvg(vario_av_delay.get() * 1000);
    _Gact.setTau(vario_av_delay.get()); // same integration time for the climb score
    _Goptimal.setTau(vario_av_delay.get());
    return true;
}


// Kalman Filter based vario filter
void VarioFilter::postProcess() {
    uint32_t now = Clock::getMillis();
    const second_t dt = (now - _prev_time) / 1000.0f;
    _prev_time = now;

    vkf.predict(dt);
    meter_t tealt_head = tecompSensor->getHead();
    meter_t pred_err = tealt_head - vkf.h;
    if (fabsf(pred_err) > 60.0f && _prepare_sim_jump) {
        // just re-/started sim mode, expect a time and height disruption, prepare KF for it
        vkf.reset(tealt_head);
        if (_prepare_sim_jump > 0) _prepare_sim_jump--;
        ESP_LOGW(FNAME, "VarioFilter SIM: large pred_err %f, re-init KF", pred_err);
        return;
    }
    vkf.R = 0.25 * (1 + fabsf(pred_err));
    vkf.R = std::clamp(vkf.R, 0.05f, 1.0f);

    vkf.update(tealt_head);
    te_vario.set(vkf.v);
    _polar_sink = Speed2Fly.getSink(ias.get());
    mps_t te_net = vkf.v - _polar_sink;

    // calc the climb score
    if ( te_net > 0.f && airborne.get()) {
        // achieved gross climb integral
        _Gact.filter(std::max(vkf.v, 0.f));
        // Gmax​(N)=N−S(vopt​)); max. gross achievable integral with current load and net vario
        _Goptimal.filter(std::max(te_net + Speed2Fly.getMinsink(), .1f));
    }
    else {
        _Gact.filter(.0f);
        _Goptimal.filter(.1f);
    }
    // thermal performance : actual gross / max. achievable gross
    float tp = _Gact.get() / _Goptimal.get();
    // ESP_LOGI(FNAME, "Varioscore TE: %.3f, sink: %.3f, Gact: %.3f, Gmax: %.3f, tp: %.3f", vkf.v, _polar_sink, _Gact.get(), _Goptimal.get(), tp);
    thermal_score.set(tp);

    // Adjust te_net for Super Netto mode if applicable
    if (CRMOD.getVMode() == CruiseMode::MODE_REL_NETTO) {
        // Super Netto, considering circling sink
        te_net += Speed2Fly.getCirclingSink();
    }
    te_netto.set(te_net);

    if (CRMOD.isGross()) {
        pushToHistory(vkf.v, now);
    } else {
        pushToHistory(te_net, now);
    }
    _avg_vario.filter(getRunningAvg());

    // speed to fly update on current vario
    s2f_ideal.set(Speed2Fly.calculate(te_netto.get(), !CRMOD.getCMode()));

    AverageVario::newSample(vkf.v); // longer term thermal average
}
