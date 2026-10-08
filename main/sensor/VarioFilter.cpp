/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "VarioFilter.h"

#include "pressure/PressureSensor.h"
#include "press_diff/AirspeedSensor.h"
#include "S2F.h"
#include "AverageVario.h"
#include "setup/CruiseMode.h"
#include "setup/SetupNG.h"
#include "logdef.h"


#include <cmath>
#include <algorithm>

VarioFilter *varioSensor = nullptr;

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
constexpr size_t HSIZE = MAX_SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS;
static __attribute__((aligned(4))) mps_t vario_buffer[ HSIZE + 1 ]; // history buffer for vario indicator

// Data and structures for different filter variants
static meter_t averageAlt = 0.f;
static meter_t Altitude = 0.f;
static meter_t predictAlt = 0.f;
static meter_t lastAltitude = 0.f;


//
// the post processing portion of the variometer
//
VarioFilter::VarioFilter() :
    SensorTP<float>(vario_buffer, HSIZE, DUTY_CYCLE_MS, 1),
    _Gact(15.f, DUTY_CYCLE_MS / 1000.f),
    _Goptimal(15.f, DUTY_CYCLE_MS / 1000.f),
    _vario_lpf(LowPassFilterT<float>::alphaFromTau(4.f, 0.1f)),
    _avg_vario(LowPassFilterT<float>::alphaFromTau(1.f, 0.1f))
{
    _id = SensorId(SensorType::VARIOMETER, 10); // prio right after the te_alt doRead filter

    // the post process organizing it's own NVS variable and filter
    setNVSVar(&te_vario);

    // preparation for a disruptive jump to the ground level
    _prepare_jump = 10;
    _prev_time = Clock::getMillis();
}

bool VarioFilter::setup() {
    float alt = altitude.get();
    ESP_LOGI(FNAME, "VarioFilter setup with alt %f", alt);
    // vario needle damping
    _vario_lpf.setTau( vario_delay.get() / 3, .1f);
    averageAlt = alt;
    Altitude = alt;
    predictAlt = alt;
    lastAltitude = alt;

    // set averager time span
    startRunningAvg(vario_av_delay.get() * 1000);
    _Gact.setTau(vario_av_delay.get()); // same integration time for the climb score
    _Goptimal.setTau(vario_av_delay.get());
    return true;
}


void VarioFilter::postProcess()
{
    static float _errorval = 1.6;
    static mps_t _TEF = 0.f;
    int now = Clock::getMillis();
    const second_t dt = (now - _prev_time) / 1000.0f;
    _prev_time = now;

    // extract from sensors according to the selected TE compensation method
    // a total energy compensated altitude.
    meter_t curr_altitude;

    if (te_comp_enable.get()) {
        // electronic compensation
        curr_altitude = altitude_isa.get(); // already read
        if (!altitude_isa.getValid() || std::isnan(curr_altitude)) {
            return;  // ignore readout when failed
        }
        // method 1
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

    // ESP_LOGI(FNAME,"TE alt: %4.3f m, ST: %.1f PI: %.1f", _currentAlt, barP, (dynP*100) );
    averageAlt += (curr_altitude - averageAlt) * 0.1;
    meter_t adiff = curr_altitude - Altitude;
    // ESP_LOGI(FNAME,"VarioFilter new alt %0.1f err %0.1f", _currentAlt, err);
    meter_t diff = (abs(adiff) * 1000) + 1;
    if (diff > 1000000 || _prepare_jump) {  // more than 100 m altitude diff in 0.1 second not plausible ( > 400 km/h vertical ) -> handled by Kalman filter
        ESP_LOGW(FNAME, "TE sensor delta OOB: %f m", diff / 10000);
        // reset filter state
        averageAlt = Altitude = curr_altitude;
        diff = adiff = 0.f;
        if (_prepare_jump > 0) _prepare_jump--;
    }
    float err = (abs(curr_altitude - predictAlt) * 1000) + 1;
    averageAlt += (curr_altitude - averageAlt) * 0.1;
    float kg = (diff / (err * _errorval + diff)) * 0.2;  // 0.2 Kalman gain
    Altitude += (adiff)*kg;
    meter_t altDiff = Altitude - lastAltitude;
    // ESP_LOGI(FNAME," altDiff %0.1f diff %0.1f", TE, diff);
    lastAltitude = Altitude;
    mps_t TEAVG = _vario_lpf.filter(altDiff / dt);  // in m/s
    predictAlt = Altitude + (TEAVG * dt);
    _TEF += ((TEAVG - _TEF)) / vario_delay.get();
    _got_positive = _TEF > .0f && te_vario.get() < .0f;

    _polar_sink = Speed2Fly.getSink(ias.get());
    mps_t te_net = _TEF - _polar_sink;

    // calc the climb score
    if ( te_net > 0.f && airborne.get()) {
        // achieved gross climb integral
        _Gact.filter(std::max(_TEF, 0.f));
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

    if ( CRMOD.getChanged() ) {
        restartRunningAvg(); // restart the running average when the cruise mode changes, avoid smeared data
        ESP_LOGI(FNAME, "Cruise mode changed, restarting running average");
    }
    if (CRMOD.isGross()) {
        pushAndPublish(_TEF, now);
    } else {
        pushAndPublish(te_net, now);
    }
    _avg_vario.filter(getRunningAvg());

    // speed to fly update on current vario
    s2f_ideal.set(Speed2Fly.calculate(te_net, !CRMOD.getCMode()));

    AverageVario::newSample(_TEF);
}
