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

// the needle at 20Hz, the altitude filter on every fresh reading of its pressure sensor (10Hz)
constexpr int DUTY_CYCLE_MS = 50;
constexpr int SLOW_CYCLE_MS = 100; // history, climb score, speed to fly, average climb: made for 10Hz
constexpr int STALE_MS = 250;      // without a fresh reading, run on the held one
// the longest averager plus the interval jitter
constexpr size_t HSIZE = (MAX_SENSOR_HISTORY_DURATION_MS + 1000) / SLOW_CYCLE_MS;
static __attribute__((aligned(4))) mps_t vario_buffer[ HSIZE + 1 ]; // history buffer for vario indicator

// Data and structures for different filter variants
static meter_t Altitude = 0.f;
static meter_t predictAlt = 0.f;


//
// the post processing portion of the variometer
//
VarioFilter::VarioFilter() :
    SensorTP<float>(vario_buffer, HSIZE, DUTY_CYCLE_MS, 1),
    _Gact(15.f, SLOW_CYCLE_MS / 1000.f),
    _Goptimal(15.f, SLOW_CYCLE_MS / 1000.f),
    _te_damper(4.f, DUTY_CYCLE_MS / 1000.f),
    _avg_vario(LowPassFilterT<float>::alphaFromTau(1.f, DUTY_CYCLE_MS / 1000.f))
{
    _valid_time_ms = 2 * STALE_MS;
    _lpf_interval.reset(SLOW_CYCLE_MS); // the history interval
    _id = SensorId(SensorType::VARIOMETER, 10); // prio right after the te_alt doRead filter

    // the post process organizing it's own NVS variable and filter
    setNVSVar(&te_vario);

    // preparation for a disruptive jump to the ground level
    _prepare_jump = 10;
    _reading_ms = Clock::getMillis();
}

bool VarioFilter::setup() {
    float alt = altitude.get();
    ESP_LOGI(FNAME, "VarioFilter setup with alt %f", alt);
    Altitude = alt;
    predictAlt = alt;

    // set averager time span
    startRunningAvg(vario_av_delay.get() * 1000);
    _Gact.setTau(vario_av_delay.get()); // same integration time for the climb score
    _Goptimal.setTau(vario_av_delay.get());
    return true;
}


// the total energy compensated altitude of the selected TE compensation method
std::optional<meter_t> VarioFilter::teAltitude(bool te_probe) const
{
    if (te_probe) {
        bool success;
        return teSensor->readAltitudeISA(success);
    }
    meter_t alt = altitude_isa.get(); // already read
    if (!altitude_isa.getValid() || std::isnan(alt)) {
        return std::nullopt;
    }
    // method 1
    mps_t ta_speed = tas.get();  // m/s
    alt += ((ta_speed * ta_speed) / (2.f * Units::g0)) * te_comp_adjust.get() / 100.0f; // Ekin ~ h = v²/2g  * adjust
    // method 2
    pascal_t barP = baroSensor->getHead();
    pascal_t dynP = asSensor->getHead();
    alt += Units::calcAltitudeISA(barP - (dynP * te_comp_adjust.get() / 100.0f));  // subtract PI pressure like TEK probe does
    return alt / 2.f; // simple average of both methods
}

void VarioFilter::filterAltitude(meter_t curr_altitude, second_t dt)
{
    static float _errorval = 1.6;

    const meter_t prev_altitude = Altitude;
    meter_t adiff = curr_altitude - Altitude;
    meter_t diff = (abs(adiff) * 1000) + 1;
    if (diff > 1000000 || _prepare_jump) {  // more than 100 m altitude diff in 0.1 second not plausible ( > 400 km/h vertical ) -> handled by Kalman filter
        ESP_LOGW(FNAME, "TE sensor delta OOB: %f m", diff / 10000);
        // reset filter state
        Altitude = curr_altitude;
        diff = adiff = 0.f;
        if (_prepare_jump > 0) _prepare_jump--;
    }
    float err = (abs(curr_altitude - predictAlt) * 1000) + 1;
    float kg = (diff / (err * _errorval + diff)) * gainPerDt(0.2f, dt);
    Altitude += (adiff)*kg;
    _te_rate = (Altitude - prev_altitude) / dt;
}

void VarioFilter::postProcess()
{
    const bool te_probe = !te_comp_enable.get();
    const PressureSensor* source = te_probe ? teSensor : baroSensor;
    if (!source) {
        return;
    }
    const int now = Clock::getMillis();
    if ((int32_t)(now - _source_clock.time_ms) < 0) {
        _source_clock = {}; // the replay clock went back
    }
    const bool fresh = _source_clock.observe(now, source->getLastUpdateTimeMs(), true);
    const bool reading = fresh || (unsigned)(now - _reading_ms) >= STALE_MS;
    second_t reading_dt = 0.f;
    if (reading) {
        const int stamp = fresh ? (int)_source_clock.time_ms : now;
        reading_dt = std::clamp(stamp - _reading_ms, 20, 500) / 1000.0f;
        _reading_ms = stamp;
        const std::optional<meter_t> alt = teAltitude(te_probe);
        if (!alt) {
            return;
        }
        filterAltitude(*alt, reading_dt);
    }

    // the needle moves every cycle, between the readings towards the rate of the last one
    _te_damper.setDamping(vario_delay.get());
    const mps_t te_gr = _te_damper.filter(_te_rate);
    if (reading) {
        predictAlt = Altitude + (_te_damper.lowPass() * reading_dt);
    }
    _got_positive = te_gr > .0f && te_vario.get() < .0f;

    _polar_sink = Speed2Fly.getSink(ias.get());
    mps_t te_ne = te_gr - _polar_sink;

    // every second cycle
    const bool slow = _slow_timer.due(now, SLOW_CYCLE_MS * 3 / 4);

    // Adjust the net for Super Netto mode if applicable
    if (CRMOD.getVMode() == CruiseMode::MODE_REL_NETTO) {
        // Super Netto, considering circling sink
        _polar_sink += Speed2Fly.getCirclingSink();
    }
    te_netto.set(te_ne);

    if ( CRMOD.getChanged() ) {
        restartRunningAvg(); // restart the running average when the cruise mode changes, avoid smeared data
        ESP_LOGI(FNAME, "Cruise mode changed, restarting running average");
    }
    if (CRMOD.isGross()) {
        publish(te_gr, now, slow); // damp
        te_gross.set(get());                // use damped value
        te_netto.set(get() - _polar_sink);
    } else {
        publish(te_ne, now, slow);
        te_gross.set(get() + _polar_sink);
        te_netto.set(get());
    }
    _avg_vario.filter(getRunningAvg());

    if (slow) {
        // calc the climb score
        if ( te_ne > 0.f && airborne.get()) {
            // achieved gross climb integral
            _Gact.filter(std::max(te_gr, 0.f));
            // Gmax​(N)=N−S(vopt​)); max. gross achievable integral with current load and net vario
            _Goptimal.filter(std::max(te_ne + Speed2Fly.getMinsink(), .1f));
        }
        else {
            _Gact.filter(.0f);
            _Goptimal.filter(.1f);
        }
        // thermal performance : actual gross / max. achievable gross
        float tp = _Gact.get() / _Goptimal.get();
        thermal_score.set(tp);

        // speed to fly update on current vario
        s2f_ideal.set(Speed2Fly.calculate(te_ne, !CRMOD.getCMode()));

        AverageVario::newSample(te_gr);
    }
}
