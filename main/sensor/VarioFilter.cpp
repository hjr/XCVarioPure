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
#include "Atmosphere.h"
#include "math/Floats.h"
#include "S2F.h"
#include "AverageVario.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"


#include <cmath>
#include <algorithm>


VarioFilter bmpVario; // static instance of it

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
static mps_t vario_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ]; // history buffer for airspeed sensor

// Data and dtructures for different filter variants
static meter_t averageAlt = 0.f;
static meter_t Altitude = 0.f;
static meter_t predictAlt = 0.f;
static meter_t lastAltitude = 0.f;

void VarioFilter::init(meter_t alt)
{
    averageAlt = alt;
    Altitude = alt;
    predictAlt = alt;
    lastAltitude = alt;
    _tealt_lpf.reset(alt);
}

#if FILTER == 3
struct VarioKF {
    // State
    meter_t h;  // altitude [m]
    mps_t   v;  // vertical speed [m/s]

    // Covariance
    float P00, P01, P10, P11;

    // Noise
    float R;     // measurement noise
    float sigma_a;

    void init(meter_t h0) {
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
        ESP_LOGI(FNAME, "VKF(%.3f/%.3f/%.3f/%.3f): K(%.3f,%.3f) pre: %.3f err:%f R:%.3f up: %.3f", P00, P01, P10, P11, K0, K1, h, y, R, v);
    }
};

static VarioKF vkf;
#endif

VarioFilter::VarioFilter() :
    SensorTP<float>(vario_buffer, DUTY_CYCLE_MS),
    _tealt_lpf(0.25f)
{
    _id = SensorId::VARIOMETER;
    setNVSVar(&te_alt);
    setFilter(&_tealt_lpf);
}
// ~VarioFilter() {} .. never going to be deleted
    
void VarioFilter::configChange() {
    // vario needle damping
    _lpf.setTau(vario_delay.get(), 0.1f); // 10 Hz
#if FILTER == 3
    vkf.setTau(vario_delay.get()); // KF
#endif
    // vario averager damping
    _avg_filter_idx = (vario_av_delay.get() / 0.1) - 1;
    ESP_LOGI(FNAME, "configChange damping:%f filter_len:%d", _lpf.getAlpha(), _avg_filter_idx);
#if FILTER == 0
    avgTE.setLength( vario_av_delay.get() );
#endif
#if FILTER == 0 || FILTER == 1
    TEavg.setLength( rint(vario_delay.get()*(10.0/3)) );
#endif
}

#if FILTER == 3
void VarioFilter::resetKF() {
    meter_t h0 = altitude_isa.get();
    vkf.init(h0); // KF
    _filter->reset(h0);
}
#endif

bool VarioFilter::setup() {
    // Decide if sensor readings come from local sensor or master (ctor gets called too early for this)
    if (SetupCommon::isMaster()) {
        _id = _id | SensorFlags::SENSOR_LOCAL;
        // mark as essential sensor to be able to simulate
        _id = _id | SensorFlags::SENSOR_ESSENTIAL;
    }

    ESP_LOGI(FNAME, "VarioFilter setup as %s sensor with alt %f", (isLocalSensor(_id) ? "local" : "remote"), altitude.get());
    init(altitude_isa.get());
    configChange();
#if FILTER == 3
    resetKF(); // reset the te_alt filter before entering the sensor loop
#endif
    _prev_time = Clock::getMillis();
    return true;
}

// call in main loop every 100 ms
// extract from real existing sensors according to the selected TE compensation method
// a total energy compensated altitude.
bool VarioFilter::doRead(float& val) {
    float curr_altitude;
    if (te_comp_enable.get() == TE_TEK_EPOT) {
        curr_altitude = altitude_isa.get();  // already read
        if (!altitude_isa.getValid() || std::isnan(curr_altitude)) {
            curr_altitude = getHead();  // ignore readout when failed
        }
        mps_t ta_speed = tas.get();  // m/s
        float cw = Speed2Fly.getCw(ta_speed);
        float ealt = ((ta_speed * ta_speed) / (2 * Units::g0)) * (1 + (te_comp_adjust.get() / 100.0)) * (1 - cw);  // Ekin ~ h = v²/2g  * adjust * (1-cw)
        curr_altitude += ealt;
        ESP_LOGD(FNAME, "Energy Alt @%0.1f km/h: %0.1f cw: %f", tas.get(), ealt, cw);
    } else if (te_comp_enable.get() == TE_TEK_PRESSURE) {
        pascal_t barP = baroSensor->getHead();
        pascal_t dynP = asSensor->getHead();
        curr_altitude = Atmosphere::calcAltitudeISA((barP - dynP) * (1 + (te_comp_adjust.get() / 100.0)));  // subtract PI pressure like TEK probe does
        // ESP_LOGI(FNAME,"TE alt: %4.3f m, ST: %.1f PI: %.1f", _currentAlt, barP, (dynP*100) );
    } else {  // TE_TEK_PROBE
        bool success;
        curr_altitude = teSensor->readAltitudeISA(success);
    }

    val = curr_altitude;
    // linear prediction and innovation gating
    // const float max_10thsec_step = 8.0f;  // max 80 m/s vertical speed
    // if (accept(curr_altitude, max_10thsec_step) || _prepare_sim_jump) {
    //     val = curr_altitude;
    //     // ESP_LOGI(FNAME, "VarioFilter: accepted alt %f", curr_altitude);
    // } else {
    //     float predicted = predict();
    //     val = curr_altitude + max_10thsec_step * ((predicted > curr_altitude) ? 1.0f : -1.0f);
    //     ESP_LOGW(FNAME, "VarioFilter: rejected alt %f, predicted %0.2f", curr_altitude, predicted);
    // }
    return true;
}

// apply filters individually as desired from settings

#if defined(FILTER) && FILTER == 0
// a legacy copy
void VarioFilter::postProcess() {
    static float _errorval = 1.6;
    static mps_t _TEF = 0.f;
    static int N = 0;

    N++;
	// ESP_LOGI(FNAME,"TE alt: %4.3f m, ST: %.1f PI: %.1f", _currentAlt, barP, (dynP*100) );
    meter_t curr_altitude = getHead();
	averageAlt += (curr_altitude - averageAlt) * 0.1;
	meter_t adiff = curr_altitude - Altitude;
	// ESP_LOGI(FNAME,"VarioFilter new alt %0.1f err %0.1f", _currentAlt, err);
	meter_t diff = (abs(adiff) * 1000) + 1;
	if(diff > 1000000){  // more than 100 m altitude diff in 0.1 second not plausible ( > 400 km/h vertical ) -> handled by Kalman filter
		 ESP_LOGW(FNAME,"TE sensor delta OOB: %f m", diff/10000 );
	}
	float err = (abs(curr_altitude - predictAlt) * 1000) + 1;
	averageAlt += (curr_altitude - averageAlt) * 0.1;
	float kg = (diff / (err*_errorval + diff)) * 0.2; // 0.2 Kalman gain
	Altitude += (adiff) * kg;
	meter_t altDiff = Altitude - lastAltitude;
	// ESP_LOGI(FNAME," altDiff %0.1f diff %0.1f", TE, diff);
	lastAltitude = Altitude;
	mps_t TEAVG = TEavg( altDiff / 0.1 ); // in m/s
	predictAlt = Altitude + (TEAVG * 0.1);
	_TEF += ((TEAVG - _TEF)) / vario_delay.get();

    te_vario.set(_TEF);
    _polar_sink = Speed2Fly.getSink(ias.get());
    te_netto.set(_TEF - _polar_sink);

	if( !(N%10) ){ // every second one sample
		_avg_vario = avgTE( _TEF );
		// ESP_LOGI(FNAME," avgTE: %f ", _avg_vario);
	}
    AverageVario::newSample(_TEF);
}
#elif defined(FILTER) && FILTER == 1
// a legacy copy with some adjustments and more logging, no change in the actual filter behavior
void VarioFilter::postProcess() {
    constexpr const float errorval = 1.6;
    static mps_t _TEF = 0.f;

    meter_t curr_altitude = getHead();
	averageAlt += (curr_altitude - averageAlt) * 0.1;
	float adiff = curr_altitude - Altitude;
	// ESP_LOGI(FNAME,"VarioFilter new alt %0.1f err %0.1f", _currentAlt, err);
	float diff = (abs(adiff) * 1000) + 1;
	if(diff > 1000000){  // more than 100 m altitude diff in 0.1 second not plausible ( > 400 km/h vertical ) -> handled by Kalman filter
		 ESP_LOGW(FNAME,"TE sensor delta OOB: %f m", diff/10000 );
	}
	float err = (abs(curr_altitude - predictAlt) * 1000) + 1;
	averageAlt += (curr_altitude - averageAlt) * 0.1;
	float kg = (diff / (err*errorval + diff)) * 0.2;
	Altitude += (adiff) * kg;
	float altDiff = Altitude - lastAltitude;
	// ESP_LOGI(FNAME," altDiff %0.1f diff %0.1f", TE, diff);
	lastAltitude = Altitude;
	float TEAVG = TEavg( altDiff / 0.1 ); // in m/s
	predictAlt = Altitude + (TEAVG * 0.1);
	_TEF += ((TEAVG - _TEF)) * _lpf.getAlpha();

    te_vario.set(_TEF);
    // ESP_LOGI(FNAME, "VarioFilter ias: %f", ias.get());
    _polar_sink = Speed2Fly.getSink(ias.get());
    te_netto.set(_TEF - _polar_sink);

    // the big AVG value
    // ESP_LOGI(FNAME, "VarioFilter::postProcess history level: %d avg_idx: %d", _history.level(), _avg_filter_idx);
    if (_history.level() > _avg_filter_idx) {
        _avg_vario = (getHead() - _history[_avg_filter_idx]) * (10.f / (_avg_filter_idx + 1));  // in m/s
        // ESP_LOGI(FNAME, "VarioFilter: H:%f 1:%f avg:%f", getHead(), _history[_avg_filter_idx], avg);
    }
    AverageVario::newSample( _TEF );
}
#elif defined(FILTER) && FILTER == 3
// Kalman Filter based TE compensation, no additional LPF
void VarioFilter::postProcess() {
    uint32_t now = Clock::getMillis();
    second_t dt = 0.1f;
    if (now - _prev_time > 200) {
        dt = (now - _prev_time) / 1000.0f;
        // ESP_LOGI(FNAME, "VKF: init timing: %f", dt);
    }

    vkf.predict(dt);
    _prev_time = now;
    meter_t pred_err = getHead() - vkf.h;
    if (fabsf(pred_err) > 60.0f && _prepare_sim_jump) {
        // just re-/started sim mode, expect a time and height disruption, prepare KF for it
        meter_t h0 = getHead();
        vkf.init(h0);
        _filter->reset(h0);
        if (_prepare_sim_jump > 0) _prepare_sim_jump--;
        ESP_LOGW(FNAME, "VarioFilter SIM: large pred_err %f, re-init KF", pred_err);
        return;
    }
    vkf.R = 0.25 * (1 + fabsf(pred_err));
    vkf.R = std::clamp(vkf.R, 0.05f, 1.0f);

    vkf.update(getHead());
    // if ( fabs( vkf.v ) > 20.0f )
        // ESP_LOGI(FNAME, "VKF(%.3f/%.3f/%.3f/%.3f): pre: %.3f inn:%f R:%.3f up: %.3f", vkf.P00, vkf.P01, vkf.P10, vkf. P11, vkf.h, pred_err, vkf.R, vkf.v);
    // if (fabsf(pred_err) < 6.0f) {
        te_vario.set(vkf.v);
        _polar_sink = Speed2Fly.getSink(ias.get());
        te_netto.set(vkf.v - _polar_sink);
    // }

    if (_history.level() > _avg_filter_idx) {
        _avg_vario = (getHead() - _history[_avg_filter_idx]) * (10.f / (_avg_filter_idx + 1));  // in m/s
        // ESP_LOGI(FNAME, "VarioFilter: H:%f 1:%f avg:%f", getHead(), _history[_avg_filter_idx], avg);
    }
    AverageVario::newSample(vkf.v); // longer term thermal average
}
#endif
