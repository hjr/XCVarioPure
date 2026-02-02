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
#include "logdef.h"


#include <cmath>
#include <algorithm>

#define FILTER 3

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
}

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
        
        // settle the filter before display a first value
        for (int i = 0; i < 10; i++) {
            predict(0.1f);
            update(h0);
        }
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
        float P00_ = (1 - K0) * P00;
        float P01_ = (1 - K0) * P01;
        float P10_ = P10 - K1 * P00;
        float P11_ = P11 - K1 * P01;

        P00 = P00_;
        P01 = P01_;
        P10 = P10_;
        P11 = P11_;
    }
};

static VarioKF vkf;

VarioFilter::VarioFilter() : SensorTP<float>(vario_buffer, DUTY_CYCLE_MS) {
    _id = SensorId::VARIOMETER;
    setNVSVar(&te_alt);
    setFilter(new LowPassFilter(0.25f));
}

void VarioFilter::configChange() {
    // vario needle damping
    _te_filter_idx = fast_iroundf(vario_delay.get() * 10.0f / 3);
    _lpf.setAlpha(1.f / (vario_delay.get() * 10.f + 0.1f));
    vkf.setTau(vario_delay.get()); // KF
    // vario averager damping
    _avg_filter_idx = (vario_av_delay.get() / 0.1) - 1;
    ESP_LOGI(FNAME, "configChange damping:%f filter_len:%d", _lpf.getAlpha(), _avg_filter_idx);

    avgTE.setLength( vario_av_delay.get() );
	TEavg.setLength( rint(vario_delay.get()*(10.0/3)) );
}

bool VarioFilter::setup() {
    // Decide if sensor readings come from local sensor or master
    if (SetupCommon::isMaster()) {
        _id = _id | SensorId::LocalSensor;
        // mark as essential sensor to be able to simulate
        _id = _id | SensorId::EssentialSensor;
    }

    ESP_LOGI(FNAME, "VarioFilter setup as %s sensor with alt %f", (isLocalSensor(_id) ? "local" : "remote"), altitude.get());
    init(altitude.get());
    vkf.init(altitude.get()); // KF
    configChange();
    return true;
}

// call in main loop every 100 ms
// extract from real existing sensors according to the selected TE compensation method
// a total energy compensated altitude.
bool VarioFilter::doRead(float& val) {
    float curr_altitude;
    pascal_t qnh = QNH.get();
    if (te_comp_enable.get() == TE_TEK_EPOT) {
        curr_altitude = altitude.get();  // already read
        if (!altitude.getValid() || std::isnan(curr_altitude)) {
            curr_altitude = getHead();  // ignore readout when failed
        }
        mps_t ta_speed = tas.get();  // m/s
        float cw = Speed2Fly.cw(ta_speed);
        float ealt = ((((ta_speed * ta_speed) / 19.62) * (1 + (te_comp_adjust.get() / 100.0)))) * (1 - cw);  // Ekin ~ h = v²/2g  * adjust * (1-cw)
        curr_altitude += ealt;
        ESP_LOGD(FNAME, "Energy Alt @%0.1f km/h: %0.1f cw: %f", tas.get(), ealt, cw);
    } else if (te_comp_enable.get() == TE_TEK_PRESSURE) {
        pascal_t barP = baroSensor->getHead();
        pascal_t dynP = asSensor->getHead();
        curr_altitude = Atmosphere::calcAltitude(qnh, (barP - dynP) * (1 + (te_comp_adjust.get() / 100.0)));  // subtract PI pressure like TEK probe does
        // ESP_LOGI(FNAME,"TE alt: %4.3f m, ST: %.1f PI: %.1f", _currentAlt, barP, (dynP*100) );
    } else {  // TE_TEK_PROBE
        bool success;
        curr_altitude = teSensor->readAltitude(qnh, success);
    }

    // linear prediction and innovation gating
    const float max_10thsec_step = 8.0f;  // max 80 m/s vertical speed
    if (accept(curr_altitude, max_10thsec_step)) {
        val = curr_altitude;
        // ESP_LOGI(FNAME, "VarioFilter: accepted alt %f", curr_altitude);
    } else {
        float predicted = predict();
        val = curr_altitude + max_10thsec_step * ((predicted > curr_altitude) ? 1.0f : -1.0f);
        ESP_LOGW(FNAME, "VarioFilter: rejected alt %f, predicted %0.2f", curr_altitude, predicted);
    }
    return true;
}

// apply filters individually as desired from settings

#if defined(FILTER) && FILTER == 0
void VarioFilter::postProcess() {
    static float _errorval = 1.6;
    static mps_t _TEF = 0.f;
    static int N = 0;
    static mps_t _avgTE = 0.f;

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
	float kg = (diff / (err*_errorval + diff)) * 0.2;
	Altitude += (adiff) * kg;
	meter_t altDiff = Altitude - lastAltitude;
	// ESP_LOGI(FNAME," altDiff %0.1f diff %0.1f", TE, diff);
	lastAltitude = Altitude;
	mps_t TEAVG = TEavg( altDiff / 0.1 ); // in m/s
	predictAlt = Altitude + (TEAVG * 0.1);
	_TEF += ((TEAVG - _TEF)) * _lpf.getAlpha();

    te_vario.set(_TEF);
    _polar_sink = Speed2Fly.sink(ias.get());
    te_netto.set(_TEF - _polar_sink);

	if( !(N%10) ){ // every second one sample
		_avg_vario = avgTE( _TEF );
		// ESP_LOGI(FNAME," _avgTE: %f ", _avg_vario);
	}
    AverageVario::newSample(_TEF);
}
#elif defined(FILTER) && FILTER == 1
void VarioFilter::postProcess() {
    // TE vario calculation
    pascal_t te = 0.f;
    if (_history.level() > _te_filter_idx) {
        pascal_t tecurr = getHead();
        te = (tecurr - _history[1]) * 10.f;  // in m/s
    }
    te_vario.set(_lpf.filter(te));
    _polar_sink = Speed2Fly.sink(ias.get());
    te_netto.set(te - _polar_sink);

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
    _polar_sink = Speed2Fly.sink(ias.get());
    te_netto.set(_TEF - _polar_sink);

    // the big AVG value
    // ESP_LOGI(FNAME, "VarioFilter::postProcess history level: %d avg_idx: %d", _history.level(), _avg_filter_idx);
    if (_history.level() > _avg_filter_idx) {
        _avg_vario = (getHead() - _history[_avg_filter_idx]) * (10.f / (_avg_filter_idx + 1));  // in m/s
        // ESP_LOGI(FNAME, "VarioFilter: H:%f 1:%f avg:%f", getHead(), _history[_avg_filter_idx], avg);
    }
    AverageVario::newSample( _TEF );
}
#elif defined(FILTER) && FILTER == 2
void VarioFilter::postProcess() {
    // TE vario calculation
    mps_t te = 0.f;
    if (_history.level() > _te_filter_idx) {
        float tecurr = getHead();
        te = (tecurr - _history[1]) * 10.f;  // in m/s
    }
    te_vario.set(_lpf.filter(te));
    _polar_sink = Speed2Fly.sink(ias.get());
    te_netto.set(te - _polar_sink);

    // the big AVG value
    // ESP_LOGI(FNAME, "VarioFilter::postProcess history level: %d avg_idx: %d", _history.level(), _avg_filter_idx);
    if (_history.level() > _avg_filter_idx) {
        _avg_vario = (getHead() - _history[_avg_filter_idx]) * (10.f / (_avg_filter_idx + 1));  // in m/s
        // ESP_LOGI(FNAME, "VarioFilter: H:%f 1:%f avg:%f", getHead(), _history[_avg_filter_idx], avg);
    }
    AverageVario::newSample(_lpf.get());
}
#elif defined(FILTER) && FILTER == 3
void VarioFilter::postProcess() {
    uint32_t now = Clock::getMillis();
    second_t dt = 0.1f;
    if (now - _prev_time > 200) {
        dt = (now - _prev_time) / 1000.0f;
        ESP_LOGI(FNAME, "VKF: init timing: %f", dt);
    }
    vkf.predict(dt);
    _prev_time = now;
    meter_t innov = getHead() - vkf.h;
    // if (fabsf(innov) > 60.0f) { // rejct innovation larger than 60m/s
    //     // reject baro glitch
    //     ESP_LOGE(FNAME, "VarioFilter: large innov %f, rejecting update", innov);
    //     return;
    // }
    vkf.R = 0.25 * (1 + fabsf(innov));
    vkf.R = std::clamp(vkf.R, 0.05f, 1.0f);

    vkf.update(getHead());
    // ESP_LOGI(FNAME, "VKF: predict: %f innov:%f R:%f update: %f", vkf.h, innov, vkf.R, vkf.v);
    // if (fabsf(innov) < 6.0f) {
        te_vario.set(vkf.v);
        _polar_sink = Speed2Fly.sink(ias.get());
        te_netto.set(vkf.v - _polar_sink);
    // }

    if (_history.level() > _avg_filter_idx) {
        _avg_vario = (getHead() - _history[_avg_filter_idx]) * (10.f / (_avg_filter_idx + 1));  // in m/s
        // ESP_LOGI(FNAME, "VarioFilter: H:%f 1:%f avg:%f", getHead(), _history[_avg_filter_idx], avg);
    }
    AverageVario::newSample(vkf.v);
}
#endif

void VarioFilter::inject(float te_alt) {
    int time = Clock::getMillis();
    pushToHistory(te_alt, time);
}
