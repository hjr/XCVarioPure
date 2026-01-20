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
#include "setup/SetupNG.h"
#include "logdef.h"


#include <cmath>

VarioFilter bmpVario; // static instance of it

constexpr int DUTY_CYCLE_MS = 100; // 10Hz
static float vario_buffer[ (SENSOR_HISTORY_DURATION_MS / DUTY_CYCLE_MS) + 1 ]; // history buffer for airspeed sensor

VarioFilter::VarioFilter() : SensorTP<float>(vario_buffer, DUTY_CYCLE_MS)
{
	// Decide if sensor readings come from local sensor or master
	_id = SensorId::VARIOMETER | ((SetupCommon::isClient()) ? SensorId::ExternalSensor : SensorId::NONE);
	setNVSVar(&te_alt);
	setFilter(new LowPassFilter(0.25f));
}


void VarioFilter::configChange() {
    // vario needle damping
    _te_filter_idx = fast_iroundf(vario_delay.get() * 10.0f / 3);
    _lpf.setAlpha(1.f / (vario_delay.get() * 10.f + 0.1f));
    // vario averager damping
    _avg_filter_idx = (vario_av_delay.get() / 0.1) - 1;
    ESP_LOGI(FNAME, "configChange damping:%f filter_len:%d", _lpf.getAlpha(), _avg_filter_idx);
}

bool VarioFilter::setup() {
    configChange();
	return true;
}


// call in main loop every 100 ms
// extract from real existing sensors according to the selected TE compensation method
// a total energy compensated altitude.
bool VarioFilter::doRead(float& val) {
    float curr_altitude;
    float qnh = QNH.get();
    if (te_comp_enable.get() == TE_TEK_EPOT) {
        curr_altitude = altitude.get();  // already read
        if (!altitude.getValid() || std::isnan(curr_altitude)) {
            curr_altitude = getHead();  // ignore readout when failed
        }
        float mps = tas.get() / 3.6;  // m/s
        float cw = Speed2Fly.cw(mps);
        float ealt = ((((mps * mps) / 19.62) * (1 + (te_comp_adjust.get() / 100.0)))) * (1 - cw);  // Ekin ~ h = v²/2g  * adjust * (1-cw)
        curr_altitude += ealt;
        ESP_LOGD(FNAME, "Energy Alt @%0.1f km/h: %0.1f cw: %f", tas.get(), ealt, cw);
    } else if (te_comp_enable.get() == TE_TEK_PRESSURE) {
        float barP = baroSensor->getHead();
        float dynP = asSensor->getHead();
        curr_altitude = Atmosphere::calcAltitude(qnh, barP - (dynP / 100.0) * (1 + (te_comp_adjust.get() / 100.0)));  // subtract PI pressure like TEK probe does
        // ESP_LOGI(FNAME,"TE alt: %4.3f m, ST: %.1f PI: %.1f", _currentAlt, barP, (dynP*100) );
    } else {  // TE_TEK_PROBE
        bool success;
        curr_altitude = teSensor->readAltitude(qnh, success);
    }

    // linear prediction and innovation check
    const float max_10thsec_step = 2.0f;  // max 20 m/s vertical speed
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
void VarioFilter::postProcess() {
    // TE vario calculation
    float te = 0.f;
    if (_history.level() > _te_filter_idx) {
        float tecurr = getHead();
        te = (tecurr - _history[_te_filter_idx]) * 5.f;  // in m/s
    }
    te_vario.set(_lpf.filter(te));

    // the big AVG value
    // ESP_LOGI(FNAME, "VarioFilter::postProcess history level: %d avg_idx: %d", _history.level(), _avg_filter_idx);
    if (_history.level() > _avg_filter_idx) {
        _avg_vario = (getHead() - _history[_avg_filter_idx]) * (10.f / (_avg_filter_idx + 1));  // in m/s
        // ESP_LOGI(FNAME, "VarioFilter: H:%f 1:%f avg:%f", getHead(), _history[_avg_filter_idx], avg);
    }
}

void VarioFilter::inject(float te_alt) {
    int time = Clock::getMillis();
    pushToHistory(te_alt, time);
}
