/*
 * S2F.cpp
 *
 *  Created on: Dec 26, 2018
 *      Author: iltis
 */

#include "S2F.h"

#include "glider/Polars.h"
#include "math/Floats.h"
#include "math/Units.h"
#include "sensor/imu/AccMPU6050.h"
#include "comm/DeviceMgr.h"
#include "protocol/NMEA.h"
#include "Flap.h"
#include "setup/SetupNG.h"
#include "sensor.h"
#include "logdefnone.h"

#include <cmath>
#include <algorithm>

S2F Speed2Fly;

float S2F::bal_percent = 0.f;

void S2F::begin(){
	if( empty_weight.get() == 0 )
		empty_weight.set( (polar_wingload.get() * polar_wingarea.get()) - 80.0 );
	calculateOverweight();
	recalculatePolar();
	changeBallast();
    changeDamping();
}

void S2F::changePolar(){
	recalculatePolar();
	recalcSinkNSpeeds();
}

void S2F::changeBallast()
{
	ESP_LOGI(FNAME,"S2F::change_ballast()" );
	[[maybe_unused]] float refw = polar_wingload.get() * polar_wingarea.get();
	ESP_LOGI(FNAME,"Reference weight: %.1f kg", refw);
	ESP_LOGI(FNAME,"Empty weight    : %.1f kg", empty_weight.get());
	ESP_LOGI(FNAME,"Crew weight     : %.1f kg", crew_weight.get());
	ESP_LOGI(FNAME,"Water Ballast   : %.1f kg", ballast_kg.get());
	ESP_LOGI(FNAME,"Gross weight    : %.1f kg", gross_weight.get());
	float max_bal = polar_max_ballast.get();
	if ( (int)(max_bal) == 0 ) { // We use 100 liters as default once its not with the polar
		max_bal = 100.f;
	}
	ESP_LOGI(FNAME,"Max ballast %.1f", max_bal );
	calculateOverweight();
	recalculatePolar();
	recalcSinkNSpeeds();
	if ( FLAP ) {
		FLAP->prepLevels();
	}
}
void S2F::changeMc()
{
	ESP_LOGI(FNAME,"S2F::change_mc(), MC: %.1f", MC.get() );
	recalcSinkNSpeeds();
}
void S2F::changeDamping() {
    ESP_LOGI(FNAME,"S2F::change_damping()" );
    _lpf_delta.setTau(s2f_delay.get(), 0.1f); // 10 Hz update
}

void S2F::setPolar()
{
	ESP_LOGI(FNAME,"S2F::setPolar()");
	t_polar p = Polars::getPolar(MyGliderPolarIndex);
	polar_speed1.set( p.speed1 );
	polar_speed2.set( p.speed2 );
	polar_speed3.set( p.speed3 );
	polar_sink1.set( p.sink1 );
	polar_sink2.set( p.sink2 );
	polar_sink3.set( p.sink3 );
	polar_wingload.set( p.wingload );
	// set default min speed as estimated stall_speed * 1.05 )
	// Vstall := sqrt( (2 * W/S * g) / ( rho * Clmax ) ) [m/s]
	_stall_speed = std::sqrtf( ( 2.f * polar_wingload.get() * Units::g0) / (Units::rho0 * 1.4f ) ) * 1.05f;
	polar_stall_speed.set(Units::mps_to_kmh(_stall_speed));
	ESP_LOGI(FNAME,"Estimated stall speed: %.1f km/h", polar_stall_speed.get());
	polar_max_ballast.set( p.max_ballast );
	polar_wingarea.set( p.wingarea, true, false );
	empty_weight.set( (p.wingload * p.wingarea) - 80.0, true, false ); // Calculate default for emtpy mass
	ProtocolItf *prtcl = DEVMAN->getProtocol(NAVI_DEV, XCVARIO_P);
	if ( prtcl ) {
		(static_cast<NmeaPrtcl*>(prtcl))->sendXCVEmptyWeight(empty_weight.get());
	}
	ESP_LOGI(FNAME,"Reference weight:%.1f, new empty_weight: %.1f", (p.wingload * p.wingarea), empty_weight.get() );
	changePolar();
}

// compare the used polar two the original one from polar store
bool S2F::isPolarEqualTo(int idx)
{
    t_polar p1 = Polars::getPolar(idx);

    ESP_LOGI(FNAME, "IDX %d", idx);
    ESP_LOGI(FNAME, "(polar_speed1.get(), p1.speed1) %f %f", polar_speed1.get(), p1.speed1);
    ESP_LOGI(FNAME, "(polar_sink1.get(), p1.sink1) %f %f", polar_sink1.get(), p1.sink1);
    ESP_LOGI(FNAME, "(polar_speed2.get(), p1.speed2) %f %f", polar_speed2.get(), p1.speed2);
    ESP_LOGI(FNAME, "(polar_sink2.get(), p1.sink2) %f %f", polar_sink2.get(), p1.sink2);
    ESP_LOGI(FNAME, "(polar_speed3.get(), p1.speed3) %f %f", polar_speed3.get(), p1.speed3);
    ESP_LOGI(FNAME, "(polar_sink3.get(), p1.sink3) %f %f", polar_sink3.get(), p1.sink3);
    ESP_LOGI(FNAME, "(polar_wingload.get(), p1.wingload) %f %f", polar_wingload.get(), p1.wingload);
    ESP_LOGI(FNAME, "(polar_max_ballast.get(), p1.max_ballast) %f %f", polar_max_ballast.get(), p1.max_ballast);
    ESP_LOGI(FNAME, "(polar_wingarea.get(), p1.wingarea) %f %f", polar_wingarea.get(), p1.wingarea);

    if ( floatEqualFast(polar_speed1.get(), p1.speed1) 
        && floatEqualFast(polar_sink1.get(), p1.sink1) 
        && floatEqualFast(polar_speed2.get(), p1.speed2) 
        && floatEqualFast(polar_sink2.get(), p1.sink2) 
        && floatEqualFast(polar_speed3.get(), p1.speed3) 
        && floatEqualFast(polar_sink3.get(), p1.sink3) 
        && floatEqualFast(polar_wingload.get(), p1.wingload) 
        && floatEqualFast(polar_max_ballast.get(), p1.max_ballast) 
        && floatEqualFast(polar_wingarea.get(), p1.wingarea) )
    {
        return true;
    }
    return false;
}

// call with 10 Hz rate after sensor read
mps_t S2F::calculate(mps_t netto_vario, bool circling)
{
    if (circling) {
        return _circling_speed; // std::clamp(_circling_speed, _min_sink_speed, v_max.get());
    }

    // // ---------- 2. Validate polar ----------
    // if (!std::isfinite(a0) || !std::isfinite(a2) || a2 <= 0.0f) {
    //     return _min_sink_speed;
    // }
    mps_t climb = a0 - MC.get();
    // ESP_LOGI(FNAME, "S2F::calculate() a0: %.3f a2: %.6f MC: %.3f climb: %.3f", a0, a2, MC.get(), climb);

    if (!s2f_blockspeed.get()) {
        climb += netto_vario;
        // ESP_LOGI(FNAME, "S2F::calculate() netto_vario: %.3f adjusted climb: %.3f", netto_vario, climb);
    }

    // radicand check
    float arg = climb / a2;
    if (!std::isfinite(arg) || arg <= 0.0f) {
        ESP_LOGI(FNAME, "S2F::calculate() invalid radicand: %.6f", arg);
        return _min_sink_speed;
    }

    mps_t stf = std::clamp(std::sqrtf(arg), _min_sink_speed, v_max.get() / 3.6f);
    ESP_LOGI(FNAME, "S2F::calculate()  %.2f", stf * 3.6f);

    _s2f_delta = _lpf_delta.filter(stf - ias.get());

    return stf;
}

// v_in : [m/s]
mps_t S2F::sink( mps_t v_in ) {
	float v_stall = _stall_speed * 0.9;
	if ( v_in < v_stall || !_valid ){
		// ESP_LOGI(FNAME,"S2F::sink, warning, airspeed %.1f below minimum speed %.1f km/h", v_in, v_stall );
		return 0.0;
	}
	float n=getN();
	float sqn = std::sqrtf(n);
	mps_t s = a0*n*sqn + a1*v_in*n + a2*v_in*v_in*sqn;
	// ESP_LOGI(FNAME,"S2F::sink() V:%0.1f sink:%2.2f G-Load:%1.2f", v_in, s, n );
	return s;
}

mps_t S2F::circlingSink(mps_t v) {
    if (v > _stall_speed * 0.6)
        return _circling_sink;
    else
        return 0;
}


// v : [m/s]
float S2F::cw( mps_t v ) {
	float cw = 0;
	if( v > 14.0 ) {
		mps_t cur_sink = sink(v);
		// ESP_LOGI(FNAME,"S2F::cw( %0.1f ) sink: %2.1f cw. %2.2f  G: %1.1f", v, sink, cw, getN() );
		cw = cur_sink / v;
	}
	return cw;
}


////////////////////////////////////////////////////////////////////
// Private helper
////////////////////////////////////////////////////////////////////


void S2F::recalculatePolar()
{
	ESP_LOGI(FNAME, "S2F::recalculatePolar() bugs: %f ", bugs.get());
	float v1 = polar_speed1.get() / 3.6;
	float v2 = polar_speed2.get() / 3.6;
	float v3 = polar_speed3.get() / 3.6;
	float w1 = polar_sink1.get();
	float w2 = polar_sink2.get();
	float w3 = polar_sink3.get();
	ESP_LOGI(FNAME, "v1/s1 %.1f/%.3f", v1 * 3.6, w1);
	ESP_LOGI(FNAME, "v2/s2 %.1f/%.3f", v2 * 3.6, w2);
	ESP_LOGI(FNAME, "v3/s3 %.1f/%.3f", v3 * 3.6, w3);
	// w= a0 + a1*v + a2*v^2   from ilec
	// w=  c +  b*v +  a*v^2   from wiki
	float d = v1 * v1 * (v2 - v3) + v2 * v2 * (v3 - v1) + v3 * v3 * (v1 - v2);
	a2 = d == 0. ? 0. : ((v2 - v3) * (w1 - w3) + (v3 - v1) * (w2 - w3)) / d;
	d = v2 - v3;
	a1 = d == 0. ? 0. : (w2 - w3 - a2 * (v2 * v2 - v3 * v3)) / d;
	a0 = w3 - a2 * v3 * v3 - a1 * v3;
	const float loading_factor = std::sqrtf((myballast + 100.0) / 100.0);
	a0 = a0 * loading_factor;
	a2 = a2 / loading_factor; // wingload  e.g. 100l @ 500 kg = 1.2 and G-Force
	a0 = a0 * ((bugs.get() + 100.0) / 100.0);
	a1 = a1 * ((bugs.get() + 100.0) / 100.0);
	a2 = a2 * ((bugs.get() + 100.0) / 100.0);
    _valid = calcValidPolar();
	ESP_LOGI(FNAME, "bugs:%d balo:%.1f%% a0=%f a1=%f  a2=%f s(80)=%f, s(160)=%f", (int)bugs.get(), myballast, a0, a1, a2, sink(80), sink(160));
}

void S2F::calculateOverweight()
{
	ESP_LOGI(FNAME,"S2F::calculateOverweight()" );
	gross_weight.set( empty_weight.get() + crew_weight.get() + ballast_kg.get() );
	myballast = ( ((100*gross_weight.get()) / (polar_wingload.get()*polar_wingarea.get())) - 100.0 );
	ESP_LOGI(FNAME,"New ballast overweight: %.2f %%", myballast );
	ballast.set( myballast );
}

// minimum sink, circling sink, best circling speed
void S2F::recalcSinkNSpeeds() {
    if (!_valid) {
        _min_sink_speed = _min_sink = _circling_speed = _circling_sink = 0.;
        return;
    }
    // 2*a2*v + a1 = 0
    _min_sink_speed = -a1 / (2 * a2);
    _min_sink = sink(_min_sink_speed);
    _circling_speed = 1.2 * _min_sink_speed; // fixme calculate to current g-load
    _circling_sink = sink(_circling_speed);
    // use user defined/confirmed stall speed
    const float loading_factor = std::sqrtf((myballast + 100.0) / 100.0);
    _stall_speed = Units::kmh_to_mps(polar_stall_speed.get()) * std::sqrtf(loading_factor);

    ESP_LOGI(FNAME, "Airspeed @ min Sink =%3.1f kmh", Units::mps_to_kmh(_min_sink_speed));
    ESP_LOGI(FNAME, "          min Sink  =%2.3f m/s", _min_sink);
    ESP_LOGI(FNAME, "Circling Speed      =%3.1f kmh", Units::mps_to_kmh(_circling_speed));
    ESP_LOGI(FNAME, "Stall    Speed      =%2.3f km/h", polar_stall_speed.get());
    ESP_LOGI(FNAME, "Stall warn @        =%2.3f", Units::mps_to_kmh(_stall_speed));
}

float S2F::getBallastPercent() {
    return bal_percent;
}

float S2F::getN() {
	float g = 1.0;
	if ( accSensor ) {
		g = accSensor->getGload();
	}
	if( g < 0.3 ) // Polars and airfoils physics behave different negative or even low g forces, we stop here impacting from g force at 0.3 g
		return 0.3;
	return g;
}

float S2F::getVn( float v ){
	float Vn = v*std::powf(getN(),0.5);
	if( Vn > _stall_speed )
		return Vn;
	else
		return _stall_speed;
}

bool S2F::calcValidPolar() {
    return (a2 < 0 && a1 > 0 && a0 < 0);
}

#ifdef S2F_Test

void S2F::test( void )
{
	ESP_LOGI(FNAME, "Minimal Sink @ %f km/h", minsink_speed());
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink(   0.0 / 3.6 ), "0");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink(  20.0 / 3.6 ), "20");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink(  40.0 / 3.6 ), "40");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink(  80.0 / 3.6 ), "80");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 100.0 / 3.6 ), "100");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 120.0 / 3.6 ), "120");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 150.0 / 3.6 ), "150");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 180.0 / 3.6 ), "180");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 220.0 / 3.6 ), "220");
	ESP_LOGI(FNAME,"MC %f  Ballast %f", MC.get(), myballast );
	for( float st=2; st >= -2; st-=0.5 )
	{
		ESP_LOGI(FNAME, "S2F %0.1f km/h vario %.1f m/s", calculate(st) * 3.6, st );
	}
}

#endif
