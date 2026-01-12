/*
 * S2F.cpp
 *
 *  Created on: Dec 26, 2018
 *      Author: iltis
 */

#include "S2F.h"

#include "glider/Polars.h"
#include "math/Floats.h"
#include "sensor/imu/KalmanMPU6050.h"
#include "comm/DeviceMgr.h"
#include "protocol/NMEA.h"
#include "Flap.h"
#include "setup/SetupNG.h"
#include "sensor.h"
#include "logdefnone.h"

#include <cmath>
#include <algorithm>

S2F::S2F() {
	a0=a1=a2=0;
	w0=w1=w2=0;
	_min_sink_speed = 0;
	_circling_speed = 0;
	_circling_sink = 0;
	_min_sink = 0;
	_stall_speed_ms = 0;
	myballast = 1.0;
}

S2F::~S2F() {
}

float S2F::getN() {
	float g = IMU::getGliderAccelZ();
	if( g < 0.3 )        // Polars and airfoils physics behave different negative or even low g forces, we stop here impacting from g force at 0.3 g
		return 0.3;
	return g;
}

void S2F::begin(){
	if( empty_weight.get() == 0 )
		empty_weight.set( (polar_wingload.get() * polar_wingarea.get())- 80.0 );
	calculateOverweight();
	recalculatePolar();
	change_ballast();
}

void S2F::modifyPolar(){
	recalculatePolar();
	recalcSinkNSpeeds();
}

bool S2F::IsValid() const
{
	return (a2 < 0 && a1 > 0 && a0 < 0);
}

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
	ESP_LOGI(FNAME, "bugs:%d balo:%.1f%% a0=%f a1=%f  a2=%f s(80)=%f, s(160)=%f", (int)bugs.get(), myballast, a0, a1, a2, sink(80), sink(160));
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
	float stall_speed = std::sqrtf( ( 2.f * polar_wingload.get() * 9.81f) / (1.225f * 1.4f ) ) * 1.05f;
	_stall_speed_ms = stall_speed;
	polar_stall_speed.set(stall_speed * 3.6);
	polar_max_ballast.set( p.max_ballast );
	polar_wingarea.set( p.wingarea, true, false );
	empty_weight.set( (p.wingload * p.wingarea) - 80.0, true, false ); // Calculate default for emtpy mass
	ProtocolItf *prtcl = DEVMAN->getProtocol(NAVI_DEV, XCVARIO_P);
	if ( prtcl ) {
		(static_cast<NmeaPrtcl*>(prtcl))->sendXCVEmptyWeight(empty_weight.get());
	}
	ESP_LOGI(FNAME,"Reference weight:%.1f, new empty_weight: %.1f", (p.wingload * p.wingarea), empty_weight.get() );
	modifyPolar();
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

float S2F::bal_percent = 0;

float S2F::getBallastPercent(){ return bal_percent; }

void S2F::calculateOverweight()
{
	ESP_LOGI(FNAME,"S2F::calculateOverweight()" );
	gross_weight.set( empty_weight.get() + crew_weight.get() + ballast_kg.get() );
	myballast = ( ((100*gross_weight.get()) / (polar_wingload.get()*polar_wingarea.get())) - 100.0 );
	ESP_LOGI(FNAME,"New ballast overweight: %.2f %%", myballast );
	ballast.set( myballast );
}

void S2F::change_ballast()
{
	ESP_LOGI(FNAME,"S2F::change_ballast()" );
	[[maybe_unused]] float refw = polar_wingload.get() * polar_wingarea.get();
	ESP_LOGI(FNAME,"Reference weight: %.1f kg", refw);
	ESP_LOGI(FNAME,"Empty weight    : %.1f kg", empty_weight.get());
	ESP_LOGI(FNAME,"Crew weight     : %.1f kg", crew_weight.get());
	ESP_LOGI(FNAME,"Water Ballast   : %.1f kg", ballast_kg.get());
	ESP_LOGI(FNAME,"Gross weight    : %.1f kg", gross_weight.get());
	[[maybe_unused]] float max_bal = polar_max_ballast.get();
	if( (int)(polar_max_ballast.get()) == 0 ) { // We use 100 liters as default once its not with the polar
		max_bal = 100;
	}
	ESP_LOGI(FNAME,"Max ballast %.1f", max_bal );
	calculateOverweight();
	recalculatePolar();
	recalcSinkNSpeeds();
	if ( FLAP ) {
		FLAP->prepLevels();
	}
}

void S2F::change_mc()
{
	ESP_LOGI(FNAME,"S2F::change_mc(), MC: %.1f", MC.get() );
	recalcSinkNSpeeds();
}

float S2F::getVn( float v ){
	float Vn = v*std::powf(getN(),0.5);
	if( Vn > _stall_speed_ms )
		return Vn;
	else
		return _stall_speed_ms;
}

// v_in : [kmh]
float S2F::sink( float v_in ) {
	float v = v_in;
	float v_stall = _stall_speed_ms * 3.6 * 0.9;
	if ( v_in < v_stall || !IsValid() ){
		// ESP_LOGI(FNAME,"S2F::sink, warning, airspeed %.1f below minimum speed %.1f km/h", v_in, v_stall );
		return 0.0;
	}
	v = v/3.6; // airspeed in meters per second
	float n=getN();
	float sqn = std::sqrtf(n);
	float s = a0*n*sqn + a1*v*n + a2*v*v*sqn;
	// ESP_LOGI(FNAME,"S2F::sink() V:%0.1f sink:%2.2f G-Load:%1.2f", v_in, s, n );
	return s;
}

float S2F::cw( float v ){  // in m/s
	float cw = 0;
	if( v > 14.0 ) {
		float cur_sink = sink(v*3.6);
		// ESP_LOGI(FNAME,"S2F::cw( %0.1f ) sink: %2.1f cw. %2.2f  G: %1.1f", v, sink, cw, getN() );
		cw = cur_sink / v;
	}
	return cw;
}


// float S2F::speed( float netto_vario, bool circling )
// {
// 	float stf = 0;
// 	if ( circling ) {
// 		// Optimum speed for a load factor of 1.4 g what corresponds 45° angle of bank and factor 1.2 speed increase; 3.6*1.2 = 4.32
// 		// fixme, calculate to current g-load
// 		stf = _circling_speed;
// 	}else{
// 		if( s2f_blockspeed.get() )
// 			stf = 3.6*std::sqrtf( ((a0-MC.get())) / a2 );  // no netto vario, no G impact
// 		else
// 			stf = 3.6*std::sqrtf( (a0-MC.get()+netto_vario) / a2 );
// 	}
// 	// ESP_LOGI(FNAME,"speed() S2F: %f netto_vario: %f circ: %d, a0: %f, MC %f", stf, netto_vario, circling, a0, MC.get() );
// 	// ESP_LOGI(FNAME,"max speed %.1f km/h", v_max.get() );
// 	if( (stf < _min_sink_speed) || std::isnan(stf) )
// 		return _min_sink_speed;
// 	if( stf > v_max.get() || std::isinf( stf) )
// 		return v_max.get();
// 	else
// 		return stf;
// }
float S2F::speed(float netto_vario, bool circling)
{
    if (circling) {
        return _circling_speed; // std::clamp(_circling_speed, _min_sink_speed, v_max.get());
    }

    // // ---------- 2. Validate polar ----------
    // if (!std::isfinite(a0) || !std::isfinite(a2) || a2 <= 0.0f) {
    //     return _min_sink_speed;
    // }

    float climb = a0 - MC.get();

    if (!s2f_blockspeed.get()) {
        climb += netto_vario;
    }

    // radicand check
    float arg = climb / a2;
    if (!std::isfinite(arg) || arg <= 0.0f) {
        return _min_sink_speed;
    }

    float stf = 3.6f * std::sqrtf(arg);

    return std::clamp(stf, _min_sink_speed, v_max.get());
}


// minimum sink, circling sink, best circling speed
void S2F::recalcSinkNSpeeds()
{
	if (!IsValid()) {
		_min_sink_speed = _min_sink = _circling_speed = _circling_sink = 0.;
		return;
	}
	// 2*a2*v + a1 = 0
	_min_sink_speed = 3.6 * (-a1 / (2 * a2));
	_min_sink = sink( _min_sink_speed );
	_circling_speed = 1.2*_min_sink_speed;
	_circling_sink = sink( _circling_speed );
	// use user defined/confirmed stall speed
	const float loading_factor = std::sqrtf((myballast + 100.0) / 100.0);
	_stall_speed_ms = polar_stall_speed.get() / 3.6 * std::sqrtf(loading_factor);

	ESP_LOGI(FNAME,"Airspeed @ min Sink =%3.1f kmh", _min_sink_speed );
	ESP_LOGI(FNAME,"          min Sink  =%2.3f m/s", _min_sink );
	ESP_LOGI(FNAME,"Circling Speed      =%3.1f kmh", _circling_speed );
	ESP_LOGI(FNAME,"Stall    Speed      =%2.3f km/h", polar_stall_speed.get() );
	ESP_LOGI(FNAME,"Stall warn @        =%2.3f",     _stall_speed_ms * 3.6f);
}

void S2F::test( void )
{
	ESP_LOGI(FNAME, "Minimal Sink @ %f km/h", minsink_speed());
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 0.0 ), "0");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 20.0 ), "20");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 40.0 ), "40");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 80.0 ), "80");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 100.0 ), "100");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 120.0 ), "120");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 150.0 ), "150");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 180.0 ), "180");
	ESP_LOGI(FNAME, "Sink %f @ %s km/h ", sink( 220.0 ), "220");
	ESP_LOGI(FNAME,"MC %f  Ballast %f", MC.get(), myballast );
	for( int st=20; st >= -20; st-=5 )
	{
		ESP_LOGI(FNAME, "S2F %g km/h vario %g m/s", speed( (double)st/10 ), (double)st/10 );
	}
}
