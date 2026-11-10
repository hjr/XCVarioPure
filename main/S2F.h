/*
 * S2F.h
 *
 *  Created on: Dec 26, 2018
 *      Author: iltis
 */

#pragma once

#include "glider/PolarModel.h"
#include "math/Units.h"
#include "sensor/Filters.h"

// #define S2F_Test 1

class S2F {
public:
	S2F() = default;
	~S2F() = default;
	void begin();
	void changePolar();
	// bool IsValid() const { return _valid;};
	void changeBallast();
	void changeMc();
	void changeDamping();
	void setPolar(int gldr_idx);
	bool hasFlaps() const;
	static bool isPolarEqualTo(int idx);
	mps_t calculate(mps_t net_vario, bool circling=false ); // call after sensor reads 10 Hz
	mps_t getDelta() const { return _s2f_delta; }
	mps_t getStallSpeed() const { return _polar.v_stall; }
	mps_t getSink( mps_t v ); // at the current load factor
	const glider::Polar& getPolar() const { return _polar; } // for the AirMass estimator
	mps_t getMinsink();
	mps_t getMinsinkSpeed();
	mps_t getStaticMinsinkSpeed() { return _min_sink_speed; };
	mps_t getCirclingSink() const { return _circling_sink; }
	float getCw( mps_t v );
	static float getLoadFactor();
    int getMyGliderIdx() const { return _glider_index; }
    float getBallastFactor() const { return myballast; }
	void test(void);

private:
	void recalculatePolar();
	void calculateOverweight();
	void recalcSinkNSpeeds();
	// float getVn( float v );
	bool calcValidPolar();

    int _glider_index = 0;
	float myballast = 1.f; // classical ballast definition (once as % over the ref weight) as plain factor
	glider::Polar _polar;
	float w0=0, w1=0, w2=0;
	mps_t _min_sink_speed = 0.f;
	mps_t _min_sink = 0.f;
	mps_t _circling_speed = 0.f;
	mps_t _circling_sink = 0.f;
	mps_t _s2f_delta = 0.f;
	LowPassFilterT<float> _lpf_delta{0.1f};
};

extern S2F Speed2Fly;
