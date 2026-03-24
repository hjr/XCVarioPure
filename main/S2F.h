/*
 * S2F.h
 *
 *  Created on: Dec 26, 2018
 *      Author: iltis
 */

#pragma once

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
	void setPolar();
	static bool isPolarEqualTo(int idx);
	mps_t calculate(mps_t net_vario, bool circling=false ); // call after sensor reads 10 Hz
	mps_t getDelta() const { return _s2f_delta; }
	mps_t getStallSpeed() const { return _stall_speed; }
	mps_t getSink( mps_t v );
	mps_t getMinsinkSpeed() { return _min_sink_speed; };
	mps_t getCirclingSink(mps_t v);
	float getCw( mps_t v );
	static float getLoadFactor();
	void test(void);

private:
	void recalculatePolar();
	void calculateOverweight();
	void recalcSinkNSpeeds();
	static float getBallastPercent();
	float getVn( float v );
	bool calcValidPolar();

	float myballast = 1.f;
	static float bal_percent;
	float a0=0, a1=0, a2=0;
	float w0=0, w1=0, w2=0;
	mps_t _min_sink_speed = 0.f;
	mps_t _min_sink = 0.f;
	mps_t _circling_speed = 0.f;
	mps_t _circling_sink = 0.f;
	mps_t _stall_speed = 0.f;
	mps_t _s2f_delta = 0.f;
	LowPassFilterT<float> _lpf_s2f{0.1f};
	bool _valid = false;
};

extern S2F Speed2Fly;
