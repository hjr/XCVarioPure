#pragma once

#include "average.h"
#include "S2F.h"

#include <driver/gpio.h>


/*
     Implementation of a stable Vario with flight optimized Iltis-Kalman filter

 */

class PressureSensor;

// #define SPS 10                   // samples per second
constexpr const int FILTER_LEN = 34; // Max Filter length
constexpr const float ALPHA = 0.2; // Kalman Gain alpha
#define ERRORVAL 1.6             // damping Factor for values off the weeds
constexpr const float STANDARD = 1013.25; // ICAO standard pressure


class VarioFilter {
public:
	VarioFilter() {
		_qnh = STANDARD;
		index = 0;
		_alpha = ALPHA;
		predictAlt = 0;
		Altitude = 0;
		lastAltitude = 0;
		_errorval = ERRORVAL;
		_TEF = 0;
		_avgTE = 0;
		averageAlt = 0;
		_damping_factor = 1.0;
		_analog_adj = 0;
		N = 0;
	}

	void setQNH( float qnh ) { _qnh = qnh; };
	void setAveragerTime(float t) { avgTE.setLength(t); };
	void setup();

	float readTE(float tas, float tePressure);   // get TE value im m/s
	float readAVGTE();   // get TE value im m/s
	// float  readS2FTE();   // get TE value im m/s for S2F
	// float readAVGalt() { return averageAlt; };    // get average Altitude
	// float readCuralt() { return _currentAlt; };   // get current Altitude
	void configChange();

private:
	float _alpha;
	float _errorval;
	float _qnh;
	float predictAlt;
	float Altitude;
	float lastAltitude;
	float averageAlt;
	Average<FILTER_LEN, float, float> TEavg;
	float _analog_adj;
	int    index;
	float _TEF;
	Average<60, float, float> avgTE;
	float _avgTE;
	float _damping_factor;
	int N;
};

extern VarioFilter bmpVario;
