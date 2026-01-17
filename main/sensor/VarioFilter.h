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
constexpr const double ALPHA = 0.2; // Kalman Gain alpha
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

	double readTE(float tas, float tePressure);   // get TE value im m/s
	double readAVGTE();   // get TE value im m/s
	// float  readS2FTE();   // get TE value im m/s for S2F
	// double readAVGalt() { return averageAlt; };    // get average Altitude
	// double readCuralt() { return _currentAlt; };   // get current Altitude
	void configChange();

private:
	double _alpha;
	double _errorval;
	float _qnh;
	double predictAlt;
	double Altitude;
	double lastAltitude;
	double averageAlt;
	Average<FILTER_LEN, double, double> TEavg;
	double _analog_adj;
	int    index;
	double _TEF;
	Average<60, float, float> avgTE;
	double _avgTE;
	double _damping_factor;
	int N;
};

extern VarioFilter bmpVario;
