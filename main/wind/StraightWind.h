/*
 * StraightWind.h
 *
 *  Created on: Mar 21, 2021
 *
 *  Author: Eckhard Völlm, Axel Pauli
 *
 *  Last update: 2021-04-18
 */
#pragma once

#include "vector.h"
#include "math/Units.h"

#include <sys/time.h>
#include <list>

// #define Wind_Test 1

class StraightWind
{
public:
	StraightWind();
	virtual ~StraightWind() {};

	void begin();

	void tick();

	// Measurement cycle for wind calculation in straight flight. Should be
	// triggered periodically, maybe once per second.
	// Returns true, if a new wind was calculated.
	bool calculateWind();

	// Return the last calculated wind. If return result is true, the wind data
	// are valid otherwise false.
	static bool getWind(int16_t *direction, mps_t *speed);

	void setWind( float direction, mps_t speed ){
		windDir = direction;
		windSpeed = speed;
		_age = 0;
	}

	void calculateWind( float tc, mps_t gs, float th, mps_t tas, float deviation  );
	static void calculateSpeedAndAngle( float angle1, mps_t speed1, float angle2, mps_t speed2, mps_t& speed, float& angle );
	void newCirclingWind( float angle, mps_t speed );
	void test();
	int getAge() { return _age; }
	static void resetAge() { _age = 0; }
	float getAsCorrection() { return airspeedCorrection; }
	float getAngle();
	float getSpeed();
	float getDeviation() { return deviation_cur; }
	bool  getGpsStatus() { return gpsStatus; }
	float getMH() { return magneticHeading; }
	const char *getStatus() { return status; }

private:
	mps_t averageTas;         // TAS
	float averageTH;          // sum of Compass true heading
	float averageTC;          // sum of GPS heading (true course)
	mps_t averageGS;		  // average ground speed
	float windDir;            // calculated wind direction
	mps_t windSpeed;          // calculated wind speed
	bool   lowAirspeed;
	float  circlingWindDir;
	float  circlingWindDirReverse;
	mps_t  circlingWindSpeed;
	int    circlingWindAge;
	float  airspeedCorrection;
	static int16_t _age;
	int    _tick;
	bool   gpsStatus;
	float  deviation_cur;
	float  magneticHeading;
	const char *status;
	float  jitter;
	std::list<Vector> windVectors;
	mps_t newWindSpeed;
	float newWindDir;
	float slipAverage;
	float lastHeading;
	float lastGroundCourse;
};

extern StraightWind *straightWind;
