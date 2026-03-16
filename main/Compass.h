/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************

File: Compass.h

Class to handle compass data access.

Author: Axel Pauli, January 2021

Last update: 2021-03-07

 **************************************************************************/

#pragma once

#include "comm/InterfaceCtrl.h"
#include "protocol/ClockIntf.h"
#include "Deviation.h"
#include "MagnetSensor.h"
#include "math/vector_3d.h"
#include "math/Units.h"
#include "average.h"

class MagnetSensor;

class CompassSink_I;

struct float_axes {
	float x;
	float y;
	float z;
};

union bitfield_compass{
	struct {
		bool xmax_green :1;
		bool xmin_green :1;
		bool ymax_green :1;
		bool ymin_green :1;
		bool zmax_green :1;
		bool zmin_green :1;
	};
	uint8_t raw;
	bitfield_compass() = default;
	constexpr bitfield_compass(bool a, bool b, bool c, bool d, bool e, bool f)
		: xmax_green(a), xmin_green(b), ymax_green(c), ymin_green(d), zmax_green(e), zmin_green(f) {}
	constexpr bitfield_compass(const bitfield_compass& other) = default;
	constexpr bitfield_compass(const uint8_t r) : raw(r) {}
	bitfield_compass& operator = ( const bitfield_compass &other ) {
		xmax_green = other.xmax_green; xmin_green = other.xmin_green;
		ymax_green = other.ymax_green; ymin_green = other.ymin_green;
		zmax_green = other.zmax_green; zmin_green = other.zmin_green;
		return *this;
	};
	bool operator == ( const bitfield_compass &other ) const {
		return( xmax_green == other.xmax_green && xmin_green == other.xmin_green &&
			    ymax_green == other.ymax_green && ymin_green == other.ymin_green &&
			    zmax_green == other.zmax_green && zmin_green == other.zmin_green  );
	};
};

class Compass: public Deviation, public Clock_I
{
private:
    // Creates instance for I2C connection with passing the desired parameters.
    // No action is done at the bus. The default address of the chip is 0x0D.
	Compass( MagnetSensor *sens );

public:
	static Compass* createCompass(InterfaceId iid);
	~Compass();
	esp_err_t selfTest();

	// system related methods
	void begin();
	void start();
	void ageIncr();

	// sensor related interface
	bool haveSensor();
	bool overflowFlag();

	// Heading related methods
	bool cur_heading( float *head );
	rad_t rawHeading( bool *okIn );      //  Returns the low pass filtered magnetic heading without deviation
	float rawX() { return fx; };
	float rawY() { return fy; };
	float rawZ() { return fz; };
	float curX() { return mysensor->curX(); };
	float curY() { return mysensor->curY(); };
	float curZ() { return mysensor->curZ(); };
	float calX() { return ((float( (float)mysensor->curX() ) - bias.x) * scale.x); };
	float calY() { return ((float( (float)mysensor->curY() ) - bias.y) * scale.y); };
	float calZ() { return ((float( (float)mysensor->curZ() ) - bias.z) * scale.z); };

	void fetchRaw() { mysensor->readRaw( magRaw ); };
	vector_i16 getRawAxes() { return magRaw; };
	// rad_t filteredHeading( bool *okIn );
	// rad_t filteredTrueHeading( bool *okIn, bool withDeviation=true );
	void setGyroHeading( rad_t hd );
	rad_t getGyroHeading( bool *ok, bool addDeclination=true );
	inline bool headingValid() { return m_headingValid;	}

	// Calibration releated methods
	// Calibrate compass by using the read x, y, z raw values.
	bool calibrate( bool (*reporter)( vector_i16 raw, vector_f scale, vector_f bias, bitfield_compass b, bool print ), bool only_show );
	 // Resets the whole compass calibration, also the saved configuration.
	void resetCalibration();
    bool isCalibrated() const;
	bool calibrationIsRunning() {  return calibrationRunning; }
	// Returns total number of read errors
	int getReadError(){ return totalReadErrors; };
	void calcCalibration();
	CompassSink_I* dataSink() const { return mysensor; }

private:
	// Calculates tilt compensated heading in degrees of 0...359. The ok flag is set to true if fine, else false
	rad_t calc_heading( bool *ok );

	// internal task management
	void progress();
	bool tick() override;  // ticker for compass reading

	// Saves a done compass calibration.
	void saveCalibration();
	// Loads a stored compass calibration. Returns true if all okay
	bool loadCalibration();

	// fully gyro fused heading
	rad_t m_gyro_fused_heading;
	/** Pure averaged magnetic heading */
	rad_t m_magn_heading;
	/** Control flag of filtered heading. */
	bool m_headingValid;

	int _tick;
	rad_t _heading_average;
	int gyro_age;

	/** Variables used by calibration. */
	vector_i16 avg_calib_sample;
	vector_f bias;
	vector_f scale;
	vector_i16 min;
	vector_i16 max;
	Average<10, int16_t> *avgX = 0; // only for calibration
	Average<10, int16_t> *avgY = 0;
	Average<10, int16_t> *avgZ = 0;
	bool calibrationRunning;
	int nrsamples;
	bitfield_compass bits;

	// Error counters
	int errors;
	int totalReadErrors;

	// Mag readings
	int age;
	MagnetSensor *mysensor = nullptr;
	float fx; //bias corrected
	float fy;
	float fz;
	rad_t _heading;
	vector_i16 magRaw;
};

extern Compass *theCompass;
