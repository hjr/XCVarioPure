/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ****************************************************************************

Magnetic Sensor Abstraction.

 ****************************************************************************/

#pragma once

#include "math/vector_3d_fwd.h"

#include <esp_err.h>

class CompassSink_I
{
public:
    virtual ~CompassSink_I() = default;
    virtual void fromExternal(const vector_i16 *magaxes) = 0;
};

// Class Template
class MagnetSensor : public CompassSink_I
{
public:
	virtual ~MagnetSensor() = default;
	virtual esp_err_t initialize() = 0;
	virtual bool haveSensor() = 0;
	bool isCalibrated() { return calibrated; }
	virtual void age_incr() = 0;
	virtual esp_err_t selfTest() = 0;
	virtual bool overflowFlag() = 0;
	virtual bool readRaw( vector_i16 &mag ) = 0;
	virtual int curX() = 0;
	virtual int curY() = 0;
	virtual int curZ() = 0;
protected:
	bool calibrated = false;
};
