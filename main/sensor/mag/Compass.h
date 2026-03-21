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

#include "math/vector_3d.h"
#include "math/Units.h"

#include <cstdint>


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
	bool allAxesGood() const {
		return xmax_green && xmin_green && ymax_green && ymin_green && zmax_green && zmin_green;
	}
};


struct CompassCalibrationData {
	int nrsamples = 0;
	vector_f sample;
	vector_f var;
	float variance = 0.f;
	vector_f min = { 20000., 20000., 20000. };
	vector_f max = { 0., 0., 0. };
	bitfield_compass bits = {};
	vector_f bias = {};
	vector_f scale = { 1.f, 1.f, 1.f };
	CompassCalibrationData() = default;
};
