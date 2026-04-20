/*
 * glider/Polars.h
 *
 *  Created on: Mar 1, 2019
 *      Author: iltis
 */

#pragma once

#include <cstdint>
#include <string_view>

struct compressed_polar;
struct flap_table;

struct t_polar {
	t_polar(const compressed_polar*);
	int16_t  index;
	const char *type;
	float    wingload;		// kg/mxm
	float    speed1;		// km/h
	float    sink1;			// m/s
	float    speed2;		// km/h
	float    sink2;			// m/s
	float    speed3;		// km/h
	float    sink3;			// m/s
	float    max_ballast;	// in liters or kg
	float    wingarea;		// mxm
	int16_t  flags;
	bool hasFlaps() const { return flags & 0x01; }
};

struct t_flap {
	t_flap(const flap_table*);
	int16_t    index;
	float      speeds[7];  // km/h 0.0 .. 255.0
	union {
        char label[4];
        int  label_int;
    }          labels[7];  // points to zero terminated string e.g. "+1"
};
struct flap_table {
	uint16_t index;
	uint8_t  speeds[7];	     // km/h 0..255
	std::string_view labels[7];  // points to zero terminated string e.g. "+1"
};

namespace Polars {
	const t_polar getPolar(int idx);
	int numPolars();
	const char *getPolarName(int i);
	int getPolarIndex(int i);
	int findMyGlider(int glider_index);
	const char *getGliderType(int i);
	bool hasFlaps(int);
	const flap_table& getFlapLevels(int idx);
	int numFlaps();
	int findMyFlapLevels(int glider_index);
};
