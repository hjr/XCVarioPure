/*
 * CenterAid.h
 *
 *  Created on: Jan 23, 2022
 *      Author: gittest3
 */
#pragma once

#include "wind/CircleWind.h"

#include <cstdint>

constexpr int CA_NUM_DIRS = 24;  // every 15°

class PolarGauge;

class CenterAid {
public:
    static CenterAid *create(PolarGauge &g);
    static void remove();
	void drawThermal( int th, int idir, bool draw_red=false );
	void tick(int tick);
	void drawCenterAid();
    void setGeometry(int r);
	void setGliderOnTop(bool onTop) { _glider_on_top = onTop; }
	void forceRedraw() { _dirty = true; }

private:
    const PolarGauge &_gauge;
	bool _glider_on_top; // circle aid reference on top, or 90° on the side.
	bool _dirty = false; // force redraw of the center aid
	CenterAid(PolarGauge &g);
	void ageThermal();
	void addThermal( int teval );
	bool maxClimb();
	void checkThermal();
	void calcFlightMode( rad_t headingDiff );
	int maxClimbIndex();
	int8_t thermals[CA_NUM_DIRS];  // every 15°: +/-127 in steps of 0.1 m/s
	int8_t drawn_thermals[CA_NUM_DIRS];
	void drawGlider();
	rad_t cur_heading;
	rad_t gps_heading;
	rad_t gyro_last;
	// int8_t gyro_foot_off; // difference from GPS to gyro heading
	int8_t idir;
	int8_t agedir;
	t_circling flightmode;
	uint8_t turn_left;
	uint8_t turn_right;
	uint8_t fly_straight;
	uint32_t last_rts;
	mps_t peak_value;
	float scale;
};

extern CenterAid  *theCenteraid;
