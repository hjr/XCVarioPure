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
	void tick(int tick);
	void drawCenterAid();
    void setGeometry(int r);
	void setGliderOnTop(bool onTop) { _glider_on_top = onTop; }
	void forceRedraw() { _dirty = true; }

private:
	CenterAid(PolarGauge &g);
	void drawThermal( int th, int idir, bool draw_red=false );
	void drawGlider();
	void ageThermal();
	void addThermal( int teval );
	bool maxClimb();
	void checkThermal();
	void calcFlightMode( rad_t headingDiff );
	int maxClimbIndex();

    const PolarGauge &_gauge;
	bool _glider_on_top; // circle aid reference on top, or 90° on the side.
	bool _dirty = false; // force redraw of the center aid

	int8_t thermals[CA_NUM_DIRS] = {0};  // every 15°: +/-127 in steps of 0.1 m/s
	rad_t cur_heading = 0.f;
	rad_t gps_heading = 0.f;
	rad_t gyro_last = 0.f;
	// int8_t gyro_foot_off; // difference from GPS to gyro heading
	int8_t idir = 0;
	int8_t agedir = 0;
	circling_t flightmode = circling_t::undefined;
	uint8_t turn_left = 0;
	uint8_t turn_right = 0;
	uint8_t fly_straight = 0;
	uint32_t last_rts = 0;
	mps_t peak_value = 1.0; // we start with expectation of 1 m/s thermals
	float scale = 1.f;
};

extern CenterAid  *theCenteraid;
