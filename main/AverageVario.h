
#pragma once

#include "math/Units.h"

#include <list>
#include <array>

class AverageVario{
	AverageVario() {};
	~AverageVario() {};
public:
	static void newSample( mps_t te );
	static void begin();
	static void recalcAvgClimb();

private:
	static mps_t averageClimbSec;
	static mps_t averageClimb;
	static std::array<mps_t, 10> avClimb100MSec;
	static std::array<mps_t, 60> avClimbSec;
	static std::list<mps_t> avClimbMin;
	static int avindex100MSec;
	static int avindexSec;
	static int samples;
};


