
#pragma once

#include "math/Units.h"

class AdaptUGC;
class FlarmMsg;
class GarminMsg;
class GpsMsg;
class FlarmScreen;

class Flarm {
	friend class FlarmMsg;
	friend class GarminMsg;
	friend class GpsMsg;
	friend class FlarmScreen;
public:
	static int alarmLevel(){ return AlarmLevel; };
	// static bool getGPS( mps_t &gndSpeed, rad_t &gndTrack );
	// static bool getGPSSpeed( float &gndSpeed );
	static bool gpsStatus() { return myGPS_OK; }
	static mps_t getGndSpeed() { return gndSpeed; }
	static rad_t getGndCourse() { return gndCourse; }
	static void setConfirmed();
	static bool isConfirmed();

private:
	static int RX,TX,GPS,Power;
	static int AlarmLevel;
	static rad_t RelativeBearing;
	static int RelativeVertical, RelativeDistance; // meter
	static mps_t gndSpeed;
	static rad_t gndCourse;
	static bool   myGPS_OK;
	static int AlarmType;
	static int IcaoId; 	// ICAO 24-bit address for Mode-S
    					// targets and a FLARM-generated ID for Mode-C targets
	static int _confirmedId;
	static int _confirmedTime;
	static int _numSat;
};


