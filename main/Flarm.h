
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
	// static bool getGPS( mps_t &gndSpeed, float &gndTrack );
	// static bool getGPSknots( float &gndSpeed );
	static bool gpsStatus() { return myGPS_OK; }
	static float getGndSpeedKnots() { return gndSpeedKnots; }
	static float getGndCourse() { return gndCourse; }
	static bool validExtAlt() { return ext_alt_timer!=0; } //fixme -> watchdog
	static void setConfirmed();
	static bool isConfirmed();

private:
	static int RX,TX,GPS,Power;
	static int AlarmLevel;
	static int RelativeBearing,RelativeVertical,RelativeDistance;
	static float gndSpeedKnots;
	static float gndCourse;
	static bool   myGPS_OK;
	static int AlarmType;
	static int IcaoId; 	// ICAO 24-bit address for Mode-S
    					// targets and a FLARM-generated ID for Mode-C targets
	static int _confirmedId;
	static int _confirmedTime;
	static int ext_alt_timer;
	static int _numSat;
};


