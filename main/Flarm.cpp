#include "Flarm.h"

#include <protocol/Clock.h>

int Flarm::RX = 0;
int Flarm::TX = 0;
int Flarm::GPS = 0;
int Flarm::Power = 0;
int Flarm::AlarmLevel = 0;
int Flarm::RelativeBearing = 0;
int Flarm::AlarmType = 0;
int Flarm::RelativeVertical = 0;
int Flarm::RelativeDistance = 0;
float Flarm::gndSpeedKnots = 0;
float Flarm::gndCourse = 0;
bool Flarm::myGPS_OK = false;
int Flarm::IcaoId = 0;
int Flarm::_confirmedId = 0;
int Flarm::_confirmedTime = 0;

int Flarm::ext_alt_timer=0;
int Flarm::_numSat=0;



bool Flarm::getGPS( float &gndSpeedKmh, float &gndTrack ) {
    if( myGPS_OK ) {
        gndSpeedKmh = Units::knots2kmh(gndSpeedKnots);
        gndTrack = gndCourse;
        return true;
    }
    else{
        return false;
    }
}
bool Flarm::getGPSknots( float &gndSpeed ) {
        if( myGPS_OK ) {
            gndSpeed = gndSpeedKnots;
            return true;
        }
        else{
            return false;
        }
}


void Flarm::setConfirmed()
{
    _confirmedId = IcaoId;
    _confirmedTime = Clock::getSeconds();
}

bool Flarm::isConfirmed()
{
    if ( _confirmedId !=0 && IcaoId != 0 && _confirmedId == IcaoId ) {
        int delta = Clock::getSeconds() - _confirmedTime;
        if ( delta < 30 ) {
            return true;
        }
    }
    return false;
}

