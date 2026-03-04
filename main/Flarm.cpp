#include "Flarm.h"

#include <protocol/Clock.h>

int Flarm::RX = 0;
int Flarm::TX = 0;
int Flarm::GPS = 0;
int Flarm::Power = 0;
int Flarm::AlarmLevel = 0;
rad_t Flarm::RelativeBearing = 0.f; // -pi .. pi
int Flarm::AlarmType = 0;
int Flarm::RelativeVertical = 0; // meter
int Flarm::RelativeDistance = 0; // meter
mps_t Flarm::gndSpeed = 0.f;
rad_t Flarm::gndCourse = 0;
bool Flarm::myGPS_OK = false;
int Flarm::IcaoId = 0;
int Flarm::_confirmedId = 0;
int Flarm::_confirmedTime = 0;

int Flarm::ext_alt_timer=0;
int Flarm::_numSat=0;



// bool Flarm::getGPS( mps_t &gndspeed_par, rad_t &gndTrack ) {
//     if( myGPS_OK ) {
//         gndspeed_par = gndSpeed;
//         gndTrack = gndCourse;
//         return true;
//     }
//     else {
//         return false;
//     }
// }
// bool Flarm::getGPSSpeed( mps_t &gndspeed_par ) {
//         if( myGPS_OK ) {
//             gndspeed_par = gndSpeed;
//             return true;
//         }
//         else {
//             return false;
//         }
// }


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

