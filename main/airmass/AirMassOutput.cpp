// AirMass outputs to the rest of the firmware: vario, NMEA

#include "AirMass.h"
#include "AirMassSensor.h"

#include "protocol/NMEA.h"
#include "protocol/nmea_util.h"
#include "comm/DataLink.h"
#include "comm/Messages.h"
#include "wind/Wind.h"

#include <cstdio>

static bool varioValid()
{
    return te_comp_enable.get() == TE_AIRMASS && (airmass_status.get() & AirMassEKF::ST_VARIO_VALID) && airmass_vario.getValid();
}

bool airmass::rawVario(mps_t& air)
{
    followMethod();
    air = airmass_vario.get();
    return varioValid();
}

bool airmass::varioFallback()
{
    return te_comp_enable.get() == TE_AIRMASS && !varioValid();
}

// the averaged air mass wind for the navigation computer, if enabled and valid (master and second seat)
static bool naviWind(WindData& w)
{
    w = WindData(static_cast<uint32_t>(synoptic_wind.get()));
    return airmass_wind_nmea.get() && (airmass_status.get() & AirMassEKF::ST_WIND_VALID) && w.isValid();
}

// System > Logging > "All Sensor Data" (master only): the estimator output $PXCVA (format in AirMassRecord.h)
//    * vario: vertical air mass movement in m/s, up positive
//    * wdir: instant wind direction in deg (the direction the wind blows from)
//    * wspeed: instant wind speed in km/h
//    * wd: raw estimated vertical wind in m/s (NED, down positive)
//    * k_tas: learned airspeed scale factor
//    * pitch_ofs: learned pitch/AoA offset in deg
//    * latency: estimated effective GNSS velocity latency in ms
//    * status: AirMassEKF status bits (hex)
// and its start, setup, GNSS velocities and compass headings, for the replay on a host together with the $SENS
// sentences of that logging (tools/airmass_sim). The sensor loop
// queues the lines in the order of its cycles.
void airmass::nmeaFeed(NmeaPrtcl* nmea, bool slow)
{
    // the queued lines in one message; no NMEA in binary mode, the queue is drained anyway
    std::string lines;
    if (airmassNmeaDrain(lines) && !nmea->getDL()->isBinActive()) {
        Message* msg = nmea->newMessage();
        msg->buffer = std::move(lines);
        DEV::Send(msg);
    }
    // the Cambridge sentence carries the wind already
    WindData wind;
    if (slow && nmea->getProtocolId() != CAMBRIDGE_P && naviWind(wind)) {
        nmea->sendXCVNmeaMWV(wind.getDeg(), wind.getVal());
    }
}

std::string airmass::cambridgeWind()
{
    WindData wind;
    if (!naviWind(wind)) {
        return "!w,0,0,0,0,";
    }
    // vector wind: the direction the air moves to, speed in 1/10 m/s
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "!w,%d,%d,0,0,", (wind.getDeg() + 180) % 360, int(wind.getVal() * 10.f + .5f));
    return buffer;
}
