// AirMass outputs to the rest of the firmware: vario, wind gauge, NMEA

#include "AirMass.h"
#include "AirMassSensor.h"

#include "sensor/Filters.h"
#include "setup/CruiseMode.h"
#include "protocol/NMEA.h"
#include "protocol/nmea_util.h"
#include "comm/DataLink.h"
#include "comm/Messages.h"
#include "wind/Wind.h"
#include "IpsDisplay.h"

#include <algorithm>
#include <cstdio>

// The air mass vario does not take horizontal gusts for lift, so it gets half the damping: just as calm,
// and a thermal entry shows about a second earlier.
constexpr float AIRMASS_DAMPING = 0.5f;
constexpr float VARIO_DT = 0.1f; // s, vario filter cycle

// the damping chain of the TE vario (VarioFilter): a low pass with tau = D/3, then y += (x - y) / D per cycle
static LowPassFilterT<mps_t> vario_lpf(LowPassFilterT<mps_t>::alphaFromTau(4.f, VARIO_DT));
static mps_t vario_damped = 0.f;
static bool vario_shown = false;

bool airmass::vario(mps_t netto, mps_t& air, bool& changed)
{
    const bool ok = CRMOD.isAirMass() && (airmass_status.get() & AirMassEKF::ST_VARIO_VALID) && airmass_vario.getValid();
    if (ok) {
        const float damping = std::max(1.f, vario_delay.get() * AIRMASS_DAMPING);
        vario_lpf.setTau(damping / 3.f, VARIO_DT);
        vario_damped += (vario_lpf.filter(airmass_vario.get()) - vario_damped) / damping;
    } else {
        vario_lpf.reset(netto);
        vario_damped = netto;
    }
    changed = ok != vario_shown;
    if (changed) {
        vario_shown = ok;
        IpsDisplay::setCruiseChanged(); // refresh the mode label
    }
    air = vario_damped;
    return ok;
}

bool airmass::varioShown()
{
    return CRMOD.isAirMass() && vario_shown;
}

void airmass::displayWind(WindData& synoptic, WindData& instant)
{
    if ((wind_enable.get() & WA_AIRMASS) && (airmass_status.get() & AirMassEKF::ST_WIND_VALID)) {
        instant.raw = airmass_inst_wind.get();
        if (!synoptic.isValid()) {
            synoptic.raw = airmass_avg_wind.get(); // the long term average of the estimator
        }
    }
}

// the averaged air mass wind for the navigation computer, if enabled and valid (master and second seat)
static bool naviWind(WindData& w)
{
    w = WindData(static_cast<uint32_t>(airmass_avg_wind.get()));
    return airmass_wind_nmea.get() && (airmass_status.get() & AirMassEKF::ST_WIND_VALID) && w.isValid();
}

// Air mass estimator output (master only)
// $PXCVA,vario,wdir,wspeed,wd,k_tas,pitch_ofs,latency,status*CHK
//    * vario: vertical air mass movement in m/s, up positive
//    * wdir: instant wind direction in deg (the direction the wind blows from)
//    * wspeed: instant wind speed in km/h
//    * wd: raw estimated vertical wind in m/s (NED, down positive)
//    * k_tas: learned airspeed scale factor
//    * pitch_ofs: learned pitch/AoA offset in deg
//    * latency: estimated effective GNSS velocity latency in ms
//    * status: AirMassEKF status bits (hex)
static void sendAirMass(NmeaPrtcl* nmea)
{
    if (nmea->getDL()->isBinActive() || !airmassSensor) {
        return;
    }
    const AirMassEKF& ekf = airmassSensor->getEKF();
    Message* msg = nmea->newMessage();

    const WindData wind = windFromVector(vector_f(ekf.windN(), ekf.windE(), 0.f));
    char str[80];
    std::snprintf(str, sizeof(str), "$PXCVA,%.2f,%d,%.1f,%.2f,%.3f,%.2f,%.0f,%X",
            ekf.airmassVario(), wind.getDeg(), Units::mps_to_kmh(wind.getVal()), ekf.windD(),
            ekf.kTas(), Units::rad_to_deg(ekf.pitchOffset()), ekf.gnssLatency() * 1000.f, (unsigned)ekf.status());
    msg->buffer = str;
    msg->buffer += "*" + NMEA::CheckSum(msg->buffer.c_str()) + "\r\n";
    DEV::Send(msg);
}

void airmass::nmeaFeed(NmeaPrtcl* nmea, bool slow)
{
    if (airmass_nmea.get()) {
        sendAirMass(nmea);
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
