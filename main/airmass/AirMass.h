#pragma once

// AirMass: the estimator's interface to the rest of the firmware. Files outside main/airmass include only this
// header and call these hooks, so the feature stays in its own directory.

#include "AirMassSettings.h"
#include "math/Units.h"

#include <cstdint>
#include <string>

class SetupMenu;
class MenuEntry;
class SetupMenuSelect;
class SetupMenuValFloat;
class NmeaPrtcl;

namespace airmass {

// system startup
void startup();
// create the estimator with the TE compensation method or the wind source Air Mass where it is supported, remove it
// without
void followMethod();
// the estimator can run here: the master with IMU and air data
bool supported();
// instrument replay ($PFLAX,S): the replayed flight or the clock jumps, restart the estimator (protocol task)
void prepareForSimJump();

// GNSS: a new ground velocity (protocol task)
void gnssVelocity(mps_t speed, rad_t course, uint32_t time_ms, uint16_t latency_ms, uint32_t fix_time_ms = UINT32_MAX);
// GNSS: true heading and wind correction angle from the estimator; false while it has none
bool heading(rad_t track);

// vario filter, every cycle: the raw vertical air mass movement, false while the TE compensation method is not Air
// Mass or the estimator has no valid one. Follows the method (followMethod).
bool rawVario(mps_t& air);
// vario needle: the method is Air Mass, but the vario shows the TE vario for now
bool varioFallback();

// setup menu: Air Mass as TE compensation method (with the help of its eAdjustment), as wind source with the wind to the navigation
// computer, and for development builds (DEBUG_AND_TEST) the status page of the vario menu and the recording in the
// help of the logging
void menuTeMethod(SetupMenuSelect* method, SetupMenuValFloat* adjustment);
void menuLogging(SetupMenuSelect* logging);
void menuVario(SetupMenu* vario);
void menuWind(SetupMenu* wind, SetupMenuSelect* source);
MenuEntry* statusPage();
void statusUpdate();
// learned flap position offsets in NVS, -1 without flaps
int learnedFlapPositions();

// NMEA, 5Hz: the recording if enabled; slow (1Hz): the averaged wind to the navigation computer
void nmeaFeed(NmeaPrtcl* nmea, bool slow);
// the wind field of the Cambridge sentence ("!w,...")
std::string cambridgeWind();

} // namespace airmass
