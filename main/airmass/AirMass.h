#pragma once

// AirMass: the estimator's interface to the rest of the firmware. Files outside main/airmass include only this
// header and call these hooks, so the feature stays in its own directory.

#include "AirMassSettings.h"
#include "math/Units.h"

#include <cstdint>
#include <string>

class SetupMenu;
class SetupMenuSelect;
class NmeaPrtcl;
union WindData;

namespace airmass {

// system startup: create the estimator (master with IMU and air data)
void startup();

// GNSS: a new ground velocity (protocol task)
void gnssVelocity(mps_t speed, rad_t course, uint32_t time_ms, uint16_t latency_ms, uint32_t fix_time_ms = UINT32_MAX);
// GNSS: true heading and wind correction angle from the estimator; false while it has none
bool heading(rad_t track);

// vario filter, every cycle: the raw vertical air mass movement, false while the estimator has no valid one
bool rawVario(mps_t& air);
// the damping of the air mass vario, from the damping of the TE vario
second_t varioDamping(second_t te_damping);

// wind gauge: the estimator's instant wind, and its average where no synoptic wind is valid; degraded: the instant
// wind is not valid any more but still the best estimate (long straight glides), to be shown marked
void displayWind(WindData& synoptic, WindData& instant, bool& degraded);

// setup menu: the Air Mass vario modes, and the Air Mass settings menu
void menuVarioModes(SetupMenuSelect* mode);
void menuVario(SetupMenu* vario);

// NMEA, 5Hz: $PXCVA if enabled; slow (1Hz): the averaged wind to the navigation computer
void nmeaFeed(NmeaPrtcl* nmea, bool slow);
// the wind field of the Cambridge sentence ("!w,...")
std::string cambridgeWind();

} // namespace airmass
