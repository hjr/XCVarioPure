#pragma once

#include "math/Units.h"

/*
 * Methods for Athmosphere Model used in Aviation
 *
 *
 */

namespace Atmosphere {

mps_t TAS(mps_t ias, pascal_t sp, kelvin_t temp);
mps_t TasFromDp(pascal_t q, kelvin_t temp);
mps_t IAS(mps_t tas, pascal_t sp, kelvin_t temp);
// float pascal2kmh(pascal_t pascal);
mps_t pascal2ms(pascal_t pascal);
pascal_t ms2pascal(mps_t speed);
meter_t calcAltitude(pascal_t seaLevelPressure, pascal_t pressure);
inline meter_t calcAltitudeISA(pascal_t pressure) { return calcAltitude(Units::P0, pressure); }
pascal_t calcPressure(pascal_t seaLevelPressure, meter_t alti);
inline pascal_t calcPressureISA(meter_t alti) { return calcPressure(Units::P0, alti); }
pascal_t calcQNHPressure(pascal_t pressure, meter_t altitude);

}; // namespace Atmosphere
