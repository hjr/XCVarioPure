#pragma once

#include "math/Units.h"

/*
 * Methods for Athmosphere Model used in Aviation
 *
 *
 */

namespace Atmosphere {

mps_t TAS(mps_t ias, meter_t altitude, kelvin_t temp);
// float TAS2(float ias, float altitude, float temp);
// float CAS(float dp);
mps_t IAS(mps_t tas, float alti, float temp);
// float pascal2kmh(pascal_t pascal);
mps_t pascal2ms(pascal_t pascal);
pascal_t kmh2pascal(float kmh);
float calcAltitude(float seaLevelPressure, float pressure);
inline meter_t calcAltitudeISA(pascal_t pressure) { return calcAltitude(Units::P0, pressure); }
pascal_t calcPressure(pascal_t seaLevelPressure, meter_t alti);
inline pascal_t calcPressureISA(meter_t alti) { return calcPressure(Units::P0, alti); }
float calcQNHPressure(float pressure, float altitude);

}; // namespace Atmosphere
