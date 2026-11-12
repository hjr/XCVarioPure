#pragma once

#include "math/Units.h"

/*
 * Methods for Athmosphere Model used in Aviation
 *
 *
 */

namespace Atmosphere {

void cacheTasPerIas();
mps_t TAS(mps_t ias, pascal_t sp, kelvin_t temp);
mps_t TasFromDp(pascal_t q, kelvin_t temp);


}; // namespace Atmosphere

namespace Units {

    void setAll();

} // namespace Units

extern const Units::unit_t *AltUnit;
extern const Units::unit_t *SpeedUnit;
extern const Units::unit_t *VarioUnit;
extern const Units::unit_t *TempUnit;
extern const Units::unit_t *DistanceUnit;
extern const Units::unit_t *PressureUnit;

