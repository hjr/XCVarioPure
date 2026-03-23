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


}; // namespace Atmosphere
