
#include "Units.h"

namespace Units {

// ---------------------------------------------------------------------------
// length
const unit_t meter      { 1.0f, 0.0f, "m" };
const unit_t kilometer  { 0.001f, 0.0f, "km" };
const unit_t mile       { 0.000621371f, 0.0f, "mi" };
const unit_t naut_mile  { 0.000539957f, 0.0f, "nm" };
// vertical length
const unit_t foot       { ft_per_m, 0.0f, "ft" };
const unit_t flightlevel { ft_per_m / 100.0f, 0.0f, "FL" }; // in hundreds of feet

// speed
const unit_t mps        { 1.0f, 0.0f, "m/s" };
const unit_t kmh        { kmh_per_mps, 0.0f, "kmh" };
const unit_t mph        { 2.2369363f, 0.0f, "mph" };
const unit_t kts        { knots_per_mps, 0.0f, "kt" };
// vertical speed
const unit_t hfpm       {hfpm_per_mps, 0.0f, "hfpm"}; // 100 feet per minute

// pressure
const unit_t pascal     { 1.0f, 0.0f, "Pa" };
const unit_t hpa        { 0.01f, 0.0f, "hPa" };
const unit_t inhg       { 0.000295299830714f, 0.0f, "inHg" };

// temperature
const unit_t kelvin     { 1.0f, 0.0f, "'K" };
const unit_t celsius    { 1.0f, -C2K, "'C" };
const unit_t fahrenheit { 9.0f / 5.0f, -459.67f, "'F" };

const unit_t none       { 1.0f, 0.0f, "%" };

}