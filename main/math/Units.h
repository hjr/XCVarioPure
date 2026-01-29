

#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

enum class quantity_t : uint8_t;

// ---------------------------------------------------------------------------
// Base SI typedefs
// ---------------------------------------------------------------------------
using meter_t    = float;
using mps_t      = float;
using pascal_t   = float;
using kelvin_t   = float;
using rad_t      = float;
using second_t   = float;
using hertz_t    = float;
using newton_t   = float;
using joule_t    = float;
using watt_t     = float;
using kilogram_t = float;

namespace Units {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr float g0            = 9.80665f;      // m/s²
constexpr float R_air         = 287.05f;       // J/(kg·K)
constexpr float rho0          = 1.225f;        // kg/m³ (ISA sea level)
constexpr kelvin_t T0         = 288.15f;       // K (alias 15°C)
constexpr kelvin_t C2K 		  = 273.15f;       // K (0°C)
constexpr pascal_t P0         = 101325.0f;     // Pa
constexpr float L             = 0.0065f;       // K/m (ISA lapse rate)
constexpr float ft_per_m      = 3.28084f;
constexpr float m_per_ft      = 1.0f / ft_per_m;
constexpr float kmh_per_mps   = 3.6f;
constexpr float mps_per_kmh   = 1.0f / kmh_per_mps;
constexpr float knots_per_mps = 1.94384f;
constexpr float mps_per_knots = 1.0f / knots_per_mps;
constexpr float fpm_per_mps   = 196.8504f;
constexpr float mps_per_fpm   = 1.0f / fpm_per_mps;
constexpr float deg_per_rad   = 57.2957795f;
constexpr float rad_per_deg   = 1.0f / deg_per_rad;

// ---------------------------------------------------------------------------
// Length
// ---------------------------------------------------------------------------
inline meter_t  ft_to_m(float ft)     { return ft * m_per_ft; }
inline float    m_to_ft(meter_t m)    { return m * ft_per_m; }

// ---------------------------------------------------------------------------
// Speed
// ---------------------------------------------------------------------------
inline constexpr mps_t kmh_to_mps(float kmh)     { return kmh * mps_per_kmh; }
inline constexpr float mps_to_kmh(mps_t mps)     { return mps * kmh_per_mps; }

inline constexpr mps_t knots_to_mps(float knots)   { return knots * mps_per_knots; }
inline constexpr float mps_to_knots(mps_t mps)     { return mps * knots_per_mps; }

inline mps_t fpm_to_mps(float fpm)     { return fpm * mps_per_fpm; }
inline float mps_to_fpm(mps_t mps)     { return mps * fpm_per_mps; }

// ---------------------------------------------------------------------------
// Pressure
// ---------------------------------------------------------------------------
inline pascal_t hpa_to_pa(float hpa)   { return hpa * 100.0f; }
inline float    pa_to_hpa(pascal_t pa) { return pa * 0.01f; }

// ---------------------------------------------------------------------------
// Temperature
// ---------------------------------------------------------------------------
inline kelvin_t C_to_K(float c)        { return c + C2K; }
inline float    K_to_C(kelvin_t k)     { return k - C2K; }

// ---------------------------------------------------------------------------
// Angles
// ---------------------------------------------------------------------------
inline rad_t deg_to_rad(float deg)     { return deg * rad_per_deg; }
inline float rad_to_deg(rad_t rad)     { return rad * deg_per_rad; }

// ---------------------------------------------------------------------------
// ISA atmosphere (troposphere, up to ~11km)
// ---------------------------------------------------------------------------
inline pascal_t isa_pressure(meter_t h_m)
{
    return P0 * powf(1.0f - (L * h_m) / T0, g0 / (R_air * L));
}

inline kelvin_t isa_temperature(meter_t h_m)
{
    return T0 - L * h_m;
}

// ---------------------------------------------------------------------------
// True Airspeed from dynamic pressure (incompressible, low Mach)
// q = 0.5 * rho * v²
// ---------------------------------------------------------------------------
inline mps_t tas_from_q(pascal_t q, kelvin_t T)
{
    float rho = P0 / (R_air * T);
    return sqrtf(2.0f * q / rho);
}



// ---------------------------------------------------------------------------
// Helper
// ---------------------------------------------------------------------------

struct unit_t {
    float scale;   // multiplicative
    float offset;  // additive
	std::string_view name;
    constexpr float apply(float v) const {
        return v * scale + offset;
    }
    constexpr float from(float v) const {
        return (v - offset) / scale;
    }
	constexpr const char* getName() const {
		return name.data();
	}
};

// ---------------------------------------------------------------------------
// length
constexpr unit_t meter      { 1.0f, 0.0f, "m" };
constexpr unit_t kilometer  { 0.001f, 0.0f, "km" };
constexpr unit_t mile       { 0.000621371f, 0.0f, "mi" };
constexpr unit_t naut_mile  { 0.000539957f, 0.0f, "nm" };
// vertical length
constexpr unit_t foot       { 3.2808399f, 0.0f, "ft" };
constexpr unit_t flightlevel { 3.2808399f / 100.0f, 0.0f, "FL" }; // in hundreds of feet

// speed
constexpr unit_t mps        { 1.0f, 0.0f, "m/s" };
constexpr unit_t kmh        { 3.6f, 0.0f, "kmh" };
constexpr unit_t mph        { 2.2369363f, 0.0f, "mph" };
constexpr unit_t kts        { 1.9438445f, 0.0f, "kt" };
// vertical speed
constexpr unit_t fpm        {196.850394f, 0.0f, "ftm"};

// pressure
constexpr unit_t pascal     { 1.0f, 0.0f, "Pa" };
constexpr unit_t hpa        { 0.01f, 0.0f, "hPa" };
constexpr unit_t inhg       { 0.000295299830714f, 0.0f, "inHg" };

// temperature
constexpr unit_t kelvin     { 1.0f, 0.0f, "K" };
constexpr unit_t celsius    { 1.0f, -C2K, "'C" };
constexpr unit_t fahrenheit { 9.0f / 5.0f, -459.67f, "'F" };

constexpr unit_t none       { 1.0f, 0.0f, "%" };

constexpr inline float convert(float value, const Units::unit_t& from, const Units::unit_t& to) {
    return to.apply(value / from.scale - from.offset);
}
constexpr inline float pipe(float v, const unit_t& u) {
    return u.apply(v);
}
constexpr inline float read(const unit_t& u, float v) {
	return u.from(v);
}

void setAll();

} // namespace Units

extern const Units::unit_t *AltUnit;
extern const Units::unit_t *SpeedUnit;
extern const Units::unit_t *VarioUnit;
extern const Units::unit_t *TempUnit;
extern const Units::unit_t *DistanceUnit;
extern const Units::unit_t *PressureUnit;

