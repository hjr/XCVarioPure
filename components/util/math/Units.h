

#pragma once

#include <cstdint>
#include <cmath>
#include <string_view>

enum class quantity_t : uint8_t;

// ---------------------------------------------------------------------------
// Base SI typedefs
// ---------------------------------------------------------------------------
using meter_t    = float;
using mps_t      = float;
using kmh_t      = float;
using knots_t    = float;
using pascal_t   = float;
using kelvin_t   = float;
using celsius_t  = float;
using rad_t      = float;
using rps_t      = float;
using degree_t   = float;
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
constexpr float R_air         = 287.058f;      // J/(kg·K)
constexpr float rho0          = 1.225f;        // kg/m³ (ISA sea level)
constexpr kelvin_t T0         = 288.15f;       // K (alias 15°C, ISA temp on sea level)
constexpr kelvin_t C2K 		  = 273.15f;       // K (0°C)
constexpr pascal_t P0         = 101325.0f;     // Pa
constexpr float L             = 0.0065f;       // K/m (ISA lapse rate)
constexpr meter_t T0divL      = T0 / L;        // 44330.76923 m
constexpr float LdivT0        = L / T0;        // 0.000022556f 1/m
constexpr float g0divRxL      = g0 / (R_air * L); // 5.25588f, exponent for ISA pressure formula
constexpr float ft_per_m      = 3.2808399f;    // feet per meter
constexpr float m_per_ft      = 1.0f / ft_per_m;
constexpr float kmh_per_mps   = 3.6f;
constexpr float mps_per_kmh   = 1.0f / kmh_per_mps;
constexpr float knots_per_mps = 1.9438445f;
constexpr float mps_per_knots = 1.0f / knots_per_mps;
constexpr float hfpm_per_mps  = 1.96850394f;   // 100 feet per minute
constexpr float mps_per_hfpm  = 1.0f / hfpm_per_mps;
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

inline constexpr mps_t hfpm_to_mps(float hfpm)   { return hfpm * mps_per_hfpm; }
inline constexpr float mps_to_hfpm(mps_t mps)    { return mps * hfpm_per_mps; }

// ---------------------------------------------------------------------------
// Pressure
// ---------------------------------------------------------------------------
inline constexpr pascal_t hpa_to_pa(float hpa)   { return hpa * 100.0f; }
inline constexpr float    pa_to_hpa(pascal_t pa) { return pa * 0.01f; }

// ---------------------------------------------------------------------------
// Temperature
// ---------------------------------------------------------------------------
inline constexpr kelvin_t C_to_K(float c)        { return c + C2K; }
inline constexpr float    K_to_C(kelvin_t k)     { return k - C2K; }

// ---------------------------------------------------------------------------
// Angles
// ---------------------------------------------------------------------------
inline constexpr rad_t deg_to_rad(float deg)     { return deg * rad_per_deg; }
inline constexpr float rad_to_deg(rad_t rad)     { return rad * deg_per_rad; }

// ---------------------------------------------------------------------------
// Inertia
// ---------------------------------------------------------------------------
inline constexpr float kgm2_to_kgfm2(float kgm2) { return kgm2 * g0; }
inline constexpr float kgfm2_to_kgm2(float kgfm2) { return kgfm2 / g0; }
inline constexpr float g_to_ms2(float g) { return g * g0; }
inline constexpr float ms2_to_g(float ms2) { return ms2 / g0; }

// ---------------------------------------------------------------------------
// ISA atmosphere (troposphere, up to ~11km)
// ---------------------------------------------------------------------------
constexpr kelvin_t isa_temperature(meter_t height) { return T0 - L * height; }
// Speed & Pressure
constexpr mps_t pascal_to_mps(pascal_t pressure) {
    if (pressure < 0.0f) { return 0.0f; }
    return std::sqrtf(2.f * pressure / rho0);
}
constexpr pascal_t mps_to_pascal(mps_t speed) { return (speed * speed) * rho0 / 2.; }

constexpr meter_t calcAltitude(pascal_t seaLevelPressure, pascal_t pressure) {
    return (Units::T0divL * (1.0f - std::powf(pressure / seaLevelPressure, 1.0f / Units::g0divRxL)));
}
inline constexpr meter_t calcAltitudeISA(pascal_t pressure) { return calcAltitude(Units::P0, pressure); }
constexpr pascal_t calcPressure(pascal_t seaLevelPressure, meter_t alti) {
    return seaLevelPressure * std::powf(1.0f - (alti / Units::T0divL), Units::g0divRxL);
}
inline constexpr pascal_t calcPressureISA(meter_t alti) { return calcPressure(Units::P0, alti); }
constexpr pascal_t calcQNH(pascal_t pressure, meter_t alti) {
    return pressure / std::powf(1.0f - (alti / Units::T0divL), Units::g0divRxL);
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
extern const unit_t meter;
extern const unit_t kilometer;
extern const unit_t mile;
extern const unit_t naut_mile;
// vertical length
extern const unit_t foot;
extern const unit_t flightlevel;

// speed
extern const unit_t mps;
extern const unit_t kmh;
extern const unit_t mph;
extern const unit_t kts;
// vertical speed
extern const unit_t hfpm; // 100 feet per minute

// pressure
extern const unit_t pascal;
extern const unit_t hpa;
extern const unit_t inhg;

// temperature
extern const unit_t kelvin;
extern const unit_t celsius;
extern const unit_t fahrenheit;

extern const unit_t none;

constexpr inline float convert(float value, const Units::unit_t& from, const Units::unit_t& to) {
    return to.apply(value / from.scale - from.offset);
}
constexpr inline float pipe(float v, const unit_t& u) {
    return u.apply(v);
}
constexpr inline float read(const unit_t& u, float v) {
	return u.from(v);
}

} // namespace Units

