

#pragma once

#include <cmath>
#include <cstdint>
#include <string_view>

enum class quantity_t : uint8_t;

namespace Units {

// ---------------------------------------------------------------------------
// Base SI typedefs
// ---------------------------------------------------------------------------
using meters_t   = float;
using mps_t      = float;
using pascal_t   = float;
using kelvin_t   = float;
using rad_t      = float;
using seconds_t  = float;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr float g0            = 9.80665f;      // m/s²
constexpr float R_air         = 287.05f;       // J/(kg·K)
constexpr float T0            = 288.15f;       // K
constexpr float P0            = 101325.0f;     // Pa
constexpr float L             = 0.0065f;       // K/m
constexpr float ft_per_m      = 3.28084f;
constexpr float m_per_ft      = 1.0f / ft_per_m;
constexpr float kmh_per_mps   = 3.6f;
constexpr float mps_per_kmh   = 1.0f / 3.6f;
constexpr float fpm_per_mps   = 196.8504f;
constexpr float mps_per_fpm   = 1.0f / 196.8504f;
constexpr float deg_per_rad   = 57.2957795f;
constexpr float rad_per_deg   = 1.0f / deg_per_rad;

// ---------------------------------------------------------------------------
// Length
// ---------------------------------------------------------------------------
inline meters_t ft_to_m(float ft)      { return ft * m_per_ft; }
inline float    m_to_ft(meters_t m)    { return m * ft_per_m; }

// ---------------------------------------------------------------------------
// Speed
// ---------------------------------------------------------------------------
inline mps_t kmh_to_mps(float kmh)     { return kmh * mps_per_kmh; }
inline float mps_to_kmh(mps_t mps)     { return mps * kmh_per_mps; }

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
inline kelvin_t C_to_K(float c)        { return c + 273.15f; }
inline float    K_to_C(kelvin_t k)     { return k - 273.15f; }

// ---------------------------------------------------------------------------
// Angles
// ---------------------------------------------------------------------------
inline rad_t deg_to_rad(float deg)     { return deg * rad_per_deg; }
inline float rad_to_deg(rad_t rad)     { return rad * deg_per_rad; }

// ---------------------------------------------------------------------------
// ISA atmosphere (troposphere, up to ~11km)
// ---------------------------------------------------------------------------
inline pascal_t isa_pressure(meters_t h_m)
{
    return P0 * powf(1.0f - (L * h_m) / T0, g0 / (R_air * L));
}

inline kelvin_t isa_temperature(meters_t h_m)
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
	constexpr const char* getName() const {
		return name.data();
	}
};

constexpr float convert(float value, unit_t from, unit_t to);
constexpr float to_display(float si_value, unit_t display);
void setAll();

} // namespace Units

extern const Units::unit_t *AltUnit;
extern const Units::unit_t *SpeedUnit;
extern const Units::unit_t *VarioUnit;
extern const Units::unit_t *TempUnit;
extern const Units::unit_t *DistanceUnit;
extern const Units::unit_t *PressureUnit;

namespace Units
{
	float Speed(float as);
	float Distance(float d);
	int SpeedRounded(float as);
	float kmh2knots(float kmh);
	float kmh2ms(float kmh);
	float ms2kmh(float ms);
	float knots2kmh(float knots);
	float Airspeed2Kmh(float as);
	float ActualWingloadCorrection(float v);
	float TemperatureUnit(float t);
	const char* TemperatureUnitStr(int idx = -1);
	const char* SpeedUnitStr(int u = -1);
	float Vario(const float te);
	float Qnh(float qnh);
	int QnhRounded(float qnh);
	float hPa2inHg(float hpa);
	float inHg2hPa(float inhg);
	float knots2ms(float knots);
	float ms2knots(float knots);
	float ms2mph(float ms);
	float ms2fpm(float ms);
	float Vario2ms(float var);
	float mcval2knots(float mc);
	const char *QnhUnit(int unit = -1);
	float Altitude(float alt, int unit = -1);
	float meters2feet(float m);
	float feet2meters(float f);
	float meters2FL(float m);
	const char *AltitudeUnit(int unit = -1);
	const char *AltitudeUnitMeterOrFeet(int unit = -1);
	float value(float val, quantity_t u);
	const char *unit(quantity_t u);
};
