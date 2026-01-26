
#include "Units.h"

#include "Floats.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

namespace Units
{
// length
constexpr unit_t meter      { 1.0f, 0.0f, "m" };
constexpr unit_t kilometer  { 0.001f, 0.0f, "km" };
constexpr unit_t mile       { 0.000621371f, 0.0f, "mi" };
constexpr unit_t nautical_mile { 0.000539957f, 0.0f, "nm" };
// vertical length
constexpr unit_t foot       { 3.2808399f, 0.0f, "ft" };
constexpr unit_t flightlevel { 3.2808399f / 100.0f, 0.0f, "FL" }; // in hundreds of feet

// speed
constexpr unit_t mps        { 1.0f, 0.0f, "m/s" };
constexpr unit_t kmh        { 3.6f, 0.0f, "kmh" };
constexpr unit_t mph        { 2.2369363f, 0.0f, "mph" };
constexpr unit_t kts        { 1.9438445f, 0.0f, "kt" };
// vertical speed
constexpr unit_t fpm        {196.850394f, 0.0f, "ft/m"};

// pressure
constexpr unit_t pascal     { 1.0f, 0.0f, "Pa" };
constexpr unit_t hpa        { 0.01f, 0.0f, "hPa" };
constexpr unit_t inhg       { 0.000295299830714f, 0.0f, "inHg" };

// temperature
constexpr unit_t kelvin     { 1.0f, 0.0f, "K" };
constexpr unit_t celsius    { 1.0f, -273.15f, "'C" };
constexpr unit_t fahrenheit { 5.0f / 9.0f, -459.67f * 5.0f / 9.0f, "'F" };

constexpr float convert(float value, Units::unit_t from, Units::unit_t to)
{
    return to.apply(value / from.scale - from.offset);
}

constexpr float to_display(float si_value, Units::unit_t display)
{
    return display.apply(si_value);
}

void setAll()
{
	AltUnit = (alt_unit.get() == ALT_UNIT_FT) ? &foot :
              (alt_unit.get() == ALT_UNIT_FL) ? &flightlevel : &meter;
	SpeedUnit = (ias_unit.get() == SPEED_UNIT_MPH) ? &mph :
	            (ias_unit.get() == SPEED_UNIT_KNOTS) ? &kts : &kmh;
	VarioUnit = (vario_unit.get() == VARIO_UNIT_KNOTS) ? &kts :
                (vario_unit.get() == VARIO_UNIT_FPM) ? &fpm : &mps;
	TempUnit = (temperature_unit.get() == T_FAHRENHEIT) ? &fahrenheit :
               (temperature_unit.get() == T_KELVIN) ? &kelvin : &celsius;
	DistanceUnit = (dst_unit.get() == DST_UNIT_FT) ? &foot :
                   (dst_unit.get() == DST_UNIT_MILES) ? &mile :
                   (dst_unit.get() == DST_UNIT_NAUTICAL_MILES) ? &nautical_mile : &meter;
	PressureUnit = (qnh_unit.get() == QNH_INHG) ? &inhg : &hpa;
}

}

const Units::unit_t *AltUnit;
const Units::unit_t *SpeedUnit;
const Units::unit_t *VarioUnit;
const Units::unit_t *TempUnit;
const Units::unit_t *DistanceUnit;
const Units::unit_t *PressureUnit;

float Units::Speed(float as)
{
	if (ias_unit.get() == SPEED_UNIT_KMH)
	{ // km/h
		return (as);
	}
	else if (ias_unit.get() == SPEED_UNIT_MPH)
	{ // mph
		return (as * 0.621371);
	}
	else if (ias_unit.get() == SPEED_UNIT_KNOTS)
	{ // knots
		return (as * 0.539957);
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for Speed");
	}
	return 0;
}

float Units::Distance(float d)
{ //
	if (dst_unit.get() == DST_UNIT_M)
	{ // meters per default
		return (d);
	}
	else if (dst_unit.get() == DST_UNIT_FT)
	{ // ft
		return (d * 3.28084);
	}
	else if (dst_unit.get() == DST_UNIT_MILES)
	{ // mi
		return (d * 0.000621371);
	}
	else if (dst_unit.get() == DST_UNIT_NAUTICAL_MILES)
	{ // sm
		return (d * 0.000539957);
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for Distance");
	}
	return d;
}

int Units::SpeedRounded(float as)
{
	float ret = 0;
	if (ias_unit.get() == SPEED_UNIT_KMH)
	{ // km/h
		ret = as;
	}
	else if (ias_unit.get() == SPEED_UNIT_MPH)
	{ // mph
		ret = as * 0.621371;
	}
	else if (ias_unit.get() == SPEED_UNIT_KNOTS)
	{ // knots
		ret = as * 0.539957;
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for AS");
	}
	return fast_iroundf_positive(ret);
}

float Units::kmh2knots(float kmh)
{
	return (kmh / 1.852);
}

float Units::kmh2ms(float kmh)
{
	return (kmh * 0.277778);
}

float Units::ms2kmh(float ms)
{
	return (ms * 3.6);
}

float Units::knots2kmh(float knots)
{
	return (knots * 1.852);
}

float Units::Airspeed2Kmh(float as)
{
	if (ias_unit.get() == SPEED_UNIT_KMH)
	{ // km/h
		return (as);
	}
	else if (ias_unit.get() == SPEED_UNIT_MPH)
	{ // mph
		return (as / 0.621371);
	}
	else if (ias_unit.get() == SPEED_UNIT_KNOTS)
	{ // knots
		return (as / 0.539957);
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for AS");
	}
	return 0;
}

float Units::ActualWingloadCorrection(float v)
{
	return v * std::sqrtf(100.0 / (ballast.get() + 100.0)); // ballast is in percent overweight
}

float Units::TemperatureUnit(float t)
{
	if (temperature_unit.get() == T_CELCIUS)
	{ // °C
		return (t);
	}
	else if (temperature_unit.get() == T_FAHRENHEIT)
	{ // °F
		return ((t * 1.8) + 32);
	}
	else if (temperature_unit.get() == T_KELVIN)
	{ // °K
		return (t + 273.15);
	}
	else
	{
		return (t); // default °C
	}
}

const char* Units::TemperatureUnitStr(int idx)
{
	if (idx == -1)
	{
		idx = temperature_unit.get();
	}
	if (idx == T_FAHRENHEIT)
	{ // °F
		return "'F";
	}
	else if (idx == T_KELVIN)
	{ // °F
		return "'K";
	}
	return "'C"; // default °C
}

const char* Units::SpeedUnitStr(int u)
{
	if (u == -1)
	{
		u = ias_unit.get();
	}
	if (u == SPEED_UNIT_KMH)
	{ // km/h
		return ("kmh");
	}
	else if (u == SPEED_UNIT_MPH)
	{ // mph
		return ("mph");
	}
	else if (u == SPEED_UNIT_KNOTS)
	{ // knots
		return ("kt");
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for airspeed");
	}
	return "none";
}

float Units::Vario(const float te)
{ // standard is m/s
	if (vario_unit.get() == VARIO_UNIT_MS)
	{
		return (te);
	}
	else if (vario_unit.get() == VARIO_UNIT_FPM)
	{
		return (te * 1.9685);
	}
	else if (vario_unit.get() == VARIO_UNIT_KNOTS)
	{
		return (te * 1.94384); // knots
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for Vario");
	}
	return 0;
}

float Units::Qnh(float qnh)
{ // standard is hPa
	if (qnh_unit.get() == QNH_HPA)
	{
		return (qnh);
	}
	else if (qnh_unit.get() == QNH_INHG)
	{
		return (hPa2inHg(qnh));
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for Vario");
	}
	return 0;
}

int Units::QnhRounded(float qnh)
{ // standard is hPa
	float qnh_value = qnh;
	if (qnh_unit.get() == QNH_INHG)
	{
		qnh_value = hPa2inHg(qnh);
	}
	return fast_iroundf_positive(qnh_value);
}

float Units::hPa2inHg(float hpa)
{ // standard is m/s
	return (hpa * 0.02952998597817832);
}
float Units::inHg2hPa(float inhg)
{ // standard is m/s
	return (inhg / 0.02952998597817832);
}

float Units::knots2ms(float knots)
{ // if we got it in knots
	return (knots / 1.94384);
}

float Units::ms2knots(float knots)
{ // if we got it in knots
	return (knots * 1.94384);
}

float Units::ms2mph(float ms)
{ // if we got it in knots
	return (ms * 2.23694);
}

float Units::ms2fpm(float ms)
{ // if we got it in knots
	return (ms * 196.85);
}

float Units::Vario2ms(float var)
{
	if (vario_unit.get() == VARIO_UNIT_MS)
	{
		return (var);
	}
	else if (vario_unit.get() == VARIO_UNIT_FPM)
	{
		return (var / 196.85);
	}
	else if (vario_unit.get() == VARIO_UNIT_KNOTS)
	{
		return (var / 1.94384); // knots
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for Vario");
	}
	return 0;
}

float Units::mcval2knots(float mc)
{ // returns MC, stored according to vario setting, in knots
	if (vario_unit.get() == VARIO_UNIT_MS)
	{ // mc is in m/s
		return (mc * 1.94384);
	}
	else if (vario_unit.get() == VARIO_UNIT_FPM)
	{ // mc is stored in feet per minute
		return (mc * 0.00987472);
	}
	else if (vario_unit.get() == VARIO_UNIT_KNOTS)
	{ // knots we already have
		return (mc);
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for Vario");
	}
	return 0;
}

const char* Units::QnhUnit(int unit)
{
	if (unit == -1)
	{
		unit = qnh_unit.get();
	}
	if (unit == QNH_HPA)
	{
		return ("hPa");
	}
	else if (unit == QNH_INHG)
	{
		return ("inHg");
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for QNH");
	}
	return "nan";
}



float Units::Altitude(float alt, int unit)
{
	int u = unit;
	if (u == -1)
	{
		u = alt_unit.get();
	}
	if (u == ALT_UNIT_METER)
	{ // m
		return (alt);
	}
	else if (u == ALT_UNIT_FT)
	{ // feet
		return (alt * 3.28084);
	}
	else if (u == ALT_UNIT_FL)
	{ // FL
		return (alt * 0.0328084);
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for altitude");
	}
	return 0;
}

float Units::meters2feet(float m)
{
	return (m * 3.28084);
}

float Units::feet2meters(float f)
{
	return (f / 3.28084);
}

float Units::meters2FL(float m)
{
	return (m * 0.0328084);
}

const char* Units::AltitudeUnit(int unit)
{
	int u = unit;
	if (u == -1)
	{
		u = alt_unit.get();
	}
	if (u == ALT_UNIT_METER)
	{ // m
		return ("m");
	}
	else if (u == ALT_UNIT_FT)
	{ // feet
		return ("ft");
	}
	else if (u == ALT_UNIT_FL)
	{ // FL
		return ("FL");
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for altitude %d", u);
	}
	return "nan";
}

const char* Units::AltitudeUnitMeterOrFeet(int unit)
{
	int u = unit;
	if (u == -1)
	{
		u = alt_unit.get();
	}
	if (u == ALT_UNIT_METER)
	{ // m
		return ("m");
	}
	else if (u == ALT_UNIT_FT || u == ALT_UNIT_FL)
	{ // feet
		return ("ft");
	}
	else
	{
		ESP_LOGE(FNAME, "Wrong unit for altitude %d", u);
	}
	return "nan";
}

float Units::value(float val, quantity_t u)
{
	switch (u)
	{
	case quantity_t::QUANT_NONE:
		return val;
	case quantity_t::QUANT_TEMPERATURE:
		return TemperatureUnit(val);
	case quantity_t::QUANT_ALT:
		return Altitude(val);
	case quantity_t::QUANT_HSPEED:
		return Speed(val);
	case quantity_t::QUANT_VSPEED:
		return Vario(val);
	case quantity_t::QUANT_QNH:
		return Qnh(val);
	default:
		return val;
	}
}

const char* Units::unit(quantity_t u)
{
	switch (u)
	{
	case quantity_t::QUANT_NONE:
		return "";
	case quantity_t::QUANT_TEMPERATURE:
		return TempUnit->getName();
	case quantity_t::QUANT_ALT:
		return AltUnit->getName();
	case quantity_t::QUANT_HSPEED:
		return SpeedUnit->getName();
	case quantity_t::QUANT_VSPEED:
		return VarioUnit->getName();
	case quantity_t::QUANT_QNH:
		return PressureUnit->getName();
	default:
		return "";
	}
}
