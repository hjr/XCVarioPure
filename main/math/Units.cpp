
#include "Units.h"

#include "Floats.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

namespace Units
{


void setAll()
{
	AltUnit = (alt_unit.get() == (int)alt_unit_t::ALT_UNIT_FT) ? &foot :
              (alt_unit.get() == (int)alt_unit_t::ALT_UNIT_FL) ? &flightlevel : &meter;
	SpeedUnit = (ias_unit.get() == SPEED_UNIT_MPH) ? &mph :
	            (ias_unit.get() == SPEED_UNIT_KNOTS) ? &kts : &kmh;
	VarioUnit = (vario_unit.get() == VARIO_UNIT_KNOTS) ? &kts :
                (vario_unit.get() == VARIO_UNIT_FPM) ? &fpm : &mps;
	TempUnit = (temperature_unit.get() == T_FAHRENHEIT) ? &fahrenheit :
               (temperature_unit.get() == T_KELVIN) ? &kelvin : &celsius;
	DistanceUnit = (dst_unit.get() == DST_UNIT_FT) ? &foot :
                   (dst_unit.get() == DST_UNIT_MILES) ? &mile :
                   (dst_unit.get() == DST_UNIT_NAUTICAL_MILES) ? &naut_mile : &meter;
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

// float Units::meters2feet(float m)
// {
// 	return (m * 3.28084);
// }

float Units::feet2meters(float f)
{
	return (f / 3.28084);
}

float Units::meters2FL(float m)
{
	return (m * 0.0328084);
}

float Units::value(float val, quantity_t u)
{
	switch (u)
	{
	case quantity_t::QUANT_NONE:
		return val;
	case quantity_t::QUANT_TEMPERATURE:
		return TempUnit->apply(val);
	case quantity_t::QUANT_ALT:
		return AltUnit->apply(val);
	case quantity_t::QUANT_HSPEED:
		return SpeedUnit->apply(val);
	case quantity_t::QUANT_VSPEED:
		return VarioUnit->apply(val);
	case quantity_t::QUANT_QNH:
		return PressureUnit->apply(val);
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
