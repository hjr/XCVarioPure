
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


