#include "Atmosphere.h"

#include <cmath>


#include "setup/SetupNG.h"

namespace Units
{

void setAll() {
    AltUnit = (alt_unit.get() == (int)alt_unit_t::ALT_UNIT_FT)   ? &foot
              : (alt_unit.get() == (int)alt_unit_t::ALT_UNIT_FL) ? &flightlevel
                                                                 : &meter;
    SpeedUnit = (ias_unit.get() == SPEED_UNIT_MPH)      ? &mph
                : (ias_unit.get() == SPEED_UNIT_KNOTS)  ? &kts 
                                                        : &kmh;
    VarioUnit = (vario_unit.get() == VARIO_UNIT_KNOTS)  ? &kts
                : (vario_unit.get() == VARIO_UNIT_FPM)  ? &hfpm 
                                                        : &mps;
    TempUnit = (temperature_unit.get() == T_FAHRENHEIT) ? &fahrenheit 
                : (temperature_unit.get() == T_KELVIN)  ? &kelvin 
                                                        : &celsius;
    DistanceUnit = (dst_unit.get() == DST_UNIT_FT)               ? &foot
                   : (dst_unit.get() == DST_UNIT_MILES)          ? &mile
                   : (dst_unit.get() == DST_UNIT_NAUTICAL_MILES) ? &naut_mile
                                                                 : &meter;
    PressureUnit = (qnh_unit.get() == QNH_INHG) ? &inhg : &hpa;
}

}  // namespace Units

const Units::unit_t *AltUnit;
const Units::unit_t *SpeedUnit;
const Units::unit_t *VarioUnit;
const Units::unit_t *TempUnit;
const Units::unit_t *DistanceUnit;
const Units::unit_t *PressureUnit;



// With density of water from: http://www.csgnetwork.com/waterinformation.html
// @ 22.8 degree: 0.997585
// earth gravity: 9.0807 m/s^2
// and standard ICAO air density with 1.225 kg/m3 there is:
// V(m/s) = sqrt(2*( <mmH2O> * 0.997585 * 9.807 ) / 1.225)


mps_t Atmosphere::TAS(mps_t ias, pascal_t sp, kelvin_t temp) {
    return ias * sqrtf( (Units::rho0 * Units::R_air * temp) / sp );
}

// True Airspeed from dynamic pressure (incompressible, low Mach)
// q = 0.5 * rho * v²
mps_t TasFromDp(pascal_t q, kelvin_t temp)
{
    float rho = Units::P0 / (Units::R_air * temp);
    return sqrtf(2.0f * q / rho);
}


// TAS=IAS/sqrt( 288.15/(T+273.15) * (P/1013.25) )
// float Atmosphere::TAS2(float ias, float altitude, float temp) {
//     return (ias / std::sqrtf(288.15 / (temp + 273.15) * (calcPressureISA(altitude) / 1013.25)));
// }

