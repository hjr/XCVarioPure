#include "Atmosphere.h"

#include <cmath>

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

mps_t Atmosphere::IAS(mps_t tas, pascal_t sp, kelvin_t temp) {
    return tas / std::sqrtf(Units::rho0 / (sp / (Units::R_air * temp)));
}

meter_t Atmosphere::calcAltitude(pascal_t SeaLevel_Pres, pascal_t pressure) {
    return (Units::T0divL * (1.0f - std::powf(pressure / SeaLevel_Pres, 1.0f / Units::g0divRxL)));
}

// respect temp laps 6.5 K / km
pascal_t Atmosphere::calcPressure(pascal_t seaLevelPressure, meter_t altitude) {
    return seaLevelPressure * std::powf(1.0f - (altitude / Units::T0divL), Units::g0divRxL);
}

pascal_t Atmosphere::calcQNHPressure(pascal_t pressure, meter_t altitude) {
    return pressure / std::powf((1.0f - (Units::L * altitude) / Units::T0), 5.255f);
}
