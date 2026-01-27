#include "Atmosphere.h"

#include <cmath>

// With density of water from: http://www.csgnetwork.com/waterinformation.html
// @ 22.8 degree: 0.997585
// earth gravity: 9.0807 m/s^2
// and standard ICAO air density with 1.225 kg/m3 there is:
// V(km/h) = sqrt(2*( <mmH2O> * 0.997585 * 9.807  )/1.225) * 3.6

//   Speed
// mmH2O m/s	km/h
// 100	 40,0	143,9
// 105          147,4
// 110	 41,9	150,9
// 116	 43,0	155,0
// 120	 43,8	157,6
// 130	 45,6	164,0
// 140	 47,3	170,2

// Standard Atmosphere Equations:
//
// p0 = air pressure at sea level
//
// ℎ = altitude above sea level
// 
// 𝑇 = 288,15𝐾
//
// T0 = 288,15K
//
// 𝑔 = 9,80665 𝑚/s^2
//
// 𝑀 = 0,0289644 𝑘𝑔/𝑚𝑜𝑙
//
// 𝑅 = 8,3144598 𝐽/(𝑚𝑜𝑙⋅𝐾)
//
// Exponent:
// E = R⋅L/g⋅M ≈ 5,255
//
// p = p0​⋅(1−​L⋅h/T0​)^E
//

mps_t Atmosphere::TAS(mps_t ias, meter_t altitude, kelvin_t temp) {
    pascal_t p = calcPressureISA(altitude);
    return ias * sqrtf( (Units::rho0 * Units::R_air * temp) / p );
}

// TAS=IAS/sqrt( 288.15/(T+273.15) * (P/1013.25) )
// float Atmosphere::TAS2(float ias, float altitude, float temp) {
//     return (ias / std::sqrtf(288.15 / (temp + 273.15) * (calcPressureISA(altitude) / 1013.25)));
// }

// float Atmosphere::CAS(float dp) {
//     return (1225.0 * std::sqrtf(5.0f * (std::powf((dp / 101325.0f) + 1.0, (2.0 / 7)) - 1.0)));
// }

mps_t Atmosphere::IAS(mps_t tas, float alti, float temp) {
    return (tas / std::sqrtf(1.225f / (calcPressureISA(alti) * 100.0f / (287.058f * (Units::C2K + temp)))));
}

// float Atmosphere::pascal2kmh(pascal_t baro) {
//     if ( baro < 0.0f ) {
//         return 0.0f;
//     }
//     return std::sqrtf(2.f * baro / 1.225f) * 3.6;
// }

mps_t Atmosphere::pascal2ms(pascal_t baro) {
    if ( baro < 0.0f ) {
        return 0.0f;
    }
    return std::sqrtf(2.f * baro / 1.225f);
}

pascal_t Atmosphere::kmh2pascal(float kmh) {
    return ((kmh / 3.6) * (kmh / 3.6)) * 1.225 / 2.;
}

float Atmosphere::calcAltitude(pascal_t SeaLevel_Pres, pascal_t pressure) {
    return (44330.0f * (1.0f - std::powf((float)(pressure) / SeaLevel_Pres, (1.0f / 5.255f))));
}

// respect temp laps 6.5 K / km
pascal_t Atmosphere::calcPressure(pascal_t seaLevelPressure, meter_t altitude) {
    return (seaLevelPressure * std::powf((1.0f - ((float)(altitude) / 44330.76923f)), 5.255f));
    // return seaLevelPressure * std::powf(1.0f - (Units::L * altitude) / Units::T0, 5.25588f);
}

float Atmosphere::calcQNHPressure(float pressure, float altitude) {
    return pressure / std::powf((1.0f - (Units::L * altitude) / Units::T0), 5.255f);
}
