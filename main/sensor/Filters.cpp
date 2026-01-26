/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Filters.h"

#include "SensorBase.h"
#include "VarioFilter.h"
#include "Atmosphere.h"
#include "math/Units.h"
#include "setup/SetupNG.h"
#include "math/Floats.h"
#include "sensor.h"

#include <cmath>

void LowPassFilter::reset(float init_val)
{
    _last_output = init_val;
}
float LowPassFilter::filter(float input)
{
    _last_output = _alpha * (input - _last_output) + _last_output;
    return _last_output;
}

mps_t AirSpeedFilter::filter(pascal_t input)
{
    float tmp = Atmosphere::pascal2ms(_lpf.filter(input));
    // clamp to zero for speeds < 15km/h (to avoid noise around zero)
    if ( tmp < Units::kmh_to_mps(15.0f) ) {
        tmp = 0.0f;
    }
    return tmp;
}

float AltimeterFilter::filter(float input)
{
    float new_alt = Atmosphere::calcAltitudeISA(input);
    altitude_isa.set(new_alt);
    // fixme units need to move to display layer

    if ( (alt_unit.get() == ALT_UNIT_FL) 
        || ((fl_auto_transition.get() == 1) 
            && ((int)Units::meters2FL(new_alt) + (int)gflags.standard_setting > transition_alt.get())) ) {
        // FL, always standard or above transition altitude
        gflags.standard_setting = true;
    } else {
        new_alt = Atmosphere::calcAltitude(QNH.get(), input);
        gflags.standard_setting = false;
    }
    return new_alt;
}

// float TEVariometerFilter::filter(float input) {

//     float tasraw = Atmosphere::TAS2( ias.get(), altitude.get(), OAT.get()); // True airspeed in km/h
//     input = std::roundf(bmpVario->readTE(tasraw, input) * 20.); // TE value caclulation
//     if ( !floatEqualFast(_oldte, input) ) {
//         _oldte = input;
//     }

//     return _oldte/20.f;
// }
