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
#include "logdef.h"

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


// float TEVariometerFilter::filter(float input) {

//     float tasraw = Atmosphere::TAS2( ias.get(), altitude.get(), OAT.get()); // True airspeed in km/h
//     input = std::roundf(bmpVario->readTE(tasraw, input) * 20.); // TE value caclulation
//     if ( !floatEqualFast(_oldte, input) ) {
//         _oldte = input;
//     }

//     return _oldte/20.f;
// }
