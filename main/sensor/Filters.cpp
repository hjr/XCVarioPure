/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Filters.h"
#include "math/vector_3d.h"


template <typename T>
T LowPassFilterT<T>::filter(T input)
{
    _last_output = (input - _last_output) * _alpha + _last_output;
    return _last_output;
}

// we explicitly need those instantiations
template class LowPassFilterT<float>;
template class LowPassFilterT<vector_f>;