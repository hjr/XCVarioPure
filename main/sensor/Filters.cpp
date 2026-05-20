/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Filters.h"
#include "math/vector_3d.h"
#include "logdefnone.h"

#include <algorithm>

template <typename T>
T LowPassFilterT<T>::filter(T input)
{
    _last_output = (input - _last_output) * _alpha + _last_output;
    return _last_output;
}

float ZeroOutGatingLPFilter::filter(float input)
{
    float tmp = LowPassFilterT<float>::filter(input);
    if (tmp < _threshold) {
        return 0.f;
    }
    return tmp;
}

template <typename T>
T AdaptiveLowPassFilterT<T>::filter(T input)
{
    float err = std::abs(input - LowPassFilterT<float>::_last_output);
    // increase faster than decrease to be more responsive to changes, but still smooth out noise
    _activity += _beta * (err - _activity) * (_activity > err ? 1.2f : 0.5f);

    float tmp = std::clamp(_activity / _threshold, 0.0f, 1.0f);
    LowPassFilterT<float>::_alpha = _alpha_min + tmp * tmp * (_alpha_max - _alpha_min);
    ESP_LOGI(FNAME, "output=%f, alpha=%f err=%f", LowPassFilterT<float>::_last_output, LowPassFilterT<float>::_alpha, err);

    return static_cast<T>(LowPassFilterT<float>::filter(input));
}

// we explicitly need those instantiations
template class LowPassFilterT<float>;
template class LowPassFilterT<vector_f>;
template class AdaptiveLowPassFilterT<float>;
