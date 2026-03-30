/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

// compare
bool floatEqual(float a, float b, float eps = 1e-6f);
bool floatEqualFast(float a, float b);
bool floatEqualFastAbs(float a, float b, float tol = 1e-5f);

// rounding to nearest int, symmetric for negative values
inline constexpr int fast_iroundf(float a) {
    return (int)((a >= 0.0f) ? (a + 0.5f) : (a - 0.5f));
}
// for purely positive values
inline constexpr int fast_iroundf_positive(float a) {
    return (int)(a + 0.5f);
}

// flooring, ceiling
float fast_floorf(float x);
float fast_ceilf(float x);

// float sign
float fast_signf(float val);
