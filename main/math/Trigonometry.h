/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "Units.h"

#include <cstdint>

// general trigonometric helper
constexpr double My_PId = 3.14159265358979323846;
constexpr float My_PIf = static_cast<float>(My_PId);
constexpr float PI2f = 2.0f * static_cast<float>(My_PId);

constexpr double deg2rad(double degrees) { return degrees * (My_PId / 180.0); }
constexpr rad_t deg2rad(degree_t degrees)   { return degrees * (My_PIf / 180.0); }
constexpr double rad2deg(double rad)     { return rad * 180.0 / My_PId; }
constexpr degree_t rad2deg(rad_t rad)       { return rad * 180.0 / My_PIf; }

// integer angle math
int normalizeDeg(int angle);
int angleDiffDeg(int a1, int a2);

// fast gauge routines with reduced precision
rad_t fast_sin_deg(degree_t angle);
rad_t fast_cos_deg(degree_t angle);
rad_t fast_sin_rad(rad_t rad);
rad_t fast_cos_rad(rad_t rad);
rad_t fast_sin_idx(int16_t idx);
rad_t fast_cos_idx(int idx);

float fast_log2f(float x);

rad_t fast_atan(float x);
rad_t fast_atan2(float y, float x);
rad_t fast_tan_deg(degree_t deg);
rad_t fast_tan_rad(rad_t rad);
int count_digits(unsigned int n);

