/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

// Shared audio tables, all generated at compile time from one set of constexpr helpers.
// Used by the sequencer voices (ESPAudio) and the vario synthesizer (VarioSynth).

#include <array>
#include <cstdint>

namespace audiotab {

constexpr double PI = 3.14159265358979323846;

// Portable constexpr sine (range reduction and Taylor series), error < 1e-12
constexpr double csin(double x)
{
    while ( x > PI ) { x -= 2. * PI; }
    while ( x < -PI ) { x += 2. * PI; }
    const double x2 = x * x;
    double term = x, sum = x;
    for (int n = 1; n < 20; n++) {
        term *= -x2 / ((2. * n) * (2. * n + 1.));
        sum += term;
    }
    return sum;
}
constexpr double ccos(double x) { return csin(x + PI / 2.); }
// 2^x for 0 <= x <= 1
constexpr double cexp2(double x)
{
    const double y = x * 0.69314718055994530942;
    double term = 1., sum = 1.;
    for (int n = 1; n < 30; n++) {
        term *= y / n;
        sum += term;
    }
    return sum;
}
constexpr long long cround(double x) { return x >= 0. ? (long long)(x + 0.5) : -(long long)(-x + 0.5); }

// One period of a sine with N samples and the given amplitude
template <typename T, int N>
constexpr std::array<T, N> makeSine(double amplitude)
{
    std::array<T, N> t{};
    for (int i = 0; i < N; i++) {
        t[i] = (T)cround(amplitude * csin(2. * PI * i / N));
    }
    return t;
}

constexpr int SIN_BITS = 8;
constexpr int SIN_LEN = 1 << SIN_BITS;
constexpr int RC_LEN = 65;     // raised cosine 0 -> 32767 in 64 steps (+1 for interpolation)
constexpr int EXP2_LEN = 65;   // 2^(i/64) in Q30 (+1 for interpolation)

// 16 bit sine, Q15
extern const std::array<int16_t, SIN_LEN> SIN_Q15;
// Raised cosine (1 - cos) / 2 over half a period, Q15
extern const std::array<int16_t, RC_LEN> RC_Q15;
// 2^(i/64), Q30 (last entry clamped to INT32_MAX)
extern const std::array<uint32_t, EXP2_LEN> EXP2_Q30;

} // namespace audiotab
