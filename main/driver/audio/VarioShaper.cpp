/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "VarioShaper.h"

#include <cmath>
#include <cstring>
#include <algorithm>

namespace {
constexpr float PI_F = 3.14159265f;
constexpr float LN10_20 = 0.11512925f;              // ln(10) / 20, dB to natural log

// vario smoothing and trend
constexpr float VS_TAU = 0.25f, DVS_TAU = 0.30f;   // s
// S2F edge outside the dead band, in units of 10 km/h
constexpr float S2F_EDGE = 0.5f;
constexpr float S2F_T_DOWN = 0.15f, S2F_T_UP = 0.30f, S2F_GAP = 0.60f;   // s to 90 %, min. pause on side change
// speed corner values for the reference glider (DG 800S/18), scaled by the polar factor
constexpr float SPD_V_LO = 90.f, SPD_V_HI = 160.f;         // km/h, full quiet below / full level above
constexpr float BRIGHT_V_LO = 110.f, BRIGHT_V_HI = 180.f;  // km/h, brightness ramp
constexpr float POLAR_V15_REF = 159.f, POLAR_F_MIN = 0.75f; // speed at 1.5 m/s sink of the reference polar
// habituation
constexpr float HAB_RAMP_S = 60.f, HAB_THR = 0.5f, HAB_AVG_S = 5.f, HAB_UP_TAU = 0.15f;

inline float cl01(float x) { return std::min(1.f, std::max(0.f, x)); }
inline float smooth(float x) { x = cl01(x); return x * x * (3.f - 2.f * x); }
inline float dbToLin(float db) { return std::exp(db * LN10_20); }
inline float lpCoef(float dt, float tau) { return tau > 0.001f ? 1.f - std::exp(-dt / tau) : 1.f; }
} // namespace

void VarioShaper::reset()
{
    _started = false;
    _snap = true;
    _zSide = 0; _zQuiet = _zGap = 0.f;
    _habInit = false; _habG = 1.f;
    _zone = _spdG = 1.f; _brightSpd = 0.f;
    _gain = 1.f; _bright = 0.f;
}

void VarioShaper::setCalmZone(float attDb, float widthMs, float tDown90, float tUp90)
{
    _rzAttDb = std::max(0.f, attDb);
    _rzWidth = std::max(0.05f, widthMs);
    _rzTauDown = tDown90 / 2.3026f; // 90 % after 2.3 time constants
    _rzTauUp = tUp90 / 2.3026f;
}

void VarioShaper::setPolar(float v1, float w1, float v2, float w2, float v3, float w3)
{
    const float p[6] = { v1, w1, v2, w2, v3, w3 };
    if ( std::memcmp(p, _polar, sizeof(p)) == 0 ) {
        return;
    }
    std::memcpy(_polar, p, sizeof(p));

    // Parabola through the three polar points, speed at 1.5 m/s sink.
    // Only scales down (slow gliders), at most to POLAR_F_MIN. Invalid polar: factor 1.
    float f = 1.f;
    const float d12 = v1 - v2, d13 = v1 - v3, d23 = v2 - v3;
    if ( std::fabs(d12) > 1.f && std::fabs(d13) > 1.f && std::fabs(d23) > 1.f ) {
        const float l1 = w1 / (d12 * d13), l2 = w2 / (-d12 * d23), l3 = w3 / (d13 * d23);
        const float a2 = l1 + l2 + l3;
        const float a1 = -(l1 * (v2 + v3) + l2 * (v1 + v3) + l3 * (v1 + v2));
        const float a0 = l1 * v2 * v3 + l2 * v1 * v3 + l3 * v1 * v2 + 1.5f;
        const float D = a1 * a1 - 4.f * a2 * a0;
        if ( a2 < 0.f && D >= 0.f ) {
            const float r1 = (-a1 + std::sqrt(D)) / (2.f * a2), r2 = (-a1 - std::sqrt(D)) / (2.f * a2);
            const float v15 = std::max(r1, r2);
            if ( v15 > 40.f && v15 < 400.f ) {
                f = std::min(1.f, std::max(POLAR_F_MIN, v15 / POLAR_V15_REF));
            }
        }
    }
    _polF = f;
}

void VarioShaper::update(const Input &in)
{
    const float dt = std::min(0.5f, std::max(0.01f, in.dt));
    const float v = in.v;
    if ( in.s2f != _s2f ) {
        // mode change: restart the mode specific states
        _s2f = in.s2f;
        _zSide = 0; _zQuiet = _zGap = 0.f;
        _habInit = false; _habG = 1.f;
    }

    // smoothed value and its rate of change (trend)
    if ( !_started ) { _vs = _vsPrev = v; _dvs = 0.f; _started = true; }
    _vs += lpCoef(dt, VS_TAU) * (v - _vs);
    _dvs += lpCoef(dt, DVS_TAU) * ((_vs - _vsPrev) / dt - _dvs);
    _vsPrev = _vs;

    // speed: quieter when slow, brighter when fast, corner values scaled by the polar, 1 s smoothing
    {
        const float f = _polF, ias = std::max(0.f, in.iasKmh);
        const float spdT = dbToLin(-_spdQuietDb * cl01((SPD_V_HI * f - ias) / ((SPD_V_HI - SPD_V_LO) * f)));
        const float brT = _brightOn ? cl01((ias - BRIGHT_V_LO * f) / ((BRIGHT_V_HI - BRIGHT_V_LO) * f)) : 0.f;
        const float a = _snap ? 1.f : lpCoef(dt, 1.f);
        _spdG += (spdT - _spdG) * a;
        _brightSpd += (brT - _brightSpd) * a;
    }

    // habituation (vario only): the 5 s mean stays close to the reference -> after start time slowly quieter
    if ( !_s2f && _habDepthDb > 0.f ) {
        if ( !_habInit ) { _habAvg = _habRef = v; _habT = 0.f; _habInit = true; }
        _habAvg += lpCoef(dt, HAB_AVG_S) * (v - _habAvg);
        if ( std::fabs(_habAvg - _habRef) > HAB_THR ) { _habRef = _habAvg; _habT = 0.f; }
        else { _habT += dt; }
        const float tg = dbToLin(-_habDepthDb * cl01((_habT - _habStartS) / HAB_RAMP_S));
        _habG = (tg < _habG) ? tg : _habG + (tg - _habG) * lpCoef(dt, HAB_UP_TAU);
    }
    else {
        _habG = 1.f; _habInit = false;
    }

    // zone: calm zone around netto zero (vario), or soft dead band edge with side change pause (S2F)
    float g = 1.f, tauDown = _rzTauDown, tauUp = _rzTauUp;
    if ( _s2f && _s2fEdge ) {
        float dbPos = in.dbPos, dbNeg = in.dbNeg;
        if ( dbPos < dbNeg ) { dbPos = dbNeg = 0.f; } // no dead band configured
        int side = 0;
        if ( v > dbPos ) {
            side = 1;
            g = 0.5f - 0.5f * std::cos(PI_F * cl01((v - dbPos) / S2F_EDGE));
        }
        else if ( v < dbNeg ) {
            side = -1;
            g = 0.5f - 0.5f * std::cos(PI_F * cl01((dbNeg - v) / S2F_EDGE));
        }
        else {
            g = 0.f;
        }
        if ( side == 0 ) {
            _zQuiet += dt;
        }
        else {
            if ( _zSide != 0 && side != _zSide && _zQuiet < S2F_GAP ) { _zGap = S2F_GAP - _zQuiet; }
            _zSide = side;
            _zQuiet = 0.f;
        }
        if ( _zGap > 0.f ) { _zGap -= dt; g = 0.f; }
        tauDown = S2F_T_DOWN / 2.3026f;
        tauUp = S2F_T_UP / 2.3026f;
    }
    else if ( !_s2f && in.circling && _rzAttDb > 0.f ) {
        const float x = std::fabs(in.netto) / _rzWidth; // half cosine up to |netto| = width
        if ( x < 1.f ) { g = dbToLin(-_rzAttDb * 0.5f * (1.f + std::cos(PI_F * x))); }
    }
    if ( _snap || (_s2f && _s2fEdge && g == 0.f && _zGap > 0.f) ) {
        _zone = g; // start, or side change pause: no fading
    }
    else {
        _zone += (g - _zone) * lpCoef(dt, g > _zone ? tauUp : tauDown);
    }

    // results
    _gain = std::min(1.f, _zone * _spdG * _habG);
    const float sinkPart = 1.f - smooth((_vs + 1.5f) / 1.5f); // brightness applies to the sink tone only
    _bright = _brightSpd * sinkPart;
    _snap = false;
}
