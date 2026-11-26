/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

// Context dependent shaping of the vario tone, independent of the sound profile in use.
// Taken from the "Klar" prototype, without its own synthesizer:
//  - Dead band:   with a depth in dB, quieter (not silent) inside the vario dead band while circling
//  - Habituation: quieter when the vario value stays the same for a long time (vario mode only)
//  - Speed:       quieter when flying slowly, brighter sink tone at speed (corner speeds scaled by the polar)
//  - S2F:         soft edge just outside the dead band, and a pause when the command changes side
//
// Runs in the audio task at ~10 Hz, float math only, no tables. The results are a level factor and
// a brightness, plus the smoothed value and its rate of change for the voices (timbre blend, trend slide).
// With all features switched off it still tracks the value, and gain() stays 1.
class VarioShaper
{
public:
    struct Input {
        float v;        // vario in m/s, or S2F delta in units of 10 km/h (+ = too fast)
        float dt;       // seconds since last call
        bool  s2f;      // true: S2F command, false: vario
        bool  circling; // the soft vario dead band applies
        float iasKmh;   // indicated airspeed in km/h, 0 = unknown (treated as slow)
        float db;       // dead band half width of the current mode, same unit as v
    };

    void reset();
    void update(const Input &in);

    // Three polar points as in the setup (km/h, m/s with sink negative). Derives a factor for the speed
    // corner values, slow gliders reach full level earlier. Cheap to call repeatedly, recalculates on change only.
    void setPolar(float v1, float w1, float v2, float w2, float v3, float w3);

    // Parameters (defaults from the prototype)
    // soft vario dead band: depth at zero (dB), fading out to the dead band edge; 90 % times quieter / louder. depthDb = 0 disables
    void setDeadbandDepth(float depthDb, float tDown90, float tUp90);
    void setHabituation(float depthDb, float startS) { _habDepthDb = depthDb; _habStartS = startS; }  // depthDb = 0 disables
    void setSpeedAdapt(float quietDb, bool bright) { _spdQuietDb = quietDb; _brightOn = bright; }
    void setS2fEdge(bool on) { _s2fEdge = on; }                                 // soft dead band edges and side change pause

    // Results
    float gain() const { return _gain; }               // 0 .. 1, level factor
    float brightness() const { return _bright; }       // 0 .. 1, brighter sink tone at speed
    float value() const { return _vs; }                // smoothed value (250 ms)
    float trend() const { return _s2f ? 0.f : _dvs; }  // rate of change of the vario, m/s^2 (0 in S2F)
    bool  silent() const { return _gain < 0.01f; }

private:
    // smoothing state
    bool  _started = false, _snap = true;
    bool  _s2f = false;
    float _vs = 0.f, _vsPrev = 0.f, _dvs = 0.f;
    float _zone = 1.f;                               // calm zone or S2F edge, smoothed
    // S2F side change pause: side (-1, 0, +1), quiet time since leaving the side, remaining pause
    int   _zSide = 0; float _zQuiet = 0.f, _zGap = 0.f;
    // speed
    float _polF = 1.f, _spdG = 1.f, _brightSpd = 0.f;
    float _polar[6] = {};
    // habituation
    float _habAvg = 0.f, _habRef = 0.f, _habT = 0.f, _habG = 1.f;
    bool  _habInit = false;
    // results
    float _gain = 1.f, _bright = 0.f;

    // parameters
    float _rzAttDb = 0.f, _rzTauDown = 2.f / 2.3026f, _rzTauUp = 0.5f / 2.3026f;
    float _habDepthDb = 3.f, _habStartS = 60.f;
    float _spdQuietDb = 6.f;
    bool  _brightOn = true;
    bool  _s2fEdge = true;
};
