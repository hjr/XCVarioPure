/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "VarioVoices.h"
#include "VarioShaper.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr int NH = SynthFrame::NH;
constexpr float FULL = 32767.f;
constexpr float TREND_MAX_ST = 1.f;      // semitones

inline float cl01(float x) { return std::min(1.f, std::max(0.f, x)); }
inline float smooth(float x) { x = cl01(x); return x * x * (3.f - 2.f * x); }
inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

// piecewise linear curve over sorted points
struct Pt { float x, y; };
template <int N>
float curve(const Pt (&p)[N], float x)
{
    if ( x <= p[0].x ) { return p[0].y; }
    for (int i = 1; i < N; i++) {
        if ( x <= p[i].x ) { return lerp(p[i - 1].y, p[i].y, (x - p[i - 1].x) / (p[i].x - p[i - 1].x)); }
    }
    return p[N - 1].y;
}

// ---- ProfileVoice constants
// Breathing sink tone, as the Pur tone: rate and level between the breaths over the vario
constexpr Pt BREATH_RATE[] = { {-3.f, .12f}, {-2.f, .16f}, {-1.f, .23f}, {-.5f, .30f}, {0.f, .40f} };
constexpr Pt BREATH_FLOOR[] = { {-3.f, .50f}, {-2.f, .58f}, {-1.f, .69f}, {-.5f, .70f}, {0.f, .70f} };
constexpr float BREATH_IN = .3f;         // share of the breath for swelling
constexpr float BEEP_IN_S = .0063f;      // beep shape: cosine in / out
constexpr float BEEP_OUT_S = .015f;
constexpr float GLIDE_S = .06f;          // pitch smoothing, short to keep the response of the sequencer tone
constexpr float DUAL_OCT = 2.f / 12.f;   // dual tone: major second up
constexpr float BRIGHT_BOOST = 2.f;      // 5th harmonic up to 3 times at speed

// level rule of the sequencer voices: -4 dB from 600 Hz up to 1500 Hz above
inline float maxGain(float f) { return std::max(0.f, (f <= 600.f) ? 128.f : 128.f - 47.f * (f - 600.f) / 1500.f); }

// ---- TableVoice constants (curve and harmonic set layout of the tables)
constexpr int   NF = 48;                 // harmonic sets over the pitch
constexpr float F_LO = 250.f;           // pitch of the first set (Hz)
constexpr float F_OCT = 3.f;            // range of all sets in octaves
constexpr int   LUT_N = 161;            // curve points
constexpr float LUT_V0 = -6.f, LUT_DV = .1f;
constexpr float GL_SPAN = 1.5f;
constexpr float TABLE_GLIDE_S = .12f;
constexpr float BRIGHT_ST = 7.f;        // brighter sink tone at speed, semitones
inline float lerpLut(const uint16_t *t, int i, float f) { return t[i] + (t[i + 1] - t[i]) * f; }
inline float lerpLut(const uint8_t *t, int i, float f) { return t[i] + (t[i + 1] - t[i]) * f; }
} // namespace

// ------------------------------------------------------------------ ProfileVoice
void ProfileVoice::configure(const Params &p)
{
    _p = p;
    _p.range = std::max(1.f, p.range);
    _maxf = p.center * p.toneVar;
    _minf = p.center / p.toneVar;
    _expMax = std::pow(2.f, p.factor);
}

float ProfileVoice::clamp(float v, bool s2f) const
{
    const float max = s2f ? 5.f : _p.range;
    return std::clamp(v, -max, max);
}

void ProfileVoice::frame(const VoiceInput &in, SynthFrame &f) const
{
    const float v = in.v;
    const float range = in.s2f ? 5.f : _p.range;
    const float center = _p.center;

    // pitch, as the sequencer tone
    const float maxVar = (v > 0.f) ? ((_maxf - center) * 2.f) : (center - _minf);
    const float mult = std::pow((std::fabs(v) / range) + 1.f, _p.factor);
    f.hz = std::max(50.f, center + ((mult * v) / range) * (maxVar / _expMax));

    // timbre: pure sine plus the 5th harmonic, in the level relation of the sequencer voices
    // (tone voice gain 220, harmonic voice gain = harmonics setting, each with the level rule)
    const float bright = in.shaper ? in.shaper->brightness() : 0.f;
    const float g0 = maxGain(f.hz) / 128.f;
    const float g5 = (g0 > 0.f) ? g0 * (maxGain(5.f * f.hz) * _p.harmonics) / (maxGain(f.hz) * 220.f) * (1.f + BRIGHT_BOOST * bright) : 0.f;
    const float norm = std::max(1.f, g0 + g5);   // peak never above full scale
    for (int k = 0; k < NH; k++) { f.harm[k] = 0.f; }
    f.harm[0] = g0 / norm * FULL;
    f.harm[4] = g5 / norm * FULL;
    f.phase = nullptr;
    f.gainGlide = true;
    f.glideS = GLIDE_S;
    f.sub = false;

    // beeps: as the sequencer tone, period 1 / (1 + 9 v / range) s, beep 0.9 T - 40 ms, pause 0.1 T + 40 ms
    const bool beeps = v > 0.f && in.chopping;
    const float rateP = beeps ? 1.f + 9.f * v / range : 1.f;
    const float dutyP = beeps ? std::max(.1f, .9f - .04f * rateP) : .5f;
    const float eP = std::min(BEEP_IN_S * rateP, .45f);
    const float erP = std::min(BEEP_OUT_S * rateP, .45f);
    const float floorP = (beeps && !_p.dual) ? 0.f : 1.f;   // steady tone, or dual tone without gap
    float t01 = 1.f; // 0 = breath, 1 = tone of the profile
    if ( _p.breathe ) {
        t01 = smooth((v + .3f) / .7f);
    }
    if ( t01 < 1.f ) {
        const float vb = std::min(v, 0.f);
        f.rate = lerp(curve(BREATH_RATE, vb), rateP, t01);
        f.duty = lerp(.5f, dutyP, t01);
        f.attack = lerp(BREATH_IN, eP, t01);
        f.release = lerp(1.f - BREATH_IN, erP, t01);
        f.floor = lerp(curve(BREATH_FLOOR, vb), floorP, t01);
    }
    else {
        f.rate = rateP;
        f.duty = dutyP;
        f.attack = eP;
        f.release = erP;
        f.floor = floorP;
    }
    f.dualOct = (beeps && _p.dual) ? DUAL_OCT : 0.f;
    const float dvs = in.shaper ? in.shaper->trend() : 0.f;
    f.trendOct = beeps ? t01 * std::clamp(dvs * _p.trendSt, 0.f, TREND_MAX_ST) / 12.f : 0.f;
}

// ------------------------------------------------------------------ TableVoice
float TableVoice::clamp(float v, bool s2f) const
{
    return s2f ? std::clamp(v, -6.f, 6.f) : std::clamp(v, -6.f, 10.f);
}

void TableVoice::frame(const VoiceInput &in, SynthFrame &f) const
{
    const VoiceTables &V = _t;
    const float v = in.v;
    const bool cont = in.s2f && !in.chopping;   // S2F continuous: set 1 (carpet with double beep)
    const float vs = in.shaper ? in.shaper->value() : v;
    const float dvs = in.shaper ? in.shaper->trend() : 0.f;
    const float bright = in.shaper ? in.shaper->brightness() : 0.f;

    // curves (0.1 table, linear interpolation)
    const uint16_t *tR = cont ? V.rate2 : V.rate;
    const uint8_t *tF = cont ? V.floor2 : V.floor, *tD = cont ? V.duty2 : V.duty, *tG = cont ? V.gain2 : V.gain;
    const float x = (v - LUT_V0) / LUT_DV;
    int i = (int)x;
    i = std::clamp(i, 0, LUT_N - 2);
    const float fr = x - i;
    const float hz = lerpLut(V.hz, i, fr) * .25f;
    const float rate = lerpLut(tR, i, fr) * (1.f / 1024.f);
    const float flo = lerpLut(tF, i, fr) * (1.f / 256.f);
    const float duty = lerpLut(tD, i, fr) * (1.f / 256.f);
    const float gain = lerpLut(tG, i, fr) * (1.f / 128.f);

    // envelope shape: soft breath in sink (swell bIn, fall the rest), beeps in climb.
    // Transition from the voice table (env), or between -0.3 and +0.4 m/s.
    const float t01 = (!cont && V.env) ? lerpLut(V.env, i, fr) * (1.f / 255.f) : smooth((v + .3f) / .7f);
    float e = (1.f - t01) * V.bIn + t01 * std::min((cont ? V.attSCont : V.attS) * rate, .2f);
    float er = (1.f - t01) * (1.f - V.bIn) + t01 * std::min(e * V.relRatio, .35f);
    // beep shape of the voice (cosine in / out in fixed times), blended in from half the transition and from shV0 m/s
    if ( !cont && V.shAttS > 0.f ) {
        const float wSh = smooth((t01 - .25f) / .5f) * smooth((v - V.shV0) / (V.shV1 - V.shV0));
        if ( wSh > 0.f ) {
            e += (std::min(V.shAttS * rate, .45f) - e) * wSh;
            er += (std::min(V.shRelS * rate, .45f) - er) * wSh;
        }
    }
    f.hz = hz;
    f.rate = rate;
    f.duty = duty;
    f.attack = e;
    f.release = er;
    f.floor = flo;
    f.dualOct = 0.f;
    f.trendOct = cont ? 0.f : std::clamp(dvs * V.trendSt, 0.f, TREND_MAX_ST) / 12.f;
    f.glideS = TABLE_GLIDE_S;
    f.sub = cont && t01 > .5f;
    f.phase = V.phase;
    f.gainGlide = V.gainGlide;

    // timbre: harmonic set of the pitch, blended between sink / climb / glanz.
    // At speed the sink part takes the set of a lower pitch (more harmonics, same pitch).
    const float mix = smooth((vs + 1.5f) / 1.5f), gl = V.glMax * smooth((vs - V.glStart) / GL_SPAN);
    float pos = (std::log2(hz / F_LO) - BRIGHT_ST * bright / 12.f) / F_OCT * (NF - 1);
    pos = std::clamp(pos, 0.f, (float)(NF - 1));
    int t = (int)pos;
    if ( t > NF - 2 ) { t = NF - 2; }
    const float tf = pos - t;
    const float gH = gain * V.harmGain;
    const float wS = gH * (1.f - mix), wC = gH * mix * (1.f - gl), wG = gH * mix * gl;
    for (int k = 0; k < NH; k++) {
        const int i0 = t * NH + k, i1 = i0 + NH;
        f.harm[k] = wS * (V.harmSink[i0] + (V.harmSink[i1] - V.harmSink[i0]) * tf)
                  + wC * (V.harmClimb[i0] + (V.harmClimb[i1] - V.harmClimb[i0]) * tf)
                  + wG * (V.harmGlanz[i0] + (V.harmGlanz[i1] - V.harmGlanz[i0]) * tf);
    }
}
