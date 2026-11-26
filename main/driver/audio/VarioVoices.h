/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

// Vario voices: describe what the vario tone sounds like, as one SynthFrame per update.
//  - ProfileVoice: computed from the tone style settings (center frequency, tone variation, response,
//    range, chopping, dual tone, harmonics, sink tone), no tables.
//  - TableVoice:   curves and harmonic sets from tables (the "Pur" tone of the Klar prototype).

#include "VarioSynth.h"

#include <cstdint>

class VarioShaper;

struct VoiceInput {
    float v;                    // value, clamped by the voice: vario in m/s, or S2F delta in 10 km/h (+ = too fast)
    bool  s2f;                  // S2F command
    bool  chopping;             // tone chopping in the current mode
    const VarioShaper *shaper;  // smoothed value, trend, brightness
};

class VarioVoice
{
public:
    virtual ~VarioVoice() = default;
    virtual float clamp(float v, bool s2f) const = 0;
    // fills everything but on and vol
    virtual void frame(const VoiceInput &in, SynthFrame &f) const = 0;
};

class ProfileVoice : public VarioVoice
{
public:
    struct Params {
        float center = 500.f;     // Hz at zero
        float toneVar = 2.f;      // max. pitch variation (fold)
        float factor = 1.f;       // response exponent
        float range = 5.f;        // vario range in m/s (S2F: 5 units)
        bool  dual = false;       // dual tone (di/da)
        int   harmonics = 21;     // level of the 5th harmonic, as e_audio_harmonics
        bool  breathe = false;    // breathing sink tone instead of a steady one
        float trendSt = 0.f;      // pitch slide per m/s^2 within the beep, semitones (0 = off)
    };
    void configure(const Params &p);
    float clamp(float v, bool s2f) const override;
    void frame(const VoiceInput &in, SynthFrame &f) const override;

private:
    Params _p;
    float _minf = 250.f, _maxf = 1000.f, _expMax = 2.f;
};

// Curves (0.1 steps from -6 to +10) and harmonic sets (48 sets over 3 octaves from 250 Hz, 12 harmonics each)
struct VoiceTables {
    const int16_t *harmClimb, *harmSink, *harmGlanz;     // NF x NH, climb / sink / glanz
    const uint16_t *hz;                                  // pitch, Q2 (Hz x 4)
    const uint16_t *rate, *rate2;                        // beeps or breaths per second, Q10; set 0 / set 1 (S2F continuous)
    const uint8_t *floor, *duty, *gain;                  // level between beeps Q8, duty Q8, level (loudness compensation) Q7
    const uint8_t *floor2, *duty2, *gain2;
    const uint8_t *env;                                  // breath -> beep, Q8 (0 = breath, 255 = beep); nullptr = default transition
    const uint8_t *phase;                                // harmonic phases
    float attS, attSCont, relRatio;                      // beep rise (s), double beep rise (s), fall / rise
    float trendSt, glStart, glMax, bIn;                  // trend (semitones per m/s^2), glanz from / at most, breath swell share
    float shAttS, shRelS, shV0, shV1;                    // beep shape: cosine in / out (s), blended in from shV0 to shV1 (m/s); shAttS = 0: none
    bool gainGlide;                                      // even out level steps on the waveform swap
    float harmGain;                                      // factor on the harmonic tables
};
extern const VoiceTables VOICE_PUR;

class TableVoice : public VarioVoice
{
public:
    explicit TableVoice(const VoiceTables &t) : _t(t) {}
    float clamp(float v, bool s2f) const override;
    void frame(const VoiceInput &in, SynthFrame &f) const override;

private:
    const VoiceTables &_t;
};
