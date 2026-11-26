/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

// Vario tone synthesizer for the 8 bit DAC, derived from the "Klar" prototype.
//
// Split of work:
//  - update() runs in the audio task (float allowed), ~10 times per second with a new SynthFrame.
//    It computes the beep envelope shape and builds one waveform (257 samples) from the harmonics
//    of the frame, then hands both over to the interrupt through a double buffer.
//  - render() runs in the DMA interrupt, integer only. Per sample: phase accumulator, one table read
//    with linear interpolation, envelope ramp and noise shaping. Every 64 samples (~1 ms) chunk()
//    advances the beep envelope, double beep gap, pitch glide, trend slide, dual tone and level.
//
// What the tone sounds like is not defined here: a VarioVoice (VarioVoices.h) fills the SynthFrame
// from the vario value, either from the tone style settings or from tables.

#include <atomic>
#include <cstdint>

// DAC level: 1 = full +-127 steps (6 dB more distance to the rounding noise), 0 = +-64
#ifndef VS_DAC_FULL
#define VS_DAC_FULL 1
#endif

struct SynthFrame {
    static constexpr int NH = 12;   // harmonics
    float hz = 500.f;               // pitch
    float rate = 1.f;               // beeps (or breaths) per second
    float duty = .5f;               // share of the period until the fall starts
    float attack = .1f;             // share of the period for the rise
    float release = .1f;            // share of the period for the fall
    float floor = 1.f;              // level between beeps, 0 .. 1
    float trendOct = 0.f;           // pitch slide over the beep, octaves at its end
    float dualOct = 0.f;            // pitch offset after the beep (dual tone), octaves
    float glideS = .12f;            // pitch smoothing time constant, s
    float harm[NH] = {};            // harmonic amplitudes, 32767 = full scale
    const uint8_t *phase = nullptr; // harmonic phases (256 = one period), nullptr = all sine phase
    bool gainGlide = false;         // even out level steps of the waveform swap over 50 ms
    bool sub = false;               // double beep (gap in the middle of the beep)
    bool on = false;                // false fades out within ~25 ms
    float vol = 1.f;                // overall level 0 .. 1 (smoothed over 50 ms in the interrupt)
};

class VarioSynth
{
public:
    static constexpr int SAMPLE_RATE = 66150;     // has to match ESPAudio.cpp
    static constexpr int CHUNK_BITS = 6;
    static constexpr int CHUNK = 1 << CHUNK_BITS;  // samples per control step
    static constexpr int WT_LEN = 256;            // waveform, power of two
    static constexpr int DAC_AMPL = VS_DAC_FULL ? 127 : 64;

    void init(float startHz = 500.f);           // the pitch glides from startHz on the first tone
    // Audio task, ~10 times per second
    void update(const SynthFrame &f);
    // Forget the last waveform, the next update builds a new one (after a voice change)
    void resetWave();

    // DMA interrupt, integer only. Writes n samples as DAC bytes (127 +- DAC_AMPL) to p, p+stride, ...
    void render(uint8_t *p, int n, int stride);

    // true while completely faded out (the amplifier may be switched off)
    bool silent() const { return _envEnd == 0 && _gate == 0; }

private:
    static constexpr int OUT_SHIFT = VS_DAC_FULL ? 23 : 24;
    struct Ctl {                  // handed over from the task to the interrupt
        int32_t  lp;              // target pitch, log2(f / F_REF) in Q16
        uint32_t binc;            // beep phase per CHUNK
        uint32_t d16;             // duty (Q16 of the period)
        uint32_t invE, invEr, invD; // 2^32 / rise, fall, duty (each Q16)
        int32_t  floor15;         // level between beeps, Q15
        int32_t  trend;           // trend slide, Q16 octaves
        int32_t  dual;            // dual tone offset after the beep, Q16 octaves
        int32_t  glideA;          // pitch smoothing per CHUNK, Q16
        int32_t  gateTarget;      // 0 or 32767
        int32_t  volTarget;       // overall level, Q15
        uint32_t gRatio;          // new waveform: level old / new, Q16 (65536 = equal), taken over into _vg on swap
        uint8_t  sub;             // 1 = double beep
        uint8_t  snap;            // 1 = level directly to target (start)
    };
    void chunk();
    void buildWave(int16_t *w, const float *a, const uint8_t *phase);

    // interrupt state
    uint32_t _ph = 0, _inc = 0, _bph = 0;
    int32_t  _env = 0, _denv = 0, _envEnd = 0, _lp = 0, _gate = 0;
    int32_t  _left = 0;                // samples until the next control step
    int32_t  _err = 0;                 // noise shaping (first order error feedback)
    int32_t  _vg = 32767 << 8;         // overall level, Q23
    const int16_t *_wt = nullptr;
    const Ctl *_cp = nullptr;          // active control set (not written by the task)

    // task -> interrupt
    Ctl _ctl[2]{};
    int16_t _wbuf[2][WT_LEN + 1]{};
    std::atomic<uint8_t> _cur{0};
    std::atomic<bool> _fresh{false};

    // task state
    float _aLast[SynthFrame::NH]{};
    const uint8_t *_phLast = nullptr;
    float _waveE = 0.f;               // energy of the last built waveform (gainGlide)
    bool _snap = true;
    uint32_t _incRef = 0;
};
