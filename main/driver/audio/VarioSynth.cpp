/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "VarioSynth.h"
#include "AudioTables.h"

#include <algorithm>
#include <cmath>
#include <cstring>

// The render path runs in the DAC DMA interrupt. CONFIG_DAC_ISR_IRAM_SAFE is not set, so the interrupt
// is held back during flash operations and the code may stay in flash (IRAM is nearly full).

namespace {
constexpr float F_REF = 62.5f;   // reference pitch of the log pitch (all tones are above)
constexpr float MAX_HZ = 7500.f; // pitch limit: 7 octaves above F_REF keep the phase increment in 32 bit (plus trend and dual tone)
// envelope peak: at full DAC level 126/128, so peak plus rounding never exceeds +-127
constexpr int32_t ENV_TOP = VS_DAC_FULL ? 32256 : 32767;
// smoothing of the overall level in the interrupt: time constant 50 ms (smooths the 10 Hz task steps)
constexpr int32_t VG_A = (int32_t)(0.0192f * 65536.f);   // 1 - exp(-64 / (66150 * 0.05))

inline __attribute__((always_inline)) int32_t rc(uint32_t x)   // x: 0..65535  ->  0..32767
{
    const uint32_t i = x >> 10, f = x & 1023;
    return audiotab::RC_Q15[i] + (((audiotab::RC_Q15[i + 1] - audiotab::RC_Q15[i]) * (int32_t)f) >> 10);
}

inline __attribute__((always_inline)) int32_t sample(uint32_t &ph, uint32_t inc, int32_t &env, int32_t denv, int32_t &err, const int16_t *wt)
{
    constexpr int OUT_SHIFT = VS_DAC_FULL ? 23 : 24;
    ph += inc;
    const uint32_t i = ph >> 24;
    const int32_t fr = (int32_t)((ph >> 16) & 0xFF);
    const int32_t a = wt[i], b = wt[i + 1];
    const int32_t s = a + (((b - a) * fr) >> 8);       // Q15
    env += denv;                                        // Q22
    const int32_t x = s * (env >> 7) + err;             // Q30 (2^30 = 128 or 64 DAC steps)
    const int32_t y = (x + (1 << (OUT_SHIFT - 1))) >> OUT_SHIFT;   // round to DAC steps
    err = x - y * (1 << OUT_SHIFT);                     // rounding error into the next sample: moves
    return y;                                           // the 8 bit noise above 10 kHz (noise shaping)
}
} // namespace

void VarioSynth::init(float startHz)
{
    _incRef = (uint32_t)(F_REF / SAMPLE_RATE * 4294967296.0);
    _cur.store(0);
    _fresh.store(false);
    _ph = _bph = 0;
    _env = _denv = _envEnd = _gate = _err = 0;
    _left = 0;
    _vg = 32767 << 8;
    _waveE = 0.f;
    resetWave();
    _snap = true;
    SynthFrame f;               // silent start state into both buffers
    f.harm[0] = 32767.f;
    f.hz = startHz;
    update(f);
    _ctl[0] = _ctl[1];
    std::memcpy(_wbuf[0], _wbuf[1], sizeof(_wbuf[0]));
    _fresh.store(false);
    _cp = &_ctl[0];
    _wt = _wbuf[0];
    _lp = _ctl[0].lp;
    _snap = true;               // the first real frame sets levels directly
}

void VarioSynth::resetWave()
{
    std::memset(_aLast, 0, sizeof(_aLast));
    _phLast = nullptr;
    _waveE = 0.f;
}

// ------------------------------------------------------------------ audio task
void VarioSynth::update(const SynthFrame &f)
{
    if ( _fresh.load(std::memory_order_acquire) ) {
        return; // the interrupt has not yet taken the last set
    }
    const float rate = std::max(0.01f, f.rate);
    const float e = std::clamp(f.attack, 0.001f, 0.95f);
    const float er = std::clamp(f.release, 0.001f, 0.95f);
    const float d = std::clamp(std::min(f.duty, 1.f - er), 0.001f, 1.f);

    const float hz = std::clamp(f.hz, F_REF, MAX_HZ);

    // waveform, band limited (no harmonics above 0.45 x sample rate), rebuilt only on a noticeable change
    float a[SynthFrame::NH];
    float dmax = 0.f, amax = 1e-6f;
    for (int k = 0; k < SynthFrame::NH; k++) {
        a[k] = ((k + 1) * hz < 0.45f * SAMPLE_RATE) ? f.harm[k] : 0.f;
        dmax = std::max(dmax, std::fabs(a[k] - _aLast[k]));
        amax = std::max(amax, std::fabs(a[k]));
    }
    const uint8_t back = _cur.load(std::memory_order_relaxed) ^ 1;
    // Level compensation on the waveform swap (gainGlide): the new waveform carries the new level. The interrupt takes it
    // with the ratio old / new into _vg and then glides to the target within 50 ms: no level steps at 10 Hz in gusts.
    float gRatio = 1.f;
    if ( dmax > 0.002f * amax || f.phase != _phLast ) {
        buildWave(_wbuf[back], a, f.phase);
        std::memcpy(_aLast, a, sizeof(_aLast));
        _phLast = f.phase;
        if ( f.gainGlide ) {   // ratio from the energy of the harmonics (level and timbre change together)
            float E = 0.f;
            for (int k = 0; k < SynthFrame::NH; k++) { E += a[k] * a[k]; }
            if ( _waveE > 0.f && E > 0.f && !_snap ) { gRatio = std::clamp(std::sqrt(_waveE / E), 0.25f, 4.f); }
            _waveE = E;
        }
    }
    else {
        std::memcpy(_wbuf[back], _wbuf[back ^ 1], sizeof(_wbuf[0]));   // unchanged
    }

    // control values for the interrupt
    Ctl &c = _ctl[back];
    c.lp = (int32_t)std::lround(std::log2(hz / F_REF) * 65536.f);
    c.binc = (uint32_t)(rate / SAMPLE_RATE * CHUNK * 4294967296.0);
    c.d16 = (uint32_t)(d * 65536.f);
    c.invEr = (uint32_t)std::min(4294967295.0, 65536.0 * 65536.0 / std::max(er * 65536.f, 1.f));
    c.invD = (uint32_t)std::min(4294967295.0, 65536.0 * 65536.0 / std::max(d * 65536.f, 1.f));
    c.invE = (uint32_t)std::min(4294967295.0, 65536.0 * 65536.0 / std::max(e * 65536.f, 1.f));
    c.floor15 = (int32_t)(std::clamp(f.floor, 0.f, 1.f) * 32767.f);
    c.trend = (int32_t)(f.trendOct * 65536.f);
    c.dual = (int32_t)(f.dualOct * 65536.f);
    c.glideA = (int32_t)((1.f - std::exp(-(float)CHUNK / (SAMPLE_RATE * std::max(f.glideS, 0.001f)))) * 65536.f);
    c.gateTarget = f.on ? 32767 : 0;
    c.volTarget = (int32_t)(std::clamp(f.vol, 0.f, 1.f) * 32767.f);
    c.gRatio = (uint32_t)(gRatio * 65536.f);
    c.sub = f.sub ? 1 : 0;
    c.snap = _snap ? 1 : 0;
    _snap = false;
    _fresh.store(true, std::memory_order_release);
}

void VarioSynth::buildWave(int16_t *w, const float *a, const uint8_t *phase)
{
    for (int j = 0; j < WT_LEN; j++) {
        float acc = 0.f;
        for (int k = 0; k < SynthFrame::NH; k++) {
            if ( a[k] == 0.f ) { continue; }
            const int ph = phase ? phase[k] : 0;
            acc += a[k] * audiotab::SIN_Q15[((k + 1) * j + ph) & (WT_LEN - 1)];
        }
        const int s = (int)std::lround(acc * (1.f / 32768.f));
        w[j] = (int16_t)std::clamp(s, -32767, 32767);
    }
    w[WT_LEN] = w[0];
}

// ------------------------------------------------------------------ DMA interrupt (integer only)
void VarioSynth::render(uint8_t *p, int n, int stride)
{
    while ( n > 0 ) {
        if ( _left == 0 ) { chunk(); }
        int m = _left < n ? _left : n;
        _left -= m;
        n -= m;
        uint32_t ph = _ph;
        const uint32_t inc = _inc;
        int32_t env = _env, err = _err;
        const int32_t denv = _denv;
        const int16_t *wt = _wt;
        for (; m > 0; m--) {
            *p = (uint8_t)(127 + sample(ph, inc, env, denv, err, wt));
            p += stride;
        }
        _ph = ph; _env = env; _err = err;
    }
}

void VarioSynth::chunk()
{
    _left = CHUNK;
    if ( _fresh.load(std::memory_order_acquire) ) {          // take the new set from the task
        const uint8_t n = _cur.load(std::memory_order_relaxed) ^ 1;
        _cp = &_ctl[n];
        _wt = _wbuf[n];
        _cur.store(n, std::memory_order_relaxed);
        _fresh.store(false, std::memory_order_release);
        if ( _cp->snap ) { _vg = _cp->volTarget << 8; }        // start: no fading
        else if ( _cp->gRatio != 65536 ) { _vg = (int32_t)(((int64_t)_vg * _cp->gRatio) >> 16); }   // level of the new waveform
    }
    const Ctl &c = *_cp;
    // pitch follows smoothly (log domain)
    _lp += (int32_t)(((int64_t)(c.lp - _lp) * c.glideA) >> 16);
    // beep phase at the end of this chunk
    _bph += c.binc;
    const uint32_t p16 = _bph >> 16;
    int32_t pu, r16;
    const bool inBeep = p16 < c.d16;
    if ( inBeep ) {
        pu = rc((uint32_t)std::min<uint64_t>(65535, ((uint64_t)p16 * c.invE) >> 16));
        r16 = (int32_t)std::min<uint64_t>(65535, ((uint64_t)p16 * c.invD) >> 16);
        if ( c.sub ) {                                      // double beep: gap in the middle of the beep
            const int32_t x16 = r16 > 32768 ? r16 - 32768 : 32768 - r16;   // distance to the middle, Q16
            if ( x16 <= 5243 ) { pu = 0; }                                  // +-8 % silent
            else if ( x16 < 10486 ) { pu = (pu * rc((uint32_t)(((x16 - 5243) * 25) >> 1))) >> 15; }   // soft up to +-16 %
        }
    }
    else {
        pu = 32767 - rc((uint32_t)std::min<uint64_t>(65535, ((uint64_t)(p16 - c.d16) * c.invEr) >> 16));
        r16 = 65535;
    }
    // fade in / out (~25 ms)
    if ( _gate < c.gateTarget ) { _gate = std::min(c.gateTarget, _gate + 1311); }
    else if ( _gate > c.gateTarget ) { _gate = std::max(c.gateTarget, _gate - 1311); }
    // overall level: 50 ms smoothing
    _vg += (int32_t)(((int64_t)((c.volTarget << 8) - _vg) * VG_A) >> 16);
    int32_t env15 = c.floor15 + (((32767 - c.floor15) * pu) >> 15);
    env15 = (env15 * _gate) >> 15;
    env15 = (env15 * std::min(_vg >> 8, (int32_t)65535)) >> 15;   // _vg may briefly exceed 1 after a level compensation
    if ( env15 > 32767 ) { env15 = 32767; }                         // never above full scale
    env15 = (env15 * ENV_TOP) >> 15;
    // envelope linear over the chunk
    _env = _envEnd;
    _envEnd = env15 << 7;
    _denv = (_envEnd - _env) >> CHUNK_BITS;
    // phase increment from log pitch + trend slide + dual tone
    const int32_t lpe = _lp + ((c.trend * r16) >> 16) + (inBeep ? 0 : c.dual);
    const uint32_t oct = (uint32_t)lpe >> 16, f6 = ((uint32_t)lpe >> 10) & 63, f10 = (uint32_t)lpe & 1023;
    const uint32_t e0 = audiotab::EXP2_Q30[f6], e1 = audiotab::EXP2_Q30[f6 + 1];
    const uint32_t ex = e0 + ((((e1 - e0) >> 4) * f10) >> 6);
    _inc = (uint32_t)(((uint64_t)(_incRef << oct) * ex) >> 30);   // _incRef << 5 fits into 32 bit
}
