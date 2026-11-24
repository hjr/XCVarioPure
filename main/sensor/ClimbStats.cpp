/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ClimbStats.h"

#include "VarioFilter.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <algorithm>
#include <cmath>

ClimbStats *climbStats = nullptr;

// Max. 180 min history (core_climb_history limit) with min. 60 sec bins
constexpr int MAX_BINS = 180;
static __attribute__((aligned(4))) uint32_t bin_buffer[ MAX_BINS + 1 ];

constexpr mps_t MAX_PLAUSIBLE_CLIMB = 20.f; // e.g. altitude jumps in sim mode

ClimbStats::ClimbStats() :
    SensorBase(SAMPLE_MS, 1),
    _bins(reinterpret_cast<Bin*>(bin_buffer), MAX_BINS)
{
    static_assert(sizeof(Bin) == sizeof(uint32_t));
    // virtual sensor, post processing only, after the vario (prio 10)
    _id = SensorId(SensorType::CLIMB_STATS, 12);
}

bool ClimbStats::setup()
{
    reset();
    return true;
}

void ClimbStats::reset()
{
    _bins.reset();
    _cur_gain = 0.f;
    _cur_climb_s = 0;
    _cur_elapsed_s = 0;
    _bin_s = (uint16_t)core_climb_period.get();
    average_climb.set(0.f);
}

// every SAMPLE_MS
void ClimbStats::postProcess()
{
    constexpr uint16_t sample_s = SAMPLE_MS / 1000;

    if ( varioSensor && airborne.get()
        && varioSensor->getLevel() * varioSensor->getDutyCycleMS() >= SAMPLE_MS ) {
        // mean of the vario indicator over the last sample period, converted to gross climb
        // HACK: see VarioFilter::getGrossOffset(), to vanish with a fixed gross vario history
        mps_t climb = varioSensor->getAVG(SAMPLE_MS) + varioSensor->getGrossOffset();
        if ( climb >= core_climb_min.get() && climb < MAX_PLAUSIBLE_CLIMB ) {
            _cur_gain += climb * sample_s;
            _cur_climb_s += sample_s;
        }
    }
    // the wall clock keeps running, also on ground or with missing vario data
    _cur_elapsed_s += sample_s;

    if ( _cur_elapsed_s >= _bin_s ) {
        closeBin();
        publish();
    }
}

void ClimbStats::closeBin()
{
    Bin b;
    b.gain_dm = (uint16_t)std::clamp(std::lround(_cur_gain * 10.f), 0L, 0xffffL);
    b.climb_s = _cur_climb_s;
    _bins.push(b);
    ESP_LOGI(FNAME, "bin: %.1fm in %ds", _cur_gain, _cur_climb_s);

    _cur_gain = 0.f;
    _cur_climb_s = 0;
    _cur_elapsed_s = 0;

    // a changed bin length invalidates the recorded history
    uint16_t bin_s = (uint16_t)core_climb_period.get();
    if ( bin_s != _bin_s ) {
        ESP_LOGI(FNAME, "bin length changed %d -> %d", _bin_s, bin_s);
        _bins.reset();
        _bin_s = bin_s;
    }
}

void ClimbStats::publish()
{
    // number of bins covering the averaging window
    int window = (int)std::ceil(core_climb_history.get() * 60.f / _bin_s);
    window = std::clamp(window, 1, std::min(_bins.level(), MAX_BINS));

    uint32_t gain_dm = 0, climb_s = 0;
    for ( int i = 0; i < window; i++ ) {
        gain_dm += _bins[i].gain_dm;
        climb_s += _bins[i].climb_s;
    }

    mps_t mean = 0.f;
    if ( climb_s >= MIN_CLIMB_S ) {
        mean = (gain_dm / 10.f) / climb_s;
    }
    ESP_LOGI(FNAME, "mean climb %.2f m/s (%d bins, %ds climbing)", mean, window, (int)climb_s);
    if ( std::fabs(mean - average_climb.get()) >= 0.01f ) {
        average_climb.set(mean);
    }
}
