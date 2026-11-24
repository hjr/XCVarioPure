/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "SensorBase.h"

//
// Long term thermal statistics: the mean climb rate over the last "core_climb_history" minutes,
// e.g. 45 min ~ the last three thermals.
//
// A low profile virtual sensor. No own sampling, it reads the vario indicator history
// every SAMPLE_MS and condenses it into time bins of "core_climb_period" seconds.
// Only the climbing portions (gross climb >= core_climb_min) contribute:
//    mean climb = sum(altitude gained while climbing) / sum(time spent climbing)
// over a wall clock window. Publishes on the black board variable "average_climb",
// 0 when there is not enough climbing in the window.
//
class ClimbStats final : public SensorBase
{
public:
    static constexpr int SAMPLE_MS = 5000;  // read the vario history every 5 sec
    static constexpr int MIN_CLIMB_S = 120; // min. climbing time in the window to publish a mean climb

    ClimbStats();
    ~ClimbStats() = default;

    const char* name() const override { return "ClimbStats"; }
    bool probe() override { return true; }
    bool setup() override;
    bool update(uint32_t) override { return false; } // virtual, no read cycle
    void postProcess() override;

    void reset();

private:
    struct Bin {
        uint16_t gain_dm;  // altitude gained while climbing [dm]
        uint16_t climb_s;  // time spent climbing [s]
    };
    void closeBin();
    void publish();

    FixedSensorHistory<Bin> _bins;
    float    _cur_gain = 0.f; // current bin, gained altitude [m]
    uint16_t _cur_climb_s = 0;
    uint16_t _cur_elapsed_s = 0;
    uint16_t _bin_s = 60;     // bin length the history was recorded with
};

extern ClimbStats *climbStats;
