/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "driver/time/ClockIntf.h"
#include "math/vector_3d.h"

class DataLink;
class Device;
struct SIMRUN;

class FlarmSim final : public Clock_I
{
public:
    static void StartSim(int variant=0);

    // Clock tick callback
    bool tick() override;

private:
    FlarmSim() = delete;
    explicit FlarmSim(Device *d, const SIMRUN *simrun);
    ~FlarmSim();

    int     _tick_count = -5;
    bool    _done = false;
    Device  *_d; // hijacked device
    // Active flagging instance
    static FlarmSim *_sim;
    // tracked vectors
    vector_f _target_pos; // all metric
    vector_f _target_inc;
    float    _target_omega;
    int      _icao_id = 0;
};
