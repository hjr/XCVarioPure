/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "setup/MenuEntry.h"
#include "driver/time/WatchDog.h"

#include <atomic>

struct Point;
struct Line;

class FlarmScreen: public MenuEntry, public WDBark_I
{
public:
    static FlarmScreen *create();
    void remove(); // triggered by time-out

    void exit(int ups=1) override;
    void display(int mode = 0) override;
    const char *value() const override { return nullptr; }
    void rot(int count) override {}
    void press() override;
    void longPress() override { press(); }

protected:
    // warn timeout
    void barked() override;

private:
    FlarmScreen();
    virtual ~FlarmScreen() = default;
    FlarmScreen(const FlarmScreen&) = delete;
    FlarmScreen& operator=(const FlarmScreen&) = delete;
    int _tick = 0;
    int _alarmtick = 0;
    WatchDog_C _time_out;
    uint16_t _prev_alarm = 0;
    std::atomic<int> _done = 0; // int because of atomic operations
};


// global access
extern FlarmScreen *FLARMSCREEN;