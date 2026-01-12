/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include <esp_timer.h>
#include <esp_attr.h>
#include <cstdint>
#include <set>

class Clock_I;
using ClockSet = std::set<Clock_I*>;

// Clock based on esp_timer
class Clock
{
public:
    static constexpr int TICK_ATOM = 10; // msec

public:
    Clock();
    ~Clock();

    // API
    static void start(Clock_I *cb); // register a tick callback
    static void stop(Clock_I *cb);
    static inline int getMillis() { return msec_counter; } // spars, but monotone time for all XCV concerns, millis since boot-up
    static int getSeconds();
    // UTC time access
    static int64_t getMillisUTC() {
        return _offset_ms + getMillis();
    }
    static inline bool isValidUTC() { return _valid; }
    static void setSimSpeed(uint8_t s) { _sim_speed = s; }
    static int getUpdateAgeMs();
    static void setTimeUTC(int64_t gps_utc_ms); // just call once with first valid GPS time stamp
    static void updateTimeUTC(int64_t gps_utc_ms);

private:
    static void IRAM_ATTR clock_timer_sr(ClockSet *registry);
    static volatile IRAM_ATTR int msec_counter; // yields for 600h of flight time monotone msec since boot
    static esp_timer_handle_t _clock_timer;
    // time sync
    static int64_t  _offset_ms;     // gps_utc - system_ms
    static bool     _valid;
    static int      _last_updated_ms;
    static uint8_t  _sim_speed;     // simulation speed multiplier
};
