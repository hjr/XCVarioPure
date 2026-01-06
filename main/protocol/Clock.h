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

class Clock_I;

// Clock based on esp_timer
class Clock
{
public:
    static constexpr int TICK_ATOM = 10; // msec

public:
    Clock();
    ~Clock();

    // API
    static void start(Clock_I *cb);
    static void stop(Clock_I *cb);
    static inline unsigned long getMillis() {
        return msec_counter;
    }
    static int getSeconds();

    static volatile IRAM_ATTR unsigned long msec_counter;

private:
    static esp_timer_handle_t _clock_timer;
};
