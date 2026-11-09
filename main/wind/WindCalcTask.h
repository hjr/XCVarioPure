/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "driver/time/ClockIntf.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// when set the bg task wants to be feed with new jobs
extern QueueHandle_t BackgroundTaskQueue;

// calculations are triggered by events on the queue
struct CalkTaskJob
{
    enum { CALK_TASK_NONE,
            CALK_TASK_EVENT_NEW_GPSPOSE,
            CALK_TASK_EVENT_NUMSAT,
            CALK_TASK_SEND_SENS,
            CALK_TASK_THERMAL_STATS,
            CALK_TASK_TOY_FEED,
            CALK_1S_CACHE
    };

    uint16_t raw;
    CalkTaskJob() = delete;
    constexpr CalkTaskJob(uint8_t typ, uint8_t arg = 0) : raw((typ << 8) | (arg & 0xff)) {}
    constexpr int getDetail() const { return (raw & 0xff); } // get the job argument
    void setDetail(int8_t v) { raw |= (v & 0xff); }
    constexpr int getJobTyp() const { return (raw >> 8); } // get the job type

};


// todo rename class
class WindCalcTask : public Clock_I
{
public:
    WindCalcTask();
    ~WindCalcTask();

    bool tick() override;

    // API
    static void createWindResources();
};

extern WindCalcTask *CalcTask;
