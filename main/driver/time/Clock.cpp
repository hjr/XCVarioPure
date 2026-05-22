/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Clock.h"
#include "ClockIntf.h"
#include "logdefnone.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <algorithm>

esp_timer_handle_t Clock::_clock_timer = nullptr;

QueueHandle_t request_queue = nullptr;
enum CmdType { CMD_START, CMD_STOP };
struct ClkRequest {
    int8_t cmd; // 0=start, 1=stop
    Clock_I* cb;
};

volatile int Clock::msec_counter = 0;
int64_t  Clock::_offset_ms = 0;
bool     Clock::_valid = false;
int      Clock::_last_updated_ms = 0;
uint8_t  Clock::_sim_speed = 1;

// Simple unique receiver registry
static ClockSet clock_registry;

// Timer SR (called in a timer task context)
void IRAM_ATTR Clock::clock_timer_sr(ClockSet*registry)
{
    // be in sync with millis, but sparse
    if ( _sim_speed == 1 ) {
        msec_counter = esp_timer_get_time() / 1000;
    } else {
        msec_counter += TICK_ATOM * _sim_speed;
    }

    // Check if there are any requests
    ClkRequest msg;
    while (xQueueReceive(request_queue, &msg, 0) == pdTRUE) {
        if (msg.cmd == CMD_START) {
            registry->insert(msg.cb);
        }
        else if (msg.cmd == CMD_STOP) {
            registry->erase(msg.cb);
        }
    }
    // tick callbacks
    for (auto it = registry->begin(); it != registry->end(); ) {
        if ((*it)->myTurn()) {
            if ((*it)->tick()) {
                it = registry->erase(it);
                continue;
            }
        }
        it++;
    }
}


// The clock
Clock::Clock()
{
    // setup clock timer only once
    if ( _clock_timer == 0 ) {
        request_queue = xQueueCreate(10, sizeof(ClkRequest));
        esp_timer_create_args_t t_args = {
            .callback = (esp_timer_cb_t)clock_timer_sr,
            .arg = (void*)(&clock_registry),
            .dispatch_method = ESP_TIMER_TASK,
            .name = "clock",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&t_args, &_clock_timer);
        esp_timer_start_periodic(_clock_timer, TICK_ATOM * 1000); // the timers API is on usec
    }
}
Clock::~Clock()
{
    if (_clock_timer) {
        esp_timer_stop(_clock_timer);
        esp_timer_delete(_clock_timer);
        clock_registry.clear();
        vQueueDelete(request_queue);
    }
}

void Clock::start(Clock_I *cb)
{
    ClkRequest req = { CMD_START, cb };
    ESP_LOGI(FNAME, "Clock start request %p", cb);
    xQueueSend(request_queue, &req, portMAX_DELAY);
}
void Clock::stop(Clock_I *cb)
{
    ClkRequest req = { CMD_STOP, cb };
    ESP_LOGI(FNAME, "Clock stop request %p", cb);
    xQueueSend(request_queue, &req, portMAX_DELAY);
}
int Clock::getSeconds()
{
    return msec_counter / 1000;
}
int32_t Clock::getMillisMidnightUTC()
{
    int64_t utc = getMillisUTC();
    int32_t day_ms = static_cast<int32_t>(utc % 86400000LL);
    // if ( day_ms < 0 ) day_ms += 86400000; // utc > 0 always
    return day_ms;
}

int Clock::getUpdateAgeMs() {
    return  msec_counter - _last_updated_ms;
}

// Time synch from GPS
void Clock::setTimeUTC(int64_t gps_utc_ms) {
    _offset_ms = gps_utc_ms - (esp_timer_get_time() / 1000); // esp time in usec
    _last_updated_ms = msec_counter;
    _valid = true;
    ESP_LOGI(FNAME, "Clock set UTC: offset %lld ms", _offset_ms);
}
void Clock::updateTimeUTC(int64_t gps_utc_ms) {
    int64_t est_now = _offset_ms + msec_counter;
    int64_t error   = gps_utc_ms - est_now;
    ESP_LOGI(FNAME, "Clock sync update: error %lldms, since %dms, new offset %lldms", error, getUpdateAgeMs(), _offset_ms);
    _last_updated_ms = msec_counter;
    _offset_ms += std::clamp(error / 12, -5LL, +5LL); // max 5 ms correction per update
}

