/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Clock.h"
#include "ClockIntf.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <set>

esp_timer_handle_t Clock::_clock_timer = nullptr;

QueueHandle_t request_queue = nullptr;
enum CmdType { CMD_START, CMD_STOP };
struct ClkRequest {
    int8_t cmd; // 0=start, 1=stop
    Clock_I* cb;
};

volatile unsigned long Clock::msec_counter = 0;

// Simple unique receiver registry
static std::set<Clock_I*> clock_registry;

// Timer SR (called in a timer task context)
static void IRAM_ATTR clock_timer_sr(std::set<Clock_I*> *registry)
{
    // be in sync with millis, but sparse
    Clock::msec_counter = esp_timer_get_time() / 1000;

    // Check if there are any requests
    ClkRequest msg;
    if (xQueueReceive(request_queue, &msg, 0) == pdTRUE) {
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
        esp_timer_create_args_t t_args = {
            .callback = (esp_timer_cb_t)clock_timer_sr,
            .arg = (void*)(&clock_registry),
            .dispatch_method = ESP_TIMER_TASK,
            .name = "clock",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&t_args, &_clock_timer);
        esp_timer_start_periodic(_clock_timer, TICK_ATOM * 1000); // the timers API is on usec
        clock_registry.clear();
        request_queue = xQueueCreate(10, sizeof(ClkRequest));
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
    xQueueSend(request_queue, &req, portMAX_DELAY);
}
void Clock::stop(Clock_I *cb)
{
    ClkRequest req = { CMD_STOP, cb };
    xQueueSend(request_queue, &req, portMAX_DELAY);
}
int Clock::getSeconds()
{
    return static_cast<int>(msec_counter / 1000);
}
