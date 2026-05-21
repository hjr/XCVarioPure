/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "driver/time/ClockIntf.h"
#include "driver/gpio/ESPRotary.h"
#include "comm/Mutex.h"

#include <deque>
#include <cstdint>
#include <string>
#include <memory>

// print text into the bottom line of the screen
// move the message, so that it is readable as a whole
// clip the message line from the current screen
//
// alarm levels are 1, 2, 3, 4
// 1: queued, stays for 10 seconds visible, blue line on top
// 2: queued, stays for 20 seconds visible, orange line
// 3: pushed to front in queue, stays for 30 seconds visible, red line
// 4: directly mapped, until recalled, red line, red letters, flashing (only short messages)
//
// precondition:
// - ucg adapter for the connected display
// - Clock timer
// - screens it interfers with

struct ScreenMsg {
    static constexpr const int CONFIRM = 180; // seconds
    int alert_level;
    std::string text;
    int _time_out;
    uint8_t _id; // e.g. to identify messages for confirmation
    ScreenMsg(int a, const char *str, int to = 0);
    bool wantsConfirmation() const { return _time_out >= CONFIRM; }
};


class MessageBox final : public Clock_I, public RotaryReceiver
{
    using MessagePtr = std::unique_ptr<ScreenMsg>;

public:
    static void createMessageBox();
    ~MessageBox();

    static constexpr const int16_t MSG_BOX_HEIGHT = 26;

    // API
    uint8_t pushMessage(int alert_level, const char* msg, int to = 0, bool confirm = false);
    void popMessage(uint8_t id = 0);
    void keepMessage(uint8_t id, int to = 0);
    bool draw();
    int16_t getBoxHeight() const { return MSG_BOX_HEIGHT; }
    // bool isVisible() const { return current != nullptr; }
    void pause() { _active = false; }
    void resume() { _active = true; }

    // Clock tick callback
    bool tick() override;

    // Rotary API to confirm messages
    void rot(int count) override {}
    void press() override;
    void longPress() override { press(); }
    void release() override {} 
    void escape() override {}
    
private:
    // helper
    MessageBox();
    bool nextMsg();
    void removeMsg();

private:
    MessagePtr current = nullptr;
    std::deque<MessagePtr> _msg_list;
    mutable SemaphoreMutex _list_mutex;
    int16_t _print_pos = 0;
    int _start_scroll = 0; // timings
    int _nr_scroll = 0;
    int _msg_to = 0;
    bool _active = false;
};

// exposed pointer to the message box
extern MessageBox *MBOX;