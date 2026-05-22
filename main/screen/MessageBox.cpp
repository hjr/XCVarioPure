/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "MessageBox.h"

#include "Colors.h"
#include "DrawDisplay.h"
#include "UiEvents.h"
#include "driver/time/Clock.h"
#include "AdaptUGC.h"
#include "logdefnone.h"

#include <mutex>

extern AdaptUGC *MYUCG;
MessageBox *MBOX; // the global representation

constexpr int CLOCK_DIVIDER = 8;

// A message is represented through
// - alert level (1,2,3,4)
// - a text message
// - a time-out in seconds (0 means default per alert level, a time-out >180 sec enforces a confirmation option)
// - has a pseudo unique id to identify messages for confirmation (e.g. to confirm a landing, or a flarm alarm)
ScreenMsg::ScreenMsg(int a, const char *str, int to) : alert_level(a), text(str), _time_out(to)
{
    _id = (rand() % 254) + 1; // generate a pseudo unique id != 0
}

void MessageBox::createMessageBox()
{
    if ( ! MBOX ) {
        MBOX = new MessageBox();
    }
}

MessageBox::MessageBox() :
    Clock_I(CLOCK_DIVIDER),
    RotaryReceiver()
{
}

MessageBox::~MessageBox()
{
    Clock::stop(this);
    _msg_list.clear();
}

uint8_t MessageBox::pushMessage(int alert_level, const char *str, int to, bool confirm)
{
    if (_msg_list.size() >= 10) {
        ESP_LOGW(FNAME, "Message list full, dropping message: %s", str);
        return 0;
    }
    
    std::unique_ptr<ScreenMsg> msg = std::make_unique<ScreenMsg>(alert_level, str, to);
    uint8_t ret = msg->_id;
    {
        std::lock_guard<SemaphoreMutex> lock(_list_mutex);
        if ( alert_level >= 3 ) {
            _msg_list.push_front(std::move(msg));
        }
        else {
            _msg_list.push_back(std::move(msg));
        }
    }
    
    if ( alert_level == 4 ) {
        _msg_to = 0; // trigger immediate display
    }
    if ( ! current && _active ) {
        Clock::start(this);
    }
    return ret;
}

// todo messages should have id's to make this work properly
void MessageBox::popMessage(uint8_t id)
{
    // only acts on the current message, if id matches or id is 0 (default)
    if ( id != 0 ) {
        if ( current && current->_id == id ) {
            _msg_to = 0; // trigger immediate display of next message
        }
    }
    else {
        // pop current message
        if ( current ) {
            _msg_to = 0; // trigger immediate display of next message
        }
    }
}

void MessageBox::keepMessage(uint8_t id, int to)
{
    if ( current && current->_id == id ) {
        int tmp_to = (to > 0) ? to :  current->alert_level * 10; // x 10 sec
        tmp_to *= (1000/(CLOCK_DIVIDER*Clock::TICK_ATOM)); // convert to ticks
        _msg_to = tmp_to; // reset time-out for current message
    }
}

// returns true in case a message is now on the display
bool MessageBox::nextMsg()
{
    const int16_t dheight = MYUCG->getDisplayHeight();
    const int16_t dwidth = MYUCG->getDisplayWidth();
    // clear message area
    MYUCG->undoClipRange();
    removeMsg();

    {
        // make this block atomic
        std::lock_guard<SemaphoreMutex> lock(_list_mutex);
        if (!_msg_list.empty()) {
            current = std::move(_msg_list.front());
            _msg_list.pop_front();
        }
        else {
            // All done
            current = nullptr;
            return false;
        }
        if ( current->wantsConfirmation() ) {
            attach(); // observe rotary for confirmation
            current->text += "  ... pls confirm";
        }
    }

    // prepare screen with a colored line on top
    if ( current->alert_level == 1 ) {
        MYUCG->setColor(COLOR_BLUE);
    }
    else if ( current->alert_level == 2 ) {
        MYUCG->setColor(COLOR_ORANGE);
    }
    else if ( current->alert_level >= 3 ) {
        MYUCG->setColor(COLOR_RED);
    }
    MYUCG->drawHLine(0, dheight - MSG_BOX_HEIGHT, dwidth);
    MYUCG->setFont(ucg_font_fub14_hr, true);
    _print_pos = 1;
    if  ( current->alert_level == 4 ) {
        // center text
        int16_t w = MYUCG->getStrWidth(current->text.c_str());
        if ( w < dwidth ) {
            _print_pos = (dwidth - w) / 2;
        }
    }
    MYUCG->setPrintPos(_print_pos, dheight-2);
    MYUCG->setColor(COLOR_WHITE);
    MYUCG->setFontPosBottom();

    MYUCG->print(current->text.c_str());

    // Exclude the message area for the rest of the system
    MYUCG->setClipRange(0, 0, dwidth, dheight - MSG_BOX_HEIGHT);

    // set time counter
    _start_scroll = 0;
    _nr_scroll = std::max(MYUCG->getStrWidth(current->text.c_str()) - dwidth +2, 0);
    _msg_to = (current->_time_out > 0) ? current->_time_out :  current->alert_level * 10; // x 10 sec
    _msg_to *= (1000/(CLOCK_DIVIDER*Clock::TICK_ATOM)); // convert to ticks

    return true;
}

void MessageBox::removeMsg()
{
    MYUCG->setColor(COLOR_BLACK);
    MYUCG->drawBox(0, MYUCG->getDisplayHeight() - MSG_BOX_HEIGHT, MYUCG->getDisplayWidth(), MSG_BOX_HEIGHT);
    MYUCG->setColor(COLOR_WHITE);
}

// drive and draw the message box
// only call when there are messages queued
// returns true when the message box has finished
bool MessageBox::draw()
{
    const int16_t dheight = MYUCG->getDisplayHeight();
    const int16_t dwidth = MYUCG->getDisplayWidth();

    ESP_LOGI(FNAME, "draw message");
    if ( _msg_to <= 0 ) {
        if ( ! nextMsg() ) {
            Clock::stop(this);
            ESP_LOGI(FNAME, "message tick stopped");
            return true;
        }
    }
    _msg_to--;

    _start_scroll++;
    if ( current->alert_level != 4 ) {
        // move text to expose the full message
        if (_start_scroll > (2000/(CLOCK_DIVIDER*Clock::TICK_ATOM)) && _nr_scroll > 0) {
            _nr_scroll--;
            _print_pos--;
            MYUCG->undoClipRange();
            MYUCG->setFont(ucg_font_fub14_hr, true);
            MYUCG->setColor(COLOR_WHITE);
            MYUCG->setFontPosBottom();
            MYUCG->setPrintPos(_print_pos, dheight-2);
            MYUCG->print(current->text.c_str());
            MYUCG->setClipRange(0, 0, dwidth, dheight - MSG_BOX_HEIGHT);
        }
    }
    else {
        // flash the message
        if ( (_start_scroll % 2) == 0 ) {
            MYUCG->undoClipRange();
            if ( _start_scroll & 0x2) {
                MYUCG->setColor(COLOR_RED);
            }
            else {
                MYUCG->setColor(COLOR_WHITE);
            }
            MYUCG->setFont(ucg_font_fub14_hr, true);
            MYUCG->setFontPosBottom();
            MYUCG->setPrintPos(_print_pos, dheight-2);
            MYUCG->print(current->text.c_str());
            MYUCG->setClipRange(0, 0, dwidth, dheight - MSG_BOX_HEIGHT);
            MYUCG->setColor(COLOR_WHITE);
        }
    }
    return false;
}

bool MessageBox::tick()
{
    int evt = ScreenEvent(ScreenEvent::MSG_BOX).raw;
    xQueueSend(uiEventQueue, &evt, 0);
    return false;
}

void MessageBox::press()
{
    popMessage();
    detach();
}