/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "S2fSwitch.h"

#include "protocol/Clock.h"
#include "setup/CruiseMode.h"
#include "setup/SetupNG.h"
#include "screen/UiEvents.h"
#include "screen/DrawDisplay.h"
#include "sensor.h"
#include "Flap.h"
#include "sensor/imu/KalmanMPU6050.h"
#include "logdefnone.h"

// auto switch filters
static bool Speed();
static bool Flap();
static bool Omega();

S2fSwitch *S2FSWITCH = nullptr;

// clock tick timer (task context)
bool IRAM_ATTR S2fSwitch::tick()
{
    // called every 10msec
    bool buttonRead = gpio_get_level(_sw) == _active_level; // gpio level 1 -- not pressed (open pull-up)

    _debounce = (buttonRead == _lastButtonRead) ? (_debounce + 1) : 0;
    _lastButtonRead = buttonRead;

    if (_debounce < _dcount || buttonRead == _state)
    {
        if (_dcount > 3) {
            _dcount--;
        }
        return false;
    }
    _dcount = 15;

    // A valid edge detected
    _state = buttonRead;
    int gotEvent;
    ESP_LOGI(FNAME, "cruise_switch _state: %d", _state);
    if (s2f_switch_type.get() != S2F_HW_PUSH_BUTTON)
    {
        // switch
        if (_state)
        {
            gotEvent = ModeEvent(ModeEvent::MODE_S2F).raw; // Button state active
        }
        else
        {
            gotEvent = ModeEvent(ModeEvent::MODE_VARIO).raw;
        }
        xQueueSend(uiEventQueue, &gotEvent, 0);
    }
    else if (!_state)
    {
        // toggle straight when pressed
        gotEvent = ModeEvent(ModeEvent::MODE_TOGGLE).raw;
        xQueueSend(uiEventQueue, &gotEvent, 0);
    }

    return false;
}

S2fSwitch::S2fSwitch(gpio_num_t sw) :
    Clock_I(1),
    _sw(sw)
{
    // prepare switch gpio
    gpio_set_direction(_sw, GPIO_MODE_INPUT);
    gpio_set_pull_mode(_sw, GPIO_PULLUP_ONLY);

    // init button semantic
    updateSwitchSetup();

    if ( s2f_switch_type.get() == S2F_HW_SWITCH || s2f_switch_type.get() == S2F_HW_SWITCH_INVERTED ) {
        ESP_LOGI(FNAME, "Init CMode _state: %d", _state);
        CRMOD.setCMode(_state);
    }
}

S2fSwitch::~S2fSwitch()
{
    Clock::stop(this);
}

void S2fSwitch::updateSwitchSetup()
{
    // setup the switch
    _active_level = 0; // default for push button is "pushed"
    _state = _lastButtonRead = false; // init state is "Vario" (CMode == false)
    if (s2f_switch_type.get() != S2F_HW_PUSH_BUTTON) {
        // in case of a bistable switch set the initial Vario Mode accordingly
        _active_level = (s2f_switch_type.get() == S2F_HW_SWITCH_INVERTED) ? 1 : 0;
        _state = _lastButtonRead = gpio_get_level(_sw) == _active_level;  // first button state read
    }
    ESP_LOGI(FNAME, "updateSwitchSetup _state: %d", _state);
    if ((s2f_switch_type.get() != S2F_SWITCH_DISABLE)) {
        Clock::start(this);
    }
    else {
        Clock::stop(this);
    }

    // setup auto switches
    _auto_lag = 0;
    _auto_state = CRMOD.getCMode(); // init event signalling state
    CRMOD.unlockCMode();
    switch (s2f_switch_mode.get())
    {
    case AM_AUTOSPEED:
        _auto_lag = static_cast<int>(s2f_auto_lag.get());
        auto_plug = Speed;
        break;
    case AM_FLAP:
        _auto_lag = static_cast<int>(s2f_auto_lag.get());
        auto_plug = Flap;
        break;
    case AM_AHRS:
        _auto_lag = static_cast<int>(s2f_auto_lag.get());
        auto_plug = Omega;
        break;
    case AM_VARIO:
    case AM_S2F:
        CRMOD.lockTo(s2f_switch_mode.get()==AM_S2F);
        [[fallthrough]];
    case AM_MANUALLY:
    default:
        auto_plug = nullptr; // alias "external", event propagation is intrinsic
        break;
    }
}

//////////////
// auto switch filter
bool Speed()
{
    if (ias.get() < Units::kmh_to_mps(s2f_threshold.get()))
    {
        return false; // vario mode
    }
    else
    {
        return true;
    }
}
bool Flap()
{
    if (FLAP) 
    {
        return FLAP->getFlapPosition() < s2f_flap_pos.get();
    }
    return true;
}
bool Omega()
{
    float ref = s2f_gyro_deg.get();
    if ( rad2deg(abs(IMU::getVerticalOmega())) < ref)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// call once a second
void S2fSwitch::checkCruiseMode()
{
    if (auto_plug)
    {
        bool cm = (*auto_plug)();
        // an auto switch decision can be reverted before becoming effective
        // when e.g. the criterion toggles in the defined auto lag duration
        if (cm != _auto_state)
        {
            // if (_lag_counter == (_auto_lag / (_auto_state ? 2 : 1))) // go twice as fast into vario mode, as out of it
            if (_lag_counter == _auto_lag )
            {
                _auto_state = cm;
                ESP_LOGI(FNAME, "New S2F auto mode: %d", cm);
                int gotEvent = ModeEvent(cm ? ModeEvent::MODE_S2F : ModeEvent::MODE_VARIO).raw;
                xQueueSend(uiEventQueue, &gotEvent, 0);
            }
            _lag_counter++;
        }
        else
        {
            _lag_counter = 0;
        }
    }
}
