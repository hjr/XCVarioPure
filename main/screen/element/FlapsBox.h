/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ScreenElement.h"
#include "sensor/Filters.h"

#include <cstdint>

class Flap; // where all the data and logic resides

union SwitchEvent {
    struct {
        uint8_t from;
        uint8_t to; // flap setting
    };
    uint16_t raw = 0;

    constexpr SwitchEvent() = delete;
    constexpr SwitchEvent(int f, int t)
        : from((f & 0xFF)), to((t & 0xFF)) {}
    constexpr bool operator!=(const SwitchEvent &other) const noexcept {
        return raw != other.raw; }
};

struct FBoxStateHash {
    int16_t wkidx10;
    int16_t top_pix;
    int16_t bottom_pix;
    union {
        struct {
            int16_t top_exseed :1;
            int16_t bottom_exseed :1;
        };
        int16_t raw = 0;
    };
    FBoxStateHash() = delete;
    FBoxStateHash(float f, float minvd, float maxvd);
    constexpr int getWkIdx() const { return (wkidx10+5) / 10; }
    constexpr float getWk() const { return (float)(wkidx10) / 10.f; }
    bool operator!=(const FBoxStateHash &other) const noexcept;
};

// a visual flaps assistant
class FlapsBox : public ScreenElement
{
public:
    FlapsBox(Flap* flap, int16_t cx, int16_t cy, bool vertical=true);
    // API
    void setLength(int16_t length);
    using ScreenElement::draw;
    void draw(mps_t ias);

private:
    void drawLabels(FBoxStateHash cs);

private: // attributes
    Flap* _flap;
    LowPassFilterT<float> _fp_filter;
    FBoxStateHash _state = {0,0,0};
    int   _last_flap_idx = 0;
    int   _snd_event_time = 0;
    SwitchEvent _last_event;
    int   _same_event_to = -1;
    bool  _vertical;
    int16_t _LFH;

public:
    static int16_t BOX_LENGTH;
    static float   PIX_PER_MPS;
};

