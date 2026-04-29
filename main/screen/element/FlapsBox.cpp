/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "FlapsBox.h"

#include "Flap.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "driver/audio/ESPAudio.h"
#include "driver/time/Clock.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"
#include "math/Floats.h"
#include "math/Units.h"

#include <cstdio>

extern AdaptUGC *MYUCG;

constexpr const int16_t BOX_WIDTH  = 28;
// constexpr const int16_t BOX_LENGTH = 100; // w/o corners
constexpr const int16_t BOX_CORNER = 8;
constexpr const int16_t LABEL_SPACING = 20;
// constexpr const float   PIX_PER_MPS = ((float)(BOX_LENGTH)/2) / Units::kmh_to_mps(10.f); // 10 km/h range on half flap box
constexpr const int     SOUND_LATENCY = 5000; // msec to wait before making sound at all

int16_t FlapsBox::BOX_LENGTH = 100;
float   FlapsBox::PIX_PER_MPS = 17.999f;

/////////////////////////
// FBoxStateHash
/////////////////////////
FBoxStateHash::FBoxStateHash(float f, float minvd, float maxvd) :
    wkidx10( fast_iroundf(f*10.) )
{
    top_pix = static_cast<int16_t>(minvd * FlapsBox::PIX_PER_MPS);
    bottom_pix = static_cast<int16_t>(maxvd * FlapsBox::PIX_PER_MPS);
    top_exseed = (bottom_pix < -FlapsBox::BOX_LENGTH/2) ? 1 : 0;
    bottom_exseed = (top_pix > FlapsBox::BOX_LENGTH/2) ? 1 : 0;
}

bool FBoxStateHash::operator!=(const FBoxStateHash &other) const noexcept
{
    // position of the wk labels
    if ( wkidx10 != other.wkidx10 ) return true;
    // position of the speed band
    if ( ((top_pix > -FlapsBox::BOX_LENGTH/2
            && top_pix < FlapsBox::BOX_LENGTH/2)
        || (other.top_pix > -FlapsBox::BOX_LENGTH/2
            && other.top_pix < FlapsBox::BOX_LENGTH/2))
        && top_pix != other.top_pix ) return true;
    if ( ((bottom_pix > -FlapsBox::BOX_LENGTH/2
            && bottom_pix < FlapsBox::BOX_LENGTH/2)
        || (other.bottom_pix > -FlapsBox::BOX_LENGTH/2
            && other.bottom_pix < FlapsBox::BOX_LENGTH/2))
        && bottom_pix != other.bottom_pix) return true;
    // excess markers
    if ( raw != other.raw ) return true;
    return false;
}

/////////////////////////
// FlapsBox
/////////////////////////
FlapsBox::FlapsBox(Flap* flap, int16_t cx, int16_t cy, bool vertical) :
    ScreenElement(cx, cy),
    _flap(flap),
    _fp_filter(0.15f),
    _last_event(0,0),
    _vertical(vertical)
{
    MYUCG->setFont(ucg_font_fub11_hr);
    _LFH = MYUCG->getFontAscent() - MYUCG->getFontDescent() + 4;
    ESP_LOGI(FNAME, "FlapsBox label height %d, a%d d%d", _LFH, MYUCG->getFontAscent(), MYUCG->getFontDescent());
}

void FlapsBox::setLength(int16_t length)
{
    BOX_LENGTH = length;
    PIX_PER_MPS = ((float)(BOX_LENGTH)/2) / Units::kmh_to_mps(10.f); // 10 km/h range on half flap box
    ESP_LOGI(FNAME, "setLength %d, PIX_PER_MPS %.3f", BOX_LENGTH, PIX_PER_MPS);
}

void FlapsBox::drawLabels(FBoxStateHash cs)
{
    ESP_LOGI(FNAME, "draw wkf=%.1f, %d/%d", cs.wkidx10/10.f, cs.top_pix, cs.bottom_pix);
    int16_t boxx = _ref.x;
    int16_t boxy = _ref.y - BOX_LENGTH / 2;
    int16_t boxw = BOX_WIDTH;
    int16_t boxh = BOX_LENGTH;

    // colored speed range (background)
    // draw excess corners in green
    if ( cs.top_exseed != _state.top_exseed ) {
        if ( cs.top_exseed ) {
            MYUCG->setColor(COLOR_DGREEN);
        } else {
            MYUCG->setColor(COLOR_BLACK);
        }
        MYUCG->setClipRange(boxx, boxy - BOX_CORNER, boxw+1, BOX_CORNER);
        MYUCG->drawRBox(boxx+1, boxy - BOX_CORNER + 1, boxw-1, 2 * BOX_CORNER, BOX_CORNER - 2);
        MYUCG->undoClipRange();
    }
    if ( cs.bottom_exseed != _state.bottom_exseed ) {
        if ( cs.bottom_exseed ) {
            MYUCG->setColor(COLOR_DGREEN);
        } else {
            MYUCG->setColor(COLOR_BLACK);
        }
        MYUCG->setClipRange(boxx, boxy + boxh, boxw+1, BOX_CORNER);
        MYUCG->drawRBox(boxx+1, boxy + boxh - BOX_CORNER - 1, boxw, 2 * BOX_CORNER, BOX_CORNER - 2);
        MYUCG->undoClipRange();
    }

    // background speed band
    MYUCG->setColor(1, COLOR_MGREY);
    MYUCG->startBuffering(boxx, boxy, boxw+1, boxh+1);
    int16_t green_top =  _ref.y + cs.top_pix;
    if ( green_top < _ref.y + boxh ) { // the green part
        MYUCG->setColor(COLOR_DGREEN);
        MYUCG->drawBox(boxx, green_top, boxw+1, _ref.y - green_top + cs.bottom_pix+1);
    }

    // foreground labels
    MYUCG->setFont(ucg_font_fub14_hr);
    const int from = std::max((int)(fast_floorf(cs.getWk() + 0.2)), 0);
    const int to   = ((from + 1) == fast_iroundf(cs.getWk() + 0.3)) ? from + 1 : from;
    for (int wk = from; wk <= to; wk++)
    {
        const char *label = _flap->getFL(wk)->label;
        int16_t pixoff = -(cs.getWk() - wk) * LABEL_SPACING; // 20 pixels per flap step
        // ESP_LOGI(FNAME, "wk %d, pixoff %d", wk, pixoff);
        int16_t lwidth = MYUCG->getStrWidth(label);
        MYUCG->setPrintPos(_ref.x + (BOX_WIDTH - lwidth)/2 + 1, _ref.y + pixoff + _LFH/2);
        MYUCG->setColor(COLOR_WHITE); // highlight the recommendation, or current position
        MYUCG->print(label);
    }
    MYUCG->setColor(COLOR_WHITE);
    MYUCG->drawHLine(boxx, _ref.y - 14, boxw);
    MYUCG->drawHLine(boxx, _ref.y + 14, boxw);
    MYUCG->finishBuffering();
    MYUCG->setColor(1, g_col_background, g_col_background, g_col_background);

    _state = cs;
}

// currently called ca. every 0,3sec, 
// but only redraws when the position changes enough to change the state hash
void FlapsBox::draw(mps_t ias)
{
    if ( _dirty ) {
        MYUCG->setColor(COLOR_WHITE);
        MYUCG->setClipRange(_ref.x - 5, _ref.y - 14, 6, 29);
        MYUCG->drawRFrame(_ref.x - 5, _ref.y - 14, 10, 28, 4);
        MYUCG->undoClipRange();
    }

    float curr_fp;
    bool have_sens = Flap::sensAvailable();
    if ( have_sens ) {
        curr_fp = _flap->getFlapPosition();
        // rasterize to .0, and .5
        float fp_base = fast_floorf(curr_fp);
        if ( curr_fp - fp_base < 0.25f ) {
            curr_fp = fp_base;
        } else if ( curr_fp - fp_base < 0.75f ) {
            curr_fp = fp_base + 0.5f;
        } else {
            curr_fp = fp_base + 1.f;
        }
    } else {
        curr_fp = fast_iroundf(_flap->getOptimum(ias));
    }
    // damp speed of indicator to make it good readable
    curr_fp = _fp_filter.filter(curr_fp);

    mps_t minv, maxv;
    minv = _flap->getSpeedBand(curr_fp, maxv);
    // enlarge the speed band just for a nicer overlapping visualization
    minv -= Units::kmh_to_mps(7.f);
    maxv += Units::kmh_to_mps(7.f);
    if ( airborne.get() == false ) {
        // on ground, set the ias virtually into the green band for the correct start position
        ias = _flap->getSpeed(flap_takeoff.get()); // pretend start speed
        ESP_LOGI(FNAME, "on ground, set ias to %.1f for flap position %.1f", ias, curr_fp);
    }
    minv -= ias;
    maxv -= ias;
    // the three variables that define the box state
    FBoxStateHash current_state( curr_fp, minv, maxv);
    if ( current_state != _state || _dirty ) {
        ESP_LOGI(FNAME,"wkf:%.1f minv:%.1f maxv:%.1f ias:%.1f", current_state.getWk(), minv, maxv, ias);
        drawLabels(current_state);
    }
    _dirty = false;

    if ( flapbox_enable.get() == 2 ) {
        return; // only show the indicator, do not play sounds
    }

    // do sounds when stepping over the speed range (with sensor),
    // or when the recommended position changes (without sensor)
    int flap_idx = fast_iroundf(curr_fp);
    if ( have_sens ) {
        _last_flap_idx = flap_idx; // keep in sync with actual position, option to not play any sound
        if ( minv > 0. && flap_idx < _flap->getNrPositions()-1 ) { // slipped below the lower speed limit
            flap_idx++;
        }
        else if ( maxv < 0. && flap_idx > 0 ) { // exceeded the upper speed limit
            flap_idx--;
        }
    }

    // play sounds after some latency
    if ( flap_idx != _last_flap_idx ) {
        SwitchEvent current_event = SwitchEvent(_last_flap_idx, flap_idx);
        if ( current_event != _last_event ) {
            if ( Clock::getMillis() - _snd_event_time > SOUND_LATENCY ) {
                ESP_LOGI(FNAME, "flap_idx changed from %d to %d", _last_flap_idx, flap_idx);
                if ( flap_idx > _last_flap_idx ) {
                    // flap back sound
                    AUDIO->startSound(AUDIO_FLAP_BACK, true);
                } else {
                    // flap forward sound
                    AUDIO->startSound(AUDIO_FLAP_FORWARD, true);
                }
                _last_event = current_event;
                _same_event_to = 200;
                _last_flap_idx = flap_idx;
            }
        }
        else {
            if ( _same_event_to > 0 ) {
                _same_event_to--;
            } else {
                // reset to allow sound again
                _last_event = SwitchEvent(0,0);
            }
        }
    }
    else {
        // reset latency counter when back in same position
        _snd_event_time = Clock::getMillis();
        _last_event = SwitchEvent(0,0);
    }

}
