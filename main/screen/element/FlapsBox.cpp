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
#include "comm/DeviceMgr.h"
#include "protocol/NMEA.h"
#include "driver/audio/ESPAudio.h"
#include "driver/time/Clock.h"
#include "math/Floats.h"
#include "math/Units.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <cstdio>
#include <algorithm>

extern AdaptUGC *MYUCG;


constexpr const int16_t BAND_OFF   = 26;
constexpr const int16_t BOX_WIDTH  = 28;
constexpr const int16_t BOX_CORNER = 6;
constexpr const int16_t LABEL_SPACING = 26;
constexpr const int     SOUND_LATENCY = 5000; // msec to wait before making sound at all

int16_t FlapsBox::BOX_LENGTH = BOX_WIDTH * 3;

/////////////////////////
// FBoxStateHash
/////////////////////////
FBoxStateHash::FBoxStateHash(float f, int16_t c_pix) :
    wkidx10( fast_iroundf(f*10.) )
{
    center_pix = c_pix;
}

bool FBoxStateHash::operator!=(const FBoxStateHash &other) const noexcept
{
    // position of the wk labels
    if ( wkidx10 != other.wkidx10 ) return true;
    // position of the speed band
    if ( center_pix != other.center_pix ) return true;
    return false;
}

/////////////////////////
// FlapsBox
/////////////////////////
FlapsBox::FlapsBox(Flap* flap, int16_t cx, int16_t cy, bool vertical) :
    ScreenElement(cx, cy),
    _flap(flap),
    _fp_filter(0.3f),
    _last_event(0,0),
    _vertical(vertical)
{
    MYUCG->setFont(ucg_font_fub11_hr);
    _LFH = MYUCG->getFontAscent() - MYUCG->getFontDescent() + 4;
    ESP_LOGI(FNAME, "FlapsBox label height %d, a%d d%d", _LFH, MYUCG->getFontAscent(), MYUCG->getFontDescent());
}

// void FlapsBox::setLength(int16_t length)
// {
//     BOX_LENGTH = length;
//     ESP_LOGI(FNAME, "setLength %d, PIX_PER_MPS %.3f", BOX_LENGTH, PIX_PER_MPS);
// }

void FlapsBox::drawLabels(FBoxStateHash cs)
{
    ESP_LOGI(FNAME, "draw wkf=%.1f, %d", cs.wkidx10/10.f, cs.center_pix);
    int16_t boxx = _ref.x;
    int16_t boxy = _ref.y - BOX_LENGTH / 2;
    int16_t boxw = BOX_WIDTH;
    int16_t boxh = BOX_LENGTH;

    // background speed band
    MYUCG->setColor(1, DARK_DGREY);
    MYUCG->startBuffering(boxx, boxy, boxw+1, boxh+1);
    int16_t green_top =  _ref.y + cs.center_pix - BOX_WIDTH/2;
    if ( green_top < _ref.y + boxh ) { // the green part
        MYUCG->setColor(COLOR_DGREEN);
        MYUCG->drawBox(boxx, green_top, boxw+1, BOX_WIDTH);
    }
    MYUCG->setColor(1, g_col_background, g_col_background, g_col_background);

    // max. three foreground labels
    MYUCG->setFont(ucg_font_fub14_hr);
    const int from = std::max((int)(fast_floorf(cs.getWk() - 0.8)), 0);
    const int to   = std::min(fast_iroundf(cs.getWk() + 1.3), _flap->getNrPositions() - 1);
    for (int wk = from; wk <= to; wk++)
    {
        const char *label = _flap->getFL(wk)->label;
        int16_t pixoff = -(cs.getWk() - wk) * LABEL_SPACING; // 20 pixels per flap step
        // ESP_LOGI(FNAME, "wk %d, pixoff %d", wk, pixoff);
        int16_t lwidth = MYUCG->getStrWidth(label);
        MYUCG->setPrintPos(_ref.x + (BOX_WIDTH - lwidth)/2 + 1, _ref.y + pixoff + _LFH/2);
        if ( cs.wkidx10/10 == wk ) {
            MYUCG->setColor(COLOR_WHITE); // highlight the recommendation, or current position
        } else {
            MYUCG->setColor(COLOR_HEADER_LIGHT);
        }
        MYUCG->print(label);
    }

    // box frame
    MYUCG->setColor(COLOR_WHITE);
    MYUCG->drawHLine(boxx, _ref.y - 14, boxw);
    MYUCG->drawHLine(boxx, _ref.y + 14, boxw);
    MYUCG->finishBuffering();
}

// currently called ca. every 0,3sec, 
// but only redraws when the position changes enough to change the state hash
void FlapsBox::draw(mps_t ias)
{
    if ( _dirty ) {
        MYUCG->setColor(DARK_DGREY);
        MYUCG->drawRBox(_ref.x, _ref.y - BOX_LENGTH/2 - BOX_CORNER, BOX_WIDTH, 2 * BOX_CORNER, BOX_CORNER);
        MYUCG->drawRBox(_ref.x, _ref.y + BOX_LENGTH/2 - BOX_CORNER, BOX_WIDTH, 2 * BOX_CORNER, BOX_CORNER);

        MYUCG->setColor(COLOR_WHITE);
        MYUCG->setClipRange(_ref.x - 5, _ref.y - BOX_WIDTH/2, 6, BOX_WIDTH + 1);
        MYUCG->drawRFrame(_ref.x - 5, _ref.y - BOX_WIDTH/2, 10, BOX_WIDTH, 4);
        MYUCG->undoClipRange();
    }

    float curr_fp;
    bool have_sens = Flap::sensAvailable();
    float flap_ideal = _flap->getOptimum(ias);
    if ( have_sens ) {
        curr_fp = Flap::getFlapPosition();
        ESP_LOGI(FNAME, "flap position from sensor: %1.2f", curr_fp);
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
        curr_fp = std::roundf(flap_ideal); // without sensor, just show the recommendation, and play sound when it changes
    }
    // damp speed of indicator to make it good readable
    curr_fp = _fp_filter.filter(curr_fp);

    if ( airborne.get() == false ) {
        // on ground, set the ias virtually into the green band for the correct start position
        flap_ideal = flap_takeoff.get();
        ias = _flap->getSpeed(flap_ideal); // pretend start speed
        ESP_LOGI(FNAME, "on ground, set ias to %.1f for flap position %.1f", ias, flap_ideal); 
    }

    mps_t minv, maxv;
    minv = _flap->getSpeedBand(curr_fp, maxv);
    float pix_per_mps = std::max(BAND_OFF / (maxv - minv), 4.f); // ensure a minimum for level 0 (ca. 180 - VNE)
    int16_t band_offset = std::clamp((int16_t)fast_iroundf((_flap->getSpeed(curr_fp) - ias) * pix_per_mps), (int16_t)-BAND_OFF, BAND_OFF);
    ESP_LOGI(FNAME,"bandoff - wkset: %.1f -> %.1fmps wkideal: %.1f pix_per_mps %.3f", curr_fp, _flap->getSpeed(curr_fp), flap_ideal, pix_per_mps);
    minv -= ias;
    maxv -= ias;
    // the three variables that define the box state
    FBoxStateHash current_state( curr_fp, band_offset);
    if ( current_state != _state || _dirty ) {
        ESP_LOGI(FNAME,"wkf:%.1f bo:%d minv:%.1f maxv:%.1f ias:%.1f", current_state.getWk(), band_offset, minv, maxv, ias);
        drawLabels(current_state);
    }

    // Check on logging
    if ( logging.get() == LOGG_RAW_SENSOR_DATA && _state.wkidx10 != current_state.wkidx10 ) {
        ProtocolItf *prtcl = DEVMAN->getProtocol(NAVI_DEV, XCVARIO_P);
        if ( prtcl ) {
            (static_cast<NmeaPrtcl*>(prtcl))->sendXcvIntItem("FLP", current_state.wkidx10);
        }
    }

    _state = current_state;
    _dirty = false;

    if ( flapbox_enable.get() == (uint8_t)flap_box_conf::FLAP_BOX_VIS ) {
        return; // only show the indicator, do not play sounds
    }

    // do sounds when stepping over the speed range (with sensor),
    // or when the recommended position changes (without sensor)
    int16_t flap_idx = fast_iroundf(curr_fp);
    if ( have_sens ) {
        _last_flap_idx = flap_idx; // keep in sync with actual position, option to not play any sound
        if ( minv > 0. && flap_idx < _flap->getNrPositions()-1 ) { // slipped below the speed band
            flap_idx++;
        }
        else if ( maxv < 0. && flap_idx > 0 ) { // exceeded the speed band
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
