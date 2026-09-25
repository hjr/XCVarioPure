/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "S2FBar.h"

#include "setup/SetupNG.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "math/Floats.h"
#include "logdefnone.h"

#include <cmath>
#include <algorithm>

extern AdaptUGC *MYUCG;

static constexpr const int16_t SYMBOL_SIZE = 10;


S2FBar::S2FBar(int16_t cx, int16_t cy, int16_t width, int16_t gap) :
    ScreenElement(cx, cy),
    _width_half(width/2),
    _gap_half(gap/2),
    _step(stepFromWidth(width))
{
    _bbox = { Point(_ref.x - _width_half - 2, _ref.y - _gap_half - 4 * _step), Point(2 * _width_half + 2, 2 * _gap_half + 8 * _step) };
}


void S2FBar::drawArrow(int16_t x, int16_t y, int16_t level, bool del)
{
    constexpr int16_t gap = 2;
    int16_t height = _step;

    if (level == 0)
        return;

    int16_t init = 1;
    if (std::abs(level) == 4)
    {
        init = 2;
        if (del) {
            MYUCG->setColor(COLOR_WGREY);
        } else {
            MYUCG->setColor(COLOR_WHITE);
        }
    }
    else if (del) {
        MYUCG->setColor(COLOR_BLACK);
    }
    else {
        MYUCG->setColor(COLOR_WGREY);
    }

    int16_t l = level - init;
    if (level < 0) {
        height = -height;
        l = level + init;
    }
    MYUCG->drawTriangle(x, y + l * (_step + gap), x, y + l * (_step + gap) + height, x - _width_half, y + l * gap);
    MYUCG->drawTriangle(x, y + l * (_step + gap), x + _width_half, y + l * gap, x, y + l * (_step + gap) + height);
}

void S2FBar::drawBlock(int16_t level)
{
    if ( level == std::abs(_prev_hash._s2f_level/2) && !_dirty) {
        return;
    }

    if (level == 0 ) {
        MYUCG->setColor(COLOR_DGREEN);
    }
    else {
        MYUCG->setColor(COLOR_WGREY);
    }
    MYUCG->drawRBox(_ref.x - _width_half + 3, _ref.y - _gap_half + 1, 2 * _width_half - 6, 2 * _gap_half - 3, 2);
}

void S2FBar::drawCircle(int16_t score)
{
    MYUCG->startBuffering(_bbox.pmin.x, _bbox.pmin.y, _bbox.pmax.x, _bbox.pmax.y);

    // draw circle
    MYUCG->setColor(COLOR_WGREY);
    MYUCG->drawCircle(_ref.x, _ref.y + 4, SYMBOL_SIZE, UCG_DRAW_UPPER_RIGHT | UCG_DRAW_LOWER_RIGHT | UCG_DRAW_LOWER_LEFT);
    MYUCG->drawCircle(_ref.x, _ref.y + 4, SYMBOL_SIZE - 1, UCG_DRAW_UPPER_RIGHT | UCG_DRAW_LOWER_RIGHT | UCG_DRAW_LOWER_LEFT);
    int tipx = _ref.x - SYMBOL_SIZE;
    constexpr const int16_t S2FTS = SYMBOL_SIZE/2;
    MYUCG->drawTriangle(tipx - S2FTS +2, _ref.y + 4 + S2FTS +1, tipx + S2FTS+2, _ref.y + 4 + S2FTS -2, tipx, _ref.y + 4);

    if ( score < 25 ) {
        MYUCG->finishBuffering();
        return;
    }

    // draw score
    MYUCG->setFont(ucg_font_fub14_hr, true);
    MYUCG->setColor(COLOR_WHITE);

    MYUCG->setPrintPos(_ref.x - 12, _ref.y + 6);
    MYUCG->print(score);
    MYUCG->finishBuffering();
}

// speed to fly delta given in m/s, s2fd > 0 means speed up
// bars dice up 10 km/h steps
void S2FBar::draw(mps_t s2fd, bool cruise)
{
    Hash current = _prev_hash;
    if ( cruise ) {
        // dice up into 10 kmh steps, map to -4..+4, 0 means no speed up/down
        // draw max. three bars, then change color of the last one
        current._s2f_level = std::clamp(fast_iroundf(s2fd / LEVEL_DELTA), -4, 4);
        current._cruise_mode = 1;
    }
    else {
        current._score = std::clamp(fast_iroundf(thermal_score.get() * 100), 0, 99);
        current._cruise_mode = 0;
    }

    if ( _dirty ) {
        // force redraw of all
        _prev_hash._s2f_level = 0;
    }
    if ( current._raw == _prev_hash._raw && !_dirty ) {
        return;
    }
    if ( cruise != _prev_hash._cruise_mode ) {
        // clear bounding box
        MYUCG->setColor(COLOR_BLACK);
        MYUCG->drawBox(_bbox.pmin.x, _bbox.pmin.y, _bbox.pmax.x, _bbox.pmax.y);
        current._s2f_level = 0; // draw all levels
    }

    // MYUCG->setColor(COLOR_WHITE);
    // MYUCG->drawFrame(_bbox.pmin.x, _bbox.pmin.y, _bbox.pmax.x + 1, _bbox.pmax.y + 1);

    if ( cruise ) {
        // ESP_LOGI(FNAME,"s2fbar %d %d", s2fd, level);
        if (current._s2f_level != _prev_hash._s2f_level) {
            ESP_LOGI(FNAME,"S2FBar::draw s2fd: %d level: %d prev: %d", s2fd, current._s2f_level, _prev_hash._s2f_level);

            int16_t inc = (current._s2f_level - _prev_hash._s2f_level > 0) ? 1 : -1;
            for (int16_t i = _prev_hash._s2f_level + ((_prev_hash._s2f_level == 0 || _prev_hash._s2f_level * inc > 0) ? inc : 0);
                i != current._s2f_level + ((i * inc < 0) ? 0 : inc); i += inc)
            {
                if (i != 0)
                {
                    drawArrow(_ref.x, _ref.y + (i > 0 ? 1 : -1) * _gap_half, i, i * inc < 0);
                    // ESP_LOGI(FNAME,"s2fbar draw %d,%d", i, (i*inc < 0)?0:inc);
                }
            }
        }

        // Fill the gap with a block if the speed is close to the target, otherwise clear it
        drawBlock(current._s2f_level);
    }
    else {
        drawCircle(current._score);
    }
    _prev_hash._raw = current._raw;
    
    _dirty = false;
}


