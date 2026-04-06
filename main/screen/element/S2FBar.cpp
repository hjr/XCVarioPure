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
#include "logdefnone.h"

#include <cmath>
#include <algorithm>

extern AdaptUGC *MYUCG;

static constexpr const int16_t SYMBOL_SIZE = 10;


S2FBar::S2FBar(int16_t cx, int16_t cy, int16_t width, int16_t gap) :
    ScreenElement(cx, cy),
    _width_half(width/2),
    _gap_half(gap/2)
{
    stepFromWidth(width);
    MYUCG->setFont(ucg_font_fub11_hr);
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
            MYUCG->setColor(ndl_color[needle_color.get()].color[0], ndl_color[needle_color.get()].color[1], ndl_color[needle_color.get()].color[2]);
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
    int16_t prev = _prev_s2f_level/2;
    level = std::abs(level/2);

    if ( level == std::abs(prev) && !_dirty) {
        return;
    }

    if (level == 0 ) {
        MYUCG->setColor(COLOR_DGREEN);
    }
    else if (level == 1) {
        MYUCG->setColor(COLOR_WGREY);
    }
    else {
        MYUCG->setColor(COLOR_BLACK);
    }
    MYUCG->drawRBox(_ref_x - _width_half + 6, _ref_y - _gap_half + 1, 2 * _width_half - 12, 2 * _gap_half - 3, 2);
}

void S2FBar::drawCircle()
{
    // draw circle
    MYUCG->setColor(COLOR_WGREY);
    MYUCG->drawCircle(_ref_x, _ref_y, SYMBOL_SIZE, UCG_DRAW_UPPER_RIGHT | UCG_DRAW_LOWER_RIGHT | UCG_DRAW_LOWER_LEFT);
    MYUCG->drawCircle(_ref_x, _ref_y, SYMBOL_SIZE - 1, UCG_DRAW_UPPER_RIGHT | UCG_DRAW_LOWER_RIGHT | UCG_DRAW_LOWER_LEFT);
    int tipx = _ref_x - SYMBOL_SIZE;
    constexpr const int16_t S2FTS = SYMBOL_SIZE/2;
    MYUCG->drawTriangle(tipx - S2FTS +2, _ref_y + S2FTS +1, tipx + S2FTS+2, _ref_y + S2FTS -2, tipx, _ref_y);

}

// speed to fly delta given in m/s, s2fd > 0 means speed up
// bars dice up 10 km/h steps
void S2FBar::draw(mps_t s2fd, bool cruise)
{
    int8_t level = 0;
    if ( cruise ) {
        // dice up into 10 kmh steps
        // draw max. three bars, then change color of the last one to red
        level = std::clamp((int)std::roundf(s2fd / LEVEL_DELTA), -4, 4);
    }

    if ( _dirty ) {
        // force redraw of all
        _prev_s2f_level = 0;
    }
    if ( cruise == _prev_cruise_mode && level == _prev_s2f_level && !_dirty ) {
        return;
    }
    if ( cruise != _prev_cruise_mode ) {
        // clear bounding box
        MYUCG->setColor(COLOR_BLACK);
        MYUCG->drawBox(_ref_x - _width_half - 2, _ref_y - _gap_half - 4 * _step, 2 * _width_half + 4, 2 * _gap_half + 8 * _step);
        // MYUCG->setColor(COLOR_WHITE);
        // MYUCG->drawFrame(_ref_x - _width_half - 2, _ref_y - _gap_half - 4 * _step, 2 * _width_half + 4, 2 * _gap_half + 8 * _step);
        _prev_s2f_level = 0;
    }

    if ( cruise ) {
        // ESP_LOGI(FNAME,"s2fbar %d %d", s2fd, level);
        if (level != _prev_s2f_level) {
            ESP_LOGI(FNAME,"S2FBar::draw s2fd: %d level: %d prev: %d", s2fd, level, _prev_s2f_level);

            int16_t inc = (level - _prev_s2f_level > 0) ? 1 : -1;
            for (int16_t i = _prev_s2f_level + ((_prev_s2f_level == 0 || _prev_s2f_level * inc > 0) ? inc : 0);
                i != level + ((i * inc < 0) ? 0 : inc); i += inc)
            {
                if (i != 0)
                {
                    drawArrow(_ref_x, _ref_y + (i > 0 ? 1 : -1) * _gap_half, i, i * inc < 0);
                    // ESP_LOGI(FNAME,"s2fbar draw %d,%d", i, (i*inc < 0)?0:inc);
                }
            }
       }

        // Fill the gap with a block if the speed is close to the target, otherwise clear it
        drawBlock(level);
    }
    else {
        drawCircle();
    }
    _prev_s2f_level = level;
    _prev_cruise_mode = cruise;
    
    _dirty = false;
}


