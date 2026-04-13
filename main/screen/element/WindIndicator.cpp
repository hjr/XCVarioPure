/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "WindIndicator.h"

#include "math/Floats.h"
#include "PolarGauge.h"
#include "LargeFigure.h"
#include "Colors.h"
#include "AdaptUGC.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <cstdio>

extern AdaptUGC *MYUCG;

int16_t WindIndicator::_cheight = 0;
int16_t WindIndicator::_cwidth = 0;


WindIndicator::WindIndicator(PolarGauge &g, bool live) :
    _gauge(g),
    _live(live)
{
    MYUCG->setFont(ucg_font_fub14_hr);
    _cheight = MYUCG->getFontAscent() - MYUCG->getFontDescent();
    _cwidth = MYUCG->getStrWidth("0");
    ESP_LOGI(FNAME, "asc %d %d", MYUCG->getFontAscent(), MYUCG->getFontDescent());
    ESP_LOGI(FNAME, "sw %d", _cwidth);

    constexpr int16_t W = 38;
    constexpr int16_t I = 18;
    _indgeom[0] = { 0, 90 };
    _indgeom[1] = { W/2, 25};
    _indgeom[2] = { I/2, 0};
    _indgeom[3] = { -I/2, 0};
    _indgeom[4] = { -W/2, 25};

    if ( _live ) {
        _color = { ndl_color[needle_color.get()].color[0], ndl_color[needle_color.get()].color[1], ndl_color[needle_color.get()].color[2] };
    }
    else {
        _color = { COLOR_LBBLUE };
    }
}

// Check if changed
// -> clear, and take over new values (but not draw)
bool WindIndicator::changed(WindData w)
{
    if (w != _wind) {
        drawWind(true);
        _gauge.drawRose(_wind.getDeg());
        _wind = w;
        ESP_LOGI(FNAME, "Wind (%d,%.2f)", _wind.getDeg(), _wind.getVal());
        if ( !_wind.isValid() ) {
            return false;
        }
        return true;
    }
    return false;
}

// direction [°], northwind as 0°; strength [any]
// wval < 0 just removes the
bool WindIndicator::draw(WindData w)
{
    ESP_LOGI(FNAME, "Wind (%d,%.2f)", w.getDeg(), w.getVal());
    if (w != _wind) {
        drawWind(true);
        _gauge.drawRose(_wind.getDeg());
        _wind = w;
        if ( !_wind.isValid() ) {
            return false;
        }
        drawWind(false);
        return true;
    }
    return false;
}

// 0° reference on top of the compass rose
void WindIndicator::drawWind(bool erase)
{
    if ( ! _wind.isValid() ) {
        return;
    }

    ESP_LOGI(FNAME, "wind deg:%d, v:%.1f ", _wind.getDeg(), _wind.getVal());
    if ( ! _live ) {
        float si = -fast_sin_idx(_wind.getDeg2());
        float co = fast_cos_idx(_wind.getDeg2());

        int16_t x0 = si * (_gauge._radius-2);
        int16_t y0 = co * (_gauge._radius-2);
        int16_t x1 = x0 / 4;
        int16_t y1 = y0 / 4;
        if (erase) {
            MYUCG->setColor(COLOR_BLACK);
        } else {
            MYUCG->setColor(COLOR_WGREY);
        }
        // a number at where the wind is coming from
        int16_t value = _wind.getVal() * _gauge._unit_fac;
        int16_t sw = _cwidth * count_digits(value);
        int16_t xshift = sw / 2;
        if (erase) {
            MYUCG->drawBox(_gauge._ref.x - x0 - x1 / 2 - xshift, _gauge._ref.y - y0 - y1 / 2 - _cheight / 2, sw, _cheight);
        } else {
            char buf[8];
            if (_wind.isValid() && value < 100) {
                sprintf(buf, "%d", value);
            } else {
                sprintf(buf, "oo");
            }
            MYUCG->setFont(ucg_font_fub14_hr);
            MYUCG->setFontPosCenter();
            MYUCG->setPrintPos(_gauge._ref.x - x0 - x1 / 2 - xshift, _gauge._ref.y - y0 - y1 / 2);
            MYUCG->print(buf);
            MYUCG->setFontPosBottom();
        }
    }

    // Calc the bounding box of the arrow
    if ( _wind.isValid()) {
        _abox[0] = _arrow[0];
        _abox[1] = _arrow[0];
        IpsDisplay::superBBox(&_arrow[1], 4, _abox);
    }
    else {
        _abox[0] = Point(_gauge._ref.x, _gauge._ref.y);
        _abox[1] = Point(_gauge._ref.x, _gauge._ref.y);
        if ( erase ) {
            return; // only a previously painted arrow can be removed
         }
    }
    if ( ! erase ) {
        // an arrow tip in direction the wind is blowing (180° other direction)
        constexpr float vref = Units::kmh_to_mps(25.f); // scale the arrow size with the wind speed
        float scale = sqrtf((std::clamp(_wind.getVal(), Units::kmh_to_mps(10.f), vref-1.f) - .4f) / vref);
        Point::scaleNshift(_indgeom, 5, Point(0, -(_gauge._radius-14)), scale, _arrow);
        Point::rotate(_arrow, 5, _wind.getDeg2(), _arrow);
        // shift to gauge center
        for (int i = 0; i < 5; i++) {
            _arrow[i] += _gauge._ref;
        }
    }
    // MYUCG->drawFrame(_abox[0].x, _abox[0].y, _abox[1].x - _abox[0].x +1, _abox[1].y - _abox[0].y +1);
    if ( (_abox[1].x - _abox[0].x +1) * (_abox[1].y - _abox[0].y +1) > (EGLIB_FRAMEBUFFER_SIZE/3) ) {
        ESP_LOGI(FNAME, "bbox too big, skip draw");
        return;
    }
    MYUCG->startBuffering(_abox[0].x, _abox[0].y, _abox[1].x - _abox[0].x +1, _abox[1].y - _abox[0].y +1);
    if ( ! erase ) {
        MYUCG->setColor(COLOR_MGREY);
        IpsDisplay::drawPolygon(_arrow, 5);
        // MYUCG->setColor(_color.color[0], _color.color[1], _color.color[2]);
        // IpsDisplay::drawPolyFrame(_arrow, 5);
    }
    if ( _gauge._figure ) {
        _gauge._figure->draw();
    }
    MYUCG->finishBuffering();
}

void WindIndicator::redrawBG()
{
    MYUCG->setColor(COLOR_MGREY);
    IpsDisplay::drawPolygon(_arrow, 5);
    // MYUCG->setColor(_color.color[0], _color.color[1], _color.color[2]);
    // IpsDisplay::drawPolyFrame(_arrow, 5);
}
