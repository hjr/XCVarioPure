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
#include "CenterAid.h"
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
    ESP_LOGI(FNAME, "wind deg:%d, v:%.1f ", _wind.getDeg(), _wind.getVal());
    float si = -fast_sin_idx(_wind.getDeg2());
    float co = fast_cos_idx(_wind.getDeg2());

    int16_t x0 = si * (_gauge._radius-6);
    int16_t y0 = co * (_gauge._radius-6);
    int16_t x1 = x0 / 4;
    int16_t y1 = y0 / 4;
    if (erase) {
        MYUCG->setColor(COLOR_BLACK);
    } else {
        MYUCG->setColor(_color.color[0], _color.color[1], _color.color[2]);
    }
    // a number at where the wind is coming from
    int16_t value = _wind.getVal() * _gauge._unit_fac;
    int16_t sw = _cwidth * count_digits(value);
    int16_t xshift = sw / 2;
    if (erase) {
        MYUCG->drawBox(_gauge._ref.x - x0 - x1 / 2 - xshift, _gauge._ref.y - y0 - y1 / 2 - _cheight / 2, sw, _cheight);
    } else {
        char buf[8];
        if (_wind.isValid()) {
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

    // a tip in direction the wind is blowing (180° other direction)
    x0 += _gauge._ref.x;
    y0 += _gauge._ref.y;
    int16_t wx = fast_iroundf(si);
    int16_t wy = fast_iroundf(co);
    if (_live) {
        int16_t bx = si * 4;
        int16_t by = co * 4;
        // ">" like arrow tip
        MYUCG->drawLine(x0 + by, y0 - bx, x0 + x1, y0 + y1); // +(by,-bx)
        MYUCG->drawLine(x0 + by + wy, y0 - bx - wx, x0 + x1 + wy, y0 + y1 - wx);
        MYUCG->drawLine(x0 - by, y0 + bx, x0 + x1, y0 + y1); // +(-by,bx)
        MYUCG->drawLine(x0 - by - wy, y0 + bx + wx, x0 + x1 - wy, y0 + y1 + wx);
    } else {
        // "-" like arrow tip
        MYUCG->drawTetragon(x0 + wy, y0 - wx,
            x0 + x1 + wy, y0 + y1 - wx, // +(wy,-wx)
            x0 + x1 - wy, y0 + y1 + wx, // +(-wy,wx)
            x0 - wy, y0 + wx);
        // ESP_LOGI(FNAME, "x0, y0 (%d,%d)", x0, y0);
    }
}

void WindIndicator::redrawBG()
{
    MYUCG->setColor(COLOR_MGREY);
    IpsDisplay::drawPolygon(_arrow, 5);
    // MYUCG->setColor(_color.color[0], _color.color[1], _color.color[2]);
    // IpsDisplay::drawPolyFrame(_arrow, 5);
}
