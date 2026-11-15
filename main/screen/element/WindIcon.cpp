/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "WindIcon.h"

#include "Colors.h"
#include "Atmosphere.h"
#include "AdaptUGC.h"
#include "sensor/gps/GpsVSensor.h"
#include "setup/SetupNG.h"
#include "logdefnone.h"

#include <cstdio>

extern AdaptUGC *MYUCG;

WindIcon::WindIcon(int16_t cx, int16_t cy, int16_t radius) :
    ScreenElement(cx, cy),
    _radius(radius)
{
    // template scale geometry to the radius
    float scale = (float)radius / 50.f;
    // The north wind arrow
    _arrow[0] = Point(0, 45 * scale); // Tipp
    _arrow[1] = Point(0, -10 * scale); // Fin center
    _arrow[2] = Point(-30 * scale, -35 * scale); // Fin left
    _arrow[3] = Point(+30 * scale, -35 * scale); // Fin right
}

// direction [°], northwind as 0°; strength [any]
// wval < 0 just removes the
void WindIcon::draw(WindData w)
{
    bool headref = wind_reference.get() == static_cast<int>(WindReference::WR_HEADING);
    int wdir_deg2 = w.getDeg2(); // new wind direction in 0..719°
    if ( headref && heading_tru.getValid() ) {
        rad_t heading = heading_tru.get();
        ESP_LOGI(FNAME, "heading %.1f", Units::rad_to_deg(heading));
        wdir_deg2 = w.relToHeading(heading);
    }
    bool dir_changed = wdir_deg2 != _wind.getDeg2();
    bool changed = w != _wind;

    if ( !changed && !dir_changed && ! _dirty ) {
        return;
    }

    _wind = w;

    if (dir_changed || _dirty) {
        drawIcon(_wind.isValid() ? wdir_deg2 : 360); // if invalid, point north
        if ( headref ) {
            // direction on top
            drawDirection();
        }
    }

    if (changed || _dirty) {
        // put the wind strength behind
        int wstrength = fast_iroundf(SpeedUnit->apply(w.getVal()));
        ESP_LOGI(FNAME, "Wind (%d,%d)", w.getDeg(), wstrength);
        MYUCG->setColor(COLOR_WGREY);
        MYUCG->setFont(ucg_font_fub20_hn, true);
        char s[16] = {"--"};
        if (_wind.isValid()) {
            snprintf(s, sizeof(s), "%2d", wstrength);
        }
        // MYUCG->drawFrame(_ref.x, _ref.y - 26, 40, 26);
        MYUCG->startBuffering(_ref.x, _ref.y - 26, 40, 26); // avoid artefacts when s shrinks
        MYUCG->setPrintPos(_ref.x, _ref.y + 3);
        MYUCG->print(s);
        MYUCG->finishBuffering();
        int16_t str_width = MYUCG->getStrWidth(s);
        MYUCG->setPrintPos(_ref.x + str_width, _ref.y);
        // MYUCG->drawFrame(_ref.x + str_width, _ref.y - 16, 55, 16);
        MYUCG->startBuffering(_ref.x + str_width, _ref.y - 16, 55, 16); // avoid artefacts when s shrinks
        MYUCG->setFont(ucg_font_fub11_hr);
        MYUCG->setColor( COLOR_HEADER );
        MYUCG->print(SpeedUnit->getName());
        MYUCG->finishBuffering();
        if (_dirty) drawStatic();
    }
    _dirty = false;
}

// 0° reference on top of the icon
void WindIcon::drawIcon(int16_t deg2) const
{
    Point center = Point(_ref.x - _radius - 3, _ref.y - _radius);
    MYUCG->setColor(COLOR_MARINE);

    MYUCG->startBuffering(center.x - _radius, center.y - _radius, 2*_radius +1, 2*_radius +1);
    MYUCG->drawDisc(center.x, center.y, _radius, UCG_DRAW_ALL);

    // an arrow tip in direction the wind is blowing (180° other direction)
    Point tmp[4];
    Point::rotate(_arrow, 4, deg2, tmp);
    // shift to gauge center
    for (int i = 0; i < 4; i++) {
        tmp[i] += center;
    }
    MYUCG->setColor(COLOR_WHITE);
    MYUCG->drawTriangle(tmp[0].x, tmp[0].y, tmp[1].x, tmp[1].y, tmp[2].x, tmp[2].y);
    MYUCG->drawTriangle(tmp[0].x, tmp[0].y, tmp[3].x, tmp[3].y, tmp[1].x, tmp[1].y);
    MYUCG->finishBuffering();
}

void WindIcon::drawStatic() const
{
    if ( wind_reference.get() == static_cast<int>(WindReference::WR_NORTH)) {
        MYUCG->setFont(ucg_font_fub11_hr);
        MYUCG->setColor( COLOR_HEADER );
        MYUCG->setPrintPos(_ref.x - _radius - MYUCG->getCharWidth('N')/2 - 2, _ref.y - 2 * _radius - 1);
        MYUCG->print("N");
    }
}

void WindIcon::drawDirection() const
{
    MYUCG->setFont(ucg_font_fub11_hr, true);
    MYUCG->setColor(COLOR_WGREY);
    char s[16] = {"  ---  "};
    if ( _wind.isValid() ) {
       sprintf(s, " %03d' ", _wind.getDeg());
    }
    MYUCG->setPrintPos(_ref.x - _radius - MYUCG->getStrWidth(s)/2 - 2, _ref.y - 2 * _radius - 1);
    MYUCG->print(s);
}