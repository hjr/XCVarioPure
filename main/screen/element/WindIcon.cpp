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
#include "setup/SetupNG.h"
#include "logdef.h"

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
bool WindIcon::draw(WindData w)
{
    ESP_LOGI(FNAME, "Wind (%d,%.2f)", w.getDeg(), w.getVal());
    if (w != _wind || _dirty) {

        if ( wind_reference.get() == static_cast<int>(WindReference::WR_HEADING) ) {
            rad_t heading = getHeading();
            ESP_LOGI(FNAME, "heading %.1f", Units::rad_to_deg(heading));
            w.inclHeading(heading);
        }

        _wind = w;
        drawIcon();
        // put the wind strength behind
        MYUCG->setColor(COLOR_WGREY);
        MYUCG->setFont(ucg_font_fub20_hn, true);
        MYUCG->setPrintPos(_ref.x + 2 * _radius + 3, _ref.y);
        if (w.getVal() >= 0) {
            char s[16];
            sprintf(s, "%3d", (int)w.getVal());
            MYUCG->print(s);
        } else {
            MYUCG->print("---");
        }
        if ( _dirty ) {
            drawUnit();
        }
        _dirty = false;
        return true;
    }
    return false;
}

// 0° reference on top of the icon
void WindIcon::drawIcon() const
{
    if ( ! _wind.isValid() ) {
        return;
    }
    Point center = Point(_ref.x + _radius, _ref.y - _radius);
    // MYUCG->setColor(COLOR_MARINE);
    MYUCG->setColor(COLOR_FIGURE);
    MYUCG->drawDisc(center.x, center.y, _radius, UCG_DRAW_ALL);
    if ( ! _wind.isValid() || _wind.getVal() <= 0 ) {
        return;
    }

    // an arrow tip in direction the wind is blowing (180° other direction)
    Point tmp[4];
    Point::rotate(_arrow, 4, _wind.getDeg2(), tmp);
    // shift to gauge center
    for (int i = 0; i < 4; i++) {
        tmp[i] += center;
    }
    MYUCG->setColor(COLOR_WHITE);
    MYUCG->drawTriangle(tmp[0].x, tmp[0].y, tmp[1].x, tmp[1].y, tmp[2].x, tmp[2].y);
    MYUCG->drawTriangle(tmp[0].x, tmp[0].y, tmp[3].x, tmp[3].y, tmp[1].x, tmp[1].y);
}

void WindIcon::drawUnit() const
{
    MYUCG->setFont(ucg_font_fub11_hr);
    MYUCG->setColor( COLOR_HEADER );
    MYUCG->print(SpeedUnit->getName());
    if ( wind_reference.get() == static_cast<int>(WindReference::WR_NORTH)) {
        MYUCG->setPrintPos(_ref.x + _radius - 3, _ref.y - 2 * _radius - 2);
        MYUCG->print("N");
    }
}