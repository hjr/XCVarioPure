/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "HorizonPage.h"


#include "math/Units.h"
#include "math/Trigonometry.h"
#include "math/Floats.h"

#include "setup/SetupNG.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "logdef.h"


#include <cstdint>
#include <cstdio>
#include <cstring>

extern AdaptUGC *MYUCG;

static int heading_old = -1;
bool BasePage::_DIRTY = true;
HorizonPage *HorizonPage::instance = nullptr;

HorizonPage* HorizonPage::HORIZON()
{
    static HorizonPage* instance = nullptr;
    if ( ! instance ) {
        instance = new HorizonPage();
    }
    return instance;
}

HorizonPage::HorizonPage()
{
    int16_t left = (DISPLAY_W-BOX_SIZE) / 2;
    int16_t top = (DISPLAY_H-BOX_SIZE) / 2;
    // ( 20, 60, 200, 200 );
    horizon_box[0] = {left, (int16_t)(top+BOX_SIZE)};
    horizon_box[1] = {(int16_t)(left+BOX_SIZE), (int16_t)(top+BOX_SIZE)};
    horizon_box[2] = {(int16_t)(left+BOX_SIZE), top};
    horizon_box[3] = {left, top};
    _DIRTY = true;
}

void HorizonPage::draw( Quaternion q )
{
    if ( _DIRTY ) {
        Display->clear();

        int16_t left = (DISPLAY_W-BOX_SIZE) / 2;
        int16_t top = (DISPLAY_H-BOX_SIZE) / 2;
        // int16_t center_x = left + BOX_SIZE / 2;
        int16_t center_y = top + BOX_SIZE / 2;
        MYUCG->setColor( COLOR_WHITE );
        MYUCG->drawTriangle(left - 19, center_y - 10, left, center_y, left - 19, center_y + 10); // Triangles l/r
        MYUCG->drawTriangle(left + 200 + 20, center_y - 10, left + 200, center_y, left + 200 + 20, center_y + 10);
        for (int i = -80; i <= 80; i += 20) { // 10° scale
            MYUCG->drawHLine(left - 19, center_y + i, 20);
            MYUCG->drawHLine(left + 200, center_y + i, 20);
        }
        for (int i = -70; i <= 70; i += 20) { // 5° scale
            MYUCG->drawHLine(left - 10, center_y + i, 10);
            MYUCG->drawHLine(left + 200, center_y + i, 10);
        }
    }
    // draw sky and earth
    Line l( q, DISPLAY_W/2, DISPLAY_H/2 );
    if ( ! l.similar(previous_horizon_line) || _DIRTY ) {
        previous_horizon_line = l;
        Point above[6], below[6];
        int na, nb;
        Display->clipRectByLine(horizon_box, l, above, &na, below, &nb);
        MYUCG->setColor( COLOR_SKYBLUE );
        Display->drawPolygon(above, na);
        MYUCG->setColor( COLOR_EARTH );
        Display->drawPolygon(below, nb);
    }

    // heading
    if (mag_hdt.getValid()) {
        int heading = fast_iroundf(Units::rad_to_deg(mag_hdt.get()));
        MYUCG->setFont(ucg_font_fub20_hn, true);
        MYUCG->setPrintPos(70, 310);
        // ESP_LOGI(FNAME,"compass enable, heading: %d", heading );
        if (heading > 0 && heading != heading_old) {
            MYUCG->setColor(COLOR_WHITE);
            MYUCG->printf("   %d°   ", heading);
            heading_old = heading;
        }
    }

    _DIRTY = false;
}

