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

#include "sensor/imu/AccMPU6050.h"
#include "setup/SetupNG.h"
#include "AdaptUGC.h"
#include "Colors.h"
#include "logdef.h"


#include <cstdint>


// #define HORIZON_TEST 1

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
        previous_horizon_line = Line();
    }
    // draw sky and earth and panel
    Line l(q);
    if ( ! l.similar(previous_horizon_line) || _DIRTY ) {
        Point above[6], below[6], opta[6], optb[6];
        int na, nb,
            oa, ob;
        Display->clipPolygonByLine(horizon_box, 4, l, above, &na, below, &nb);
        // ESP_LOGI(FNAME, "nA/nB %d,%d", na, nb);
        // int i = 0;
        // for ( ; i < std::min(na,nb); i++ ) {
        //     ESP_LOGI(FNAME, "A/B %d: %d,%d - %d,%d", i, above[i].x, above[i].y, below[i].x, below[i].y);
        // }
        // if ( i < na ) {
        //     ESP_LOGI(FNAME, "A extra %d: %d,%d", i, above[i].x, above[i].y);
        // }
        // if ( i < nb ) {
        //     ESP_LOGI(FNAME, "B extra %d: %d,%d", i, below[i].x, below[i].y);
        // }
        if ( previous_horizon_line.isDefined() ) {
            // also clip the previous horizon line for a minimal transition
            Display->clipPolygonByLine(above, na, previous_horizon_line, optb, &ob, opta, &oa);
            MYUCG->setColor( COLOR_SKYBLUE );
            Display->drawPolygon(opta, oa);
            Display->clipPolygonByLine(below, nb, previous_horizon_line, optb, &ob, opta, &oa);
            MYUCG->setColor( COLOR_EARTH );
            Display->drawPolygon(optb, ob);
        }
        else {
            ESP_LOGI(FNAME, "First draw full horizon");
            MYUCG->setColor( COLOR_SKYBLUE );
            Display->drawPolygon(above, na);
            MYUCG->setColor( COLOR_EARTH );
            Display->drawPolygon(below, nb);
        }

        MYUCG->setColor( COLOR_YELLOW );
        MYUCG->drawDisc(DISPLAY_W/2, DISPLAY_H/2, 2, UCG_DRAW_ALL);
        MYUCG->drawHLine(DISPLAY_W/2 + 15, DISPLAY_H/2, 30);
        MYUCG->drawVLine(DISPLAY_W/2 + 15, DISPLAY_H/2, 5);
        MYUCG->drawHLine(DISPLAY_W/2 - 15, DISPLAY_H/2, -30);
        MYUCG->drawVLine(DISPLAY_W/2 - 15, DISPLAY_H/2, 5);
        MYUCG->setColor(COLOR_WHITE);
        MYUCG->setFont(ucg_font_fub20_hn, true);
        char buf[20];
        sprintf(buf, " %d° ", fast_iroundf(accSensor->getRollDeg()));
        MYUCG->setPrintPos(DISPLAY_W/2 - MYUCG->getStrWidth(buf)/2, DISPLAY_H/2 - BOX_SIZE/2 - 10);
        MYUCG->print(buf);
        previous_horizon_line = l;
    }

    // heading
    if (mag_hdt.getValid()) {
        int heading = fast_iroundf(Units::rad_to_deg(mag_hdt.get()));
        MYUCG->setFont(ucg_font_fub20_hn, true);
        MYUCG->setPrintPos(DISPLAY_W/2 - 50, DISPLAY_H/2 + BOX_SIZE + 50);
        // ESP_LOGI(FNAME,"compass enable, heading: %d", heading );
        if (heading > 0 && heading != heading_old) {
            MYUCG->setColor(COLOR_WHITE);
            MYUCG->printf("   %d°   ", heading);
            heading_old = heading;
        }
    }

    _DIRTY = false;

#ifdef HORIZON_TEST
    // axes for testing
    vector_f north = {500,0,0};
    vector_f east  = {500,20,0};
    vector_f down  = {500,0,20};

    Point pe = Point::centralProjection(east, 1000.f);
    pe = l.mapToHorizon(pe);
    Point pn = Point::centralProjection(north, 1000.f);
    pn = l.mapToHorizon(pn);
    Point pd = Point::centralProjection(down, 1000.f);
    pd = l.mapToHorizon(pd);

    // ESP_LOGI(FNAME, "North %d,%d", pn.x, pn.y);
    // ESP_LOGI(FNAME, "East %d,%d", pe.x, pe.y);
    // ESP_LOGI(FNAME, "Down %d,%d", pd.x, pd.y);

    MYUCG->setPrintPos(pn.x, pn.y);
    MYUCG->setColor(COLOR_RED);
    MYUCG->print("N");
    MYUCG->setPrintPos(pe.x, pe.y);
    MYUCG->setColor(COLOR_GREEN);
    MYUCG->print("E");
    MYUCG->setPrintPos(pd.x, pd.y);
    MYUCG->setColor(COLOR_BLUE);
    MYUCG->print("D");

    // target point test on horizon line
    vector_f test_nav = {1000,0,0};
    Point p = Point::centralProjection(test_nav, 1000.f);
    p = l.mapToHorizon(p);
    MYUCG->setColor(COLOR_MGREY);
    MYUCG->drawDisc( p.x, p.y, 7, UCG_DRAW_ALL);

    // above / below test
    vector_f above = {1000,0,-50};
    p = Point::centralProjection(above, 1000.f);
    p = l.mapToHorizon(p);
    MYUCG->setColor(COLOR_RED);
    MYUCG->drawDisc( p.x, p.y, 7, UCG_DRAW_ALL);
    vector_f below = {1000,0,50};
    p = Point::centralProjection(below, 1000.f);
    p = l.mapToHorizon(p);
    MYUCG->setColor(COLOR_BLUE);
    MYUCG->drawDisc( p.x, p.y, 7, UCG_DRAW_ALL);
#endif
}

