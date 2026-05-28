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
#include "sensor/gps/GpsVSensor.h"
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
        // --- REMOVED: side triangles + side scales ---
        previous_horizon_line = Line();
    }

    // draw sky and earth and panel
    Line l(q);
    if ( ! l.similar(previous_horizon_line) || _DIRTY ) {
        Point above[6], below[6], opta[6], optb[6];
        int na, nb, oa, ob;

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
            Line shifted = previous_horizon_line;
            shifted._d += 1.0f;   // 1 Pixel down 

            Display->clipPolygonByLine(above, na, shifted, optb, &ob, opta, &oa);
            MYUCG->setColor( COLOR_SKYBLUE );
            Display->drawPolygon(opta, oa);

            shifted._d -= 2.0f;   // 1-2 = -1 so 1 Pixel up 

            Display->clipPolygonByLine(below, nb, shifted, optb, &ob, opta, &oa);
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

        // --- Aircraft symbol ---
        const int cx = DISPLAY_W / 2;
        const int cy = DISPLAY_H / 2;
        MYUCG->setColor(COLOR_WHITE);
        // center ring
        MYUCG->drawCircle(cx, cy, 5, UCG_DRAW_ALL);
        MYUCG->drawCircle(cx, cy, 6, UCG_DRAW_ALL);

        // small top post
        MYUCG->drawVLine(cx, cy - 10, 5);

        // thicker wings (+20% span, shifted 5px outward)
        MYUCG->drawHLine(cx + 13, cy,     32);
        MYUCG->drawHLine(cx + 13, cy + 1, 32);

        MYUCG->drawHLine(cx - 45, cy,     32);
        MYUCG->drawHLine(cx - 45, cy + 1, 32);

        // vertical hooks near fuselage moved outward too
        MYUCG->drawVLine(cx + 13, cy, 4);
        MYUCG->drawVLine(cx - 13, cy, 4);

        // wing tips upward
        MYUCG->drawLine(cx + 45, cy, cx + 51, cy - 4);
        MYUCG->drawLine(cx - 45, cy, cx - 51, cy - 4);


        int full_len = 30;
        int short_len = (int)(full_len * 0.3f);
        float px_per_5deg = 12.0f;

        // text config
        MYUCG->setFont(ucg_font_fub11_hr, false );   // small, readable
        int gap = 8;                        // space between line and number

        for (int deg = -35; deg <= 35; deg += 5) {

            if (deg == 0) continue;
            if (deg == 5)
                MYUCG->setColor(COLOR_WHITE);
            else
                MYUCG->setColor(COLOR_POINTER);

            int y = cy - (int)((deg / 5.0f) * px_per_5deg);
            bool isLong = (deg % 10 == 0);
            int len = isLong ? full_len : short_len;

            // draw tick
            MYUCG->drawHLine(cx - len/2, y, len);

            // numbers only for 10/20/30
            if (isLong) {
                int absDeg = abs(deg);
                if (absDeg == 20) {
                    char buf[4];
                    sprintf(buf, "%d", absDeg);

                    int str_w = MYUCG->getStrWidth(buf);

                    int ascent  = MYUCG->getFontAscent();
                    int descent = MYUCG->getFontDescent();
                    // center text vertically on the tick line
                    int text_y = y + (ascent - descent) / 2;

                    // LEFT number
                    int tx = cx - len/2 - gap - str_w;
                    int center_x = tx + str_w / 2;
                    int center_y = text_y - (ascent - descent) / 2;
                    float side = l.fct(Point(center_x, center_y));

                    // choose background color
                    if(side > 0) {
                        MYUCG->setColor(1, COLOR_SKYBLUE); // bg color sky
                    } else {
                        MYUCG->setColor(1, COLOR_EARTH);   // bg color earth
                    }

                    MYUCG->setPrintPos(cx - len/2 - gap - str_w, text_y);
                    MYUCG->print(buf);

                    // RIGHT number
                    tx = cx + len/2 + gap;
                    center_x = tx + str_w / 2;
                    center_y = text_y - (ascent - descent) / 2;
                    side = l.fct(Point(center_x, center_y));

                    // choose background color
                    if(side > 0) {
                        MYUCG->setColor(1, COLOR_SKYBLUE); // bg color sky
                    } else {
                        MYUCG->setColor(1, COLOR_EARTH);   // bg color earth
                    }

                    MYUCG->setPrintPos(cx + len/2 + gap, text_y);
                    MYUCG->print(buf);
                }
            }
        }

        // --- bank label ---
        MYUCG->setColor(1, COLOR_BLACK); // bg color
        MYUCG->setFont(ucg_font_fub14_hn, true);
        MYUCG->setColor(COLOR_HEADER_LIGHT);
        int baseY = DISPLAY_H / 2 - BOX_SIZE / 2 - 10;
        MYUCG->setPrintPos((DISPLAY_W-BOX_SIZE)/2, baseY-4);
        MYUCG->print("BANK");
        char buf[20];
        // --- bank value
        int roll = fast_iroundf(accSensor->getRollDeg());
        MYUCG->setFont(ucg_font_fub20_hn, true);
        snprintf(buf, sizeof(buf), " % 4d°   ", roll);
        int strWidth = MYUCG->getStrWidth(buf);
        MYUCG->setColor(COLOR_WHITE);
        MYUCG->setPrintPos(DISPLAY_W / 2 - strWidth / 2 +5, baseY);
        MYUCG->print(buf);

        previous_horizon_line = l;
    }

    // --- Magnetic Heading (HDG) / Track (TRK) ---
    static bool was_valid = false;
    bool valid = heading_tru.getValid() && GpsVSensor::getValid();
    if (valid) {
        int heading;
        bool isMag = heading_tru.getValid();
        if (isMag) {
            heading = fast_iroundf(Units::rad_to_deg(heading_tru.get()));
        } else {
            heading = fast_iroundf(Units::rad_to_deg(gnd_course.get()));
        }
        if (heading >= 0 && (heading != heading_old || !was_valid)) {
            int baseY = DISPLAY_H / 2 + BOX_SIZE / 2 + 25;
            MYUCG->setColor(1, COLOR_BLACK);
            // hdg/trk label
            MYUCG->setFont(ucg_font_fub14_hn, true);
            MYUCG->setColor(COLOR_HEADER_LIGHT);
            MYUCG->setPrintPos((DISPLAY_W - BOX_SIZE) / 2, baseY + 3);
            MYUCG->print(isMag ? "HDG" : "TRK");
            // hdg/trk value
            MYUCG->setFont(ucg_font_fub20_hn, true);
            MYUCG->setColor(COLOR_WHITE);
            char buf[20];
            snprintf(buf, sizeof(buf), " % 4d°", heading);
            int strWidth = MYUCG->getStrWidth(buf);
            MYUCG->setPrintPos((DISPLAY_W / 2) - strWidth / 2 +5, baseY + 7);
            MYUCG->print(buf);
            heading_old = heading;
        }

    } else {
        // clear display when signal lost ---
        if (was_valid) {
            int baseY = DISPLAY_H / 2 + BOX_SIZE / 2;
            MYUCG->setColor(COLOR_BLACK);
            MYUCG->drawBox((DISPLAY_W - BOX_SIZE) / 2, baseY, BOX_SIZE, 40);
            heading_old = -1;   // force redraw when signal returns
        }
    }
    was_valid = valid;

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
