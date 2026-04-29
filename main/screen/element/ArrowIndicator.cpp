/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "ArrowIndicator.h"

#include "PolarGauge.h"
#include "math/Trigonometry.h"
#include "Colors.h"
#include "math/Floats.h"
#include "logdefnone.h"
#include "IpsDisplay.h" //bbox

#include <cmath>

extern AdaptUGC *MYUCG;


////////////////////////////
// ArrowIndicator

ArrowIndicator::ArrowIndicator(PolarGauge &g, int16_t tipradius, int16_t length, int16_t half_base, int16_t headlen) :
    _gauge(g),
    _tip(tipradius),
    _arrowhead(headlen),
    _root((_gauge._flavor == PolarGauge::CLUB) ? tipradius - length : 
        cos(atan(static_cast<float>(half_base)/(_tip-length))) * (_tip-length)),
    _shoulder_val_offset(atan(static_cast<float>(half_base)/(_tip-length))*g.IDX_SCALE), // correct for cutting the radius from base points
    _half_width(half_base)
{
    color = { COLOR_RED };
    ESP_LOGI(FNAME,"Base  tl:%d off:%d tip:%d base:%d", _tip-length, _shoulder_val_offset, _tip, _root );
    prev.x_0 = _gauge.CosCenteredDeg2(0, _root); // root center
    prev.y_0 = _gauge.SinCenteredDeg2(0, _root);
    prev.x_1 = prev.x_0 - 0; // lower shoulder
    prev.y_1 = prev.y_0 + _half_width;
    prev.x_0 += 0; // upper shoulder
    prev.y_0 -= _half_width;
    prev.x_2 = _gauge.CosCenteredDeg2(0, _tip); // tip
    prev.y_2 = _gauge.SinCenteredDeg2(0, _tip);
    if ( _gauge._flavor == PolarGauge::CLUB) {
        prev.x_2 = _gauge.CosCenteredDeg2(0, _tip); // tip upper corner
        prev.y_2 = _gauge.SinCenteredDeg2(0, _tip);
        prev.x_3 = prev.x_2; // tip lower corner
        prev.y_3 = prev.y_2 + _half_width;
        prev.y_2 -= _half_width;
        prev.x_4 = _gauge.CosCenteredDeg2(0, _tip - _arrowhead); // tip shoulder top corner 
        prev.y_4 = _gauge.SinCenteredDeg2(0, _tip - _arrowhead);
        prev.x_5 = prev.x_4;
        prev.y_5 = prev.y_4 + _half_width;
        prev.y_4 -= _half_width;
        prev.x_6 = _gauge.CosCenteredDeg2(0, _tip); // tip centered
        prev.y_6 = _gauge.SinCenteredDeg2(0, _tip);
    }
}


// > 0,5° step counter
// return status on changed pointer position
bool ArrowIndicator::draw(int16_t val)
{
    ArrowPoints n;
    bool change = val != _needle_pos;
    if (!change && !_gauge._dirty)
    {
        return false; // nothing painted
    }

    // same about ~5000 cycles effort, but with tiny artifacts remaining
    // n.x_0 = _gauge.CosCenteredDeg2(val, _base); // base center
    // n.y_0 = _gauge.SinCenteredDeg2(val, _base);
    // int16_t bc = fast_cos_idx(val) * _half_width;
    // int16_t bs = fast_sin_idx(val) * _half_width;
    // n.x_1 = n.x_0 - bs; // lower shoulder
    // n.y_1 = n.y_0 + bc;
    // n.x_0 += bs; // top shoulder
    // n.y_0 -= bc;
    // n.x_2 = _gauge.CosCenteredDeg2(val, _tip); // tip
    // n.y_2 = _gauge.SinCenteredDeg2(val, _tip);
    n.x_0 = _gauge.CosCenteredDeg2(val + _shoulder_val_offset, _root); // top shoulder
    n.y_0 = _gauge.SinCenteredDeg2(val + _shoulder_val_offset, _root);
    n.x_1 = _gauge.CosCenteredDeg2(val - _shoulder_val_offset, _root); // lower shoulder
    n.y_1 = _gauge.SinCenteredDeg2(val - _shoulder_val_offset, _root);
    n.x_2 = _gauge.CosCenteredDeg2(val, _tip); // tip
    n.y_2 = _gauge.SinCenteredDeg2(val, _tip);
    // ESP_LOGI(FNAME,"draw Triangle  x0:%d y0:%d x1:%d y1:%d x2:%d y2:%d", n.x_0, n.y_0, n.x_1 ,n.y_1, n.x_2, n.y_2 );

    if (std::abs(val - _needle_pos) < _gauge.dice_rad(deg2rad(9.f)))
    {
        // new indicator position overlaps
        // Clear just a smal triangle
        int16_t x_2 = _gauge.CosCenteredDeg2(_needle_pos, _root + 7);
        int16_t y_2 = _gauge.SinCenteredDeg2(_needle_pos, _root + 7);
        if( change ){
            MYUCG->setColor(COLOR_MGREY);
            MYUCG->drawTriangle(prev.x_0, prev.y_0, prev.x_1, prev.y_1, x_2, y_2);
        }

        // draw pointer
        MYUCG->setColor(color.color[0], color.color[1], color.color[2]);
        MYUCG->drawTriangle(n.x_0, n.y_0, n.x_1, n.y_1, n.x_2, n.y_2);

        // cleanup respecting overlap
        if( change ){  // we need to cleanup only if position has changed, otherwise a redraw at same position is enough
            MYUCG->setColor(COLOR_MGREY);
            // clear area to the side
            if (val > _needle_pos)
            {
                // up
                MYUCG->drawTetragon(prev.x_2, prev.y_2, prev.x_1, prev.y_1, n.x_1, n.y_1, n.x_2, n.y_2);
            }
            else
            {
                MYUCG->drawTetragon(prev.x_2, prev.y_2, n.x_2, n.y_2, n.x_0, n.y_0, prev.x_0, prev.y_0);
            }
        }
    }
    else
    {
        if( change ){
            // cleanup previous incarnation
            MYUCG->setColor(COLOR_MGREY);
            MYUCG->drawTriangle(prev.x_0, prev.y_0, prev.x_1, prev.y_1, prev.x_2, prev.y_2);
            // draw pointer
            MYUCG->setColor(color.color[0], color.color[1], color.color[2]);
            MYUCG->drawTriangle(n.x_0, n.y_0, n.x_1, n.y_1, n.x_2, n.y_2);
        }
    }
    // ESP_LOGI(FNAME,"change=%d  prev=%d  now=%d", change, _needle_pos, val  );
    prev = n;
    _needle_pos = val;
    return change;
}

bool ArrowIndicator::drawOver(int16_t val,float a)
{
    if (val == _needle_pos && !_gauge._dirty) {
        return false; // nothing painted
    }
    
    ESP_LOGI(FNAME,"draw over val %d", val);
    float si = fast_sin_idx(val);
    float co = fast_cos_idx(val);
    ArrowPoints n {
        (int16_t)(_gauge._ref.x - fast_iroundf(co * _root - si * _half_width)),
        (int16_t)(_gauge._ref.y - fast_iroundf(si * _root + co * _half_width)),
        (int16_t)(_gauge._ref.x - fast_iroundf(co * _root + si * _half_width)),
        (int16_t)(_gauge._ref.y - fast_iroundf(si * _root - co * _half_width)),
        (int16_t)(_gauge._ref.x - fast_iroundf(co * _tip + si * _half_width)),
        (int16_t)(_gauge._ref.y - fast_iroundf(si * _tip - co * _half_width)),
        (int16_t)(_gauge._ref.x - fast_iroundf(co * _tip - si * _half_width)),
        (int16_t)(_gauge._ref.y - fast_iroundf(si * _tip + co * _half_width)),
        (int16_t)(_gauge._ref.x - fast_iroundf(co * (_tip - _arrowhead) + si * _half_width)),
        (int16_t)(_gauge._ref.y - fast_iroundf(si * (_tip - _arrowhead) - co * _half_width)),
        (int16_t)(_gauge._ref.x - fast_iroundf(co * (_tip - _arrowhead) - si * _half_width)),
        (int16_t)(_gauge._ref.y - fast_iroundf(si * (_tip - _arrowhead) + co * _half_width)),
        (int16_t)(_gauge._ref.x - fast_iroundf(co * _tip)),
        (int16_t)(_gauge._ref.y - fast_iroundf(si * _tip)),
    };
    if (_gauge._dirty) {
        // the moment when the old position is probably invalid, just draw the new position
        _last_a = a;
        prev = n;
    }
    ESP_LOGI(FNAME,"drawTetragon  x0:%d y0:%d x1:%d y1:%d x2:%d y2:%d x3:%d y3:%d", n.x_0, n.y_0, n.x_1, n.y_1, n.x_2, n.y_2, n.x_3, n.y_3 );
    BoundingBox pbox = {{prev.x_0, prev.y_0}, {prev.x_0, prev.y_0}};
    IpsDisplay::superBBox((Point*)(&prev.x_1), 6, pbox);
    BoundingBox box = {pbox[0], pbox[1]};
    IpsDisplay::superBBox((Point*)(&n.x_0), 7, box);
    // Enlarge box by one pixel in each direction
    box[0].x -= 1;
    box[0].y -= 1;
    box[1].x += 1;
    box[1].y += 1;
    // try first if the combined box fits into the frame buffer
    ESP_LOGI(FNAME, "draw over frame size %dbyte", (box[1].x - box[0].x +1) * (box[1].y - box[0].y +1) * 3 );
    bool extra_draw = false;
    if ( ((box[1].x - box[0].x +1) * (box[1].y - box[0].y +1)) > (EGLIB_FRAMEBUFFER_SIZE/3) ) {
        extra_draw = true;
        box[0].x = pbox[0].x - 1;
        box[0].y = pbox[0].y - 1;
        box[1].x = pbox[1].x + 1;
        box[1].y = pbox[1].y + 1;
        ESP_LOGI(FNAME, "draw over frame size %dbyte", (box[1].x - box[0].x +1) * (box[1].y - box[0].y +1) * 3 );
        if ( ((box[1].x - box[0].x +1) * (box[1].y - box[0].y +1)) > (EGLIB_FRAMEBUFFER_SIZE/3) ) {
            // too much for the 14k frame buffer, guess prev unset
            prev = n;
            return false;
        }
    }
    // ESP_LOGI(FNAME,"draw over  minx:%d maxx:%d miny:%d maxy:%d", box[0].x, box[1].x, box[0].y, box[1].y );
    // MYUCG->drawFrame(box[0].x-1, box[0].y-1, box[1].x - box[0].x + 2, box[1].y - box[0].y + 2);
    MYUCG->startBuffering(box[0].x, box[0].y, box[1].x - box[0].x +1, box[1].y - box[0].y +1);
    float diag = (float)(std::min(box[1].x - box[0].x +1, box[1].y - box[0].y +1)) / (_gauge.getDist05()); // rough estimate
    ESP_LOGI(FNAME,"draw over diag: %.2f dist05: %d", diag, _gauge.getDist05() );
    _gauge.drawScale(_last_a + diag, _last_a - diag); // redraw scale at last position to clean up artifacts
    MYUCG->setColor(color.color[0], color.color[1], color.color[2]);
    MYUCG->drawTetragon(n.x_0, n.y_0, n.x_1, n.y_1, n.x_4, n.y_4, n.x_5, n.y_5);
    if ( _arrowhead > 0 ) {
        MYUCG->drawTriangle(n.x_4, n.y_4, n.x_5, n.y_5, n.x_6, n.y_6);
    }
    // draw a black frame around the needle
    MYUCG->setColor(COLOR_BLACK);
    MYUCG->drawLine(n.x_0, n.y_0, n.x_1, n.y_1);
    MYUCG->drawLine(n.x_1, n.y_1, n.x_4, n.y_4);
    MYUCG->drawLine(n.x_4, n.y_4, n.x_6, n.y_6);
    MYUCG->drawLine(n.x_6, n.y_6, n.x_5, n.y_5);
    MYUCG->drawLine(n.x_5, n.y_5, n.x_0, n.y_0);
    MYUCG->finishBuffering();

    if ( extra_draw ) {
        // redraw arrow at new position again to have the parts that did not fit into the frame buffer
        MYUCG->setColor(color.color[0], color.color[1], color.color[2]);
        MYUCG->drawTetragon(n.x_0, n.y_0, n.x_1, n.y_1, n.x_4, n.y_4, n.x_5, n.y_5);
        if ( _arrowhead > 0 ) {
            MYUCG->drawTriangle(n.x_4, n.y_4, n.x_5, n.y_5, n.x_6, n.y_6);
        }
        // draw a black frame around the needle
        MYUCG->setColor(COLOR_BLACK);
        MYUCG->drawLine(n.x_0, n.y_0, n.x_1, n.y_1);
        MYUCG->drawLine(n.x_1, n.y_1, n.x_4, n.y_4);
        MYUCG->drawLine(n.x_4, n.y_4, n.x_6, n.y_6);
        MYUCG->drawLine(n.x_6, n.y_6, n.x_5, n.y_5);
        MYUCG->drawLine(n.x_5, n.y_5, n.x_0, n.y_0);
    }

    prev = n;
    _needle_pos = val;
    _last_a = a;
    return true;
}

void ArrowIndicator::testDraw(int16_t val)
{
    // static ArrowPoints p = {};
    // ESP_LOGI(FNAME,"testDraw val %d", val);
    // int16_t minx = p.x_0;
    // int16_t maxx = p.x_0;
    // for (int i = 1; i < 3; i++) {
    //     int16_t *x = (int16_t *)&p + i*2;
    //     if (*x < minx) {
    //         minx = *x;
    //     } else if (*x > maxx) {
    //         maxx = *x;
    //     }
    // }
    // int16_t miny = p.y_0;
    // int16_t maxy = p.y_0;
    // for (int i = 1; i < 3; i++) {
    //     int16_t *y = (int16_t *)&p + i*2 + 1;
    //     if (*y < miny) {
    //         miny = *y;
    //     } else if (*y > maxy) {
    //         maxy = *y;
    //     }
    // }
    // ArrowPoints n {
    //     _gauge.CosCenteredDeg2(val - 7, 25),
    //     _gauge.SinCenteredDeg2(val - 7, 25),
    //     _gauge.CosCenteredDeg2(val + 7, 25),
    //     _gauge.SinCenteredDeg2(val + 7, 25),
    //     _gauge._ref_x,
    //     _gauge._ref_y,
    //     0,0,0,0,0,0
    // };

    // if ( p.x_2 == 0 ) {
    //     p = n;
    //     return;
    // }
    // ESP_LOGI(FNAME,"testDraw  x0:%d y0:%d x1:%d y1:%d x2:%d y2:%d", n.x_0, n.y_0, n.x_1 ,n.y_1, n.x_2, n.y_2 );

    // MYUCG->setColor(COLOR_WHITE);
    // MYUCG->startBuffering(minx, miny, maxx - minx, maxy - miny);

    // MYUCG->drawTriangle(n.x_0, n.y_0, n.x_1, n.y_1, n.x_2, n.y_2);
    // MYUCG->finshBuffering();

    // p = n;

    // static bool toggle = true;
    // MYUCG->setFontPosCenter();
    // // MYUCG->setFontPosBottom();
    // // MYUCG->setFontPosTop();
    // MYUCG->setFont(ucg_font_fub20_hn, true);
    // int16_t x=50, y=60;
    // MYUCG->setPrintPos(x, y);
    // if ( toggle ) {
    //     MYUCG->setColor(COLOR_BLACK);
    //     MYUCG->clearScreen();
    //     MYUCG->setColor(COLOR_RED);
    //     MYUCG->drawLine(x-15, y, x+40, y);
    //     MYUCG->setColor(COLOR_BLUE);
    //     MYUCG->print(val);
    //     toggle = false;
    // }
    // else {
    //     MYUCG->setColor(COLOR_WHITE);
    //     MYUCG->drawFrame(x-1, y-21, 21, 31);
    //     // MYUCG->drawLine(x-1, y-20, x-1, y+10);
    //     // MYUCG->drawLine(x+20, y-20, x+20, y+10);
    //     // MYUCG->drawLine(x-15, y+10, x+15, y+10);
    //     MYUCG->startBuffering(x, y-20, 20, 30);
    //     MYUCG->print(val);
    //     MYUCG->setColor(COLOR_GREEN);
    //     MYUCG->drawLine(x-5, y, x+40, y);
    //     MYUCG->drawTriangle(x+5, y+5, x+5, y-5, x, y);
    //     //MYUCG->drawTetragon(x+16, y+1, x+50, y+1, x+50, y-1, x+16, y-1);
    //     MYUCG->drawTetragon(x, y-20, x, y-10, x+15, y+8, x+17, y+8);
    //     MYUCG->finshBuffering();
    //     toggle = true;
    // }
}