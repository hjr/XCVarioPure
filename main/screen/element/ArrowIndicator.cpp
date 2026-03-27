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

#include <cmath>

extern AdaptUGC *MYUCG;


////////////////////////////
// ArrowIndicator

ArrowIndicator::ArrowIndicator(PolarGauge &g, int16_t tipradius, int16_t length, int16_t half_base) :
    _gauge(g),
    _tip(tipradius),
        // correct for cutting the radius from base points
    _root(cos(atan(static_cast<float>(half_base)/(_tip-length))) * (_tip-length)),
    _shoulder_val_offset(atan(static_cast<float>(half_base)/(_tip-length))*g.IDX_SCALE),
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
        prev.x_2 = prev.x_2; // tip lower corner
        prev.y_2 = prev.y_2 + _half_width;
        prev.y_2 -= _half_width;
        _root = tipradius - length;
    }
}


// > 0,5° step counter
// return status on changed pointer position
bool ArrowIndicator::draw(int16_t val)
{
    Triangle_t n;
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
    int16_t l1 = _root;
    int16_t w = _half_width;
    int16_t l2 = _tip;
    Triangle_t n {
        (int16_t)(_gauge._ref_x - fast_iroundf(co * l1 - si * w)),
        (int16_t)(_gauge._ref_y - fast_iroundf(si * l1 + co * w)),
        (int16_t)(_gauge._ref_x - fast_iroundf(co * l1 + si * w)),
        (int16_t)(_gauge._ref_y - fast_iroundf(si * l1 - co * w)),
        (int16_t)(_gauge._ref_x - fast_iroundf(co * l2 + si * w)),
        (int16_t)(_gauge._ref_y - fast_iroundf(si * l2 - co * w)),
        (int16_t)(_gauge._ref_x - fast_iroundf(co * l2 - si * w)),
        (int16_t)(_gauge._ref_y - fast_iroundf(si * l2 + co * w)),
    };
    ESP_LOGI(FNAME,"drawTetragon  x0:%d y0:%d x1:%d y1:%d x2:%d y2:%d x3:%d y3:%d", n.x_0, n.y_0, n.x_1, n.y_1, n.x_2, n.y_2, n.x_3, n.y_3 );
    MYUCG->setColor(COLOR_BLACK);
    MYUCG->drawTetragon(prev.x_0, prev.y_0, prev.x_1, prev.y_1, prev.x_2, prev.y_2, prev.x_3, prev.y_3);
    _gauge.drawScale(_last_a, _last_a); // redraw scale at last position to clean up artifacts
    MYUCG->setColor(color.color[0], color.color[1], color.color[2]);
    MYUCG->drawTetragon(n.x_0, n.y_0, n.x_1, n.y_1, n.x_2, n.y_2, n.x_3, n.y_3);
    prev = n;
    _needle_pos = val;
    _last_a = a;
    return true;
}