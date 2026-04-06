/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "LargeFigure.h"

#include "Colors.h"
#include "AdaptUGC.h"
#include "math/Floats.h"

#include <cmath>
#include <cstdio>

extern AdaptUGC *MYUCG;


////////////////////////////
// LargeFigure

LargeFigure::LargeFigure(int16_t x, int16_t y) :
    ScreenElement(x, y)
{
    MYUCG->setFont(ucg_font_fub35_hn, false);
}

void LargeFigure::draw(float val) {
    int16_t ival = fast_iroundf(val * 10); // integer value in steps of 10th

    // only print if there is a change in rounded numeric string
    if (_value != ival || _dirty) {
        char s[32];
        MYUCG->setFont(ucg_font_fub35_hn);
        MYUCG->setFontPosCenter();
        MYUCG->setColor(COLOR_WHITE);
        if ( std::abs(ival) < 100 ) {
            sprintf(s, "% 2.1f", float(ival) / 10.0f);
        }
        else if ( std::abs(ival) < 1000 ) {
            sprintf(s, "% d", ival / 10);
        }
        else {
            sprintf(s, "oo");
        }
        int16_t tmp = MYUCG->getStrWidth(s+1)/2;
        int16_t signwidth = MYUCG->getCharWidth(s[0]);
        MYUCG->setPrintPos(_ref_x - tmp + 2-signwidth, _ref_y + 2);
        MYUCG->startBuffering(_ref_x - tmp + 2-10, _ref_y + 2 - 54/2, 2*tmp + 16, 54);
        MYUCG->print(s);
        _value = ival;
        MYUCG->finishBuffering();
        MYUCG->setFontPosBottom();
    }
    _dirty = false;
}
