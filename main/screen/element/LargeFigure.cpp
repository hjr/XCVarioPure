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
#include "Atmosphere.h"
#include "math/Floats.h"
#include "setup/CruiseMode.h"
#include "logdef.h"

#include <cmath>
#include <cstdio>

extern AdaptUGC *MYUCG;


////////////////////////////
// LargeFigure

LargeFigure::LargeFigure(int16_t x, int16_t y) :
    ScreenElement(x, y)
{
    constexpr int16_t tmp = 33;
    _bbox[0] = Point(_ref.x - tmp + 3-10, _ref.y + 2 - 36/2);
    _bbox[1] = Point( 2*tmp + 7, 36);
}

bool LargeFigure::changed(float v)
{
    int16_t ival = fast_iroundf(v * 10); // integer value in steps of 10th
    if (_value != ival || _dirty) {
        return true;
    }
    return false;
}

void LargeFigure::draw() {
    char s[32];
    MYUCG->setFont(ucg_font_fub35_hn);
    MYUCG->setFontPosCenter();
    MYUCG->setColor(COLOR_WHITE);
    if ( std::abs(_value) < 100 ) {
        sprintf(s, "% 2.1f", float(_value) / 10.0f);
    }
    else if ( std::abs(_value) < 1000 ) {
        sprintf(s, "% d", _value / 10);
    }
    else {
        sprintf(s, " oo");
    }
    int16_t tmp = MYUCG->getStrWidth(s+1)/2;
    int16_t signwidth = MYUCG->getCharWidth(s[0]);
    MYUCG->setPrintPos(_ref.x - tmp + 2-signwidth, _ref.y + 2);
    MYUCG->startBuffering(_ref.x - tmp + 3-10, _ref.y + 2 - 36/2, 2*tmp + 7, 36);
    MYUCG->print(s);
    MYUCG->finishBuffering();

    // MYUCG->drawFrame(_ref.x - tmp + 3-10, _ref.y + 2 - 36/2, 2*tmp + 7, 36);
    // MYUCG->drawFrame(_ref.x - tmp + 3-signwidth, _ref.y + 2 - fh/2, 
    //     2 * tmp + signwidth, fh);
    MYUCG->setFontPosBottom();
    // ESP_LOGI(FNAME, "draw large figure w%d, a%d,d%d", tmp, MYUCG->getFontAscent(), MYUCG->getFontDescent());
}

void LargeFigure::drawStatic() {
    MYUCG->setFont(ucg_font_fub11_hr, true);
    MYUCG->setColor(COLOR_HEADER);
    const char *mode = "average";
    if ( CRMOD.getVMode() == CruiseMode::MODE_REL_NETTO) {
        mode = " s-netto ";
    }
    else if ( CRMOD.getVMode() == CruiseMode::MODE_NORMAL_NETTO) {
        mode = "  netto  ";
    }
    int16_t str_width = MYUCG->getStrWidth(mode);
    MYUCG->setPrintPos(_ref.x - str_width / 2, _bbox[0].y - 3);
    MYUCG->print(mode);

    str_width = MYUCG->getStrWidth(VarioUnit->getName());
    MYUCG->setPrintPos(_ref.x - str_width / 2, _bbox[0].y + _bbox[1].y + MYUCG->getFontLineSpace());
    MYUCG->setColor(COLOR_HEADER);
    MYUCG->print(VarioUnit->getName());
}

void LargeFigure::draw(float val) {
    int16_t ival = fast_iroundf(val * 10); // integer value in steps of 10th

    // only print if there is a change
    if (_value != ival || _dirty) {
        _value = ival;
        draw();
        if ( _dirty ) {
            drawStatic();
        }
    }
    _dirty = false;
}
