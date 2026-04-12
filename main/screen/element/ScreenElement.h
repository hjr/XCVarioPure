/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "IpsDisplay.h"
#include "AdaptUGC.h"

#include <cstdint>

extern const ucg_color_t ndl_color[3];

class ScreenElement
{
public:
    ScreenElement() = delete;
    constexpr ScreenElement(int16_t refx, int16_t refy) : _ref(refx, refy) {}
    virtual ~ScreenElement() = default;
    void setRef(int16_t x, int16_t y) { _ref.x=x; _ref.y=y; }
    const BoundingBox& getBoundingBox() const { return _bbox; }
    void forceRedraw() { _dirty = true; }
    // virtual void update(const UiEvent&) {}
    virtual void draw() {};
    virtual void draw(float a) { } // default implementation ignores the value
    virtual void drawBG() {};
    virtual void clearBG() {};
    // virtual void drawStatic() { draw(); }
    // virtual void clearStatic() { clearBG(); }
    // virtual void drawProgressive(float a) { draw(a); }
    // virtual void clearProgressive(float a) { clearBG(); }

public:
    Point _ref;
    bool _dirty = true;
    BoundingBox _bbox;
};
