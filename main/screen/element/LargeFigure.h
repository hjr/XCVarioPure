/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "ScreenElement.h"
#include "IpsDisplay.h"

// a related prominent figure to the gauge visual indicator
class LargeFigure : public ScreenElement
{
public:
    LargeFigure(int16_t x, int16_t y);
    bool changed(float v);
    void draw() override;
    void draw(float a) override;
    

private:
    int16_t _value = 0;
};
