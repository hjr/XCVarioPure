/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "math/vector_3d.h"

// Hesse form of a 3d plane: Normal x Pxyz + d = 0
struct Plane
{
    Plane() = default;
    Plane(const vector_f &p0, const vector_f &n);
    vector_f _nn;
    float _d;
    float fct(const vector_f &p);
    float signedDist(const vector_f& p);
};

