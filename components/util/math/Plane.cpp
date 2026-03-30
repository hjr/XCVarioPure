/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "Plane.h"

#include <cmath>


Plane::Plane(const vector_f &p0, const vector_f &n)
{
    // normalize
    float len = std::sqrtf(n.x*n.x + n.y*n.y + n.z*n.z);
    _nn.x = n.x/len;
    _nn.y = n.y/len;
    _nn.z = n.z/len;

    // d = n_unit · point
    _d = _nn.x*p0.x + _nn.y*p0.y + _nn.z*p0.z;
}

float Plane::fct(const vector_f &p)
{
    return _nn.x*p.x + _nn.y*p.y + _nn.z*p.z - _d;
}

// distance is positive, if point is in direction of normal from plane
float Plane::signedDist(const vector_f& p) {
    return _nn.x*p.x + _nn.y*p.y + _nn.z*p.z - _d;
}
