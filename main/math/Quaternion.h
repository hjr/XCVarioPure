/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#pragma once

#include "math/vector_3d_fwd.h"
#include "math/Units.h"


// #define Quaternionen_Test 1

float Compass_atan2( float y, float x );

// Quaternion class in the form:
// q = a + bi + cj + dk

class Quaternion {
public:
    float _w;
    float _x;
    float _y;
    float _z;
    constexpr Quaternion() : _w(1.), _x(0), _y(0), _z(0) {};
    Quaternion(float w, float b, float c, float d);
    Quaternion(rad_t angle, const vector_f& axis);
    Quaternion(Quaternion &&) = default; // Allow std::move
    Quaternion(const Quaternion &) = default;
    Quaternion& operator=(const Quaternion&) = default;
    bool operator==(const Quaternion r) { return _w==r._w && _x==r._x && _y==r._y && _z==r._z; };

    // API
    rad_t getAngle() const;
    rad_t getAngleAndAxis(vector_f& axis) const;
    friend Quaternion operator*(const Quaternion& left, const Quaternion& right);
    Quaternion get_normalized() const;
    Quaternion& normalize();
    Quaternion& conjugate();
    Quaternion get_conjugate() const;
    vector_f rotate(const vector_f& p) const;
    // vector_d rotate(const vector_d& p) const;
    friend Quaternion slerp(Quaternion q1, Quaternion q2, double lambda);
    static Quaternion AlignVectors(const vector_f &start, const vector_f &dest);
    static Quaternion fromRotationMatrix(const vector_f &X, const vector_f &Y);
    static Quaternion fromAccelerometer(const vector_f& accel);
    static Quaternion fromGyro(const vector_f& w, float time);
    vector_f toEulerRad() const;

    // something like a unit test
    static void quaternionen_test();
};
