/***********************************************************************
 **
 **   vector.cpp
 **
 **   This file is part of Cumulus
 **
 ************************************************************************
 **
 **   Copyright (c):  2002      by André Somers
 **                   2009-2015 by Axel Pauli
 **
 **   This file is distributed under the terms of the General Public
 **   License. See the file COPYING for more information.
 **
 ***********************************************************************/

#include "vector.h"
#include "math/Units.h"
#include "math/Floats.h"
#include "math/Trigonometry.h"

#include <cmath>

Vector::Vector() : _angle(0.0), _x(0.0), _y(0.0), _speed(0.0) {
    flags.dirtyXY = false;
    flags.dirtyDR = true;
}

Vector::Vector(rad_t angle, mps_t speed) {
    // ESP_LOGI(FNAME, "New Vector ang:%f speed %f", angle, speed );
    _x = 0.0;
    _y = 0.0;
    flags.dirtyXY = false;
    flags.dirtyDR = false;
    _speed = speed;
    setAngleRad(angle);
    flags.dirtyDR = false;
    flags.dirtyXY = true;
}

//  in: -oo < angle < oo
// out: 0 .. 2*pi
rad_t Vector::normalizePI2(rad_t angle) {
    return angle - PI2f * fast_floorf(angle / PI2f);
}

//  in: -oo < angle < oo
// out: 0 .. 2*pi
rad_t Vector::normalizePI(rad_t angle) {
    return angle - PI2f * fast_floorf((angle + My_PIf) / PI2f);
}

//  in: -oo < angle < oo
// out: 0 .. 360
degree_t Vector::normalizeDeg(degree_t angle) {
    return angle - 360.0f * fast_floorf(angle / 360.0f);
}

//  in: -oo < angle < oo
// out: -180 .. 180
degree_t Vector::normalizeDeg180(degree_t angle) {
    return angle - 360.0f * fast_floorf((angle + 180.0f) / 360.0f);
}

//  in: -oo < angle < oo
// out: -180 .. 180
degree_t Vector::reverseBearing(degree_t angle) {
    return normalizeDeg180(angle + 180.0f);
}

rad_t Vector::reverseBearingRad(rad_t angle) {
    return normalizePI(angle + My_PIf);
}

//  in: -oo < x,y < oo
// out: 0 .. 2*pi
rad_t Vector::polar(float y, float x) {
    rad_t angle = atan2f(y, x);  // range: (–π, π]
    if (angle < 0.0f) {
        angle += PI2f;  // convert to [0, 2π)
    }
    return angle;
}

//  in: -oo < ang1, ang2 < oo
// out: -180 .. 180
degree_t Vector::angleDiffDeg(degree_t ang1, degree_t ang2) {
    return normalizeDeg180(ang1 - ang2);
}

//  in: -oo < ang1, ang2 < oo
// out: -pi .. pi
rad_t Vector::angleDiff(rad_t ang1, rad_t ang2) {
    return normalizePI(ang1 - ang2);
}

Vector::~Vector() {}

void Vector::reset() {
    _angle = 0.0;
    _x = 0.0;
    _y = 0.0;
    _speed = 0.0;
    flags.dirtyXY = true;
    flags.dirtyDR = true;
}

Vector Vector::cross(Vector& v) {
    if (flags.dirtyXY) {
        recalcXY();
    }
    if (v.flags.dirtyXY) {
        v.recalcXY();
    }
    Vector result;
    result._x = _y * v._x - _x * v._y;
    result._y = _x * v._y - _y * v._x;
    result.flags.dirtyDR = true;
    return result;
}

float  Vector::dot(Vector& v) {
    if (flags.dirtyXY) {
        recalcXY();
    }
    if (v.flags.dirtyXY) {
        v.recalcXY();
    }
    return _x * v._x + _y * v._y;
}

degree_t Vector::getAngleDeg() {
    if (flags.dirtyDR) {
        recalcDR();
    }

    return Units::rad_to_deg(_angle);
}

/** Get angle in radian. */
rad_t Vector::getAngleRad() {
    if (flags.dirtyDR) {
        recalcDR();
    }

    return _angle;
}

void Vector::setAngle(degree_t angle) {
    // ESP_LOGI(FNAME, "setAngle D ang:%f", angle );
    if (flags.dirtyDR) {
        recalcDR();
    }

    _angle = normalizePI2(Units::deg_to_rad(angle));
    flags.dirtyXY = true;
    // ESP_LOGI(FNAME, "New angle ang:%f", _angle );
}

/**
 * set the angle in degrees and the speed.
 */
// void Vector::setAngleAndSpeed(const int angle, const mps_t& spd) {
//     if (flags.dirtyDR) {
//         recalcDR();
//     }

//     setAngle(angle);
//     _speed = spd;
//     flags.dirtyDR = false;
//     flags.dirtyXY = true;
// }

/** Set property of float angle as radian. */
void Vector::setAngleRad(rad_t angle) {
    if (flags.dirtyDR) {
        recalcDR();
    }

    _angle = normalizePI2(angle);
    flags.dirtyXY = true;
}

/**
 * Set the speed
 */
void Vector::setSpeedKmh(kmh_t speed) {
    if (flags.dirtyDR) {
        recalcDR();
    }

    _speed = Units::kmh_to_mps(speed);
    flags.dirtyXY = true;
}

/**
 * Set the speed. Expected unit is meter per second.
 */
void Vector::setSpeed(const mps_t mps) {
    if (flags.dirtyDR) {
        recalcDR();
    }

    _speed = mps;
    flags.dirtyXY = true;
}

/**
 * @return The speed
 */
mps_t Vector::getSpeed() {
    if (flags.dirtyDR) {
        recalcDR();
    }

    return _speed;
}

/** Recalculates the the angle and the speed from the known x and y values. */
void Vector::recalcDR() {
    _angle = normalizePI2(polar(_y, _x));
    _speed = hypot(_y, _x);
    flags.dirtyDR = false;
}

/** Recalculates the X and Y values from the known angle and speed. */
void Vector::recalcXY() {
    _y = _speed * sin(_angle);
    _x = _speed * cos(_angle);
    flags.dirtyXY = false;
}

/** returns the speed in X (latitude) direction (north is positive, south is negative) */
float Vector::getXMps() {
    if (flags.dirtyXY) {
        recalcXY();
    }

    return _x;
}

/** Returns the speed in Y (longitude) direction (east is positive, west is negative) */
mps_t Vector::getYMps() {
    if (flags.dirtyXY) {
        recalcXY();
    }

    return _y;
}

/** Sets the Y (longitudinal) speed in meters per second. */
void Vector::setY(const float& y) {
    if (flags.dirtyXY) {
        recalcXY();
    }

    _y = y;
    flags.dirtyDR = true;
}

/** Sets the X (latitudinal) speed in meters per second. */
void Vector::setX(const float& x) {
    if (flags.dirtyXY) {
        recalcXY();
    }

    _x = x;
    flags.dirtyDR = true;
}

/** = operator for Vector. */
Vector& Vector::operator=(const Vector& x) {
    setX(x._x);
    setY(x._y);
    _speed = x._speed;
    _angle = x._angle;
    flags.dirtyXY = x.flags.dirtyXY;
    flags.dirtyDR = x.flags.dirtyDR;

    return *this;
}

/** + operator for Vector. */
Vector Vector::operator+(Vector& v) {
    if (flags.dirtyXY) {
        recalcXY();
    }
    if (v.flags.dirtyXY) {
        v.recalcXY();
    }
    Vector result;
    result._x = _x + v._x;
    result._y = _y + v._y;
    result.flags.dirtyDR = true;
    return result;
}

/** - operator for Vector. */
Vector Vector::operator-(Vector& v) {
    if (flags.dirtyXY) {
        recalcXY();
    }
    if (v.flags.dirtyXY) {
        v.recalcXY();
    }
    Vector result;
    result._x = _x - v._x;
    result._y = _y - v._y;
    result.flags.dirtyDR = true;
    return result;
}

// /** * operator for Vector. */
// Vector Vector::operator*(float left) {
//     if (flags.dirtyDR) {
//         recalcDR();
//     }

//     return Vector(_angle, left * _speed);
// }

// Vector Vector::operator*(int left) {
//     if (!flags.dirtyDR) {
//         return Vector(_angle, left * _speed);
//     } else if (!flags.dirtyXY) {
//         return Vector(left * _x, left * _y);
//     } else {
//         recalcXY();
//         return Vector(left * _x, left * _y);
//     }
// }

/** / operator for Vector. */
float Vector::normBy(float x) {
    if (flags.dirtyDR)
        recalcDR();
    return _speed / x;
}

Vector Vector::operator/(float x) {
    if (flags.dirtyXY) {
        recalcXY();
    }
    Vector result;
    result._x = _x / x;
    result._y = _y / x;
    result.flags.dirtyDR = true;
    return result;
}

/** * operator for Vector. */
float Vector::operator*(Vector& x) {
    if (flags.dirtyDR)
        recalcDR();
    if (x.flags.dirtyDR)
        x.recalcDR();
    return _speed * x._speed;
}

/** == operator for Vector */
// bool Vector::operator==(Vector& x) {
//     Vector t(x);
//     Vector u(*this);

//     if (u.flags.dirtyDR) {
//         u.recalcDR();
//     }

//     if (t.flags.dirtyDR) {
//         t.recalcDR();
//     }

//     return ((t._speed == u._speed) && (t._angle == u._angle));
// }

/** != operator for Vector */
// bool Vector::operator!=(Vector& x) {
//     if (flags.dirtyDR) {
//         recalcDR();
//     }

//     return ((x._speed != _speed) || (x._angle != _angle));
// }

/** - prefix operator for Vector */
// Vector Vector::operator-() {
//     // there are two options for this. We use the one that involves the least conversions.
//     if (!flags.dirtyDR) {
//         return Vector(_angle + My_PIf, _speed);
//     } else if (!flags.dirtyXY) {
//         return Vector(-_x, -_y); // fixme wrong usage of ctor
//     } else {  // should not happen! Either XY or DR should be clean, or both.
//         recalcXY();
//         return Vector(-_x, -_y);
//     }
// }

/** * operator for vector. */
// Vector operator*(Vector& left, float right) {
//     return Vector(left.getAngleRad(), right * left.getSpeed());
// }

/** * operator for vector. */
// Vector operator*(float left, Vector& right) {
//     return Vector(right.getAngleRad(), left * right.getSpeed());
// }

/** / operator for vector. */
Vector operator/(Vector& left, float right) {
    return Vector(left.getAngleRad(), left.getSpeed() / right);
}

/** / operator for vector. */
Vector operator/(Vector& left, int right) {
    return Vector(left.getAngleRad(), left.getSpeed() / right);
}

/** Poor man's solution for not getting the + operator to work properly. */
void Vector::add(Vector arg) {
    if (arg.flags.dirtyXY) {
        arg.recalcXY();
    }

    if (flags.dirtyXY) {
        recalcXY();
    }

    _x += arg.getXMps();
    _y += arg.getYMps();

    flags.dirtyDR = true;
}

Vector Vector::reverse() {
    if (flags.dirtyXY) {
        recalcXY();
    }
    _x = -_x;
    _y = -_y;
    flags.dirtyDR = true;
    return *this;
}
