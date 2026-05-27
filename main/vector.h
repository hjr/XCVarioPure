/***********************************************************************
**
**   vector.h
**
**   This file is part of Cumulus.
**
************************************************************************
**
**   Copyright (c):  2002 by André Somers
**                   2002    Eckhard Völlm
**                   2009-2015 Axel Pauli
**
**   This file is distributed under the terms of the General Public
**   License. See the file COPYING for more information.
**
***********************************************************************/

#ifndef VECTOR_H
#define VECTOR_H

#include "math/vector_3d_fwd.h"
#include "math/Units.h"

/**
 * \class Vector
 *
 * \author André Somers, Axel Pauli, Eckhard Völlm
 *
 * \brief A vector represents a speed in a certain (2d) direction.
 *
 * A vector represents a speed in a certain (2d) direction.
 * It is a subclass from the @ref float class, meaning you can use it as a
 * normal float object. The values returned or set in that way are the
 * values in the given direction. You can read or set that direction using
 * the getAngle... and setAngle member functions (or their variants).
 * You can also access the components of the speed in the X and Y directions.
 * Note that X counts latitudinal, Y count longitudinal.
 *
 * \date 2002-2015
 */

typedef struct bitfield_vector {
	   bool dirtyXY  :1;   // True if X and/or Y have been set, and speed and direction need to be recalculated
       bool dirtyDR  :1;
}bitfield_vector_t;

class Vector
{
public:

    Vector();
    Vector(const Vector&) = default;
    Vector(rad_t angle, mps_t speed);
    Vector(const vector_f& v);

    ~Vector();

    static rad_t polar(float y, float x);
    static rad_t normalizePI2(rad_t angle);
    static rad_t normalizePI(rad_t angle);
    static degree_t normalizeDeg(degree_t angle);
    static degree_t normalizeDeg180(degree_t angle);
    static rad_t angleDiff(rad_t ang1, rad_t ang2);    // RAD
    static degree_t angleDiffDeg(degree_t ang1, degree_t ang2);    // DEG
    static degree_t reverseBearing(degree_t angle);
    static rad_t reverseBearingRad(rad_t angle);

    void reset();
    bool isValid() const { return flags.dirtyDR == false || flags.dirtyXY == false; }
    float cross(Vector& v);
    float dot(Vector& v);

    /**
     * Get angle in degrees.
     */
    degree_t getAngleDeg();

    /**
     * Get angle in radian.
     */
    rad_t getAngleRad();

    /**
     * set the angle in degrees
     */
    void setAngle(degree_t angle);

    /**
     * set the angle in degrees  and the speed
     */
    // void setAngleAndSpeed(const int angle, const mps_t& spd);

    /**
     * set the angle in radian
     */
    void setAngleRad(rad_t angle);

    /**
     * Set the speed. Expected unit is meter per second.
     */
    void setSpeedKmh(kmh_t kmh);
    void setSpeed(const mps_t mps);

    /**
     * @return The speed
     */
    mps_t getSpeed();  // in internal units of m/s

    /**
     * @returns The speed in Y (longitude) direction
     * (east is positive, west is negative) in meters per second
     */
    mps_t getYMps();

    /**
     * @return The speed in X (latitude) direction
     * (north is positive, south is negative) in meters per second
     */
    mps_t getXMps();


    /**
     * Sets the X (latitudinal) speed in meters per second.
     */
    void setX(const mps_t& x);

    /**
     * Sets the Y (longitudinal) speed in meters per second.
     */
    void setY(const mps_t& y);

    /* Operators */
    /**
     * = operator for Vector.
     */
    Vector& operator = (const Vector& x);

    Vector operator+(Vector& v);
    Vector operator-(Vector& v);
    float normBy(float x);
    Vector operator/(float x);

    /**
     * operator for Vector.
     */
    float operator*(Vector& x);

    /**
     * != operator for Vector
     */
    // bool operator != ( Vector& x);

    /**
      * == operator for Vector
      */
    // bool operator == ( Vector& x);

    /**
     * minus prefix operator for Vector
     */
    // Vector operator - ();

    // /**
    //  * * prefix operator for Vector
    //  */
    // Vector operator * (float left);

    // Vector operator * (int left);

    /**
     * Poor man's solution for not getting the +
     * operator to work properly.
     */
    void add(Vector arg);

    /**
     * Reverses the direction of the vector.
     */
    Vector reverse();

protected: // Protected attributes

    bitfield_vector_t flags;
    /**
     * Contains the angle of the speed. 0 is north, pi/2 east, pi south, etc.
     */
    rad_t _angle;

    /**
     * float in X (latitudinal) direction in meters per second, north being positive.
     */
    mps_t _x;

    /**
     * float in Y (longitudinal) direction in meters per second, east being positive.
     */
    mps_t _y;

    /**
     * float in mps
     */
    mps_t _speed;

private:
    /**
     * Recalculates the X and Y values from the known angle and speed.
     */
    void recalcXY();

    /**
     * Recalculates the the angle and the distance from the known x and y values.
     */
    void recalcDR();
};

/** operators for vector. */
// Vector operator * (float left, Vector& right);
Vector operator * (Vector& left, float right);
Vector operator / (Vector& left, float right);
Vector operator / (Vector& left, int right);

#endif
