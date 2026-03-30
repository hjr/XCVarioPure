/***********************************************************
 ***   THIS DOCUMENT CONTAINS PROPRIETARY INFORMATION.   ***
 ***    IT IS THE EXCLUSIVE CONFIDENTIAL PROPERTY OF     ***
 ***     Rohs Engineering Design AND ITS AFFILIATES.     ***
 ***                                                     ***
 ***       Copyright (C) Rohs Engineering Design         ***
 ***********************************************************/

#include "math/Quaternion.h"
#include "math/Trigonometry.h"
#include "math/vector_3d.h"
#include "esp_log.h"

//#include <esp_timer.h>

#include <cmath>

#define TAG "Quaternion"

// basic constructor
Quaternion::Quaternion(float w, float x, float y, float z)
{
    _w = w;
    _x = x;
    _y = y;
    _z = z;
}

// rotate radian angle around axis
Quaternion::Quaternion(rad_t angle, const vector_f& axis)
{
    float fac = std::sinf(0.5f * angle);

    _w = std::cosf(0.5f * angle);
    _x = fac * axis.x;
    _y = fac * axis.y;
    _z = fac * axis.z;
}
// radian
rad_t Quaternion::getAngle() const
{
    return 2.f * std::acos(_w);
}

// radians and a normalized vector
rad_t Quaternion::getAngleAndAxis(vector_f& axis) const
{
    rad_t angle = getAngle();
    float sinphi2 = fabsf(angle) > 1e-7 ? 1.0f / sinf(0.5f * angle) : 0.0;

    // null rotation -> return null vector
    axis.x = _x;
    axis.y = _y;
    axis.z = _z;
    axis *= sinphi2;
    // ESP_LOGI(TAG,"naxisv: %.3f", axis.get_norm() );

    return angle;
}


// concatenatin of two quaternions
Quaternion operator*(const Quaternion& q1, const Quaternion& q2)
{
    // q = q1*q2 -- IMPORTANT: read the "*" operator as "after"
    //                         q1*q2 is first rotating q2, then q1 (!)
    Quaternion q( (q1._w*q2._w) - (q1._x*q2._x) - (q1._y*q2._y) - (q1._z*q2._z),
                  (q1._w*q2._x) + (q1._x*q2._w) + (q1._y*q2._z) - (q1._z*q2._y),
                  (q1._w*q2._y) - (q1._x*q2._z) + (q1._y*q2._w) + (q1._z*q2._x),
                  (q1._w*q2._z) + (q1._x*q2._y) - (q1._y*q2._x) + (q1._z*q2._w) );
    q.normalize();
    return q;
}

// Spherically interpolates between quaternions q1 and q2 by ratio lambda.
// The parameter lambda is clamped to the range [0, 1].
// Use this to create a rotation which smoothly interpolates between the first
// quaternion q1 to the second quaternion q2, based on the value of the parameter lambda.
// If the value of the parameter is close to 0, the output will be close to q1,
// if it is close to 1, the output will be close to q2.

Quaternion slerp(Quaternion q1, Quaternion q2, double lambda)
{
	float dotproduct = q1._x * q2._x + q1._y * q2._y + q1._z * q2._z + q1._w * q2._w;
	float theta, st, sut, sout, coeff1, coeff2;
	// algorithm adapted from Shoemake's paper
	lambda=lambda/2.0;

	theta = (float) acos(dotproduct);
	if (theta<0.0) theta=-theta;

	st = (float) sinf(theta);
	sut = (float) sin(lambda*theta);
	sout = (float) sin((1-lambda)*theta);
	coeff1 = sout/st;
	coeff2 = sut/st;

	Quaternion qr( coeff1*q1._w + coeff2*q2._w, coeff1*q1._x + coeff2*q2._x, coeff1*q1._y + coeff2*q2._y, coeff1*q1._z + coeff2*q2._z );
	return qr.normalize();
}

// Get a normalized version of quaternion
Quaternion Quaternion::get_normalized() const
{
    float len = std::sqrtf(_w*_w + _x*_x + _y*_y + _z*_z);
    Quaternion q2( _w/len, _x/len, _y/len, _z/len );
    // ESP_LOGI(TAG,"Q1: a=%.3f b=%.3f c=%.3f d=%.3f  Q2: a=%.3f b=%.3f c=%.3f d=%.3f", a, b, c, d, q2.a, q2.b, q2.c, q2.d );
    return q2;
}
// Normalize quaternion
Quaternion& Quaternion::normalize()
{
    float len = std::sqrtf(_w*_w + _x*_x + _y*_y + _z*_z);
    _w = _w/len;
    _x = _x/len;
    _y = _y/len;
    _z =_z/len;
    return *this;
}

// conjugat quaternion
Quaternion& Quaternion::conjugate()
{
    _x = -_x;
    _y = -_y;
    _z = -_z;
    return *this;
}

// return the conjugated quaternion
Quaternion Quaternion::get_conjugate() const
{
    Quaternion q2( _w, -_x, -_y, -_z );
    return q2;
}

// rotate vector v by quaternion
vector_f Quaternion::rotate(const vector_f& vec) const
{
    float a00 = _w * _w;
    float a01 = _w * _x;
    float a02 = _w * _y;
    float a03 = _w * _z;
    float a11 = _x * _x;
    float a12 = _x * _y;
    float a13 = _x * _z;
    float a22 = _y * _y;
    float a23 = _y * _z;
    float a33 = _z * _z;
    vector_f result;
    result.x = vec.x * (+a00 + a11 - a22 - a33)
        + 2.0f * (a12 * vec.y + a13 * vec.z + a02 * vec.z - a03 * vec.y);
    result.y = vec.y * (+a00 - a11 + a22 - a33)
        + 2.0f * (a12 * vec.x + a23 * vec.z + a03 * vec.x - a01 * vec.z);
    result.z = vec.z * (+a00 - a11 - a22 + a33)
        + 2.0f * (a13 * vec.x + a23 * vec.y - a02 * vec.x + a01 * vec.y);
    return result;
}
// vector_d Quaternion::rotate(const vector_d& vec) const
// {
//     double a00 = _w * _w;
//     double a01 = _w * _x;
//     double a02 = _w * _y;
//     double a03 = _w * _z;
//     double a11 = _x * _x;
//     double a12 = _x * _y;
//     double a13 = _x * _z;
//     double a22 = _y * _y;
//     double a23 = _y * _z;
//     double a33 = _z * _z;
//     vector_d result;
//     result.x = vec.x * (+a00 + a11 - a22 - a33)
//         + 2.0 * (a12 * vec.y + a13 * vec.z + a02 * vec.z - a03 * vec.y);
//     result.y = vec.y * (+a00 - a11 + a22 - a33)
//         + 2.0 * (a12 * vec.x + a23 * vec.z + a03 * vec.x - a01 * vec.z);
//     result.z = vec.z * (+a00 - a11 - a22 + a33)
//         + 2.0 * (a13 * vec.x + a23 * vec.y - a02 * vec.x + a01 * vec.y);
//     return result;
// }

// return euler angles roll, pitch, yaw in radian
vector_f Quaternion::toEulerRad() const
{
    vector_f result;

    // roll
    result.x = atan2f(2.f*(_w*_x + _y*_z),1.f - 2.f*(_x*_x + _y*_y));

    // pitch
    result.y = (-(My_PIf)/2.f + 2.f* atan2f(std::sqrtf(1.f+ 2.f*(_w*_y - _x*_z)), std::sqrtf(1- 2*(_w*_y - _x*_z))));
    // or asin(2*(a*c - d*b));

    // yaw
    if (_z==0)
        result.z = 0.0f;
    else
        result.z = atan2f(2.f*(_w*_z + _x*_y),1.f - 2.f*(_y*_y + _z*_z));
    return result;


    // vector_f result;
    // double q0 = _w;
    // double q1 = b;
    // double q2 = c;
    // double q3 = d;
    // result.a = atan2(2*(q0*q1 + q2*q3),1 - 2*(q1*q1 + q2*q2));
    // // float xx = asin(2*(q0*q2 - q3*q1))*180/My_PIf;
    // result.y = (-My_PIf/2. + 2* atan2(sqrt(1+ 2*(q0*q2 - q1*q3)), sqrt(1- 2*(q0*q2 - q1*q3))));
    // //result.pitch = asin(2*(q0*q2 - q3*q1))*180/My_PIf;
    // // ESP_LOGI( TAG,"EulerPitch sin:%.4f atan2:%.4f", xx, result.pitch);
    // if (d==0)
    //     result.z = 0.0;
    // else
    //     result.z = atan2(2*(q0*q3 + q1*q2),1 - 2*(q2*q2 + q3*q3));
    // return result;
}

// Creat a rotation through two vectors, aligning the first to the second
Quaternion Quaternion::AlignVectors(const vector_f& start, const vector_f& dest) {
    vector_f from = start;
    from.normalize();
    vector_f to = dest;
    to.normalize();

    float cosTheta = from.dot(to);
    vector_f rotationAxis;

    if ( cosTheta > 1.f- 0.0001f) {
        // vectors are the same
        return Quaternion();
    } else if (cosTheta < -1.f + 0.001f) {
        // special case when vectors in opposite directions:
        // there is no "ideal" rotation axis
        // So guess one; any will do as long as it's perpendicular to start
        rotationAxis = from.cross(vector_f(0.0f, 0.0f, 1.0f));
        if (rotationAxis.get_norm2() < 0.01)  // bad luck, they were parallel, try again!
            rotationAxis = from.cross(vector_f(1.0f, 0.0f, 0.0f));

        rotationAxis.normalize();
        return Quaternion(deg2rad(180.0f), rotationAxis);
    }
    rotationAxis = from.cross(to);

    float s = std::sqrt((1.f + cosTheta) * 2.f);
    float k = 1.f / s;
    Quaternion q(s * 0.5f, rotationAxis.x * k, rotationAxis.y * k, rotationAxis.z * k);
    return q.normalize();
}

Quaternion Quaternion::fromRotationMatrix(const vector_f &X, const vector_f &Y)
{
    vector_f mat[3];
    mat[0] = X;
    mat[1] = Y;
    mat[2] = X.cross(Y);
    // ESP_LOGI(TAG, "Z: %f,%f,%f", mat[2].x, mat[2].y, mat[2].z);

    Quaternion result;
    const double trace = mat[0].x + mat[1].y + mat[2].z;
    // ESP_LOGI(TAG, "trace: %f", trace);

    // check the diagonal
    if ( trace > 0.0 ) {
        double s = std::sqrt(trace + 1.0);
        result._w = s / 2.0;
        s = 0.5 / s;

        result._x = (mat[1][2] - mat[2][1]) * s;
        result._y = (mat[2][0] - mat[0][2]) * s;
        result._z = (mat[0][1] - mat[1][0]) * s;
    } else {
        // find largest diag. element with index i
        int i = 0;
        if ( mat[1][1] > mat[0][0] ) {
            i = 1;
        }
        if ( mat[2][2] > mat[i][i] ) {
            i = 2;
        }
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        double s = std::sqrt((mat[i][i] - (mat[j][j] + mat[k][k])) + 1.0);
        vector_f v;
        v[i] = s * 0.5;
        s = 0.5 / s;

        result._w = (mat[j][k] - mat[k][j]) * s;
        v[j] = (mat[i][j] + mat[j][i]) * s;
        v[k] = (mat[i][k] + mat[k][i]) * s;
        result._x = v[0]; result._y = v[1]; result._z = v[2];
    }
    return result;
}

// https://ahrs.readthedocs.io/en/latest/filters/aqua.html
// accel_par in m/(sec*sec)
Quaternion Quaternion::fromAccelerometer(const vector_f& accel_par)
{
    // Normalize
    vector_f an = accel_par;
    an.normalize();
    // ESP_LOGI(TAG,"ax=%.3f ay=%.3f az=%.3f", an.x, an.y, an.z);

    // There is a singularity at an.z == -1
    if ( an.z < -0.999999f ) {
        float half_cos = std::sqrtf(0.5f*(1.0f - an.z));
        float temp = .5f / half_cos;
        Quaternion orientation( -an.y*temp, half_cos, 0.0, an.x*temp );
        //ESP_LOGI(TAG,"Quat: %.3f %.3f %.3f %.3f", orientation.a, orientation.b, orientation.c, orientation.d );
        return orientation;
    } else {
        float half_cos = std::sqrtf(0.5f*(1.0f + an.z));
        float temp = .5f / half_cos;
        Quaternion orientation( half_cos, -an.y*temp, an.x*temp, 0.0 );
        //ESP_LOGI(TAG,"Quat: %.3f %.3f %.3f %.3f", orientation.a, orientation.b, orientation.c, orientation.d );
        return orientation;
    }
}

// omega in radians per second: dtime in seconds
Quaternion Quaternion::fromGyro(const vector_f& omega, float dtime)
{
    // ESP_LOGI(TAG,"Quat: %.3f %.3f %.3f", omega.a, omega.b, omega.c );
    float alpha = 0.5*dtime;
    float a,b,c,d;
    b = alpha*(omega.x);
    c = alpha*(omega.y);
    d = alpha*(omega.z);
    a = 1.f - 0.5f*(b*b+c*c+d*d);
    Quaternion result(a,b,c,d);
    result.normalize();
    // ESP_LOGI(TAG,"Quat1: %.3f %.3f %.3f %.3f", result.a, result.b, result.c, result.d );

    // Often found in literature and identic for small angles.
    // // omega=(alpha,beta,gamma)
    // // theta = ||omega||*dt; //length of angular velocity vector
    // float norm = omega.normalize(); // normalized orientation of angular velocity vector
    // float theta_05 = norm * 0.5 * dtime;
    // Quaternion result2(cos(theta_05), omega.a * sin(theta_05), omega.b * sin(theta_05), omega.c * sin(theta_05));
    // result = result2;
    // ESP_LOGI(TAG,"Quat2: %.3f %.3f %.3f %.3f", result.a, result.b, result.c, result.d );
    return result;
}

#ifdef Quaternionen_Test

// Test quaternion performance against a matrix mapping
class Matrix
{
public:
    Matrix() = default;
    Matrix(const vector_f& x, const vector_f& y) {
        vector_f z = x.cross(y);
        m[0][0] = x.x; m[1][0] = x.x; m[2][0] = x.x;
        m[0][1] = y.y; m[1][1] = y.y; m[2][1] = y.y;
        m[0][2] = z.z; m[1][2] = z.z; m[2][2] = z.z;
    }
    vector_f operator*(vector_f& v2) {
        vector_f r;
        // not correct, just for a performance comparison
        for ( int row = 0; row < 3; ++row ) {
            for ( int col = 0; col < 3; ++col ) {
                r[row] += m[row][col] * v2[row];
            }
        }
        return r;
    }

    float m[3][3];
};

void Quaternion::quaternionen_test()
{
    vector_f v1(1,0,0),
        v2(0,0,1),
        vt(1,2,3),
        x_axes(1,0,0),
        y_axes(0,1,0),
        z_axes(0,0,1),
        v3;

    // v to v setup
    int64_t t0 = esp_timer_get_time();
    Quaternion q = Quaternion::AlignVectors(v1,v2);
    int64_t t1 = esp_timer_get_time();
    ESP_LOGI(TAG,"Setup");
    ESP_LOGI(TAG,"Align v1 to v2: (%f,%f,%f) - (%f,%f,%f)", v1.x, v1.y, v1.z, v2.x, v2.y, v2.z );
    ESP_LOGI(TAG,"%-4lldusec: %f %f %f %f a:%f", t1-t0, q._w, q._x, q._y, q._z, rad2deg(q.getAngle()) );
    // explicit setup, rotate around y
    Quaternion qex = Quaternion(deg2rad(-90.f), y_axes);
    ESP_LOGI(TAG,"equal to: %f %f %f %f a:%f", qex._w, qex._x, qex._y, qex._z, rad2deg(qex.getAngle()) );

    // rotate
    t0 = esp_timer_get_time();
    v3 = q.rotate(v1);
    t1 = esp_timer_get_time();
    ESP_LOGI(TAG,"Mapping (%lldusec)", t1-t0);
    ESP_LOGI(TAG,"rotate v1 -> v2: %f %f %f", v3.x, v3.y, v3.z );
    // compare to matrix mapping
    Matrix m(v2, y_axes);
    t0 = esp_timer_get_time();
    v3 = m * v1;
    t1 = esp_timer_get_time();
    ESP_LOGI(TAG,"Matrixing (%lldusec)", t1-t0);
    ESP_LOGI(TAG,"rotate v1 -> v2: %f %f %f", v3.x, v3.y, v3.z );

    // Zero rotation
    q = Quaternion(1,0,0,0);
    v3 = q.rotate(v1);
    ESP_LOGI(TAG,"rotate zero v1 -> v1: %f %f %f", v3.x, v3.y, v3.z );

    // slerp
    ESP_LOGI(TAG,"Slerp: (v1+v2)/2, v2");
    Quaternion q2 = Quaternion::AlignVectors(v1,vector_f(0.707106781, 0, 0.707106781));
    ESP_LOGI(TAG,"Q: %f %f %f %f a:%f", q2._w, q2._x, q2._y, q2._z, rad2deg(q2.getAngle()) );
    Quaternion qs = slerp(q, q2, 1.f);
    ESP_LOGI(TAG,"-> (45°+90°)/2");
    ESP_LOGI(TAG,"slerp: %f %f %f %f a:%f", qs._w, qs._x, qs._y, qs._z, rad2deg(qs.getAngle()) );

    // toEuler
    q = Quaternion(1,0,0,0);
    vector_f e = q.toEulerRad() * rad2deg(1.f);
    ESP_LOGI( TAG,"Euler zero r/p/y %.4f/%.4f/%.4f", e.Roll(), e.Pitch(), e.Yaw());
    // 45° around X
    q = Quaternion::AlignVectors(vector_f(0,0,1), vector_f(0,-1,1));
    e = q.toEulerRad() * rad2deg(1.f);
    ESP_LOGI( TAG,"Euler +45X r/p/y %.4f/%.4f/%.4f", e.Roll(), e.Pitch(), e.Yaw());

    // compass atan2
    ESP_LOGI(TAG,"Check compass atan2");
    ESP_LOGI(TAG,"  0? %f", Compass_atan2(0, 0));
    ESP_LOGI(TAG,"  0: %f", Compass_atan2(0, 1));
    ESP_LOGI(TAG," 45: %f", Compass_atan2(1, 1));
    ESP_LOGI(TAG," 45: %f", Compass_atan2(0.6, 0.6)); // norm?
    ESP_LOGI(TAG," 90: %f", Compass_atan2(1., 0.));
    ESP_LOGI(TAG,"135: %f", Compass_atan2(0.6, -0.6));
    ESP_LOGI(TAG,"180: %f", Compass_atan2(0., -1.));
    ESP_LOGI(TAG,"180: %f", Compass_atan2(-0.00001, -1.));
    ESP_LOGI(TAG,"225: %f", Compass_atan2(-0.6, -0.6));
    ESP_LOGI(TAG,"270: %f", Compass_atan2(-1., 0.));
    ESP_LOGI(TAG,"315: %f", Compass_atan2(-0.6, 0.6));


    // fromRotationMatrix
    ESP_LOGI(TAG, "Test import of 3x3 matrix");
    q = fromRotationMatrix(vector_f(1,0,0), vector_f(0,1,0));
    ESP_LOGI(TAG, "Benign case: 1,0,0/0,1,0 - %f %f %f %f", q._w, q._x, q._y, q._z);
    ESP_LOGI(TAG, "Rotate 90°/Z");
    q = fromRotationMatrix(vector_f(0,1,0), vector_f(-1,0,0));
    ESP_LOGI(TAG, "Quaternion : %f %f %f %f a:%f", q._w, q._x, q._y, q._z, rad2deg(q.getAngle()) );
    // explicit setup, rotate around z
    qex = Quaternion(deg2rad(90.f), z_axes);
    ESP_LOGI(TAG, "check equal: %f %f %f %f a:%f", qex._w, qex._x, qex._y, qex._z, rad2deg(qex.getAngle()) );

    ESP_LOGI(TAG, "Rotate 90°/X");
    q2 = fromRotationMatrix(vector_f(1,0,0), vector_f(0,0,1));
    ESP_LOGI(TAG, "Quaternion : %f %f %f %f a:%f", q2._w, q2._x, q2._y, q2._z, rad2deg(q2.getAngle()) );
    // explicit setup, rotate around z
    qex = Quaternion(deg2rad(90.f), x_axes);
    ESP_LOGI(TAG, "check equal: %f %f %f %f a:%f", qex._w, qex._x, qex._y, qex._z, rad2deg(qex.getAngle()) );

    ESP_LOGI(TAG, "Concat Rotate 90°/Z and Rotate 90°/X");
    q = q * q2; // concatenate
    ESP_LOGI(TAG, "Quaternion : %f %f %f %f a:%f", q._w, q._x, q._y, q._z, rad2deg(q.getAngle()) );
    v1 = q.rotate(x_axes);
    ESP_LOGI(TAG, "image of x-axes: %f %f %f", v1.x, v1.y, v1.z );
    v1 = q.rotate(y_axes);
    ESP_LOGI(TAG, "image of y-axes: %f %f %f", v1.x, v1.y, v1.z );
    v1 = q.rotate(vector_f(5,5,5));
    ESP_LOGI(TAG, "image of 5,5,5: %f %f %f", v1.x, v1.y, v1.z );
    // concatenate
    qex = fromRotationMatrix(vector_f(0,1,0), vector_f(0,0,1));
    ESP_LOGI(TAG, "check equal: %f %f %f %f a:%f", qex._w, qex._x, qex._y, qex._z, rad2deg(qex.getAngle()) );
    v1 = qex.rotate(x_axes);
    ESP_LOGI(TAG, "image of x-axes: %f %f %f", v1.x, v1.y, v1.z );
    v1 = qex.rotate(y_axes);
    ESP_LOGI(TAG, "image of y-axes: %f %f %f", v1.x, v1.y, v1.z );
    v1 = qex.rotate(vector_f(5,5,5));
    ESP_LOGI(TAG, "image of 5,5,5: %f %f %f", v1.x, v1.y, v1.z );

    // fromAccelerometer
    ESP_LOGI(TAG, "Test accelerometer vector conversion");
    q = fromAccelerometer(vector_f(0,0,1));
    vector_f euler = q.toEulerRad() * rad2deg(1.f);
    ESP_LOGI(TAG, "Check zero case: R/P/Y: %f %f %f", euler.Roll(), euler.Pitch(), euler.Yaw());
    q = fromAccelerometer(vector_f(0,1,1));
    euler = q.toEulerRad() * rad2deg(1.f);
    ESP_LOGI(TAG, "Check Roll-45°X case: R/P/Y: %f %f %f", euler.Roll(), euler.Pitch(), euler.Yaw());

    // fromGyro
    ESP_LOGI(TAG, "Test gyro vector conversion");
    q = fromGyro(vector_f(deg2rad(30.f),0,0), 0.5f);
    euler = q.toEulerRad() * rad2deg(1.f);
    ESP_LOGI(TAG, "Check X:30°/s,0.5s: X/Y/Z: %f %f %f", euler.Roll(), euler.Pitch(), euler.Yaw());
    q = fromGyro(vector_f(0,deg2rad(-30.f),0), 0.5f);
    euler = q.toEulerRad() * rad2deg(1.f);
    ESP_LOGI(TAG, "Check Y:-30°/s,0.5s: X/Y/Z: %f %f %f", euler.Roll(), euler.Pitch(), euler.Yaw());
    q = fromGyro(vector_f(0,0,deg2rad(30.f)), 0.5f);
    euler = q.toEulerRad() * rad2deg(1.f);
    ESP_LOGI(TAG, "Check Z:30°/s,0.5s: X/Y/Z: %f %f %f", euler.Roll(), euler.Pitch(), euler.Yaw());
    q = fromGyro(vector_f(deg2rad(30.f),deg2rad(30.f),deg2rad(30.f)), 0.5f);
    euler = q.toEulerRad() * rad2deg(1.f);
    ESP_LOGI(TAG, "Check All:30°/s,0.5s: X/Y/Z: %f %f %f", euler.Roll(), euler.Pitch(), euler.Yaw());
}

#endif
