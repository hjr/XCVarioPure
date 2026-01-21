#pragma once

#include <cmath>


template <typename T>
class vector_3d
{
public:
	vector_3d() = default;
	vector_3d( const vector_3d &o ) { x=o.x; y=o.y; z=o.z; };
	vector_3d(T px, T py, T pz);
	vector_3d(vector_3d&&) = default; // Allow the move optimization
	vector_3d<T>& operator=(const vector_3d<T>&) = default;

	// indexed access
	T& operator[](int i) { return (&x)[i]; }

	// API
	void set(const T p1, const T p2, const T p3) { x=p1, y=p2, z=p3; }
	vector_3d<T>& operator+=(const vector_3d<T>& v2);
	vector_3d<T> operator+(const vector_3d<T>& v2) const;
	vector_3d<T>& operator-=(const vector_3d<T>& v2);
	vector_3d<T> operator-(const vector_3d<T>& v2) const;
	vector_3d<T>& operator*=(const T s2);
	vector_3d<T> operator*(const T s2);
	vector_3d<T>& operator/=(const T s2);
	T dot(const vector_3d<T>& v2) const;
	vector_3d<T> cross(const vector_3d<T> &v2 ) const;
	T get_norm() const { return std::sqrt(x*x + y*y + z*z); }
	T get_norm2() const { return x*x + y*y + z*z; }
	T normalize();
	vector_3d<T> get_normalized() const;
	// Interpretation as Euler angles
	float Roll()  const { return x; }
	void setRoll(float roll) { x=roll; }
	float Pitch() const { return y; }
	void setPitch(float pitch) { y=pitch; }
	float Yaw()   const { return z; }
	void setYaw(float yaw) { z=yaw; }

// todo private:
	T x;
	T y;
	T z;
};

template <typename T>
vector_3d<T>& operator+(const vector_3d<T> v1, const vector_3d<T> v2);

#include "math/vector_3d_fwd.h"

