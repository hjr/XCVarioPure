#include "vector_3d.h"

#include <cstdint>
#include <cassert>

template <typename T>
vector_3d<T>::vector_3d(T _a, T _b, T _c)
{
    x = _a;
    y = _b;
    z = _c;
}

template <typename T>
vector_3d<T>& vector_3d<T>::operator+=(const vector_3d<T>& v2)
{
    x = x + v2.x;
    y = y + v2.y;
    z = z + v2.z;
    return *this;
}

template <typename T>
vector_3d<T> vector_3d<T>::operator+(const vector_3d<T>& v2) const
{
    vector_3d<T> result(*this);
    return result += v2;
}

template <typename T>
vector_3d<T>& vector_3d<T>::operator-=(const vector_3d<T>& v2)
{
    x = x - v2.x;
    y = y - v2.y;
    z = z - v2.z;
    return *this;
}

template <typename T>
vector_3d<T> vector_3d<T>::operator-(const vector_3d<T>& v2) const
{
    vector_3d<T> result(*this);
    return result -= v2;
}

template <typename T>
vector_3d<T>& vector_3d<T>::operator*=(const T s2)
{
    x = x * s2;
    y = y * s2;
    z = z * s2;
    return *this;
}

template <typename T>
vector_3d<T> vector_3d<T>::operator*(const T s2)
{
    vector_3d<T> result(*this);
    return result *= s2;
}

template <typename T>
vector_3d<T>& vector_3d<T>::operator/=(const T s2)
{
    //if (s2 == 0) throw
    x = x / s2;
    y = y / s2;
    z = z / s2;
    return *this;
}

template <typename T>
T vector_3d<T>::dot(const vector_3d<T>& v2) const
{
    return (x*v2.x + y*v2.y + z*v2.z);
}

template <typename T>
vector_3d<T> vector_3d<T>::cross(const vector_3d &v2) const
{
    vector_3d tmp;
    tmp.x = y*v2.z - z*v2.y;
    tmp.y = z*v2.x - x*v2.z;
    tmp.z = x*v2.y - y*v2.x;
    return tmp;
}

template <typename T>
T vector_3d<T>::normalize()
{
    T one_by_sqrt;
    T norm = get_norm();
    if ( norm > 0.00001 ) {
        one_by_sqrt = 1/norm;
        x = x*one_by_sqrt;
        y = y*one_by_sqrt;
        z = z*one_by_sqrt;
    }
    return norm;
}

template <typename T>
vector_3d<T> vector_3d<T>::get_normalized() const
{
    vector_3d<T> ret = *this;
    ret.normalize();
    return ret;
}

template class vector_3d<float>; // explicit instantiation
template class vector_3d<double>;
template class vector_3d<int>;
template class vector_3d<int16_t>;
