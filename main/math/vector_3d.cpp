#include "vector_3d.h"

#include <cstdint>
#include <cassert>
#include <algorithm>

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
vector_3d<T>& vector_3d<T>::operator*=(const vector_3d<T>& v2) {
    x = x * v2.x;
    y = y * v2.y;
    z = z * v2.z;
    return *this;
}

template <typename T>
vector_3d<T> vector_3d<T>::operator*(const vector_3d<T>& v2) const {
    vector_3d<T> result(*this);
    return result *= v2;
}

template <typename T>
vector_3d<T>& vector_3d<T>::operator/=(const vector_3d<T>& v2) {
    x = x / v2.x;
    y = y / v2.y;
    z = z / v2.z;
    return *this;
}

template <typename T>
vector_3d<T> vector_3d<T>::operator/(const vector_3d<T>& v2) const {
    vector_3d<T> result(*this);
    return result /= v2;
}

template <typename T>
vector_3d<T>& vector_3d<T>::operator+=(const T s2)
{
    x = x + s2;
    y = y + s2;
    z = z + s2;
    return *this;
}

template <typename T>
template <typename S>
vector_3d<std::common_type_t<T, S>> vector_3d<T>::operator+(S s) const {
    using R = std::common_type_t<T, S>;
    return {static_cast<R>(x) + s, static_cast<R>(y) + s, static_cast<R>(z) + s};
}

template <typename T>
vector_3d<T>& vector_3d<T>::operator*=(const T s2)
{
    x = x * s2;
    y = y * s2;
    z = z * s2;
    return *this;
}

// template <typename T>
// vector_3d<T> vector_3d<T>::operator*(const T s2) const
// {
//     vector_3d<T> result(*this);
//     return result *= s2;
// }
template <typename T>
template <typename S>
vector_3d<std::common_type_t<T,S>>
vector_3d<T>::operator*(S s) const {
    using R = std::common_type_t<T,S>;
    return {static_cast<R>(x) * s, static_cast<R>(y) * s, static_cast<R>(z) * s};
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
template <typename S>
vector_3d<std::common_type_t<T,S>>
vector_3d<T>::operator/(S s) const {
    using R = std::common_type_t<T,S>;
    return {static_cast<R>(x) / s, static_cast<R>(y) / s, static_cast<R>(z) / s};
}

template <typename T>
T vector_3d<T>::dot(const vector_3d<T>& v2) const
{
    return (x*v2.x + y*v2.y + z*v2.z);
}

template <typename T>
vector_3d<T> vector_3d<T>::cross(const vector_3d<T> &v2) const
{
    vector_3d<T> tmp;
    tmp.x = y*v2.z - z*v2.y;
    tmp.y = z*v2.x - x*v2.z;
    tmp.z = x*v2.y - y*v2.x;
    return tmp;
}

template <typename T>
vector_3d<T> vector_3d<T>::normalize()
{
    double one_by_sqrt;
    double norm = get_norm();
    if ( norm > 0.00001 ) {
        one_by_sqrt = 1./norm;
        x = x*one_by_sqrt;
        y = y*one_by_sqrt;
        z = z*one_by_sqrt;
    }
    return *this;
}

template <typename T>
vector_3d<T> vector_3d<T>::get_normalized() const
{
    vector_3d<T> ret = *this;
    ret.normalize();
    return ret;
}

// template <typename T>
// vector_3d<std::common_type_t<T, float>> vector_3d<T>::clamp(float max_norm) const {
//     float n = get_norm();
//     if (n <= max_norm)
//         return *this;
//     return (*this) * (max_norm / n);
// }

template <typename T>
void vector_3d<T>::clamp_inplace(float max_norm)
{
    float n = get_norm();   // norm() sollte float/double liefern

    if (n <= max_norm || n < 1e-12f)
        return;

    float scale = max_norm / n;

    x = static_cast<T>(x * scale);
    y = static_cast<T>(y * scale);
    z = static_cast<T>(z * scale);
}

template <typename T>
vector_3d<T> vector_3d<T>::clamp(T minl, T maxl) {
    x = std::clamp(x, minl, maxl);
    y = std::clamp(y, minl, maxl);
    z = std::clamp(z, minl, maxl);
    return *this;
}


template class vector_3d<float>; // explicit instantiation
template class vector_3d<double>;
template class vector_3d<int>;
template class vector_3d<int16_t>;

template vector_3d<float>
vector_3d<float>::operator+<float>(float) const;

template vector_3d<float>
vector_3d<float>::operator*<float>(float) const;

template vector_3d<float>
vector_3d<float>::operator/<float>(float) const;