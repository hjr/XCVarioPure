#pragma once

#include <cmath>

template <typename T>
class vector_3d {
   public:
    vector_3d() = default;
    vector_3d(const vector_3d& o) {
        x = o.x;
        y = o.y;
        z = o.z;
    };
    vector_3d(T px, T py, T pz);
    static inline vector_3d make_vector(int16_t, int16_t, int16_t, float) = delete;
    vector_3d(vector_3d&&) = default;  // Allow the move optimization
    vector_3d<T>& operator=(const vector_3d<T>&) = default;

    // indexed access
    T& operator[](int i) { return (&x)[i]; }

    // API
    void set(const T p1, const T p2, const T p3) { x = p1, y = p2, z = p3; }
    // element wise operators
    vector_3d<T>& operator+=(const vector_3d<T>& v2);
    vector_3d<T> operator+(const vector_3d<T>& v2) const;
    vector_3d<T>& operator-=(const vector_3d<T>& v2);
    vector_3d<T> operator-(const vector_3d<T>& v2) const;
    vector_3d<T>& operator*=(const vector_3d<T>& v2);
    vector_3d<T> operator*(const vector_3d<T>& v2) const;
    vector_3d<T>& operator/=(const vector_3d<T>& v2);
    vector_3d<T> operator/(const vector_3d<T>& v2) const;
    // scalar operators
    vector_3d<T>& operator+=(const T s2);
    template <typename S>
    vector_3d<std::common_type_t<T, S>> operator+(S s) const;
    vector_3d<T>& operator*=(const T s2);
    //	vector_3d<T> operator*(const T s2) const;
    template <typename S>
    vector_3d<std::common_type_t<T, S>> operator*(S s) const;
    vector_3d<T>& operator/=(const T s2);
    template <typename S>
    vector_3d<std::common_type_t<T, S>> operator/(S s) const;
    T dot(const vector_3d<T>& v2) const;
    vector_3d<T> cross(const vector_3d<T>& v2) const;
    std::common_type_t<T, float> get_norm() const { return std::sqrt(x * x + y * y + z * z); }
    std::common_type_t<T, float> get_norm2() const { return x * x + y * y + z * z; }
    T normalize();
    vector_3d<T> get_normalized() const;
    // vector_3d<std::common_type_t<T, float>> clamp(float max_norm) const;
    void clamp_inplace(float max_norm);
    vector_3d<T> clamp(T minl, T maxl);
    // Interpretation as Euler angles
    float Roll() const { return x; }
    void setRoll(float roll) { x = roll; }
    float Pitch() const { return y; }
    void setPitch(float pitch) { y = pitch; }
    float Yaw() const { return z; }
    void setYaw(float yaw) { z = yaw; }
    T operator[](int i) const { return (&x)[i]; }

    // todo private:
    T x = T{};
    T y = T{};
    T z = T{};
};

template <>
inline vector_3d<float> vector_3d<float>::make_vector(int16_t a, int16_t b, int16_t c, float scale) {
    return {a * scale, b * scale, c * scale};
}

#include "math/vector_3d_fwd.h"
