#pragma once

#include <cmath>
#include <cstddef>

// Small fixed-size row-major float matrix for embedded filters (e.g. EKF covariance).
// Header only, no heap usage.
template <size_t R, size_t C>
class Matrix {
   public:
    constexpr Matrix() = default;

    float& operator()(size_t r, size_t c) { return _m[r][c]; }
    float operator()(size_t r, size_t c) const { return _m[r][c]; }

    // force symmetry, for covariance matrices after updates
    void symmetrize() {
        static_assert(R == C, "symmetrize needs a square matrix");
        for (size_t i = 0; i < R; ++i) {
            for (size_t j = i + 1; j < C; ++j) {
                const float v = 0.5f * (_m[i][j] + _m[j][i]);
                _m[i][j] = _m[j][i] = v;
            }
        }
    }

    bool allFinite() const {
        for (size_t i = 0; i < R; ++i) {
            for (size_t j = 0; j < C; ++j) {
                if (!std::isfinite(_m[i][j])) {
                    return false;
                }
            }
        }
        return true;
    }

   private:
    float _m[R][C] = {};
};

// Scalar (sequential) Kalman measurement update, avoids any matrix inversion.
//   h:  measurement Jacobian row
//   y:  innovation (z - h(x))
//   r:  measurement variance
//   gate: innovation gate in sigma, <= 0 disables gating
// Returns false if the measurement got rejected by the gate (state untouched).
template <size_t N>
bool ekfScalarUpdate(Matrix<N, N>& P, float (&x)[N], const float (&h)[N], float y, float r, float gate = 0.f) {
    float pht[N];
    float s = r;
    for (size_t i = 0; i < N; ++i) {
        float acc = 0.f;
        for (size_t j = 0; j < N; ++j) {
            acc += P(i, j) * h[j];
        }
        pht[i] = acc;
        s += h[i] * acc;
    }
    if (!(s > 0.f) || !std::isfinite(y)) {
        return false;
    }
    if (gate > 0.f && y * y > gate * gate * s) {
        return false;
    }
    const float inv_s = 1.f / s;
    for (size_t i = 0; i < N; ++i) {
        x[i] += pht[i] * inv_s * y;
    }
    // P = P - K * (H P), with K = P H' / s and (H P) = (P H')' for symmetric P
    for (size_t i = 0; i < N; ++i) {
        const float ki = pht[i] * inv_s;
        for (size_t j = 0; j < N; ++j) {
            P(i, j) -= ki * pht[j];
        }
    }
    P.symmetrize();
    return true;
}
