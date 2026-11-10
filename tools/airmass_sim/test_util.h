#pragma once

// The checks of the host tests: each prints PASS or FAIL with its name and counts the failures, which main returns.

#include <cmath>
#include <cstdio>

inline int failures = 0;

inline void check(bool ok, const char* name) {
    std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
    failures += !ok;
}

// a finite a within tolerance of b
inline bool near(float a, float b, float tolerance = 1e-5f) { return std::isfinite(a) && std::fabs(a - b) <= tolerance; }

inline void expect(const char* name, float value, float wanted, float tolerance) {
    const bool ok = near(value, wanted, tolerance);
    std::printf("%s: %-48s %10.6f (expected %10.6f)\n", ok ? "PASS" : "FAIL", name, value, wanted);
    failures += !ok;
}
