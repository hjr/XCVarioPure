#pragma once

// rms and maximum of an error series, for the simulation and the replay

#include <cmath>

struct Stats {
    double sum2 = 0;
    long n = 0;
    double maxabs = 0;
    void add(double e) {
        sum2 += e * e;
        n++;
        maxabs = std::fmax(maxabs, std::fabs(e));
    }
    double rms() const { return n ? std::sqrt(sum2 / n) : NAN; }
};
