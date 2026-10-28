#pragma once

// Pitch/AoA offset per flap setting. Flaps shift the zero lift angle of attack by several degrees, so the
// estimator's pitch offset is learned and kept for each lever position (notch). While the lever moves between
// two notches the offset follows it, interpolated, since the flaps move with the lever.
// Plain C++, verified by tools/airmass_sim (asw20 tests).

#include "AirMassEKF.h"

#include <cmath>

namespace airmass {

class FlapOffsets {
public:
    static constexpr int N = 7;               // Flap::MAX_NR_POS
    static constexpr float UNKNOWN = 1.f;     // rad, marks a position never learned (beyond any plausible offset)
    static constexpr float SIGMA_NEW = 0.08f; // rad, a position flown the first time: flaps shift by degrees
    static constexpr float SIGMA_MIN = 0.003f;// rad, a learned position
    static constexpr float NOTCH = 0.25f;     // lever positions: a reading this close to a notch is at the notch

    // learned offsets per position, UNKNOWN where never learned
    void load(const float ofs[N]) {
        for (int i = 0; i < N; ++i) {
            _ofs[i] = ofs[i];
            _sig[i] = known(i) ? SIGMA_MIN : SIGMA_NEW;
        }
        _slot = -1;
        _active = false;
    }

    // Called every cycle with the lever position (0 .. N-1, fractional between notches; NAN without a flap
    // sensor): sets the estimator's pitch offset for the lever.
    void update(AirMassEKF& ekf, float lever) {
        if (!std::isfinite(lever)) {
            return;
        }
        _active = true;
        lever = std::fmin(std::fmax(lever, 0.f), N - 1.f);
        const int notch = std::lround(lever);
        if (std::fabs(lever - notch) <= NOTCH) {
            // at a notch: the estimator learns its offset
            if (notch != _slot) {
                keep(ekf);
                _slot = notch;
                ekf.setPitchOffset(guess(notch, ekf), std::fmax(_sig[notch], SIGMA_MIN)); // never flown: SIGMA_NEW
            }
            return;
        }
        // moving between two notches: follow the lever
        keep(ekf);
        _slot = -1;
        const int lo = static_cast<int>(lever);
        const float frac = (lever - lo - NOTCH) / (1.f - 2.f * NOTCH);
        const float ofs = guess(lo, ekf) + frac * (guess(lo + 1, ekf) - guess(lo, ekf));
        ekf.setPitchOffset(ofs, std::fmax(std::fmax(_sig[lo], _sig[lo + 1]), SIGMA_MIN));
    }

    // take over what the estimator learned for the current notch
    void keep(const AirMassEKF& ekf) {
        if (_slot >= 0) {
            _ofs[_slot] = ekf.pitchOffset();
            _sig[_slot] = ekf.sigma(AirMassEKF::THB);
        }
    }

    bool active() const { return _active; } // a flap lever is read
    float offset(int i) const { return _ofs[i]; }
    float sigma(int i) const { return _sig[i]; }
    bool known(int i) const { return std::fabs(_ofs[i]) < UNKNOWN; }

private:
    // the offset of a notch; never flown: the nearest learned one, or what the estimator has now
    float guess(int i, const AirMassEKF& ekf) const {
        if (known(i)) {
            return _ofs[i];
        }
        const int near = nearestKnown(i);
        return near >= 0 ? _ofs[near] : ekf.pitchOffset();
    }

    int nearestKnown(int i) const {
        for (int d = 1; d < N; ++d) {
            if (i - d >= 0 && known(i - d)) return i - d;
            if (i + d < N && known(i + d)) return i + d;
        }
        return -1;
    }

    float _ofs[N] = {UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN};
    float _sig[N] = {SIGMA_NEW, SIGMA_NEW, SIGMA_NEW, SIGMA_NEW, SIGMA_NEW, SIGMA_NEW, SIGMA_NEW};
    int _slot = -1;
    bool _active = false;
};

} // namespace airmass
