#pragma once

#include <algorithm>
#include <cstdint>

namespace airmass {

// RMC UTC is an epoch identifier, not a clock synchronized to the sensor task. Differences between source
// epochs and arrival times reveal additional transport delay. A constant receiver delay remains learned by
// the EKF. Without source epochs we can only check arrival ordering, not detect re-timestamped duplicates.
class GnssTiming {
public:
    static constexpr uint32_t UNKNOWN = UINT32_MAX;
    static constexpr uint32_t DAY_MS = 86400000;

    void reset() { *this = GnssTiming(); }

    bool observe(uint32_t arrival, uint32_t epoch, uint32_t& delay) {
        delay = 0;
        if (_have_arrival && ((int32_t)(arrival - _arrival) < 0 ||
            (arrival == _arrival && epoch == UNKNOWN))) return false;
        if (epoch != UNKNOWN) {
            if (epoch >= DAY_MS) return false;
            if (_have_epoch) {
                int32_t source_dt = (int32_t)epoch - (int32_t)_epoch;
                if (source_dt < -(int32_t)DAY_MS / 2) source_dt += DAY_MS; // midnight
                if (source_dt > (int32_t)DAY_MS / 2) source_dt -= DAY_MS;
                if (source_dt <= 0) return false;
                const int64_t extra = (int64_t)_delay + (uint32_t)(arrival - _epoch_arrival) - source_dt;
                _delay = (uint32_t)std::clamp<int64_t>(extra, 0, DAY_MS);
            }
            _epoch = epoch;
            _epoch_arrival = arrival;
            _have_epoch = true;
            delay = _delay;
        }
        _arrival = arrival;
        _have_arrival = true;
        return true;
    }

private:
    uint32_t _arrival = 0, _epoch_arrival = 0, _epoch = 0, _delay = 0;
    bool _have_arrival = false, _have_epoch = false;
};

} // namespace airmass
