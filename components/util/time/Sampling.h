#pragma once

#include <cstdint>

// Source clocks, not polling clocks: a held reading is not another independent measurement.
struct SampleClock {
    uint32_t time_ms = 0;
    uint32_t interval_ms = 0;
    bool have = false;

    bool observe(uint32_t now, uint32_t stamp, bool valid) {
        if (!valid || (int32_t)(now - stamp) < 0 || (have && (int32_t)(stamp - time_ms) <= 0)) return false;
        interval_ms = have ? stamp - time_ms : 0;
        time_ms = stamp;
        have = true;
        return true;
    }
    uint32_t age(uint32_t now) const { return have ? now - time_ms : UINT32_MAX; }
};

// Wrap-safe elapsed-time scheduling. Missed periods do not produce a burst of repeated work.
struct Periodic {
    uint32_t last_ms = 0;
    bool started = false;
    bool due(uint32_t now, uint32_t period_ms) {
        if (started && now - last_ms < period_ms) return false;
        last_ms = now;
        started = true;
        return true;
    }
};
