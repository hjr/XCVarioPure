
#pragma once

#include "SensorBase.h"
#include "Filters.h"
#include "S2F.h"
#include "average.h"

#include <driver/gpio.h>


#define FILTER 0

//
// Implementation of a stable Vario with flight optimized Iltis-Kalman filter
//

class VarioFilter final : public SensorTP<meter_t> {
   public:
    using TekComp_Type = enum : uint8_t { TE_TEK_PROBE, TE_TEK_EPOT, TE_TEK_PRESSURE, TE_TEK_IMU, TE_TEK_MAX_TYPES };

    VarioFilter();
    const char* name() const override { return "tek_vario"; }
    bool probe() override { return true; }
    bool setup() override;

    bool doRead(meter_t& val) override;
    void postProcess() override;

    void configChange();
    void prepareForSimJump() { _prepare_sim_jump = 40; } // prepare for a disruptive jump in altitude in simulation mode
    float getAvgVario() const { return _avg_vario; }
    float getPolarSink() const { return _polar_sink; }

   private:
    void init(meter_t alt);

    LowPassFilterT<float> _tealt_lpf;
    uint32_t _prev_time = 0;
    Average<34, float, float> TEavg;
#if FILTER == 0
    Average<60, float, float> avgTE;
#endif
    LowPassFilterT<float> _lpf{0.25f};
    int16_t _avg_filter_idx;
    // some usefull derived values
    float _avg_vario = 0.f;
    float _polar_sink = 0.f;
    int8_t _prepare_sim_jump = 0;
};

extern VarioFilter bmpVario;
