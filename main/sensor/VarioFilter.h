
#pragma once

#include "SensorBase.h"
#include "Filters.h"
#include "S2F.h"
#include "average.h"

#include <driver/gpio.h>


#define FILTER 1

//
// Implementation of a stable Vario with flight optimized Iltis-Kalman filter
//

class VarioFilter final : public SensorTP<float> {
   public:
    using TekComp_Type = enum : uint8_t { TE_TEK_PROBE, TE_TEK_EPOT, TE_TEK_PRESSURE, TE_TEK_IMU, TE_TEK_MAX_TYPES };

    VarioFilter();
    const char* name() const override { return "tek_vario"; }
    bool probe() override { return true; }
    bool setup() override;

    bool doRead(float& val) override;
    void postProcess() override;

    void configChange();
    void resetKF();  // cope with disruptive events like a QNH adjustment
    void prepareForSimJump() { _prepare_sim_jump = 40; } // prepare for a disruptive jump in altitude in simulation mode
    float getAvgVario() const { return _avg_vario; }
    float getPolarSink() const { return _polar_sink; }

   private:
    LowPassFilterT<float> _tealt_lpf;
    void init(meter_t alt);
    uint32_t _prev_time = 0;
    Average<34, float, float> TEavg;
#if FILTER == 0
    Average<60, float, float> avgTE;
#endif
    int16_t _te_filter_idx;
    LowPassFilterT<float> _lpf{0.25f};
    int16_t _avg_filter_idx;
    // some usefull derived values
    float _avg_vario = 0.f;
    float _polar_sink = 0.f;
    int8_t _prepare_sim_jump = 0;
};

extern VarioFilter bmpVario;
