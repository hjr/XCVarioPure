
#pragma once

#include "SensorBase.h"
#include "Filters.h"
#include "S2F.h"
#include "average.h"

#include <driver/gpio.h>


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
    float getAvgVario() const { return _avg_vario; }
    float getPolarSink() const { return _polar_sink; }

   private:
    LowPassFilterT<float> _tealt_lpf;
    void init(meter_t alt);
    uint32_t _prev_time = 0;
    Average<34, float, float> TEavg;
    Average<60, float, float> avgTE;
    int16_t _te_filter_idx;
    LowPassFilterT<float> _lpf{0.25f};
    int16_t _avg_filter_idx;
    // some usefull derived values
    float _avg_vario = 0.f;
    float _polar_sink = 0.f;
};

extern VarioFilter bmpVario;
